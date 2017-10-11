#***********************************************************
# This version of the application is for communication with
# measurement boards on the I2C bus
#***********************************************************

from Tkinter import *
import ttk
import tkFont
from ConfigParser import ConfigParser
import BME_280
import alicat
import time
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
from matplotlib.figure import Figure
import matplotlib.animation as an
import chip_fixture
import serial
from datetime import datetime as dt
from threading import Thread

class MFC_ControlThread(Thread):
	def __init__(self,num_seconds):
		Thread.__init__(self)
		self.num_seconds = num_seconds
		
	def run(self):
		sec = 1
		while(sec < self.num_seconds):
			print "waiting %d"%(sec)
			sec += 1
			time.sleep(1)

debug=True
MFCdebug=False

GPIO_TRIGGER_CHANNEL = 23

import platform
if platform.system() == 'Windows':
    pass
elif platform.system() == 'Linux':
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    
    GPIO.setup(GPIO_TRIGGER_CHANNEL,GPIO.OUT)
    GPIO.output(GPIO_TRIGGER_CHANNEL,0)
    
    import pigpio
    import smbus
    bus = smbus.SMBus(1)	#I2C bus



class mfc_display:
	def __init__(self):
		self.flowController = None

		self.address = None
		self.flow_setpoint = None
		self.temperature = None
		self.mass_flow = None
		self.mass_flow2 = None
		self.volumetric_flow = None
		self.gas = None
		self.pressure = None

	def update_readings(self):
		readings = self.flowController.get()

		self.flow_setpoint['text'] = "%.2f"%(readings['flow_setpoint'])
		self.temperature['text'] = "%.2f"%(readings['temperature'])
		self.mass_flow['text'] = "%.2f"%(readings['mass_flow'])
		self.mass_flow2['text'] = "%.2f"%(readings['mass_flow'])
		self.volumetric_flow['text'] = "%.2f"%(readings['volumetric_flow'])
		self.gas['text'] = "%s"%(readings['gas'])
		self.pressure['text'] = "%.2f"%(readings['pressure'])
		
	def get_id(self):
		return self.address
	
	def get_gas(self):
		return self.gas['text']
	
	def get_flow_setpoint(self):
		return self.flow_setpoint['text']
	
	def get_mass_flow(self):
		return self.mass_flow['text']

	def close(self):
		self.flowController.close()

class App:
	def __init__(self):
		self.root = Tk()

		self.mainPanel = Frame(self.root)
		self.mainPanel.grid(sticky=N)

		self.tabs = ttk.Notebook(self.mainPanel)
		self.tabs.grid(row=0,column=0,padx=10,pady=10)
		self.tabs.bind("<<NotebookTabChanged>>",self.onTabChanged)
		self.tab1_frame = Frame(self.tabs)
		self.tab2_frame = Frame(self.tabs)
		self.tab3_frame = Frame(self.tabs)

		self.read_config_file()
		self.search_I2C_devices()
		self.create_chip_fixture_instances()
		self.search_mfc_devices()
		self.create_mfc_instances()
		self.create_circuit_control_section()

		self.create_mfc_displays()
		self.create_test_configuration_section()
		self.create_fixture_monitoring_section()

		self.tabs.add(self.tab1_frame,text='Fixture Control')
		self.tabs.add(self.tab2_frame,text='Test Condition Setup')
		self.tabs.add(self.tab3_frame,text='Test Circuit Control')

		self.root.protocol("WM_DELETE_WINDOW", self.onExit)

		self.create_menu()

		self.plot_span_seconds = 30
		self.sampleTime = 0
		self.sampleTimes = []
		self.temperatures = []
		self.humidities = []
		self.t_color = 'g'
		self.create_plot()
		self.monitor_state = 'stopped'
		self.samples_since_write = 0

		if platform.system() == 'Linux':
			self.pi = pigpio.pi()

			if not self.pi.connected:
				raise RuntimeError("Pi not connected. Check pigpio use")

		self.onUpdate()
		
#		t=MFC_ControlThread(15)
#		t.start()
		
		self.root.title("NEA Sensor Research")
		self.root.geometry('1350x950')
		self.root.mainloop()

	def read_config_file(self):
		self.config_file_name = "settings.cfg"
		self.c_file = open(self.config_file_name, 'r')
		self.config_file = ConfigParser()
		self.config_file.readfp(self.c_file)

		self.MFC_com_port = self.config_file.get('MFC Controls','MFC COM port')
		self.MFC_baud_rate = int(self.config_file.get('MFC Controls','MFC baud rate'))
		self.MFC_search_delay = float(self.config_file.get('MFC Controls','MFC search response delay msec')) / 1000.0

		self.sample_interval_msec = int(self.config_file.get('Measurement Controls','sample interval msec'))
		self.trigger_width_msec = int(self.config_file.get('Measurement Controls','measurement trigger pulse width msec'))
		self.read_delay_msec_after_trigger = int(self.config_file.get('Measurement Controls','msec delay after trigger'))
		
		self.DAC1_voltage_min_setting = float(self.config_file.get('Circuit Controls','DAC1 min voltage setting'))
		self.DAC1_voltage_max_setting = float(self.config_file.get('Circuit Controls','DAC1 max voltage setting'))
		self.DAC2_voltage_min_setting = float(self.config_file.get('Circuit Controls','DAC2 min voltage setting'))
		self.DAC2_voltage_max_setting = float(self.config_file.get('Circuit Controls','DAC2 max voltage setting'))
		self.min_Rbias_setting = int(self.config_file.get('Circuit Controls','min Rbias setting'))
		self.max_Rbias_setting = int(self.config_file.get('Circuit Controls','max Rbias setting'))
		self.min_measurement_delay_msec = int(self.config_file.get('Circuit Controls','min measurement delay msec'))
		self.max_measurement_delay_msec = int(self.config_file.get('Circuit Controls','max measurement delay msec'))
		self.min_sensor_number = int(self.config_file.get('Circuit Controls','min sensor number'))
		self.max_sensor_number = int(self.config_file.get('Circuit Controls','max sensor number'))	

		self.plot_span_seconds = int(self.config_file.get('Plot Controls','scroll seconds'))
		self.scroll_checkbox_tmp = self.config_file.getboolean('Plot Controls','scrolling enabled')

		self.outDir = self.config_file.get('Output','data path')
		self.samples_before_write = int(self.config_file.get('Output','samples before write'))

		self.num_comments = int(self.config_file.get('User Input','number of comments'))
		self.num_test_conditions = int(self.config_file.get('User Input','number of test conditions'))
		self.num_sensors_on_chip = int(self.config_file.get('User Input','number of sensors on chip'))

	def search_I2C_devices(self):
		if debug: print "Searching for connected I2C devices..."
		self.fixture_addresses = []
		self.fixture_addr_strings = []
		for id in range(128):
			try:
				bus.write_quick(id)
				self.fixture_addresses.append(id)
				self.fixture_addr_strings.append("ID: %#x"%(id))
			except:
				pass
			
		if len(self.fixture_addresses) == 0:
			self.fixture_addr_strings.append("None")
			
		if debug: print "Found I2C device addresses: ",self.fixture_addr_strings
		
	def create_chip_fixture_instances(self):
		self.i2c_test_fixtures = []
		for id in self.fixture_addresses:
			self.i2c_test_fixtures.append(chip_fixture.ChipFixture(id,bus))

	def search_mfc_devices(self):
		if debug: print "Searching for connected MFCs..."
		ser = serial.Serial(self.MFC_com_port, self.MFC_baud_rate, timeout=0.1)
		self.found_MFC_IDs = []

		#scan ascii 'A' to ascii 'Z' for connected MFCs
		for id in range(65,91,1):
			if MFCdebug: print "searching %s: "%(chr(id))
			ser.flushInput()
			ser.flushOutput()
			ser.write("%s\r"%(chr(id)))
			time.sleep(self.MFC_search_delay)
			ret=ser.read(1)
			if MFCdebug: print "  %s"%(ret)
			if ret==chr(id):
				self.found_MFC_IDs.append(chr(id))

		self.num_connected_MFCs = len(self.found_MFC_IDs)
		if debug: print "Found MFCs: ",self.found_MFC_IDs
		
		return self.found_MFC_IDs
		
	def create_mfc_instances(self):
		if platform.system() == 'Linux':
			self.connected_MFCs = []
			
			for mfc_id in self.found_MFC_IDs:
				mfc = alicat.FlowController(address=mfc_id,baud=self.MFC_baud_rate)
				self.connected_MFCs.append(mfc)
				
	def onNumLoops_click(self):
		pass
		
	def onNumLoops_enter(self,event=None):
		self.tab2_frame.focus_set()
		
		new_val = float(self.num_loops.get())
		
		if new_val < 0:
			new_val = 0
			self.num_loops.set(new_val)

		elif new_val > 999:
			new_val = 999
			self.num_loops.set(new_val)
		
	def onDAC1_click(self):
		fixture_index = self.fixture_addr_strings.index(self.selected_fixture_id.get())
		self.i2c_test_fixtures[fixture_index].setDAC(1,float(self.DAC1_voltage.get()))

	def onDAC1_enter(self,event=None):
		new_val = float(self.DAC1_voltage.get())
		
		if new_val < self.DAC1_voltage_min_setting:
			new_val = self.DAC1_voltage_min_setting
			self.DAC1_value.set(new_val)

		elif new_val > self.DAC1_voltage_max_setting:
			new_val = self.DAC1_voltage_max_setting
			self.DAC1_value.set(new_val)

		self.onDAC1_click()
		self.focus_to_tab()

	def onDAC2_click(self):
		fixture_index = self.fixture_addr_strings.index(self.selected_fixture_id.get())
		self.i2c_test_fixtures[fixture_index].setDAC(2,float(self.DAC2_voltage.get()))

	def onDAC2_enter(self,event=None):
		new_val = float(self.DAC2_voltage.get())
		
		if new_val < self.DAC2_voltage_min_setting:
			new_val = self.DAC2_voltage_min_setting
			self.DAC2_value.set(new_val)
			
		elif new_val > self.DAC2_voltage_max_setting:
			new_val = self.DAC2_voltage_max_setting
			self.DAC2_value.set(new_val)

		self.onDAC2_click()
		self.focus_to_tab()

	def onDelayMsec_spinClick(self):
		fixture_index = self.fixture_addr_strings.index(self.selected_fixture_id.get())
		self.i2c_test_fixtures[fixture_index].setMeasurementDelayMsec(int(self.delay_msec.get()))

	def onDelayMsec_enter(self,event=None):
		new_val = float(self.delay_msec.get())
		
		if new_val < self.min_measurement_delay_msec:
			new_val = self.min_measurement_delay_msec
			self.delay_msec.set(new_val)
			
		elif new_val > self.max_measurement_delay_msec:
			new_val = self.max_measurement_delay_msec
			self.delay_msec.set(new_val)

		self.onDelayMsec_spinClick()
		self.focus_to_tab()

	def onSelectedSensor_spinClick(self):
		fixture_index = self.fixture_addr_strings.index(self.selected_fixture_id.get())
		self.i2c_test_fixtures[fixture_index].selectSensor(int(self.selected_sensor.get()))

	def onSelectedSensor_enter(self,event=None):
		new_val = float(self.selected_sensor.get())
		
		if new_val < self.min_sensor_number:
			new_val = self.min_sensor_number
			self.selected_sensor.set(new_val)
			
		elif new_val > self.max_sensor_number:
			new_val = self.max_sensor_number
			self.selected_sensor.set(new_val)

		self.onSelectedSensor_spinClick()
		self.focus_to_tab()
		
	def onRbias_click(self):
		fixture_index = self.fixture_addr_strings.index(self.selected_fixture_id.get())
		self.i2c_test_fixtures[fixture_index].selectRbias(int(self.selected_Rbias.get()))

	def onRbias_enter(self,event=None):
		new_val = float(self.selected_Rbias.get())
		
		if new_val < self.min_Rbias_setting:
			new_val = self.min_Rbias_setting
			self.selected_Rbias.set(new_val)
			
		elif new_val > self.max_Rbias_setting:
			new_val = self.max_Rbias_setting
			self.selected_Rbias.set(new_val)

		self.onRbias_click()
		self.focus_to_tab()
		
	def onUsingCCcircuit_clicked(self):
		fixture_index = self.fixture_addr_strings.index(self.selected_fixture_id.get())
		if self.usingCCcircuit.get():
			self.i2c_test_fixtures[fixture_index].selectConstantCurrent()
		else:
			self.i2c_test_fixtures[fixture_index].selectConstantVoltage()
		
	def onSensorsGrounded_clicked(self):
		fixture_index = self.fixture_addr_strings.index(self.selected_fixture_id.get())
		if self.sensorsGrounded.get():
			self.i2c_test_fixtures[fixture_index].groundSensors()
		else:
			self.i2c_test_fixtures[fixture_index].ungroundSensors()
		
	def onCcCircuitGrounded_clicked(self):
		fixture_index = self.fixture_addr_strings.index(self.selected_fixture_id.get())
		if self.ccCircuitGrounded.get():
			self.i2c_test_fixtures[fixture_index].groundCCcircuit()
		else:
			self.i2c_test_fixtures[fixture_index].ungroundCCcircuit()
		
	def onCcCircuitAutorange_clicked(self):
		fixture_index = self.fixture_addr_strings.index(self.selected_fixture_id.get())
		if self.ccCircuitAutorange.get():
			self.i2c_test_fixtures[fixture_index].enableAutorange()
		else:
			self.i2c_test_fixtures[fixture_index].disableAutorange()

	def onTabChanged(self,event):
		selection = event.widget.select()
		tab = event.widget.tab(selection,"text")

		if tab == 'Test Circuit Control':
			self.update_displayed_circuit_status()

	def update_displayed_circuit_status(self):
		fixture_index = self.fixture_addr_strings.index(self.selected_fixture_id.get())
		returned_status_data = self.i2c_test_fixtures[fixture_index].readStatus()
		
		if len(returned_status_data) == 5:
			self.DAC1_voltage.set("%.3f"%(returned_status_data[0]))
			self.DAC2_voltage.set("%.3f"%(returned_status_data[1]))
			
			self.selected_Rbias.set((returned_status_data[2] & 0x03) + 1)
			
			if (returned_status_data[2] & 0x4):
				self.usingCCcircuit.set(1)
			else:
				self.usingCCcircuit.set(0)

			if (returned_status_data[2] & 0x8):
				self.sensorsGrounded.set(1)
			else:
				self.sensorsGrounded.set(0)

			if (returned_status_data[2] & 0x10):
				self.ccCircuitGrounded.set(1)
			else:
				self.ccCircuitGrounded.set(0)

			if (returned_status_data[2] & 0x20):
				self.ccCircuitAutorange.set(1)
			else:
				self.ccCircuitAutorange.set(0)

			self.delay_msec.set(returned_status_data[3])
			self.selected_sensor.set(returned_status_data[4])
			
	def focus_to_tab(self):
		self.tab3_frame.focus_set()

	def create_circuit_control_section(self):
		self.circuit_control_frame = LabelFrame(self.tab3_frame,padx=3,pady=3)
		self.circuit_control_frame.grid(row=0,column=0,padx=10,pady=10,sticky=NW)

		#DAC1 voltage spin control
		self.DAC1_voltage = StringVar()
		self.DAC1_voltage_spin = Spinbox(self.circuit_control_frame,from_=0,to=2.5,
						increment=0.05,width=6,command=self.onDAC1_click,textvariable=self.DAC1_voltage)
		self.DAC1_voltage_spin.grid(row=2,column=5,pady=2,sticky=E)
		self.DAC1_voltage_spin.bind('<Return>',self.onDAC1_enter)
		self.DAC1_voltage_spin.bind('<KP_Enter>',self.onDAC1_enter)
		Label(self.circuit_control_frame,text="DAC1 voltage(volts)").grid(row=2,column=10,sticky=W)

		#DAC2 voltage spin control
		self.DAC2_voltage = StringVar()
		self.DAC2_voltage_spin = Spinbox(self.circuit_control_frame,from_=0,to=2.5,
						increment=0.05,width=6,command=self.onDAC2_click,textvariable=self.DAC2_voltage)
		self.DAC2_voltage_spin.grid(row=3,column=5,pady=2,sticky=E)
		self.DAC2_voltage_spin.bind('<Return>',self.onDAC2_enter)
		self.DAC2_voltage_spin.bind('<KP_Enter>',self.onDAC2_enter)
		Label(self.circuit_control_frame,text="DAC2 voltage(volts)").grid(row=3,column=10,sticky=W)

		ttk.Separator(self.circuit_control_frame,orient='horizontal').grid(row=4,column=5,columnspan=10,sticky='ew',pady=5)

		#selected bias resistor in constant current circuit
		self.selected_Rbias = StringVar()
		self.selected_Rbias_spin = Spinbox(self.circuit_control_frame,from_=1,to=4,
						increment=1,width=6,command=self.onRbias_click,textvariable=self.selected_Rbias)
		self.selected_Rbias_spin.grid(row=5,column=5,pady=2,sticky=E)
		self.selected_Rbias_spin.bind('<Return>',self.onRbias_enter)
		self.selected_Rbias_spin.bind('<KP_Enter>',self.onRbias_enter)
		Label(self.circuit_control_frame,text="Selected Constant Current Bias Resistor(1-4)").grid(row=5,column=10,sticky=W)

		#using either constant current or constant voltage circuit
		self.usingCCcircuit = IntVar()
		self.usingCCcircuitCb = Checkbutton(self.circuit_control_frame,variable=self.usingCCcircuit,
											command=self.onUsingCCcircuit_clicked,onvalue=1,offvalue=0)
		self.usingCCcircuitCb.grid(row=7,column=5)
		Label(self.circuit_control_frame,text="Use Constant Current Circuit").grid(row=7,column=10,sticky=W)

		#one side of sensors grounded if using chips with 22 sensors
		self.sensorsGrounded = IntVar()
		self.sensorsGroundedCb = Checkbutton(self.circuit_control_frame,variable=self.sensorsGrounded,
											command=self.onSensorsGrounded_clicked,onvalue=1,offvalue=0)
		self.sensorsGroundedCb.grid(row=10,column=5)
		Label(self.circuit_control_frame,text="Fixture Grounds One Side of Sensors").grid(row=10,column=10,sticky=W)

		#constant current circuit grounded through 1kohm resistor
		self.ccCircuitGrounded = IntVar()
		self.ccCircuitGroundedCb = Checkbutton(self.circuit_control_frame,variable=self.ccCircuitGrounded,
												command=self.onCcCircuitGrounded_clicked,onvalue=1,offvalue=0)
		self.ccCircuitGroundedCb.grid(row=15,column=5)
		Label(self.circuit_control_frame,text="Constant Current Circuit Grounded").grid(row=15,column=10,sticky=W)

		#autorange in constant current circuit turned off or on
		self.ccCircuitAutorange = IntVar()
		self.ccCircuitAutorangeCb = Checkbutton(self.circuit_control_frame,variable=self.ccCircuitAutorange,
												command=self.onCcCircuitAutorange_clicked,onvalue=1,offvalue=0)
		self.ccCircuitAutorangeCb.grid(row=20,column=5)
		Label(self.circuit_control_frame,text="Autorange Constant Current Circuit Measurements").grid(row=20,column=10,sticky=W)

		ttk.Separator(self.circuit_control_frame,orient='horizontal').grid(row=25,column=5,columnspan=10,sticky='ew',pady=5)

		#delay between sensor selection and adc measurement in milliseconds
		self.delay_msec = StringVar()
		self.delay_msec_spin = Spinbox(self.circuit_control_frame,from_=0,to=255,
						increment=1,width=6,command=self.onDelayMsec_spinClick,textvariable=self.delay_msec)
		self.delay_msec_spin.grid(row=30,column=5,pady=2,sticky=E)
		self.delay_msec_spin.bind('<Return>',self.onDelayMsec_enter)
		self.delay_msec_spin.bind('<KP_Enter>',self.onDelayMsec_enter)
		Label(self.circuit_control_frame,text="Delay Between Sensor Selection and ADC Read(msec)").grid(row=30,column=10,sticky=W)

		#sensor number that is currently selected
		self.selected_sensor = StringVar()
		self.selected_sensor_spin = Spinbox(self.circuit_control_frame,from_=0,to=22,
						increment=1,width=6,command=self.onSelectedSensor_spinClick,textvariable=self.selected_sensor)
		self.selected_sensor_spin.grid(row=35,column=5,pady=2,sticky=E)
		self.selected_sensor_spin.bind('<Return>',self.onSelectedSensor_enter)
		self.selected_sensor_spin.bind('<KP_Enter>',self.onSelectedSensor_enter)
		Label(self.circuit_control_frame,text="Selected Sensor").grid(row=35,column=10,sticky=W)

	def create_mfc_displays(self):
		self.mfc_monitors_frame = LabelFrame(self.tab1_frame,text="MFC Monitors",padx=3,pady=3)
		self.mfc_monitors_frame.grid(row=0,column=1,columnspan=3,padx=10,pady=10,sticky=NW)

		small_mfc_font = tkFont.Font(family="Helvetica",size=9)
		big_mfc_font = tkFont.Font(family="Helvetica",size=20)
		self.mfc_monitors = []

		for i in range(self.num_connected_MFCs):
			m = mfc_display()

			Label(self.mfc_monitors_frame,text="PSIA",font=small_mfc_font).grid(row=0,column=4*i+0)
			lbl=Label(self.mfc_monitors_frame,text="14.00",font=small_mfc_font)
			lbl.grid(row=1,column=4*i+0)
			m.pressure = lbl
			Label(self.mfc_monitors_frame,text="*C",font=small_mfc_font).grid(row=0,column=4*i+1)
			lbl=Label(self.mfc_monitors_frame,text="25.76",font=small_mfc_font)
			lbl.grid(row=1,column=4*i+1,padx=8)
			m.temperature = lbl
			Label(self.mfc_monitors_frame,text="SETPT",font=small_mfc_font).grid(row=0,column=4*i+2)
			lbl=Label(self.mfc_monitors_frame,text="0.0",font=small_mfc_font)
			lbl.grid(row=1,column=4*i+2)
			m.flow_setpoint = lbl
			Label(self.mfc_monitors_frame,text=" ",font=small_mfc_font).grid(row=2,column=4*i+0)
			Label(self.mfc_monitors_frame,text="SCCM",font=small_mfc_font).grid(row=3,column=4*i+2)
			lbl=Label(self.mfc_monitors_frame,text="Air",font=small_mfc_font)
			lbl.grid(row=4,column=4*i+2)
			m.gas = lbl
			lbl=Label(self.mfc_monitors_frame,text="1.3",font=big_mfc_font)
			lbl.grid(row=3,rowspan=3,column=4*i+0,columnspan=2,sticky=NE)
			m.mass_flow = lbl
			Label(self.mfc_monitors_frame,text=" ",font=small_mfc_font).grid(row=5,column=4*i+0)
			lbl=Label(self.mfc_monitors_frame,text="1.3",font=small_mfc_font)
			lbl.grid(row=6,column=4*i+0)
			m.volumetric_flow = lbl
			Label(self.mfc_monitors_frame,text="CCM",font=small_mfc_font).grid(row=7,column=4*i+0)
			lbl=Label(self.mfc_monitors_frame,text="1.2",font=small_mfc_font)
			lbl.grid(row=6,column=4*i+1)
			m.mass_flow2 = lbl
			Label(self.mfc_monitors_frame,text="SCCM",font=small_mfc_font).grid(row=7,column=4*i+1)

			Label(self.mfc_monitors_frame,text=self.found_MFC_IDs[i],font=small_mfc_font).grid(row=6,column=4*i+2)
			Label(self.mfc_monitors_frame,text="ID",font=small_mfc_font).grid(row=7,column=4*i+2)

			m.flowController = self.connected_MFCs[i]
			m.address = self.found_MFC_IDs[i]
			m.update_readings()

			self.mfc_monitors.append(m)

			if i != self.num_connected_MFCs-1:
				ttk.Separator(self.mfc_monitors_frame,orient='vertical').grid(row=0,column=4*i+3,rowspan=7,sticky='ns',padx=10)

	def create_test_configuration_section(self):
		mfc_control_frame = LabelFrame(self.tab2_frame,text="Test Configuration")
		mfc_control_frame.grid(row=0,column=0,padx=10,pady=10,ipadx=5,ipady=5,sticky=NW)

		for mfc_num in range(self.num_connected_MFCs):
			#MFC ID for communication
			Label(mfc_control_frame,text="      MFC ID:").grid(row=1,column=3+mfc_num*4,sticky=E)
			mfc_id = Label(mfc_control_frame,text=self.found_MFC_IDs[mfc_num],
					borderwidth=2,relief='sunken',width=2)
			mfc_id.grid(row=1,column=4+mfc_num*4,sticky=W)
			#type of gas MFC will control
			Label(mfc_control_frame,text="gas:").grid(row=1,column=5+mfc_num*4,sticky=E)
			gas = Label(mfc_control_frame,text=self.mfc_monitors[mfc_num].get_gas(),
					borderwidth=2,relief='sunken',width=5)
			gas.grid(row=1,column=6+mfc_num*4,sticky=W)

		self.test_condition_checkboxes = []
		test_condition_times = []
		test_condition_settings = []
		for test_condition in range(self.num_test_conditions):
			#checkbox to enable test condition
			cb = Checkbutton(mfc_control_frame)
			cb.grid(row=2+test_condition,column=0,sticky=W,pady=3)
			self.test_condition_checkboxes.append(cb)
			#time this test condition will last
			Label(mfc_control_frame,text="seconds:").grid(row=2+test_condition,column=1,sticky=E)
			e = Entry(mfc_control_frame,width=5)
			e.grid(row=2+test_condition,column=2,sticky=W)
			test_condition_times.append(e)
			for mfc_num in range(self.num_connected_MFCs):
				Label(mfc_control_frame,text="scfm:").grid(row=2+test_condition,column=4+mfc_num*4,sticky=E)
				e = Entry(mfc_control_frame,width=5)
				e.grid(row=2+test_condition,column=5+mfc_num*4,sticky=W)
				test_condition_settings.append(e)
				
		#checkbox for loop control
		self.loop_cb = Checkbutton(mfc_control_frame)
		self.loop_cb.grid(row=2+self.num_test_conditions,column=0,sticky=W,pady=3)
		Label(mfc_control_frame,text="Loop All Test Conditions").grid(row=2+self.num_test_conditions,column=1,columnspan=3,sticky=W)
		self.num_loops = StringVar()
		self.num_loops_spin = Spinbox(mfc_control_frame,from_=0,to=999,
									increment=1,width=5,textvariable=self.num_loops)
		self.num_loops_spin.grid(row=2+self.num_test_conditions,column=4,columnspan=2,sticky=W)
		self.num_loops_spin.bind('<Return>',self.onNumLoops_enter)
		self.num_loops_spin.bind('<KP_Enter>',self.onNumLoops_enter)
		
		#user input to describe chips and test conditions
		test_description_frame = LabelFrame(self.tab2_frame,text="Test Description")
		test_description_frame.grid(row=5,column=0,padx=10,pady=10,ipadx=15,ipady=40,sticky=W)
		#descriptions for each test fixture
		self.fixture_descriptions = []
		for i in range(len(self.fixture_addresses)):
			Label(test_description_frame,text="Fixture "+self.fixture_addr_strings[i]).grid(row=i,column=0,sticky=W)
			e = Entry(test_description_frame,width=30)
			e.grid(row=i,column=1,sticky=W,pady=5)
			if debug: e.insert(0,"description text %d"%(i+1))
			self.fixture_descriptions.append(e)
		#general comments
		self.comments = []
		for i in range(self.num_comments):
			Label(test_description_frame,text="  Comment %d:"%(i+1)).grid(row=i,column=2)
			e = Entry(test_description_frame,width=40)
			e.grid(row=i,column=3,sticky=W,pady=5)
			if debug: e.insert(0,"gen comment %d"%(i+1))
			self.comments.append(e)

	def create_fixture_monitoring_section(self):

		PHT_sensor_font = tkFont.Font(family="Helvetica",size=11)

		self.chip_fixture_monitor_frame = LabelFrame(self.tab1_frame,text="Test Fixture Measurements",padx=10,pady=10)
		self.chip_fixture_monitor_frame.grid(row=0,rowspan=2,column=0,padx=10,pady=10,sticky=N)

		#fixture selection drop down box
		Label(self.chip_fixture_monitor_frame,text="Selected Fixture:",font=PHT_sensor_font).grid(row=5,column=0,columnspan=3,sticky=E)
		self.selected_fixture_id = StringVar()
		self.selected_fixture_id.set(self.fixture_addr_strings[0])
		self.fixture_popup_menu = ttk.OptionMenu(self.chip_fixture_monitor_frame,
												self.selected_fixture_id,
												self.fixture_addr_strings[0],
												*self.fixture_addr_strings)
		self.fixture_popup_menu.grid(row=5,column=3,columnspan=2,sticky=W)
		ttk.Separator(self.chip_fixture_monitor_frame,orient='horizontal').grid(row=7,column=0,columnspan=5,sticky='ew',pady=5)

		#temperature, humidity, pressure readings
		Label(self.chip_fixture_monitor_frame,text="Temperature:",font=PHT_sensor_font).grid(row=10,column=0,columnspan=3,sticky=E)
		self.BME280_Temp = Label(self.chip_fixture_monitor_frame,text="N/A",font=PHT_sensor_font,borderwidth=2,relief='sunken',width=6)
		self.BME280_Temp.grid(row=10,column=3)
		Label(self.chip_fixture_monitor_frame,text="*C",font=PHT_sensor_font).grid(row=10,column=4,sticky=W)
		Label(self.chip_fixture_monitor_frame,text="Humidity:",font=PHT_sensor_font).grid(row=11,column=0,columnspan=3,sticky=E)
		self.BME280_Hum = Label(self.chip_fixture_monitor_frame,text="N/A",font=PHT_sensor_font,borderwidth=2,relief='sunken',width=6)
		self.BME280_Hum.grid(row=11,column=3)
		Label(self.chip_fixture_monitor_frame,text="%RH",font=PHT_sensor_font).grid(row=11,column=4,sticky=W)
		Label(self.chip_fixture_monitor_frame,text="Pressure:",font=PHT_sensor_font).grid(row=12,column=0,columnspan=3,sticky=E)
		self.BME280_Pres = Label(self.chip_fixture_monitor_frame,text="N/A",font=PHT_sensor_font,borderwidth=2,relief='sunken',width=6)
		self.BME280_Pres.grid(row=12,column=3)
		Label(self.chip_fixture_monitor_frame,text="Pa",font=PHT_sensor_font).grid(row=12,column=4,sticky=W)
		ttk.Separator(self.chip_fixture_monitor_frame,orient='horizontal').grid(row=13,column=0,columnspan=5,sticky='ew',pady=5)

		#individual sensor readings
		Label(self.chip_fixture_monitor_frame,text="Gas Sensor Readings (Kohms)").grid(row=14,column=0,columnspan=5,pady=1)

		self.sensor_readouts = []
		self.sensor_checkboxes = []
		for sensor in range(self.num_sensors_on_chip):
			cb = Checkbutton(self.chip_fixture_monitor_frame)
			cb.grid(row=15+sensor,column=0)
			self.sensor_checkboxes.append(cb)
			Label(self.chip_fixture_monitor_frame,text="%2d"%(sensor+1)).grid(row=15+sensor,column=1,sticky=E)
			l=Label(self.chip_fixture_monitor_frame,text="5.12",borderwidth=2,relief='sunken',width=7)
			l.grid(row=15+sensor,column=2)
			self.sensor_readouts.append(l)

		self.monitor_controls_frame = LabelFrame(self.chip_fixture_monitor_frame,borderwidth=0,padx=10,pady=10)
		self.monitor_controls_frame.grid(row=50,column=0,columnspan=5,padx=10,pady=10,sticky=N)
		button_width = 6
		self.monitor_start_button = Button(self.monitor_controls_frame,text="Start",command=self.on_start_monitor,
									height=1,width=button_width).grid(row=0,column=0)
		self.monitor_stop_button = Button(self.monitor_controls_frame,text="Stop",command=self.on_stop_monitor,
																	height=1,width=button_width).grid(row=0,column=1)
		self.monitor_pause_button = Button(self.monitor_controls_frame,text="Pause",command=self.on_pause_monitor,
																	height=1,width=button_width).grid(row=0,column=2)

	def create_exit_button(self):
		exitButton = Button(self.mainPanel, text="Exit", command=self.onExit, height=2,width=10).grid(pady=10)

	def create_menu(self):
		menubar = Menu(self.root)

		filemenu = Menu(menubar,tearoff=0)
		filemenu.add_command(label="Set Data Output Dir",state=DISABLED)
		filemenu.add_separator()
		filemenu.add_command(label="Exit",command=self.onExit)
		menubar.add_cascade(label="File",menu=filemenu)
		
		testConfigMenu = Menu(menubar, tearoff=0)
		testConfigMenu.add_command(label="Save Test Configuration",state=DISABLED)
		testConfigMenu.add_command(label="Save Test Configuration as...",state=DISABLED)
		testConfigMenu.add_command(label="Open Test Configuration",state=DISABLED)
		menubar.add_cascade(label="Test_Config",menu=testConfigMenu)


		helpmenu = Menu(menubar,tearoff=0)
		helpmenu.add_command(label="About...",state=DISABLED)
		menubar.add_cascade(label="Help",menu=helpmenu)

		self.root.config(menu=menubar)

	def create_plot(self):
		self.fig = Figure(figsize=(10,6), dpi=100)
		self.fig.suptitle("Sensor Resistance")
		self.ax = self.fig.add_subplot(111)
		self.t_line, = self.ax.plot(self.sampleTimes,self.temperatures,self.t_color+'-')
		self.ax.set_xlabel('time(seconds)')
		self.ax.set_ylabel('Resistance(kohms)')
		self.ax.set_ylim(20,30)
		self.ax.set_xlim(0,self.plot_span_seconds)
		self.ax.ticklabel_format(useOffset=False)
		#self.ax.tick_params('y',colors=self.t_color)
		self.ax.set_axis_bgcolor('black')
		self.ax.grid(True,color='gray')

		canvas = FigureCanvasTkAgg(self.fig, self.tab1_frame)
		canvas.show()
		canvas.get_tk_widget().grid(row=1,rowspan=2,column=1,columnspan=6,padx=5,pady=10)

		self.animation = an.FuncAnimation(self.fig,self.animate,interval=self.sample_interval_msec,blit=False)


	def animate(self,i):
	#        if self.monitor_state == 'running':
		if False:
			self.t_line.set_data(self.sampleTimes,self.temperatures)
			print 'got here'
			self.update_plot()

			return self.t_line,

	def update_plot(self):
		if len(self.temperatures) > 0:
			self.ax.set_ylim([min(self.temperatures)-1,max(self.temperatures)+1])

		if self.scroll_checkbox.get():

			if self.sampleTime >= self.plot_span_seconds-1.0:
				self.ax.set_xlim(self.sampleTime-self.plot_span_seconds+1.0,self.sampleTime+1.0)
			else:
				self.ax.set_xlim(0,self.plot_span_seconds)
		else:

			if self.sampleTime >= self.plot_span_seconds-1.0:
				self.ax.set_xlim(0,max(self.sampleTimes)+5)
			else:
				self.ax.set_xlim(0,INITIAL_XSPAN_SEC)

	def onUpdate(self):
		if platform.system() == 'Linux':

			if self.monitor_state == 'running':
				self.read_fixture_measurements()
								
				self.current_sample_number += 1
				self.samples_since_write += 1
				if self.samples_since_write == self.samples_before_write:
					self.write_samples_to_file()
					self.samples_since_write = 0

				self.update_measurements_display()
		
			for monitor in self.mfc_monitors:
				monitor.update_readings()
				

			self.root.after(self.sample_interval_msec,self.onUpdate)

	def update_measurements_display(self):
		
		#check for selected fixture
		selected_fixture = self.selected_fixture_id.get()
		
		#use selected fixture's last data entry to populate diplays
		
		self.BME280_Temp['text'] = "%.2f"%(self.data_containers[selected_fixture]['Temps'][self.current_sample_number-1])
		self.BME280_Hum['text'] = "%.2f"%(self.data_containers[selected_fixture]['Humidities'][self.current_sample_number-1])
		self.BME280_Pres['text'] = "%.0f"%(self.data_containers[selected_fixture]['Pressures'][self.current_sample_number-1])

		for i in range(len(self.data_containers[selected_fixture]['Res'][self.current_sample_number-1])):
			self.sensor_readouts[i]['text'] = "%.1f"%(
								self.data_containers[selected_fixture]['Res'][self.current_sample_number-1][i])

	def on_start_monitor(self):
		if self.monitor_state == 'stopped':
			self.create_new_outfile()
			self.current_sample_number = 0
			self.samples_since_write = 0
			
			self.data_containers = self.create_data_containers()
			
			self.monitor_state = 'running'

	def on_stop_monitor(self):
		self.monitor_state = 'stopped'
		self.outfile.close()
#		self.print_data()

	def on_pause_monitor(self):
		self.monitor_state = 'paused'
		
	def create_data_containers(self):
		""" each fixture has a dictionary with keyed lists(5) for temperature,
		    humidity, pressure, sensor readings, and timestamps"""

		data_containers = dict()
		for _id in self.fixture_addr_strings:
			data_containers[_id]=dict()
			data_containers[_id]['Temps']=list()
			data_containers[_id]['Humidities']=list()
			data_containers[_id]['Pressures']=list()
			data_containers[_id]['Res']=list()
		data_containers['Timestamps']=list()
		

		return data_containers

	def print_data(self):
		""" just for testing that all data containers are created correctly and 
		    populated with the right data"""
		for fixture in self.data_containers:
			print fixture
			for data_category in self.data_containers[fixture].keys():
				print "  %s:"%(data_category),self.data_containers[fixture][data_category]

	def create_new_outfile(self):
		time_string = dt.now().strftime("%Y_%m_%d__%H_%M_%S")
		self.outfile = open(self.outDir + "/" + time_string + "_Test_Data.csv",'w')

		# write fixture description comments
		for i in range(len(self.fixture_addresses)):
			self.outfile.write("Fixture %s %s"%(self.fixture_addr_strings[i],
										self.fixture_descriptions[i].get()))
			if i < (len(self.fixture_addresses)-1):
				self.outfile.write(",")
		self.outfile.write("\n")

		# write general comments
		for i in range(self.num_comments):
			self.outfile.write("Comment%d: %s"%(i+1,self.comments[i].get()))
			if i < (self.num_comments-1):
				self.outfile.write(",")
		self.outfile.write("\n\n")
		
		self.outfile.write(",Fixture I2C Address,Temperature(*C),%RH,Pressure(Pa),Sensor#1")
		for i in range(2,45):
			self.outfile.write(",#%d"%(i))
		self.outfile.write("\n")

		self.outfile.flush()

	def write_samples_to_file(self):
		
		for i in range(self.samples_before_write,0,-1):
		
			sampleNum = self.current_sample_number - i
			
			#write line with timestamp and mfc readings
			self.outfile.write(self.data_containers['Timestamps'][sampleNum])
			self.outfile.write("\n")
		
			#write separate line for each fixture
			for f in self.fixture_addr_strings:
				
				#write line with temperature, RH, pressure, and sensor readings
				self.outfile.write(",%s,%.2f,%.2f,%.0f,"%(f,self.data_containers[f]['Temps'][sampleNum],
												self.data_containers[f]['Humidities'][sampleNum],
												self.data_containers[f]['Pressures'][sampleNum]))
				for j in range(len(self.data_containers[f]['Res'][self.current_sample_number-1])):
					self.outfile.write("%.3f,"%(self.data_containers[f]['Res'][sampleNum][j]))
		
				self.outfile.write("\n")
		
		self.outfile.flush()
		print "write done, outfile flushed"

	def read_fixture_measurements(self):

		timestamp_line = dt.now().strftime("%Y_%m_%d__%H_%M_%S")
		timestamp_line += ","

		self.send_measurement_trigger()

		#wait for measurements to complete before reading results
		time.sleep(float(self.read_delay_msec_after_trigger)/1000.0)

		for mfc in self.mfc_monitors:
			timestamp_line+="MFCid:%s(%s)_setpoint:%s_reading:%s,"%(
								mfc.get_id(),mfc.get_gas(),
								mfc.get_flow_setpoint(),mfc.get_mass_flow())
		self.data_containers['Timestamps'].append(timestamp_line)

		for fixture_index in range(len(self.i2c_test_fixtures)):

			#find the address for this fixture
			addr = self.fixture_addr_strings[fixture_index]

			#read the measurement data from this fixture
			fixture_data = self.i2c_test_fixtures[fixture_index].readAll(25)

			#first 3 readings are Temperature,Humidity,Pressure
			self.data_containers[addr]['Temps'].append(fixture_data[0])
			self.data_containers[addr]['Humidities'].append(fixture_data[1])
			self.data_containers[addr]['Pressures'].append(fixture_data[2])

			#rest of readings are sensor data so put those readings in a list
			sensor_data = list()
			for i in range(3,len(fixture_data)):
				sensor_data.append(fixture_data[i])

			#append the list of sensor data
			self.data_containers[addr]['Res'].append(sensor_data)

	def send_measurement_trigger(self):
		GPIO.output(GPIO_TRIGGER_CHANNEL,1)
		time.sleep(self.trigger_width_msec/1000.0)
		GPIO.output(GPIO_TRIGGER_CHANNEL,0)

	def onExit(self):
		if platform.system() == 'Linux':
			print 'exiting...'
			GPIO.cleanup()
			self.pi.stop()
			for mfc in self.mfc_monitors:
				mfc.close()
		self.root.quit()

	def onChange(self):
		pass


app=App()
