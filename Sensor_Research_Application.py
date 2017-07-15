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

debug=True

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


#screen layout
num_MFCs = 6
num_MFC_displays = 2
num_test_conditions = 20
num_sensors_on_chip = 22

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

    def close(self):
        self.flowController.close()

class App:
    def __init__(self):
        self.root = Tk()

        self.mainPanel = Frame(self.root)
        self.mainPanel.grid(sticky=N)

        self.tabs = ttk.Notebook(self.mainPanel)
        self.tabs.grid(row=0,column=0,padx=10,pady=10)
        self.tab1_frame = Frame(self.tabs)
        self.tab2_frame = Frame(self.tabs)
	
	self.read_config_file()
	self.search_I2C_devices()
	self.create_test_fixture_instances()

        self.create_test_configuration_section()
        self.create_fixture_monitoring_section()
        self.create_mfc_displays()

        self.tabs.add(self.tab1_frame,text='Fixture Control')
        self.tabs.add(self.tab2_frame,text='Test Condition Setup')

        self.root.protocol("WM_DELETE_WINDOW", self.onExit)

        self.create_menu()

        self.sample_interval_msec = 500
        self.plot_span_seconds = 30
        self.sampleTime = 0
        self.sampleTimes = []
        self.temperatures = []
        self.humidities = []
        self.t_color = 'g'
        self.create_plot()
        self.monitor_state = None

        if platform.system() == 'Linux':
            self.pi = pigpio.pi()

            if not self.pi.connected:
    			raise RuntimeError("Pi not connected. Check pigpio use")

        self.onUpdate()

        self.root.title("NEA Sensor Research")
        self.root.geometry('1350x950')
        self.root.mainloop()

    def read_config_file(self):
        self.config_file_name = "settings.cfg"
        self.c_file = open(self.config_file_name, 'r')
        self.config_file = ConfigParser()
        self.config_file.readfp(self.c_file)

	self.MFC_com_port = self.config_file.get('MFC Controls','MFC COM port')
	self.MFC_ids = self.config_file.get('MFC Controls','MFC IDS').split(',')
	self.MFC_gasses = self.config_file.get('MFC Controls','MFC gasses').split(',')
	
        self.sample_interval_msec = int(self.config_file.get('Measurement Controls','sample interval msec'))
	
        self.plot_span_seconds = int(self.config_file.get('Plot Controls','scroll seconds'))
        self.scroll_checkbox_tmp = self.config_file.getboolean('Plot Controls','scrolling enabled')
	
        #self.fixture_addresses = self.config_file.get('Connected Fixtures','I2C addresses').split(',')
	
    def search_I2C_devices(self):
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
	    
	if debug: print "detected i2c fixture addresses: ",self.fixture_addresses
	if debug: print "fixture address strings: ",self.fixture_addr_strings
	    
    def create_test_fixture_instances(self):
	self.i2c_test_fixtures = []
	for id in self.fixture_addresses:
	    self.i2c_test_fixtures.append(chip_fixture.ChipFixture(id))

    def create_test_configuration_section(self):
        mfc_control_frame = LabelFrame(self.tab2_frame,text="Test Configuration")
        mfc_control_frame.grid(row=0,column=0,padx=10,pady=10,ipadx=5,ipady=5,columnspan=2)

        mfc_checkboxes = []
        mfc_ids = []
        mfc_gas = []
        for mfc_num in range(num_MFCs):
            #checkbox to enable this MFC
            Label(mfc_control_frame,text="enable").grid(row=0,column=3+mfc_num*4,columnspan=2,sticky=E)
            cb = Checkbutton(mfc_control_frame)
            cb.grid(row=0,column=5+mfc_num*4,sticky=W)
            mfc_checkboxes.append(cb)
            #MFC ID for communication
            Label(mfc_control_frame,text="    MFC ID").grid(row=1,column=3+mfc_num*4,sticky=E)
            e = Entry(mfc_control_frame,width=2)
            e.grid(row=1,column=4+mfc_num*4,sticky=W)
            mfc_ids.append(e)
            #type of gas MFC will control
            Label(mfc_control_frame,text="gas").grid(row=1,column=5+mfc_num*4,sticky=E)
            e = Entry(mfc_control_frame,width=4)
            e.grid(row=1,column=6+mfc_num*4,sticky=W)
            mfc_gas.append(e)

        test_condition_checkboxes = []
        test_condition_times = []
        test_condition_settings = []
        for test_condition in range(num_test_conditions):
            #checkbox to enable test condition
            cb = Checkbutton(mfc_control_frame)
            cb.grid(row=2+test_condition,column=0,sticky=W,pady=3)
            test_condition_checkboxes.append(cb)
            #time this test condition will last
            Label(mfc_control_frame,text="seconds:").grid(row=2+test_condition,column=1,sticky=E)
            e = Entry(mfc_control_frame,width=4)
            e.grid(row=2+test_condition,column=2,sticky=W)
            test_condition_times.append(e)
            for mfc_num in range(num_MFCs):
                Label(mfc_control_frame,text="scfm:").grid(row=2+test_condition,column=4+mfc_num*4,sticky=E)
                e = Entry(mfc_control_frame,width=3)
                e.grid(row=2+test_condition,column=5+mfc_num*4,sticky=W)
                test_condition_settings.append(e)

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
        for sensor in range(num_sensors_on_chip):
            cb = Checkbutton(self.chip_fixture_monitor_frame)
            cb.grid(row=15+sensor,column=0)
            self.sensor_checkboxes.append(cb)
            Label(self.chip_fixture_monitor_frame,text="%2d"%(sensor+1)).grid(row=15+sensor,column=1,sticky=E)
            l=Label(self.chip_fixture_monitor_frame,text="5.12",borderwidth=2,relief='sunken',width=5)
            l.grid(row=15+sensor,column=2)
            self.sensor_readouts.append(l)
	
        self.monitor_controls_frame = LabelFrame(self.chip_fixture_monitor_frame,borderwidth=0,padx=10,pady=10)
        self.monitor_controls_frame.grid(row=50,column=0,columnspan=5,padx=10,pady=10,sticky=N)
	button_width = 6
        self.monitor_start_button = Button(self.monitor_controls_frame,text="Start",command=self.start_monitor,
								    height=1,width=button_width).grid(row=0,column=0)
        self.monitor_stop_button = Button(self.monitor_controls_frame,text="Stop",command=self.stop_monitor,
                                                                    height=1,width=button_width).grid(row=0,column=1)
        self.monitor_pause_button = Button(self.monitor_controls_frame,text="Pause",command=self.pause_monitor,
                                                                    height=1,width=button_width).grid(row=0,column=2)

    #def create_monitor_control_section(self):
        #self.monitor_controls_frame = LabelFrame(self.tab1_frame,text="Monitor Controls",padx=10,pady=10)
        #self.monitor_controls_frame.grid(row=2,column=0,padx=10,pady=10,sticky=N)

	#button_width = 6
        #self.monitor_start_button = Button(self.monitor_controls_frame,text="Start",command=self.start_monitor,
								    #height=1,width=button_width).grid(row=0,column=0)
        #self.monitor_stop_button = Button(self.monitor_controls_frame,text="Stop",command=self.stop_monitor,
                                                                    #height=1,width=button_width).grid(row=0,column=1)
        #self.monitor_pause_button = Button(self.monitor_controls_frame,text="Pause",command=self.pause_monitor,
                                                                    #height=1,width=button_width).grid(row=0,column=2)

    def create_mfc_displays(self):
        self.mfc_monitors_frame = LabelFrame(self.tab1_frame,text="MFC Monitors",padx=3,pady=3)
        self.mfc_monitors_frame.grid(row=0,column=1,columnspan=3,padx=10,pady=10,sticky=NW)

        small_mfc_font = tkFont.Font(family="Helvetica",size=9)
        big_mfc_font = tkFont.Font(family="Helvetica",size=20)
        self.mfc_monitors = []

        for i in range(num_MFC_displays):
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

            self.mfc_monitors.append(m)

            if i != num_MFC_displays-1:
                ttk.Separator(self.mfc_monitors_frame,orient='vertical').grid(row=0,column=4*i+3,rowspan=7,sticky='ns',padx=10)

        if platform.system() == 'Linux':
            self.mfc_B = alicat.FlowController(address='B')
            self.mfc_monitors[0].flowController = self.mfc_B
            self.mfc_monitors[0].address = 'B'

            self.mfc_C = alicat.FlowController(address='C')
            self.mfc_monitors[1].flowController = self.mfc_C
            self.mfc_monitors[1].address = 'C'

    def create_exit_button(self):
        exitButton = Button(self.mainPanel, text="Exit", command=self.onExit, height=2,width=10).grid(pady=10)

    def create_menu(self):

        menubar = Menu(self.root)

        filemenu = Menu(menubar,tearoff=0)
        filemenu.add_command(label="Save")
        filemenu.add_command(label="Save as...")
        filemenu.add_separator()
        filemenu.add_command(label="Exit",command=self.onExit)
        menubar.add_cascade(label="File",menu=filemenu)

        helpmenu = Menu(menubar,tearoff=0)
        helpmenu.add_command(label="About...")
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
        if self.monitor_state == 'running':
	    data = self.read_fixture_measurements()
	    self.BME280_Temp['text'] = "%.2f"%(data[0])
	    self.BME280_Hum['text'] = "%.2f"%(data[1])
	    self.BME280_Pres['text'] = "%.0f"%(data[2])

        if platform.system() == 'Linux':
            for monitor in self.mfc_monitors:
                monitor.update_readings()


        self.root.after(1000,self.onUpdate)

    def start_monitor(self):
        self.monitor_state = 'running'
	
    def stop_monitor(self):
	self.monitor_state = 'stopped'
	
    def pause_monitor(self):
	self.monitor_state = 'paused'

    def select_fixture(self):
	pass

    def read_fixture_measurements(self):
	self.send_measurement_trigger()
	
	fixture_index = self.fixture_addr_strings.index(self.selected_fixture_id.get())
	
	return self.i2c_test_fixtures[fixture_index].readAll()
	
	
    def send_measurement_trigger(self):
	GPIO.output(GPIO_TRIGGER_CHANNEL,1)
	time.sleep(0.1)
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

    #-----------------------------------------------------------


app=App()
