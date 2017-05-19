#***********************************************************
# This version of the application is for communication with
# switch boards using the SPI bus
#***********************************************************

from Tkinter import *
import ttk
import tkFont
import RPi.GPIO as GPIO
from ConfigParser import ConfigParser
import BME_280
import alicat
import pigpio
import time
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
from matplotlib.figure import Figure
import matplotlib.animation as an



#list of which GPIO ports are used as fixture selection lines
fixture_GPIO_ports = [4,17,22,5,6,13]

#setup GPIO for selecting a test fixture to monitor
GPIO.setmode(GPIO.BCM)
GPIO.setup(fixture_GPIO_ports, GPIO.OUT, initial=GPIO.HIGH)

#screen layout
num_MFCs = 5
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

        self.create_fixture_select_section()
        self.create_test_configuration_section()
        self.create_fixture_monitoring_section()
        self.create_monitor_control_section()
        self.create_mfc_displays()
        self.create_exit_button()

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


        self.fixture_selected = False
        self.pi = pigpio.pi()

        if not self.pi.connected:
			raise RuntimeError("Pi not connected. Check pigpio use")

        self.onUpdate()

        self.root.title("NEA Sensor Research")
        self.root.geometry('1300x850')
        self.root.mainloop()

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

    def create_fixture_select_section(self):

        #create a list of tuples used to build radio buttons for fixture selection
        i = 1
        self.fixture_selections = []
        for port in fixture_GPIO_ports:
        	self.fixture_selections.append(("fixture #%d"%(i),port))
        	i = i+1
        self.fixture_selections.append(("all off",0))

        #variable shared by all fixture selection radio buttons
        self.fixture_selection = IntVar()
        #initialize to 0 for no fixture currently selected
        self.fixture_selection.set(0)

        #create the border grouping all fixture selection radio buttons
        fixture_selection_border = LabelFrame(self.tab1_frame,text="Test Fixture Selection",padx=3,pady=3)
        fixture_selection_border.grid(row=0,column=1,padx=10,pady=10,sticky=N)

        #create fixture selection radio buttons
        for lbl, portNum in self.fixture_selections:
        	Radiobutton(fixture_selection_border,
        				text=lbl,
        				padx=5,
        				variable=self.fixture_selection,
        				command=self.select_fixture,
        				value=portNum).grid(sticky=W)

    def create_fixture_monitoring_section(self):
		#create chip fixture monitor section
        self.chip_fixture_monitor_frame = LabelFrame(self.tab1_frame,text="No Test Fixture Selected",padx=10,pady=10)
        self.chip_fixture_monitor_frame.grid(row=0,rowspan=2,column=0,padx=10,pady=10,sticky=N)

        PHT_sensor_font = tkFont.Font(family="Helvetica",size=11)
        Label(self.chip_fixture_monitor_frame,text="Temperature:",font=PHT_sensor_font).grid(row=0,column=0,columnspan=3,sticky=E)
        self.BME280_Temp = Label(self.chip_fixture_monitor_frame,text="N/A",font=PHT_sensor_font,borderwidth=2,relief='sunken',width=6)
        self.BME280_Temp.grid(row=0,column=3)
        Label(self.chip_fixture_monitor_frame,text="*C",font=PHT_sensor_font).grid(row=0,column=4,sticky=W)
        Label(self.chip_fixture_monitor_frame,text="Humidity:",font=PHT_sensor_font).grid(row=1,column=0,columnspan=3,sticky=E)
        self.BME280_Hum = Label(self.chip_fixture_monitor_frame,text="N/A",font=PHT_sensor_font,borderwidth=2,relief='sunken',width=6)
        self.BME280_Hum.grid(row=1,column=3)
        Label(self.chip_fixture_monitor_frame,text="%RH",font=PHT_sensor_font).grid(row=1,column=4,sticky=W)
        Label(self.chip_fixture_monitor_frame,text="Pressure:",font=PHT_sensor_font).grid(row=2,column=0,columnspan=3,sticky=E)
        self.BME280_Pres = Label(self.chip_fixture_monitor_frame,text="N/A",font=PHT_sensor_font,borderwidth=2,relief='sunken',width=6)
        self.BME280_Pres.grid(row=2,column=3)
        Label(self.chip_fixture_monitor_frame,text="Pa",font=PHT_sensor_font).grid(row=2,column=4,sticky=W)
        ttk.Separator(self.chip_fixture_monitor_frame,orient='horizontal').grid(row=3,column=0,columnspan=5,sticky='ew',pady=5)
        Label(self.chip_fixture_monitor_frame,text="Gas Sensor Readings (Kohms)").grid(row=4,column=0,columnspan=5,pady=5)

        self.sensor_readouts = []
        self.sensor_checkboxes = []
        for sensor in range(num_sensors_on_chip):
            cb = Checkbutton(self.chip_fixture_monitor_frame)
            cb.grid(row=5+sensor,column=0)
            self.sensor_checkboxes.append(cb)
            Label(self.chip_fixture_monitor_frame,text="%2d"%(sensor+1)).grid(row=5+sensor,column=1,sticky=E)
            l=Label(self.chip_fixture_monitor_frame,text="5.12",borderwidth=2,relief='sunken',width=5)
            l.grid(row=5+sensor,column=2)
            self.sensor_readouts.append(l)

    def create_monitor_control_section(self):
        self.monitor_controls_frame = LabelFrame(self.tab1_frame,text="Monitor Controls",padx=10,pady=10)
        self.monitor_controls_frame.grid(row=2,column=0,padx=10,pady=10,sticky=N)

        self.monitor_start_button = Button(self.monitor_controls_frame,text="Start",command=self.start_monitor,
                                                                                                        height=1,width=5).grid(row=0,column=0)
        self.monitor_stop_button = Button(self.monitor_controls_frame,text="Stop",command=self.start_monitor,
                                                                                                        height=1,width=5).grid(row=0,column=1)
        self.monitor_pause_button = Button(self.monitor_controls_frame,text="Pause",command=self.start_monitor,
                                                                                                        height=1,width=5).grid(row=0,column=2)

    def create_mfc_displays(self):
        self.mfc_monitors_frame = LabelFrame(self.tab1_frame,text="MFC Monitors",padx=3,pady=3)
        self.mfc_monitors_frame.grid(row=0,column=2,padx=10,pady=10,sticky=N)

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

        self.fig = Figure(figsize=(10,5), dpi=100)
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
        if self.monitor_state == 'running':
            self.read_BME280()

            self.t_line.set_data(self.sampleTimes,self.temperatures)

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



    #-----------------------------------------------------------
    #----------------function definitions-----------------------
    def onUpdate(self):
        if self.fixture_selected:
                self.readBME280()

        for monitor in self.mfc_monitors:
            monitor.update_readings()


        self.root.after(1000,self.onUpdate)

    def readBME280(self):
        t, p, h = self.BME280sensor.read_data()
        self.BME280_Temp['text'] = "%.2f"%(t)
        self.BME280_Hum['text'] = "%.2f"%(h)
        self.BME280_Pres['text'] = "%.0f"%(p)

        return (t,p,h)

    def start_monitor(self):
        pass


    def select_fixture(self):

        #testing previous selection state to handle BME280 sensor switching
        if self.fixture_selected:
            self.BME280sensor.cancel()

        #find GPIO port associated with selected radio button
        fx = self.fixture_selection.get()

    	if fx:
            self.fixture_selected = True
        else:
            self.fixture_selected = False

    	#find index of radio button selected
    	radio_index = self.fixture_selections.index([tup for tup in self.fixture_selections if fx in tup][0])

    	#set all GPIO ports high to disable
    	for port in fixture_GPIO_ports:
            GPIO.output(port,GPIO.HIGH)

    	#test for valid fixture selection or all fixtures off
    	if self.fixture_selected:
            GPIO.output(fx,GPIO.LOW)
            self.chip_fixture_monitor_frame.configure(text="Fixture #%d Readings"%(radio_index+1))
            time.sleep(0.2)
            self.BME280sensor = BME_280.sensor(self.pi)
    	else:
			self.chip_fixture_monitor_frame.configure(text="No Test Fixture Selected")
			self.BME280_Temp['text'] = "N/A"
			self.BME280_Hum['text'] = "N/A"
			self.BME280_Pres['text'] = "N/A"


    def onExit(self):
        GPIO.cleanup()
        #self.BME280sensor.cancel()
        self.pi.stop()
        for mfc in self.mfc_monitors:
            mfc.close()
        self.root.quit()


    def onChange(self):
    	pass

    #-----------------------------------------------------------


app=App()
