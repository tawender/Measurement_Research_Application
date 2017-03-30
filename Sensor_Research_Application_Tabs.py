from Tkinter import *
import ttk
import tkFont
import RPi.GPIO as GPIO
from ConfigParser import ConfigParser
import BME_280
import pigpio
import time



#list of which GPIO ports are used as fixture selection lines
fixture_GPIO_ports = [4,17,22,5,6,13]

#setup GPIO for selecting a test fixture to monitor
GPIO.setmode(GPIO.BCM)
GPIO.setup(fixture_GPIO_ports, GPIO.OUT, initial=GPIO.HIGH)

#screen layout
num_MFCs = 6
num_test_conditions = 8
num_sensors_on_chip = 22


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
        self.create_exit_button()

        self.tabs.add(self.tab1_frame,text='Fixture Control')
        self.tabs.add(self.tab2_frame,text='Test Condition Setup')

        self.create_menu()
        
        self.fixture_selected = False
        self.pi = pigpio.pi()
        
        if not self.pi.connected:
			raise RuntimeError("Pi not connected. Check pigpio use")
	
        self.onUpdate()

        self.root.title("NEA Sensor Research")
        self.root.geometry('1300x750')
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
        fixture_selection_border = LabelFrame(self.tab1_frame,text="Test Fixture Selection",padx=10,pady=10)
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
        self.chip_fixture_monitor_frame.grid(row=0,column=0,padx=10,pady=10,sticky=N)

        PHT_sensor_font = tkFont.Font(family="Helvetica",size=11)
        Label(self.chip_fixture_monitor_frame,text="Temperature:",font=PHT_sensor_font).grid(row=0,column=0,sticky=E)
        self.BME280_Temp = Label(self.chip_fixture_monitor_frame,text="N/A",font=PHT_sensor_font,borderwidth=2,relief='sunken',width=6)
        self.BME280_Temp.grid(row=0,column=1)
        Label(self.chip_fixture_monitor_frame,text="*C",font=PHT_sensor_font).grid(row=0,column=2,sticky=W)
        Label(self.chip_fixture_monitor_frame,text="Humidity:",font=PHT_sensor_font).grid(row=1,column=0,sticky=E)
        self.BME280_Hum = Label(self.chip_fixture_monitor_frame,text="N/A",font=PHT_sensor_font,borderwidth=2,relief='sunken',width=6)
        self.BME280_Hum.grid(row=1,column=1)
        Label(self.chip_fixture_monitor_frame,text="%RH",font=PHT_sensor_font).grid(row=1,column=2,sticky=W)
        Label(self.chip_fixture_monitor_frame,text="Pressure:",font=PHT_sensor_font).grid(row=2,column=0,sticky=E)
        self.BME280_Pres = Label(self.chip_fixture_monitor_frame,text="N/A",font=PHT_sensor_font,borderwidth=2,relief='sunken',width=6)
        self.BME280_Pres.grid(row=2,column=1)
        Label(self.chip_fixture_monitor_frame,text="Pa",font=PHT_sensor_font).grid(row=2,column=2,sticky=W)
        ttk.Separator(self.chip_fixture_monitor_frame,orient='horizontal').grid(row=3,column=0,columnspan=3,sticky='ew',pady=10)
        
        self.sensor_readouts = []
        for sensor in range(num_sensors_on_chip):
            Label(self.chip_fixture_monitor_frame,text="Sensor #%2d"%(sensor+1)).grid(row=4+sensor,column=0,sticky=E)
            l=Label(self.chip_fixture_monitor_frame,text="5.12",borderwidth=2,relief='sunken',width=7)
            l.grid(row=4+sensor,column=1)
            self.sensor_readouts.append(l)
            Label(self.chip_fixture_monitor_frame,text="Kohms").grid(row=4+sensor,column=2)

    def create_exit_button(self):
        #create button used to exit the application
        exitButton = Button(self.mainPanel, text="Exit", command=self.exitProgram, height=2,width=10).grid(pady=10)

    def create_menu(self):

        menubar = Menu(self.root)

        filemenu = Menu(menubar,tearoff=0)
        filemenu.add_command(label="Save")
        filemenu.add_command(label="Save as...")
        filemenu.add_separator()
        filemenu.add_command(label="Exit",command=self.exitProgram)
        menubar.add_cascade(label="File",menu=filemenu)

        helpmenu = Menu(menubar,tearoff=0)
        helpmenu.add_command(label="About...")
        menubar.add_cascade(label="Help",menu=helpmenu)

        self.root.config(menu=menubar)

    #-----------------------------------------------------------
    #----------------function definitions-----------------------
    def onUpdate(self):
		if self.fixture_selected:
			self.readBME280()
			
		self.root.after(1000,self.onUpdate)
		
    def readBME280(self):
		t, p, h = self.BME280sensor.read_data()
		self.BME280_Temp['text'] = "%.2f"%(t)
		self.BME280_Hum['text'] = "%.2f"%(h)
		self.BME280_Pres['text'] = "%.0f"%(p)
		
		return (t,p,h)


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


    def exitProgram(self):
        GPIO.cleanup()
        self.pi.stop()
        self.root.quit()


    def onChange(self):
    	pass

    #-----------------------------------------------------------


app=App()
