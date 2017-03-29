from Tkinter import *
import ttk
import tkFont
#import RPi.GPIO as GPIO
from ConfigParser import ConfigParser



#list of which GPIO ports are used as fixture selection lines
fixture_GPIO_ports = [4,17,22]

#setup GPIO for selecting a test fixture to monitor
#GPIO.setmode(GPIO.BCM)
#GPIO.setup(fixture_GPIO_ports, GPIO.OUT, initial=GPIO.HIGH)

#screen layout
num_MFCs = 6
num_test_conditions = 8


class App:
    def __init__(self):
        self.root = Tk()

        self.mainPanel = Frame(self.root)
        self.mainPanel.grid(sticky=N)

        self.tabs = ttk.Notebook(self.mainPanel)
        self.tabs.grid(row=0,column=0)
        self.tab1_frame = Frame(self.tabs)
        self.tab2_frame = Frame(self.tabs)

        self.create_fixture_select_section()
        self.create_test_configuration_section()
        self.create_fixture_monitoring_section()
        self.create_exit_button()

        self.tabs.add(self.tab1_frame,text='Fixture Control')
        self.tabs.add(self.tab2_frame,text='Test Condition Setup')

        self.create_menu()

        self.root.title("NEA Sensor Research")
        self.root.geometry('1300x550')
        self.root.mainloop()



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
        fixture_selection_border.grid(row=0,column=0,padx=10,pady=10,sticky=W)

        #create fixture selection radio buttons
        for lbl, portNum in self.fixture_selections:
        	Radiobutton(fixture_selection_border,
        				text=lbl,
        				padx=5,
        				variable=self.fixture_selection,
        				command=self.select_fixture,
        				value=portNum).grid(sticky=W)


    def create_test_configuration_section(self):
        mfc_control_frame = LabelFrame(self.tab2_frame,text="Test Configuration")
        mfc_control_frame.grid(row=0,column=0,padx=10,pady=10,ipady=5,columnspan=2)

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
		#create chip fixture monitor section
        chip_fixture_monitor_frame = LabelFrame(self.tab1_frame,text="PHT Sensor Measurements")
        chip_fixture_monitor_frame.grid(row=0,column=1)

        PHT_sensor_font = tkFont.Font(family="Helvetica",size=11)
        Label(chip_fixture_monitor_frame,text="Temperature:",font=PHT_sensor_font).grid(row=0,column=0,sticky=E)
        Label(chip_fixture_monitor_frame,text="25.1",font=PHT_sensor_font,borderwidth=2,relief='sunken',width=6).grid(row=0,column=1)
        Label(chip_fixture_monitor_frame,text="*C",font=PHT_sensor_font).grid(row=0,column=2,sticky=W)
        Label(chip_fixture_monitor_frame,text="Humidity:",font=PHT_sensor_font).grid(row=1,column=0,sticky=E)
        Label(chip_fixture_monitor_frame,text="45",font=PHT_sensor_font,borderwidth=2,relief='sunken',width=6).grid(row=1,column=1)
        Label(chip_fixture_monitor_frame,text="%RH",font=PHT_sensor_font).grid(row=1,column=2,sticky=W)
        Label(chip_fixture_monitor_frame,text="Pressure:",font=PHT_sensor_font).grid(row=2,column=0,sticky=E)
        Label(chip_fixture_monitor_frame,text="1000.5",font=PHT_sensor_font,borderwidth=2,relief='sunken',width=6).grid(row=2,column=1)
        Label(chip_fixture_monitor_frame,text="mbar",font=PHT_sensor_font).grid(row=2,column=2,sticky=W)

    def create_exit_button(self):
		#create button used to exit the application
		exitButton = Button(self.mainPanel, text="Exit", command=self.exitProgram, height=2,width=10).grid()



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
    #def read_config_file():


    def select_fixture(self):
    	#find current radio button selection
    	fx = self.fixture_selection.get()

    	#set all GPIO ports high to disable
    	for port in fixture_GPIO_ports:
    		GPIO.output(port,GPIO.HIGH)

    	#if a fixture is selected set its select signal low
    	if fx != 0:
    		GPIO.output(fx,GPIO.LOW)


    def exitProgram(self):
        GPIO.cleanup()
        self.root.quit()


    def onChange(self):
    	pass

    #-----------------------------------------------------------


app=App()
