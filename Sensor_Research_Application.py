from Tkinter import *
import tkFont
import RPi.GPIO as GPIO
from ConfigParser import ConfigParser

root = Tk()

#list of which GPIO ports are used as fixture selection lines
fixture_GPIO_ports = [4,17,22]

#setup GPIO for selecting a test fixture to monitor
GPIO.setmode(GPIO.BCM)
GPIO.setup(fixture_GPIO_ports, GPIO.OUT, initial=GPIO.HIGH)

#screen layout
num_MFCs = 6
num_test_conditions = 8


		
#-----------------------------------------------------------
#----------------function definitions-----------------------
#def read_config_file():
	
	
def select_fixture():
	#find current radio button selection
	fx = fixture_selection.get()
	
	#set all GPIO ports high to disable
	for port in fixture_GPIO_ports:
		GPIO.output(port,GPIO.HIGH)
		
	#if a fixture is selected set its select signal low
	if fx != 0:	
		GPIO.output(fx,GPIO.LOW)
        
        
def exitProgram():
    GPIO.cleanup()
    root.quit()
    

def onChange():
	pass
	
#-----------------------------------------------------------





#-------------------
#create a list of tuples used to build radio buttons for fixture selection
i = 1
fixture_selections = []
for port in fixture_GPIO_ports:
	fixture_selections.append(("fixture #%d"%(i),port))
	i = i+1
fixture_selections.append(("all off",0))

#variable shared by all fixture selection radio buttons
fixture_selection = IntVar()
#initialize to 0 for no fixture currently selected
fixture_selection.set(0)	
		
#create the border grouping all fixture selection radio buttons    
fixture_selection_frame = LabelFrame(root,text="Test Fixture Selection",padx=10,pady=10)
fixture_selection_frame.grid(row=1,column=0,padx=10,pady=10,sticky=W)  

#create fixture selection radio buttons
for lbl, portNum in fixture_selections:
	Radiobutton(fixture_selection_frame,
				text=lbl,
				padx=5,
				variable=fixture_selection,
				command=select_fixture,
				value=portNum).grid(sticky=W)
#-------------------


#-------------------
#create MFC control section
mfc_control_frame = LabelFrame(root,text="MFC Control")
mfc_control_frame.grid(row=0,column=0,padx=10,pady=10,ipady=5,columnspan=2)

mfc_checkboxes = []
mfc_ids = []
mfc_gas = []
for mfc_num in range(num_MFCs):
	#checkbox to enable this MFC
	Label(mfc_control_frame,text="enable").grid(row=0,column=1+mfc_num*4,columnspan=2,sticky=E)
	cb = Checkbutton(mfc_control_frame)
	cb.grid(row=0,column=3+mfc_num*4,sticky=W)
	mfc_checkboxes.append(cb)
	#MFC ID for communication
	Label(mfc_control_frame,text="MFC ID").grid(row=1,column=1+mfc_num*4,sticky=E)
	e = Entry(mfc_control_frame,width=2)
	e.grid(row=1,column=2+mfc_num*4,sticky=W)
	mfc_ids.append(e)
	#type of gas MFC will control
	Label(mfc_control_frame,text="gas").grid(row=1,column=3+mfc_num*4,sticky=E)
	e = Entry(mfc_control_frame,width=4)
	e.grid(row=1,column=4+mfc_num*4,sticky=W)
	mfc_gas.append(e)
	
test_condition_checkboxes = []
test_condition_times = []
test_condition_settings = []
for test_condition in range(num_test_conditions):
	#checkbox to enable test condition
	cb = Checkbutton(mfc_control_frame)
	cb.grid(row=2+test_condition,column=0,sticky=W,pady=3)
	test_condition_checkboxes.append(cb)
	for mfc_num in range(num_MFCs):
		#time this test condition will last
		Label(mfc_control_frame,text="    seconds:").grid(row=2+test_condition,column=1+mfc_num*4,sticky=E)
		e = Entry(mfc_control_frame,width=3)
		e.grid(row=2+test_condition,column=2+mfc_num*4,sticky=W)
		test_condition_times.append(e)
		Label(mfc_control_frame,text="scfm:").grid(row=2+test_condition,column=3+mfc_num*4,sticky=E)
		e = Entry(mfc_control_frame,width=3)
		e.grid(row=2+test_condition,column=4+mfc_num*4,sticky=W)
		test_condition_settings.append(e)
#-------------------




#-------------------
#create chip fixture monitor section
chip_fixture_monitor_frame = LabelFrame(root,text="Chip Fixture Monitor")
chip_fixture_monitor_frame.grid(row=1,column=1,padx=10,pady=10,ipady=5,sticky=W)

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

#-------------------



#-------------------
#create button used to exit the application
exitButton = Button(root, text="Exit", command=exitProgram, height=2,
                    width=10).grid()
#-------------------



#-------------------
#create button used to exit the application
exitButton = Button(root, text="Change", command=onChange, height=2,
                    width=10).grid()
#-------------------

root.title("NEA Sensor Research")
root.geometry('1300x550')
mainloop()
