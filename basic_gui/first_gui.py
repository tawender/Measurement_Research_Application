from Tkinter import *
import tkFont
import RPi.GPIO as GPIO

led_pin = 16

GPIO.setmode(GPIO.BCM)
GPIO.setup(led_pin, GPIO.OUT)
GPIO.output(led_pin, GPIO.LOW)

win = Tk()

myFont = tkFont.Font(family='Helvetica', size=36, weight='bold')

def ledON():
    if GPIO.input(led_pin):
        GPIO.output(led_pin,GPIO.LOW)
        ledButton["text"]="LED ON"
    else:
        GPIO.output(led_pin,GPIO.HIGH)
        ledButton["text"]="LED OFF"
        
def exitProgram():
    GPIO.cleanup()
    win.quit()

win.title("First GUI")
win.geometry('800x550')

exitButton = Button(win, text="Exit", font=myFont, command=exitProgram, height=2,
                    width=8)
exitButton.pack(side=BOTTOM)

ledButton = Button(win, text="LED ON", font=myFont, command=ledON, height=2,
                   width=8)
ledButton.pack()

mainloop()
