# master.py
#
# -program file for senior design 1 project 2
# calculation of boltzmann's constant by reading noise voltage
#
# Written and Designed by Michael Thompson


from Log import Log
from UI import Page
from os import system

#testing only
from time import time_ns

class program:
    def __init__(self, width):
        self.state = 0
        self.change = False

        #-UI-
        #Menu Page
        self.menu = Page("Menu", width, '.')
        self.menu.changeContents(["Senior Design Project 2",
                                            "by",
                                            "Bo Rogers",
                                            "Steven Gaiko", 
                                            "Michael Thompson", 
                                            "Jackson Ball",
                                            ""])

        #Recording Page
        self.record = Page("Recording Voltage", width, "*")

        #Calculate Boltzmann Constant Page
        self.calculateB = Page("Calculate Boltzmann Constant", width, "#")
        self.calculateB.changePage("", "Calculating Boltzmann's Constant",["",
                                    "        Vrms^2   ",
                                    "BC = ____________",
                                    "     4*Temp*ENB*R",""])

        #Calculate Resistor Value Page
        self.calculateR = Page("Calculate Resistor Value", width, "^")
        self.calculateR.changePage("","Calculating Unknown Resistor",["",
                                    "        Vrms^2   ",
                                    "R = ____________",
                                    "     4*Temp*ENB*BC",""])

        #-Log-
        self.Log = Log()
    
    #shows the page based on the state
    def show_state(self, clear=True):
        if self.state == 0:
            self.menu.show(clear)
        elif self.state == 1:
            self.record.show(clear)
        elif self.state == 2:
            self.calculateB.show(clear)
        elif self.state == 3:
            self.calculateR.show(clear)
    
    #Run program
    def run(self):
        answer = "NULL" #variable to store command line answers
        while self.state == 0: #run loop
            #menu state
            if self.state == 0: 
                #set menu UI with commands
                self.menu.changeCommand("Select task:\n"+"1- Log Data\n"+
                                        "2- Calculate Boltzmann Constant\n"+
                                        "3- Calculate Unknow Resistor\n"+
                                        "4- Exit")
                #check for answers
                while(answer != "1" and answer != "2" and answer != "3" and answer != "4"):
                    self.show_state()
                    answer = input("->")
                if (answer == "4"): quit() #quit if Exit is selected

                answer1 = 0 #2nd answer variable used so that answer will not be overridden 

                #select log file prompt
                self.menu.changeCommand("Select Log File:\n"+
                                        "1- sample_1.log\n"+
                                        "2- sample_2.log")
                while(answer1 != "1" and answer1 != "2"):
                    self.show_state()
                    answer1 = input("->")
                if answer1 == "1": 
                    self.Log.selectLog(0)
                elif answer1 == "2":
                    self.Log.selectLog(1)

                if answer == "1":
                    answer = "NULL"
                    #clear log or append prompt
                    self.menu.changeCommand("Continue Log?")
                    while(answer != "yes" and answer != "no"):
                        self.show_state()
                        answer = input("->").strip().lower()
                    if answer == "no":
                        answer = "NULL"
                        self.menu.changeCommand("Are you sure you would like to clear the log?\n"+
                                                "This can not be undone.\n"+
                                                "Type CLEAR to clear log or NO to back")
                        while(answer != "clear" and answer != "no"):
                            self.show_state()
                            answer = input("->").strip().lower()
                        #clear selected log
                        if answer == "clear": 
                            self.menu.changeTask("Clearing Log")
                            self.show_state()
                            self.Log.clearLog()
                    self.state = 1 #set state to 1
                elif answer == "2":
                    self.state = 2 #set state to 2
                elif answer == "3":
                    self.state = 3 #set state to 3
                self.show_state()
            #read voltage
            if self.state == 1:
                ##################FOR ESP32 USE###################
                self.record.changePage("Close PuTTY to continue", "Executing PuTTY.exe",
                                ["", "Logging:", "*All session output", "!*Flush log frequently" "*Always overwrite it", "place log file in program src","",
                                "Session:","*Serial","set port to COM3", "set speed to 115200","Open",""])
                self.show_state()
                system('D:\Coding\Putty\putty.exe')
                self.record.changeTask("Converting putty.log to program log")
                self.record.changePage("", "Converting putty.log to program log",
                                ["","Please wait"])
                self.show_state()
                self.Log.cleanLog()
                ##################FOR PI USE######################
                #self.record.changePage("Press [SPACE] then [ENTER] to end", "Executing Pi.exe",
                #                      ["","Currently taking readings", "from the test circuit."])
                #self.Log.close() #close log so c++ file doesn't have issues writing to the logs
                #system("./pi.exe") #run external c++ file to read voltage from the ADC
                #self.Log.open() #reopen logs
                #self.record.changeTask("Pi.exe Executed")
                #self.show_state()
                self.state = 0 #set state to menu to allow for calculations
            #calculate boltzmanns constant
            if self.state == 2:
                self.calculateB.changeCommand("Calculated Boltzmann's Constant: " + str(self.Log.calculateBC()))
                self.calculateB.changeTask("Calculated Boltzmann's Constant")
                
            #calculate unknown resistor
            if self.state == 3:
                self.calculateR.changeCommand("Calculated Resistor: " + str(self.Log.calculateResistor()))
                self.calculateR.changeTask("Calculated Unknown Resistor")
            self.show_state()
        self.Log.close()

#the execution of the program
program = program(50)
program.run()