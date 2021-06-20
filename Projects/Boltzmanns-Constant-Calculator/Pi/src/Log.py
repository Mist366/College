# Log.py
#
# -Holds the Log Class
# -Log class takes in data from .log files and makes calculations for Coltzmann's Constant
# -Used in master.py
#
# Written and Designed by Michael Thompson

from os import remove, path
from re import split
from numpy import sqrt
import string

#files being used
file1 = "sample_1.log"
file2 = "sample_2.log"
fileb = "boltzmann.log"
fileI = "info.v"
fileT = "go_between.tmp"
class Del:
    def __init__(self, keep=string.digits):
        self.comp = dict((ord(c),c) for c in keep)
    def __getitem__(self, k):
        return self.comp.get(k)

class Log:
    def __init__(self):
        #selector for which log is being used
        self.select = 0

        self.D = Del()

        #files
        self.log_name = [file1, file2]
        self.log = [open(file1,"a"), open(file2, "a")]
        self.bfile = open(fileb, "a")

        #takes in variable values from fileI
        info = open(fileI,"r")
        info.seek(0,0)
        R6 = self.detlimit(info)
        R5 = self.detlimit(info)
        R9 = self.detlimit(info)
        R11 = self.detlimit(info)
        R12 = self.detlimit(info)
        R14 = self.detlimit(info)
        self.Temp = self.detlimit(info)
        self.ENB = self.detlimit(info)
        self.R_Test = self.detlimit(info) 
        info.close()

        #calculates variables from fileI variables
        pre_gain = R5/(R6 + R9)
        filter_gain = 1
        post_gain = R11/(R12+R14)
        self.gain = pre_gain*filter_gain*post_gain
        #PI conversion
        #self.to_voltage = float(5)/float(((2**16)-1)) 
        #self.offset = 2.5

        #PI conversion
        self.to_voltage = 3.3/4095
        self.offset = 3.3/float(2)
    
    #deconstructor
    def __del__(self):
        del self.R_Test
        del self.ENB
        del self.Temp
        del self.gain
        del self.to_voltage
        del self.offset
        if path.exists(fileT): remove(fileT)

    #selects log file to use and writes the name to fileT
    def selectLog(self, select, mode="w"):
        self.select = select
        tmp = open(fileT, mode)
        if select == 0:
            tmp.write(file1)
        elif select == 1:
            tmp.write(file2)
        tmp.close()

    #closes current log file and fileb
    def close(self):
        self.log[0].close()
        self.log[1].close()
        self.bfile.close()

    def open(self, mode="r"):
        if not (self.log[0].closed and self.log[1].closed and self.bfile.closed): 
            self.close()

        self.bfile = open(fileb, mode)
        self.log[0] = open(self.log_name[0], mode)
        self.log[1] = open(self.log_name[1], mode)
            

    #seperates string and returns what is after =
    def detlimit(self, info):
        tmp = info.readline().split('=')
        return float(tmp[1])
    
    #clears out the current log file
    def clearLog(self):
        self.log[self.select].close()
        remove(self.log_name[self.select])

    #changes the file mode of fileb
    def changeModeB(self, mode):
        self.bfile.close()
        self.bfile = open(fileb, mode)

    def changeModeL(self, mode):
        self.log[0].close()
        self.log[1].close()
        self.log[0] = open(self.log_name[0], mode)
        self.log[1] = open(self.log_name[1], mode)

    #convers 16 bit number into actual voltage^2
    def calculateSample(self, sample):
        return ((float(sample)*self.to_voltage-self.offset)/self.gain)**2

    #calculates the average RMS voltage from the contents of the current log
    def calculateRMS(self):
        self.changeModeL("r")
        value = 0
        ammount = 0
        self.log[self.select].seek(0, 0)
        #reads in entire log file
        while True:
            tmp = self.log[self.select].read().split("\n")[0]
            if tmp == '':
                break
            tmp = self.calculateSample(int(tmp))
            ammount += 1
            value += tmp
        return value/ammount #returns the average of the log

    #calculates the resistor value from the RMS from the current 
    #log and the average of the calculated boltzmanns contents
    def calculateResistor(self):
        self.changeModeB("r")
        value = 0
        ammount = 0
        self.bfile.seek(0, 0)
        while True:
            tmp = self.bfile.read().split("\n")[0]
            if tmp == '':
                break
            ammount += 1
            value += float(tmp)
        if ammount == 0: raise Exception("No values in "+ fileb) #if there is nothing in bfile then exception is raised
        BC = value/ammount                                       #takes the average of the boltzmanns contents
        return (self.calculateRMS())/(4*self.Temp*self.ENB*BC)   #calculates the resistor value

    #calculates boltzmann constant from the current log file
    def calculateBC(self):
        self.changeModeB("a")
        BC = (self.calculateRMS())/(4*self.Temp*self.ENB*self.R_Test)
        self.bfile.write(str(BC) + "\n")
        return BC

    def cleanLog(self):
        putty = open("putty.log", "a")
        putty.write("\n")
        putty.write("^^^^^^")
        putty.close()
        putty = open("putty.log", "r")
        self.open("a")
        current = "-1"
        
        while current != '^^^^^^':
            go = True
            tmp = str(putty.readline())
            if len(tmp) > 10 or len(tmp) <= 1:
                continue
            current = ""
            i = 0
            while(i < len(tmp)):
                current += tmp[i]
                i += 1
            try:
                int(current)
            except:
                go = False
            if go:
                if current == '^^^^^^' or int(current) == -1:
                    dumb = 0
                else:
                    self.log[self.select].write(str(current))
        putty.close()
        remove("putty.log")
