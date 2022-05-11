# UI.py
#
# -Holds the Page class
# -Page class is an object that prints a page to the command line
# -Used in master.py
#
# Written and Designed by Michael Thompson

from os import system, name
class Page:
    def __init__(self, title, width, border='_'):
        self.title = title
        self.width = width
        self.border = border
        self.command_line = "uninitialized"
        self.task = "no task running"
        self.contents = [""]
    
    #change all page variables
    def changePage(self, command_line, task, contents):
        self.command_line = command_line
        self.task = task
        self.contents = contents

    #change command line variable
    def changeCommand(self, command_line):
        self.command_line = command_line

    #change task variable
    def changeTask(self, task):
        self.task = task

    #change contents variable
    def changeContents(self, contents):
        self.contents = contents

    #Show page
    def show(self, clear=True):
        if clear: self.clear()
        #divider
        for x in range(self.width):
            print(self.border, end='', flush=True)
        #title
        print("")
        print(self.title.center(self.width))
        #running task
        print("Task:" + self.task.rjust(self.width-len("Task:")))
        #print contents
        i = 0
        while i < len (self.contents):
            print(self.contents[i].center(self.width))
            i += 1
        #print command
        print(self.command_line.ljust(self.width))
    
    #clear function
    def clear(self):
        # for windows
        if name == 'nt':
            _ = system('cls')
        # for mac and linux(here, os.name is 'posix')
        else:
            _ = system('clear')


    

