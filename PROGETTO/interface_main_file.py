#------------------------------------------------------------------------------------------------------------------------------------------------------------
#----------------------------------IMPORT--------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------------------------------------------
from PySide2 import QtWidgets
import numpy as np
import time
#------------------------------------------------------------------------------------------------------------------------------------------------------------
#-------------------------------------FUNCTIONS--------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------------------------------------------
"""
Here you can add the function you used in your "old main file". This is because your "old main file" now will become a function itself
In this example we use "Imperio" as the function that contain the main code and we have just the "print_" function as additional function.
"""

#In order to print something important on the window, we create a new simple function "print_" to use as alternative to the classic "print".
#while "print" print something on the terminal, our new "print_" print the string "string" on the text-box fo "Dialog."
#so if you want to maintain some of your "print" in you main file, you just have to change
#"print(string)"-->"print_(Dialog,string)" and it will be printed also in your interface.
def print_(Dialog,string):
    Dialog.bigEditor.append(string)

#This is the function that we called "Imperio" (if you want tio change this name remember to change also the name in the code in "Interface.py")
#in this function you have to copy your "main code", the one you want to execute once you click on th button "start".
#It is very important to maintain its first argument "Dialog", but you can remove, change or add all the other arguments.
#just for exaple, we have a second argument "numero" where we store a random integer we generate in the file Interface.py for print it
#on the dialog form with our new "print_" function.
def Imperio(Dialog,numero):
    #------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----------------------------------SET UP--------------------------------------------------------------------------------------------------------------------
    #------------------------------------------------------------------------------------------------------------------------------------------------------------
    """
    Here you can declare all the variables you want to use in the next lines.
    """
    #------------------------------------------------------------------------------------------------------------------------------------------------------------
    #------------------------------------CODE--------------------------------------------------------------------------------------------------------------------
    #------------------------------------------------------------------------------------------------------------------------------------------------------------
    #This is just a simple example of "main code" where we print on the dialog form the random integer passed from the file "Interface.py".
    #after the print we lose time with a cicle of a lot of iterations just to try that the  "consecutive" clicks on the button "start"
    #won't produce any effects because we disable that button during all the execution of this code.
    print_(Dialog,"Numero : " + str(numero))
    i=0

    while i<100000000 :
        i=i+1
