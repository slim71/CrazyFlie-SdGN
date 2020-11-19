README.txt

REQUIREMENTS:

1)
VICON SYSTEM:
	- TRACKER SOFTWARE
	- ACTIVE WAND

2)
BITCRAZE HARDWARE:
	- CRAZYFLIE
	- FLOWDECK	
	- CRAZYRADIO

3)
WINDOWS PC (Not the same PC where the Tracker runs)

4)
Python 3.7 Version is HIGHLY RECOMMENDED. We invite you to
delete all other versions in order to avoid conflicts.

5)
Matlab version 2020 in order to plot the results from the log
file. The file you have to open is called "plot_log_file.m"





INSTALLATION OF THE REQUIRED LIBRARIES:

	
Install numpy: 
	From the Prompt go to the directory
	"C:\Users\'your username'\AppData\Local\Programs\Python\Python37\Scripts"
	Type the command "pip install numpy"
    
Install Vicon Datastream_SDK: 
	Double-Click on "install_vicon_dssdk" to install.
	Delete file "pip".

Install USB:
	From the Prompt go to the directory
	"C:\Users\'your username'\AppData\Local\Programs\Python\Python37\Scripts"
	Type the command "pip install pyusb"
	Type the command "pip install libusb1"

Install threading:
	From the Prompt go to the directory
	"C:\Users\'your username'\AppData\Local\Programs\Python\Python37\Scripts"
	Type the command "pip install threaded" 

Install CrazyRadio Driver:
	Connect the CrazyRadio to a USB port
	Go to the link: "https://zadig.akeo.ie/" and download the last version
	Use this link as Guide for installation:
    "https://wiki.bitcraze.io/doc:crazyradio:install_windows_zadig"




INTERFACE.py

If you want to use the "Interface.py" please search for the
latest version of the QtCreator package and download it.

You also have to :
a) 
Chose one of the files : "INSEGUIMENTO.py" , "RELATIVO.py" and
change its name in "interface_main_file.py"


b)
Crate a new function where you "transfer" all the code. You can
call that function "Imperio(Dialog)" if you don't want to
change anything else in the code.





OBSERVATIONS: 

0)
Before running the code it is highly recommended to read all
comments in the file ! In that comments in fact we explained
also everything about all the values you have to set before
launching the experiment.


1) 
During the execution we continuously receive the position of
the Wand from the Vicon Tracker and when we want to interrupt
the experiment we have to turn off the wand.
When this happens, the function "GetSegmentGlobalTranslation(..)"
used for receiving the Wand Position from the Vicon tracker
doesn't return an exception but it just returns the position
values: [0,0,0]. Taking into consideration the fact that it is
highly improbable that the Wand will have this exact position
for more than two consecutive frames, we decide to consider
"MAX_LOSS" consecutive Wand's position with value [0,0,0] as
the error encoding for the end of the experiment.





PROCEDURE FOR THE EXECUTION OF THE EXPERIMENT: 

0) 
It is highly recommended to start a new calibration of the
cameras for each session.


1) 
Place markers on both Drone and Wand in order to crate the
relative objects with the Vicon Tracker.


2) 
Set the body frame for the Drone-Object on the Vicon in order
to have the same orientation of the body frame of the Drone.


3) 
Check that both Wand and Drone are correctly seen by the Vicon
Tracker. Otherwise try a different placing for the markers.
Check also that the body frame for the drone object has this
axis orientation: 
- Z up
- X stern-bow 
- Y right-left watching form stern to bow

4) 
Choose a starting point for the Drone and place it on the floor
of the flight room. Only when you placed the drone on the floor
you can turn it on. It is important to maintain the drone
placed on the floor with no pitch and no roll angles also WHILE
you press the button for turning it on, this is because when
you press the button the drone considers its current
orientation and position as initial values for its body frame's
origin and orientation so, if you don't follow this procedure,
it will consider wrong values of initial orientation. Regarding
to the position instead, when you turn on the drone it will
consider the current position as the origin of its body frame
so you don't have to change its position once you pressed the
start button.


5) 
Once both Drone and Wand are turned on you can start the
experiment pressing the START button of the interface.
If you don't use the "Interface.py" you can simply run your
main file. 

6) 
The experiment will be concluded when you will turn off the
ActiveWand. If something wrong happens and you want to analyse
it critically, we suggest to set = 1 the flag "MAKE_LOG_FILE"
so at the end of the experiment you will find all information
you need. 


7)
Every time that the experiment ends (correctly or not) it is
very important to turn off the Drone and then turn it on again
from the new starting position of the new experimentation.
In this way you can be sure that both orientation and position
are correctly initialised. 

8)
Enjoy the flight !