dsacrts
=======
If this document doesn't help feel free to email fclaytor@aol.com

This adapts the Cognitive Radio Test System to be able to use and test various aspects of Dynamic Spectrum Access

Normal CRTS
===========
If you want to check out the non DSA version of CRTS go to this github repository

https://github.com/fmromano/crtb

Here is a copy of the README file written for normal CRTS
=======================================================

CRTS: Cognitive Radio Test System

The Cognitive Radio Test System (CRTS) is software designed to enable testing, evaluation, and development of cognitive engines (CEs) for cognitive radios (CRs). Users of CRTS are able to design and test a variety of CEs through custom combinations of the built-in CRTS cognitive abilities. Similarly, these CEs can be tested in a variety of signal environments by combining several of the built-in signal environment emulation capabilities. Through the parameterization of the cognitive abilities and of the signal environments, and through the ability to test each CE against each signal environment in succession, CRTS enables effective testing and development of CEs.

CRTS comes with the ability to test CEs with both a) simulated signal environments and b)  USRPs for live radio transmissions in emulated signal environments.  

Building CRTS:
    1. Install the necessary dependencies
            Required: libconfig>=1.4, liquid-dsp, uhd>=3.5, liquid-usrp (fmromano version)
            Recommended: linux>=3.10
    2. Build with
            $ make

Running CRTS:
    CRTS must be run from the root of the source tree. 
    To run CRTS in simulation mode, use:
            $./crts -c
       The crts binary will simulate both a transmitter and a receiver, perform the cognitive functions, and record experiment data.

    To run CRTS using USRPs, use both
            $./crts -rc
        for the controller and
            $./crts -r
        for the receiver node.
        They can be run on separate machines, but they must be networked. Check the command line options for specifying IP addresses and ports. 
        When using CRTS with USRPs, the crts contorller connects to the transmitter USRP, performs the cognitive functions, and records experiment data. The crts receiver, on the other hand, connects to the receiver USRP and sends feedback to the transmitter over a TCP/IP connection. 

    For available command line options, use:
            $ ./crts -h

Creating and Testing Cognitive Engines
    Cognitive Engines are represented by configuration files located in the ceconfigs/ directory of the source tree. All CE config files should be placed there. An example CE config file, called 'ce1.txt', is provided. Read it to learn about the currently supported options for CEs.
    To test one or more CEs, they should be listed in the CE master file, 'master_cogengine_file.txt'. CRTS will run each listed CE through each test scenario. More details can be found in the CE master file.

Creating and Using Scenarios/Signal Environments
    Testing Scenarios are represented by configuration files located in the scconfigs/ directory of the source tree. All scenario config files should be placed there. Several example scenario config files are provided with CRTS. 
    To use a scenario file in a test, it should be listed in scenario master file, 'master_scenario_file.txt', which uses the same format as the CE master file. 

Data logging
    All data files are logged in the data/ directory. The file names have the date and time of the start of the test appended to them.
    
    
We would like to give thanks to Virginia Tech ICTAS for their support in funding our research into Cognitive Radio Testing and Evaluation.
=============================================

DSACRTS User Guide:
===================

DSACRTS builds upon the basic CRTS framework to allow the simulation and testing of DSA networks
To run crts you have to compile the code with the make command then use the terminal command ./crts to execute it
But just using ./crts will just run default crts. To run DSACRTS you need to add different command line arguments

./crts -rcD will start running the node as a Controller

./crts -rP -a (controller addresss) will start running the node as a primary transmitter
All transmitter must be given the IP address of the Controller using the -a argument

./crts -rS -a (controller address) initializes a secondary transmitter

./crts -rPB -a (controller address) initializes a primary transmitter that can receive feedback from receivers
The B stands for broadcasting and tells the program to start up a thread to average together receiver feedback

./crts -rSB -a (controller address) starts the node as a secondary transmitter that can receive feedback from receivers

./crts -rRP -a (primary transmitter address) starts running the node as a primary receiver. Primary receivers will receive
the frames transmitted by the transmitters and run a callback function to interpret them and find out various metrics
Then they send the feedback through a TCP link back to the transmitter

./crts -rRS -a (secondary transmitter address) starts running a node as a secondary receiver. They act just like a primary receiver but interpret secondary frames (they can differentiate between the two by using the header)

./crts -rE -a (secondary transmitter address) starts a node as an energy detector that can sense the power of the spectrum to detect the primary user. It runs an fft function over and over and sends messages to the secondary transmitter about the results. The energy detector cannot receive and interpret secondary frames because having a usrp object for receiving fft samples and having a liquid usrp object to receive frames causes overflow and errors and problems and such

./crts -rI starts a node as an interference emitter. Currently the node uses the cognitve engine file ce1.txt. It assembles a frames and then distorts them with scenario sc1.txt, which applies a massive amount of AWGN. So it should essentially emit a bunch of AWGN. Using liquid frames for this might make it less noisy than would be liked and currently all it does it raise the power of the channel slightly and interfere with some frames, but it could have it uses


DSA Types
=========

The thing that makes the secondary transmitter different than the primary transmitter is that it can sense the spectrum
and tell whether the primary user is transmitting or not. That way it can intelligently switch off and on to use the
spectrum holes the primary user isn't using while avoiding interfering with the primary transmissions

To change the detection type just change the character for the detectiontype parameter in the master_dsa_file.txt

Currently DSA CRTS supports 4 spectrum sensing detection types

Header Matching 'm'
Energy Detection 'e'
Header Matching Receivers 'r'
Energy Detection Receivers 't' (due to technical limitations the energy detecting receivers can't actually
				receive liquid frames)

Header Matching is done using the dsacallback function. This is a function that is called whenever a liquid transceiver
object receives a liquid frame. First the function looks at the received frames header. The primary and secondary
transmitters are coded so they create frames with different headers. Primary frames have a header of 111's while
secondary frames have a header of 0's. The callback function allows the secondary user to receive liquid frames and analyze
the headers to determine where they came from. If the callback function discovers that a frame is primary a variable is
changed and the primary user is detected. The secondary transmitter stops transmitting and switches to sensing mode,
where it is stuck in a while loop until it doesn't detect any primary frames for a set amount of time. Then it will
resume transmitting until its callback functino detects another primary frame, and so on and so forth

Energy Detection usingn FFT to find the power of parts of the spectrum to see if the primary user is transmitting or if
there is just noise. To do this first the function noise_floor is called. It is given a usrp object that it uses to
receive samples. It then calculates the FFT of the received samples a certain number of times. Each time it computes the
FFT it adds the first 30 values computed to a variable called totalpower. It uses the first ones because the values put
in the earliest spots of the output vector come from a more central position of the FFT, so they are all close to the
central frequency that the primary user is on. It doesn't have to be 30, you can change the number of measured FFT bins
in the master_dsa_file.txt. After the FFT is run enough times totalpower will be divided by the number of bins measured
and by the number of times the FFT was computed to find the average power of a single bin. This is the noise floor. The noise floor is then multiplied by noisemult and added to noiseadder (both variables in the master dsa file) to create a threshold that is higher than the noise floor. If the energy detector detects a power above this threshold it will assume the primary user is transmitting. The secondary user should have a power somewhere between the noise floor and the threshold.
The second function used to energy detect is fftscan. This is just like the noise_floor function except instead of finding a noise floor it uses the same math used by the noise floor function to find the current power of the spectrum. It then compares it to the threshold and returns a 0 if it detects the primary user and returns a 1 if it doesn't
All of the variables for the fft function are passed to it with the fftstruct struct. Variables like gain, bandwidth,
number of bins, noiseadder, etc.

Header Matching with receivers is just like normal Header Matching but the receivers use their callback functions to do the sensing while the transmitter just transmits and doesn't make any decisions about sensing unless the receivers use TCP to
tell it to switch off or on.

Energy Detection with receivers is also pretty similar to its nonreceiver counterpart. But the energy detecting receivers
are incapable of receiving liquid frames so they aren't really receivers at all, more like energy detectors detached from
the secondary transmitter. It the liquid transceiver receiver is started while the fft usrp object exists overflows and
bad things happen. But the energy detector works for energy detection. It is suggested to start the secondary transmitter
before finding the noise floor so that it doesn't accidentally surpass the threshold. Once the energy detector is calibrated
it starts running and will warn the secondary user with a TCP message when it detects a power level that is high enough
to be the primary user

TCP Communication:
==================

Both normal CRTS and DSA CRTS use TCP to send messages between different computers to facilitate feedback receiving and
timing info. Normal CRTS just sends feedback structs from the receiver to the controller. DSA CRTS's TCP messages
are a bit more complicated

TCP works by creating a connection between a server and a client. After the connection is created the client can use the
write function to send data to the server. The server can then use the read function to read the data into a readbuffer
so it can be interpreted

In DSA CRTS the clients send a special struct called a message with their write commands. The message is just a struct
that is a hodge podge of things that you might want to send with TCP. It has a feedback struct, some ints, some
chars, various things

One important variable in the message struct is the type. Type is a char variable, which means it is a single character
The type of the message tells the receiving server where it came from
'P' means it came from a primary transmitter
'S' means it came from a secondary transmitter
'p' means it came from a primary receiver
's' means it came from a secondary receiver

Another char variable is the purpose. This tells the server why the client sent the message
't' The client transmitter is turning on
'r' The client transmitter is turning off (resting/sensing)
'f' Either a receiver is sending feedback to a transmitter or a transmitter is sending feedback to the controller
'F' Primary user telling the controller it has finished all of its cycles and the test is complete
'D' Energy detector telling secondary transmitter that it detected the primary user
'd' Energy detector telling secondary transmitter that it didn't detect the primary user
'u' Receiver telling its transmitter that it detected a frame with an unknown header
'P' Receiver telling its transmitter that it detected a primary frame
'S' Receiver telling its transmitter that it detected a secondary frame

The message struct also has a variable called number. This is a int and it increments with each message sent. This
is to make sure that if a server receives the same message twice it doesn't read it again because it will only accept
messages from a client with a number higher than the last message it received

The message struct also has a variable called feed. Feed is a feedback struct and is used whenever feedback is passed
from client to server

Normal CRTS sets up the TCP links with the starttcpserver function. DSA CRTS does the same thing. Normal CRTS reads in
feedback from TCP using a threaded tcpclient function. Threading means that it uses pthread to run a function at the same
time as another function. That way the controller and transmitter can transmit at the same time they receive receiver
feedback. The two functions can run at the same time. The tcpclient function reads in a feedback struct, the function
analyzes it and does stuff with it, and when it is done it changes a variable in a pointer given to the pthread to tell
it it is ready for another feedback. The tcpclient will read in a new feedback struct and then wait again for the main
function to signal for the next one.

DSA CRTS follows a similar strategy but it uses TCPDSAclient. Instead of reading in feedback it reads in messages
One client function runs for each client linked up to the TCP server. Each client function is stuck in an infinite while loop
where it reads in a message, checks to see if the number is higher than the last received, checks that it has a valid type,
and then sends it to the main function that will then interpret it and do stuff based on the purpose of the message

Messages are very useful. They allow receivers to give feedback to the transmitters, and the transmitters to send the
collected and averaged feedback to the main controller. The transmitters also send messages to the controller when
they turn off and on so that the controller can calculate things like evacuation time, rendezvous time, spectrum usage, etc.
This makes sure that all timings done are synchronized because they are all done by the same computer

The transmitters have special threads running to interpret messages from receivers. A function called feedbackThread is
running at the same time as the main function. When the TCPDSAclient finishes checking a message it sends it to the
feedbackThread which then checks it and performs certain actions based on its type and purpose. The feedbackThread
function is used both to receive error feedback and to interpret signal from spectrum sensing secondary receivers
It also will try to average together feedback from multiple receivers that all correspond to the same frame
It does this by having an array of ints called clientlist
Whenever it receives feedback it checks clientlist. If it doesn't contain the number for the client the number will be
added and the feedback will be added to the other feedback received
Once the transmitter receives feedback from a client already in the list then it will think "Huh, I already received
feedback from this client, so this feedback I just got must be from a new frame" and it will assume that the new frame
feedback starts with the feedback it just got. So it will take all of the feedback for the previous frame it has collected,
average them together, and send them to the controller so that it can look at them and write them to the data file
It will then zero out its feedbacks and clientlist and then add the newly received feedback to the zeroed out feedback
collector.

The controller also interprets and does stuff based on messages, but since its only job is to read messages it doesn't
require something like feedbackThread to run as a thread. The controller will stay in a while loop until it receives
the quit message from the primary transmitter (purpose of 'F'). In this while loop it will take all messages received
by TCPDSAclient and do things based on their purpose. If it gets a 't' or an 'r' it will record the time that that
particular transmitter turned off or on. If the secondary transmitter is changing states the controller will determine
the evacuation time, or rendezvous time, or whether the change was a false alarm or missed detection.

If the purpose is 'f' the controller will take the received feedback and write it to the data file. It will also use the
feedbackadder function (it adds feedback, that's it) to add it to the proper special feedback structs

The controller has 7 special feedback structs that it adds to throughout its while loop and at the end it will
average them togeter and print them out to show the metrics of various frames from the test

The 7 feedback structs are:

Total Feedback: The averaged feedback of every frame received

Primary Feedback: Averaged feedback of every primary frame

Safe Primary Feedback: Averaged feedback of every primary frame received when secondary user was off

Primary Collision Feedback: Averaged feedback of every primary frame received when secondary user was on

Secondary Feedback: Averaged feedback of every secondary frame

Safe Secndary Feedback: Averaged feedback of every secondary frame received when primary user was off

Secondary Collision Feedback: Averaged feedback of every secondary frame received when primary user was on

If the controller receives a purpose of 'u' it will increment the unknownheader variable

The while loop the controller is in also contains some nested if statements related to spectrum usage
Every time interval, determined by the float variable spectrumchecktime (currently 0.05 seconds), the if
statement will trigger and the controller will check its variables to see which transmitters are on and which are
off. Since it has variables for each it is simple to do so. It will then increment the proper variables.
As these variables are being incremented many many times over the course of the test they should be pretty accurate
Once the primary transmitter sends the finish message the controller will take these incremented spectrum usage variables
and divide them to find the percentages of when certain transmitters were on and how efficiently they used the spectrum
There are 5 different measurements found

Spectrum Usage: Percentage of the time that the spectrum was used at all
Primary Spectrum Usage: Percentage of the time that the primary user used the spectrum
Secondary Spectrum Usage: Percentage of the time that the secondary user used the spectrum
Secondary Free Spectrum Usage: What percent of the empty spectrum time the secondary user utilized
Overlap Spectrum Usage: Percentage of the time that both transmitters were transmitting

This can help the user see how efficiently the secondary user used the spectrum and how much overlap and
interference there was

Proper DSA CRTS Initialization Order
====================================

When using DSA CRTS you have to start each part seperately with its own set of command line arguments.
Also note that whenever a node is a TCP server (it can receive TCP connections) it will pause before starting so that
you can connect all clients to its server before pressing enter and telling it to start
There is an optimal order for doing this

Controller (don't press enter to start it yet)
Secondary Transmitter (if using receivers don't press enter to start it yet)
Primary Transmitter (if using receivers don't press enter to start it yet)
All secondary receivers
All primary receivers
Controller (press enter to start it and it will start checking for messages and running its while loop)
Secondary Transmitter (press enter to start it transmitting)
Energy Detector (If you are using energy detectors try to start them after the secondary transmitter starts so it can
		take the secondary power into account when making the threshold)
Primary Transmitter (press enter to start)

Once the primary transmitter starts the controller will start timing and records rendezvous times, evacuation times,
and other metrcis

This order of activating nodes should work well for testing


Feedback
========

Feedback and feedback structs allow receivers to determine various metrics from the frame they receive
Feedback is determined by the callback function
The default rxCallback and the DSA CRTS dsacallBack work pretty much the same way except the dsa one uses header matching
The error vector magnitude, received signal strength indicator, and central frequency offset is determined automatically
by the liquid framesync and put into the stats object. So the callback function just takes that object and puts some
of its variables into the feedback struct
Byte and bit errors are done by comparing the received payload to the expected payload. The payload is an array of bytes
First each byte in the received payload is compared to its corresponding one in the expected payload
If the byte aren't the same the byte errors variable is incremented
After the bytes are compared each bit in each byte is compared the same way, which each error leading to incrementing
the bit error variable one more time
The callback function also automatically determines whether the header and payload are valid
This allows the callback function to find many different metrics, combine them in one feedback struct, and then use
TCP to send the feedback struct to the transmitter so it can interpret it, analyze it, and choose to adapt or not

Scenarios
=========

Scenarios allow the simulation of different network conditions. Additive gaussian white noise, fading, and continuous wave interference can all be simulated by the transmitter or receiver. This allows frames to be distorted without having to
produce actual interference. The different scenarios are listed in the scconfigs file
To use a scenario with the primary transmitter got to the master_dsa_file.txt and set the PU's usescenario variable to
1. Then type the name of the scenario file you wish to use into the scenario variable. Then when the primary user transmits
it will apply the scenario first to distort its transmissions so you can observe the effects
It works the same for the secondary transmitter
DSA CRTS does not yet support receiver based scenarios

Cognitive Engines
=================

The primary and secondary users both use cognitive engines to give their transceiver usrps certain parameters. All
transmitters and receivers of the same type (primary or secondary) use the same cognitive engine
The cognitive engine parameters are stored in configuration files found in the ceconfigs folder
The ce variables in the master_dsa_file.txt should have the name of the ceconfig file you wish to use for the
primary and secondary cogntive engines
Currently the primary user uses primary.txt and the secondary uses secondary.txt
You can edit many parameters by changing these ceconfig files, though problems might arise
For some reason changing the bandwidth or modulation scheme makes it hard for receives to receive the transmissions
The uhd_txgain variable can be changed to change the power of the transmitter
Use uhd_usrp_probe to find the range of values that the usrp is capable of supporting for different parameters

Configuration Files
===================

One of the strengths of CRTS is that it uses configuration files to allow the user to easily modify the parameters of
the cognitive engines and scenarios they are testing without having to edit and debug the code itself
CRTS uses the library libconfig to read through the configuration files and pull out values to assign to variables
It can read in ints, floats, chars, strings, and other things, making configuration files a convenient way to
assign different values to parameters

DSA CRTS uses configuration files just like normal CRTS does but it uses a new one called master_dsa_file.txt
The master dsa file contains the parameters for the DSA tests, the primary user, secondary user, and for the
parameters for the secondary user's energy detecting fft function
The master dsa file can be easily edited to test different DSA strategies

Check out the documentation for libconfig for details
http://www.hyperrealm.com/libconfig/libconfig_manual.html

Data Files
===========

Whichever computer is the controller will create a data file every time a test is run
The data file will be put in the data folder and will be named with the date and time that it was created
The data file contains all of the data that is printed out by the controller, as well as some data from every
piece of feedback struct received by the controller
The data is written to the data file with the function fprintf
It is also possible to make the data that would be printed to the file be printed to the terminal by using
the d command line argument to set the data file name to stdout
Data files are very useful because they allow you to save a lot of data instead of struggling to record it all as
it rushes past on the terminal
