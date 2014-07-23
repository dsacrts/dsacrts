dsacrts
=======

This adapts the Cognitive Radio Test System to be able to use and test various aspects of Dynamic Spectrum Access

DSACRTS User Guide:

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

./crts -rI starts a node as an interference emitter. Currently the node uses the cognitve engine file ce1.txt. It aseembles a frames and then distorts them with scenario sc1.txt, which applies a massive amount of AWGN. So it should essentially emit a bunch of AWGN. Using liquid frames for this might make it less noisy than would be liked and currently all it does it raise the power of the channel slightly and interfere with some frames, but it could have it uses


DSA Types

The thing that makes the secondary transmitter different than the primary transmitter is that it can sense the spectrum
