CC=g++
CFLAGS=-Wall

all: crts crts1 crts2

crts: crts.cpp
	$(CC) $(CFLAGS) crts.cpp -o crts -lm -lliquid -lpthread -lconfig -luhd -lliquidusrp -lfftw3f

crts1: crts1.cpp
	$(CC) $(CFLAGS) crts1.cpp -o crts1 -lm -lliquid -lpthread -lconfig -luhd -lliquidusrp

crts2: crts2.cpp
	$(CC) $(CFLAGS) crts2.cpp -o crts2 -lm -lliquid -lpthread -lconfig -luhd -lliquidusrp
clean:
	rm crts
	rm crts1
	rm crts2
