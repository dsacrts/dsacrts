CC=g++
CFLAGS=-Wall

all: crts

crts: crts.cpp
	$(CC) $(CFLAGS) crts.cpp -o crts -lm -lliquid -lpthread -lconfig -luhd -lliquidusrp -lfftw3f

clean:
	rm crts
