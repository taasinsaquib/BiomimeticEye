CC=g++
# CFLAGS=-I.

default: main.cpp
	$(CC) -o prog main.cpp

clean:
	rm prog