CC=g++
CFLAGS=-Wno-deprecated-declarations -framework OpenGL -lglut

default:
	$(CC) $(CFLAGS) -o prog main.cpp
clean:
	rm prog