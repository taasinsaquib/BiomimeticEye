CC=g++
CFLAGS=-Wno-deprecated-declarations -framework OpenGL -lglut

default:
	$(CC) $(CFLAGS) -o prog main_eye.cpp glutFuncs.cpp helpers.cpp

exp:
	$(CC) $(CFLAGS) -o prog main.cpp

clean:
	rm prog