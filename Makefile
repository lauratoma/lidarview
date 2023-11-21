PLATFORM = $(shell uname)

CFLAGS = -g  -Wall 
LDFLAGS=


ifeq ($(PLATFORM),Darwin)
## Mac OS X
CFLAGS += -m64 -isystem/usr/local/include  -Wno-deprecated 
LDFLAGS+= -m64 -lc -framework AGL -framework OpenGL -framework GLUT -framework Foundation

else
## Linux
CFLAGS += -m64
INCLUDEPATH  = -I/usr/include/GL/ 
LIBPATH = -L/usr/lib64 -L/usr/X11R6/lib
LDFLAGS+=  -lGL -lglut -lrt -lGLU -lX11 -lm  -lXmu -lXext -lXi
endif


CC = g++ -O3 -Wall $(INCLUDEPATH)


PROGS = lidarview

default: $(PROGS)

lidarview: lidarview.o  lidar.o 
	$(CC) -o $@ lidarview.o  lidar.o $(LDFLAGS)

lidarview.o: lidarview.cpp lidar.hpp  
	$(CC) -c $(INCLUDEPATH) $(CFLAGS)   lidarview.cpp  -o $@

lidar.o: lidar.cpp lidar.hpp   
	$(CC) -c $(INCLUDEPATH) $(CFLAGS)   lidar.cpp  -o $@


clean::	
	rm *.o
	rm lidarview


