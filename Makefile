OPENCVLIBS=$(shell pkg-config --libs opencv)
OPENCVINCS=$(shell pkg-config --cflags opencv)
INCLUDES=-Iinclude
LIBS=-L$(PWD)/lib -lcamera
RPATH=$(PWD)/lib
GITHASH=githash:$(shell git rev-parse --verify HEAD)
CFLAGS=-ggdb -std=c++11
CXX=g++

all: lib/libcamera.so examples/test_camera 

lib/libcamera.so: src/camera.cpp include/camera.h
	$(CXX) $(CFLAGS) -pthread -shared -fPIC $(OPENCVINCS) $(INCLUDES) -o lib/libcamera.so src/camera.cpp src/colorspaces.cpp $(OPENCVLIBS) -DFWVERSION='"$(FWVERSION)"' -DGITHASH='"$(GITHASH)"'

examples/test_camera: examples/test_camera.cpp include/camera.h
	$(CXX) $(CFLAGS) $(OPENCVINCS) $(INCLUDES) examples/test_camera.cpp -o examples/test_camera $(OPENCVLIBS) $(LIBS) -Wl,-rpath,$(RPATH)

clean:
	rm bin/*
	rm lib/*

