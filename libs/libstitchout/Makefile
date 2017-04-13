.PHONY: all release clean debug
TARGET := libstitchout.a

CXXFLAGS += -I ./include
CXXFLAGS += -I ../libmat/include/
CXXFLAGS += -I ../CImg/include
CXXFLAGS += -I ../flann/include
CXXFLAGS += -I ../libadjuster/include
CXXFLAGS += -I ../libblend/include
CXXFLAGS += -I ../libcamera/include
CXXFLAGS += -I ../libcolor/include
CXXFLAGS += -I ../libconfig/include
CXXFLAGS += -I ../libdist/include
CXXFLAGS += -I ../libfeature/include
CXXFLAGS += -I ../libhomography/include
CXXFLAGS += -I ../libimg/include
CXXFLAGS += -I ../libkdtree/include
CXXFLAGS += -I ../liblodepng/include
CXXFLAGS += -I ../libmatch/include
CXXFLAGS += -I ../libmat/include
CXXFLAGS += -I ../libplanedrawer/include
CXXFLAGS += -I ../libpolygon/include
CXXFLAGS += -I ../libstitch/include
CXXFLAGS += -I ../libtransform/include
CXXFLAGS += -I ../libwarp/include
CXXFLAGS += -I ../liblut/include
CXXFLAGS += `pkg-config --cflags opencv`
EIGEN_PATH ?= /usr/include/eigen3
CXXFLAGS += -isystem $(EIGEN_PATH)
CXXFLAGS += -std=c++11 -O3 -fopenmp -Wall -Wextra -march=native  -msse2 -msse3

# -Werror
objects := $(patsubst %.cpp, %.o, $(wildcard src/*.cpp))

all : release

$(TARGET):$(objects)
	if [ ! -d lib ] ; then mkdir lib ; fi
	ar rs ./lib/$@ $^

release : $(objects) $(TARGET)
release : CXXFLAGS += -DNDEBUG

debug : $(objects) $(TARGET)
debug : CXXFLAGS += -DDEBUG -g

clean:
	rm -rf ./lib $(objects)
