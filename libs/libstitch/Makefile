.PHONY: all release clean debug
TARGET := libstitch.a
CXXFLAGS += -I ./include
CXXFLAGS += -I ../flann/include/
CXXFLAGS += -I ../libplanedrawer/include/
CXXFLAGS += -I ../libpolygon/include/
CXXFLAGS += -I ../libdist/include/
CXXFLAGS += -I ../libfeature/include/
CXXFLAGS += -I ../libmat/include/
CXXFLAGS += -I ../libhomography/include/
CXXFLAGS += -I ../libcolor/include/
CXXFLAGS += -I ../libimg/include/
CXXFLAGS += -I ../libmatch/include/
#
CXXFLAGS += -I ../libconfig/include/
CXXFLAGS += -I ../libtransform/include/
CXXFLAGS += -I ../libblend/include/
CXXFLAGS += -I ../libcamera/include/
CXXFLAGS += -I ../libwarp/include/
CXXFLAGS += -std=c++11  -O3 -fopenmp -Wall -Wextra -march=native  -msse2 -msse3
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
