.PHONY: all release clean debug
TARGET := libflann.a
CXXFLAGS += -I ./include 
CXXFLAGS += -I /usr/lib/gcc/i686-linux-gnu/5/include/
CXXFLAGS +=  -O3 -fopenmp -Wall -Wextra -march=native  -msse2 -msse3
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
