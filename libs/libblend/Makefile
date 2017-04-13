.PHONY: all release clean debug
TARGET := libblend.a
CXXFLAGS += -I ./include 
CXXFLAGS += -I ../libmat/include/
CXXFLAGS += -I ../libcolor/include/
CXXFLAGS += -I ../libconfig/include/
CXXFLAGS += -I ../libimg/include/
CXXFLAGS += -I ../libfeature/include/
CXXFLAGS += -I ../liblut/include/
CXXFLAGS += -I ../libimg/include/
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
