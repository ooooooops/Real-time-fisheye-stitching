.PHONY: all release clean debug

all : release

TARGET = image-stitching


CXX = g++
#Matrix 
EIGEN_PATH ?= /usr/include/eigen3
INCLUDE_DIR = -isystem $(EIGEN_PATH)
INCLUDE_DIR += -I ./libs/CImg/include
INCLUDE_DIR += -I ./libs/flann/include
INCLUDE_DIR += -I ./libs/libadjuster/include
INCLUDE_DIR += -I ./libs/libblend/include
INCLUDE_DIR += -I ./libs/libcamera/include
INCLUDE_DIR += -I ./libs/libcolor/include
INCLUDE_DIR += -I ./libs/libconfig/include
INCLUDE_DIR += -I ./libs/libdist/include
INCLUDE_DIR += -I ./libs/libfeature/include
INCLUDE_DIR += -I ./libs/libhomography/include
INCLUDE_DIR += -I ./libs/libimg/include
INCLUDE_DIR += -I ./libs/libkdtree/include
INCLUDE_DIR += -I ./libs/liblodepng/include
INCLUDE_DIR += -I ./libs/libmatch/include
INCLUDE_DIR += -I ./libs/libmat/include
INCLUDE_DIR += -I ./libs/libplanedrawer/include
INCLUDE_DIR += -I ./libs/libpolygon/include
INCLUDE_DIR += -I ./libs/libstitch/include
INCLUDE_DIR += -I ./libs/libstitchout/include
INCLUDE_DIR += -I ./libs/libtransform/include
INCLUDE_DIR += -I ./libs/libwarp/include
INCLUDE_DIR += -I ./libs/liblut/include
INCLUDE_DIR += `pkg-config --cflags opencv`

OPTFLAGS ?= -O3 -march=native -msse2 -msse3
#OPTFLAGS ?= -g -O0
CXXFLAGS = $(INCLUDE_DIR) -fopenmp -Wall -Wextra 
#-Werror
CXXFLAGS += $(DEFINES) -std=c++11 $(OPTFLAGS)
# LD PATH
LDFLAGS = -L ./libs/libstitchout/lib/ -lstitchout
LDFLAGS += -L ./libs/flann/lib/ -lflann
LDFLAGS += -L ./libs/libstitch/lib/ -lstitch
LDFLAGS += -L ./libs/libcamera/lib/ -lcamera
LDFLAGS += -L ./libs/libadjuster/lib/ -ladjuster
LDFLAGS += -L ./libs/libhomography/lib/ -lhomography
LDFLAGS += -L ./libs/libblend/lib/ -lblend
LDFLAGS += -L ./libs/libtransform/lib/ -ltransform
LDFLAGS += -L ./libs/libpolygon/lib/ -lpolygon
LDFLAGS += -L ./libs/libwarp/lib/ -lwarp
LDFLAGS += -L ./libs/libmatch/lib/ -lmatch
LDFLAGS += -L ./libs/libdist/lib/ -ldist
LDFLAGS += -L ./libs/libmat/lib/ -lmat
LDFLAGS += -L ./libs/libplanedrawer/lib/ -lplanedrawer
LDFLAGS += -L ./libs/libdist/lib/ -ldist
LDFLAGS += -L ./libs/libfeature/lib/ -lfeature
LDFLAGS += -L ./libs/libimg/lib/ -limg
LDFLAGS += -L ./libs/libcolor/lib/ -lcolor
LDFLAGS += -L ./libs/libkdtree/lib/ -lkdtree
LDFLAGS += -L ./libs/liblodepng/lib/ -llodepng
LDFLAGS += -L ./libs/libimg/lib/ -limg
LDFLAGS += -L ./libs/liblut/lib -llut
LDFLAGS += -L ./libs/libconfig/lib/ -lconfig
#LDFLAGS += -L /usr/lib/i386-linux-gnu/ -ljpeg
LDFLAGS += -L /usr/lib/i386-linux-gnu/ -ldl
LDFLAGS += `pkg-config --libs opencv`
LDFLAGS += -L /usr/lib/i386-linux-gnu/ -lpng -ltiff -ljpeg -lz
LDFLAGS += -fopenmp -lstdc++ 

release:
	@make release -C libs
	@$(CXX)  main.cpp $(INCLUDE_DIR) $(CXXFLAGS) -DNDEBUG $(OPTFLAGS) $(LDFLAGS) -o $(TARGET)
	@strip $(TARGET)

debug:
	@make debug -C libs
	@$(CXX)  main.cpp $(INCLUDE_DIR) $(CXXFLAGS) -DDEBUG -g $(OPTFLAGS) $(LDFLAGS) -o $(TARGET)

clean:
	@make clean -C libs
	@rm -rf $(TARGET)

