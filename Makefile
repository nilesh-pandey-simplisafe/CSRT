TARGET = csrt

ARCH=$(shell bash -c "ldd `which dpkg` | grep libc.so | sed 's:.*/lib/\(.*\)/libc.*:\1:g'")

# ENABLE THESE FOR CUSTOM OPENCV INSTALL
OPENCV_INC="/usr/local/include/opencv2/" 
OPENCV_LIBS="/usr/local/lib/"
LIBS = -lopencv_core -lopencv_videoio -lopencv_imgproc -lopencv_highgui -lopencv_photo -lopencv_video -o csrt
#OPENCV_INC="/home/nile649/opencv_build/CSRT-tracker-standalone/" 

LIB_COMMON = -L$(OPENCV_LIBS) -lgcov

CXX = g++

# STEP 0
CXXFLAGS = -std=c++14 -march=native -O0 -g3 -Wall -I . -I $(OPENCV_INC)

# STEP 1
# CXXFLAGS = -std=c++14 -march=native -O3 -g0 -fprofile-generate -fprofile-dir=/tmp -Wall -I . -I $(OPENCV_INC)

# STEP 2
#CXXFLAGS = -std=c++14 -march=native -O3 -g0 -fprofile-use -fprofile-dir=/tmp -fprofile-correction -Wall -I . -I $(OPENCV_INC)

# PROFIT!

.PHONY: default all clean

default: $(TARGET)
all: default

OBJECTS = $(patsubst %.cpp, %.o, $(wildcard *.cpp))
HEADERS = $(wildcard *.hpp)

%.o: %.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

.PRECIOUS: $(TARGET) $(OBJECTS)

$(TARGET): $(OBJECTS)
	$(CXX) -Wall $(OBJECTS) $(LIB_COMMON) $(LIBS) -o $@

clean:
	-rm -f *.o
	-rm -f $(TARGET)

# CC = g++
# CFLAGS = -Wall -std=c++11
# LIBS = -lopencv_core -lopencv_videoio -lopencv_imgproc -lopencv_highgui -lopencv_photo -lopencv_video
# INCLUDES = "/home/nile649/opencv_build/opencv/build/include"

# SRCS = $(wildcard *.cpp)
# OBJS = $(SRCS:.cpp=.o)
# TARGET = main

# all: $(TARGET)

# $(TARGET): $(OBJS)
# 	$(CC) -L $(OBJS) $(LIBS) -o $(TARGET)

# %.o: %.cpp
# 	$(CC) $(CFLAGS) $(INCLUDES) -c $<

# clean:
# 	rm -f $(OBJS) $(TARGET)