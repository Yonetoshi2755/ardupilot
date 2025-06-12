# Simple Makefile for hexacopter control system

CXX = c++
CXXFLAGS = -std=c++11 -Wall -Wno-unused-variable -Wno-unused-function -I./include
LDFLAGS = -lm

SOURCES = src/hexacopter_sysid.cpp \
          src/ekf_attitude_velocity.cpp \
          src/velocity_feedback_controller.cpp \
          tests/test_build.cpp

OBJECTS = $(SOURCES:.cpp=.o)
TARGET = test_hexacopter

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(OBJECTS) -o $(TARGET) $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJECTS) $(TARGET)

test: $(TARGET)
	./$(TARGET)

.PHONY: all clean test