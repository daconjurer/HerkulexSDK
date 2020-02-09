######################################################
###### Robótica de La Mixteca
###### Example's for Hovis HerkuleX servos Makefile
###### based on the Dynamixel's Makefile template
###### for projects using HerkulexSDK
######################################################

#---------------------------------------------------------------------
# Makefile template for projects using HerkulexSDK
#
# Please make sure to follow these instructions when setting up your
# own copy of this file:
#
#   1- Enter the name of the target (the TARGET variable)
#   2- Add additional source files to the SOURCES variable
#   3- Add additional static library objects to the OBJECTS variable
#      if necessary
#   4- Ensure that compiler flags, INCLUDES, and LIBRARIES are
#      appropriate to your needs

TARGET = poly

# important directories used by assorted rules and other variables
DIR_HERKULEX = ../..
DIR_OBJECTS = .objects

# compiler options
CX = g++
CXFLAGS = -Wall -std=c++11 $(INCLUDES) $(FORMAT) -g
LNKCC = $(CX)
LNKFLAGS = $(CXFLAGS) #-Wl,-rpath,$(DIR_THOR)/lib
FORMAT = -m64

# Core components (all of these are likely going to be needed)
INCLUDES += -I$(DIR_HERKULEX)/include/herkulex_sdk
LIBRARIES += -lhklx_linux_x86
LIBRARIES += -lrt

# Files
SOURCES = ros_hklx.cpp \
  # *** OTHER SOURCES GO HERE ***

OBJECTS = $(addsuffix .o,$(addprefix $(DIR_OBJECTS)/,$(basename $(notdir $(SOURCES)))))
#OBJETCS += *** ADDITIONAL STATIC LIBRARIES GO HERE ***

# Compiling Rules
$(TARGET): make_directory $(OBJECTS)
	$(LNKCC) $(LNKFLAGS) $(OBJECTS) -o $(TARGET) $(LIBRARIES)

all: $(TARGET)

clean:
	rm -rf $(TARGET) $(DIR_OBJECTS) core *~ *.a *.so *.lo

make_directory:
	mkdir -p $(DIR_OBJECTS)/

$(DIR_OBJECTS)/%.o: %.cpp
	$(CX) $(CXFLAGS) -c $? -o $@
