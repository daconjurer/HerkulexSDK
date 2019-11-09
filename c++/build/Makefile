######################################################
###### Robótica de La Mixteca
###### Herkulex_SDK for Hovis HerkuleX servos Makefile
######################################################

# Compiler, Flags and Target name

DIR_HERKULEX      = ..
DIR_OBJECTS       = ./.objects

INSTALL_ROOT      = /usr/local

TARGET            = libhklx_linux_x86.so

CHECK_DIR_EXISTS  = test -d
LD                = g++
LDFLAGS           = -shared -fPIC $(FORMAT)#-Wl,-soname,hklx
LD_CONFIG         = ldconfig
CP                = cp
CP_ALL            = cp -r
RM                = rm
RM_ALL            = rm -rf
SYMLINK           = ln -s
MKDIR             = mkdir
CX                = g++
CXFLAGS           = -Wall -std=c++11 -c $(INCLUDES) $(FORMAT) -fPIC -g
FORMAT            = -m64
INCLUDES          += -I$(DIR_HERKULEX)/include/herkulex_sdk

# Required external libraries
LIBRARIES         += -lrt

# SDK Files
SOURCES           = $(DIR_HERKULEX)/src/herkulex_sdk/port_handler.cpp \
                    $(DIR_HERKULEX)/src/herkulex_sdk/packet_manager.cpp \
                    $(DIR_HERKULEX)/src/herkulex_sdk/servo_herkulex.cpp

OBJECTS           = $(addsuffix .o,$(addprefix $(DIR_OBJECTS)/,$(basename $(notdir $(SOURCES)))))

# COMPILING RULES
$(TARGET): makedirs $(OBJECTS)
	$(LD) $(LDFLAGS) -o ./$(TARGET) $(OBJECTS) $(LIBRARIES)

makedirs:
	mkdir -p $(DIR_OBJECTS)/

clean:
	rm -f $(OBJECTS) ./$(TARGET)

install: $(TARGET)
# copy the binaries into the lib directory
	@$(CHECK_DIR_EXISTS) $(INSTALL_ROOT)/lib || $(MKDIR) $(INSTALL_ROOT)/lib
	-$(CP) "./$(TARGET)" "$(INSTALL_ROOT)/lib/$(TARGET)"

# copy the headers into the include directory
	@$(CHECK_DIR_EXISTS) $(INSTALL_ROOT)/include/herkulex_sdk || $(MKDIR) $(INSTALL_ROOT)/include/herkulex_sdk
	$(CP_ALL) $(DIR_HERKULEX)/include/herkulex_sdk/* $(INSTALL_ROOT)/include/herkulex_sdk

	$(LD_CONFIG)

uninstall:
	$(RM) $(INSTALL_ROOT)/lib/$(TARGET)

	$(RM) $(INSTALL_ROOT)/include/herkulex_sdk/herkulex_sdk.h
	$(RM_ALL) $(INSTALL_ROOT)/include/herkulex_sdk/*

reinstall: uninstall install

# Make rules for .cpp files in each directory
$(DIR_OBJECTS)/%.o: $(DIR_HERKULEX)/src/herkulex_sdk/%.cpp
	$(CX) $(CXFLAGS) -c $? -o $@

