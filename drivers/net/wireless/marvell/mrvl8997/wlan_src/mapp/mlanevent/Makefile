# File : mlanevent/Makefile
#
# Copyright 2014-2020 NXP

# Path to the top directory of the wlan distribution
PATH_TO_TOP = ../..

# Determine how we should copy things to the install directory
ABSPATH := $(filter /%, $(INSTALLDIR))
RELPATH := $(filter-out /%, $(INSTALLDIR))
INSTALLPATH := $(ABSPATH)
ifeq ($(strip $(INSTALLPATH)),)
INSTALLPATH := $(PATH_TO_TOP)/$(RELPATH)
endif

# Override CFLAGS for application sources, remove __ kernel namespace defines
CFLAGS := $(filter-out -D__%, $(ccflags-y))
# remove KERNEL include dir
CFLAGS := $(filter-out -I$(KERNELDIR)%, $(CFLAGS))


#CFLAGS += -DAP22 -fshort-enums
CFLAGS += -Wall
#ECHO = @
LIBS = -lrt

.PHONY: default tags all

OBJECTS = mlanevent.o
HEADERS = mlanevent.h

TARGET = mlanevent.exe

build default: $(TARGET)
	@cp -f $(TARGET) $(INSTALLPATH)

all : tags default

$(TARGET): $(OBJECTS) $(HEADERS)
	$(ECHO)$(CC) $(LIBS) -o $@ $(OBJECTS)

%.o: %.c $(HEADERS)
	$(ECHO)$(CC) $(CFLAGS) -c -o $@ $<

tags:
	ctags -R -f tags.txt

distclean clean:
	$(ECHO)$(RM) $(OBJECTS) $(TARGET)
	$(ECHO)$(RM) tags.txt

