#
# Standard Makefile for a Webots controller
#
# Author: Olivier.Michel@cyberbotics.com
# Date:   August 29th, 2006.
#

ifndef WEBOTS_HOME_PATH
ifeq ($(OS),Windows_NT)
nullstring :=
space := $(nullstring) # a string containing a single space
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
else
WEBOTS_HOME_PATH=$(WEBOTS_HOME)
endif
endif

include $(WEBOTS_HOME_PATH)/projects/default/controllers/Makefile.include
