#
# File:
#    Makefile
#
# Description:
#    Makefile Michael Kohl's program to take MPD data.
#
#
DEBUG ?= 1

LINUXVME_LIB	?= ${CODA}/extensions/linuxvme/libs
LINUXVME_INC	?= ${CODA}/extensions/linuxvme/include

CROSS_COMPILE		=
CC			= $(CROSS_COMPILE)gcc
AR                      = ar
RANLIB                  = ranlib
CFLAGS			= -I. -I${LINUXVME_INC} -I./libconfig \
			  -L. -L${LINUXVME_LIB} -L./libconfig
ifeq ($(DEBUG),1)
CFLAGS			+= -Wall -g
endif

PROGS			= DAQevent_Noinhibit_MK

all: $(PROGS)

install: all

clean distclean:
	@rm -f $(PROGS) *~ *.so

%: %.c
	echo "Making $@"
	$(CC) $(CFLAGS) -o $@ $(@:%=%.c) -I../ -L../ -lrt -ljvme -lmpd -lconfig

.PHONY: all clean distclean
