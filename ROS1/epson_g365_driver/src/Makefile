CC=gcc

# If no MODEL macro is defined when calling make it defaults to G354
MODEL ?= G354
DEFINES= $(MODEL)
INCLUDES= .
CFLAGS= -I$(INCLUDES) -D$(DEFINES) -std=gnu99
LIBS=
DEPS= hcl.h hcl_gpio.h sensor_epsonCommon.h main_helper.h
DEPS_IMUMODEL= sensor_epsonG354.h sensor_epsonG34DC0.h sensor_epsonG34DCA.h sensor_epsonV340.h sensor_epsonG320.h sensor_epsonG365DC0.h sensor_epsonG365DF0.h sensor_epsonG370DC0.h sensor_epsonG370DF0.h sensor_epsonG325PDF0.h
DEPS_UART= hcl_uart.h

_OBJ= hcl_gpio.o hcl_linux.o hcl_uart.o sensor_epsonCommon.o sensor_epsonUart.o main_helper.o

ifeq ($(MODEL), G354)
	_OBJ+= sensor_epsonG354.o
endif

ifeq ($(MODEL), G364PDC0)
	_OBJ+= sensor_epsonG364.o
endif

ifeq ($(MODEL), G364PDCA)
	_OBJ+= sensor_epsonG364.o
endif

ifeq ($(MODEL), V340)
	_OBJ+= sensor_epsonV340.o
endif

ifeq ($(MODEL), G320)
	_OBJ+= sensor_epsonG320.o
endif

ifeq ($(MODEL), G365PDC0)
	_OBJ+= sensor_epsonG365.o
endif

ifeq ($(MODEL), G365PDF0)
	_OBJ+= sensor_epsonG365.o
endif

ifeq ($(MODEL), G370PDC0)
	_OBJ+= sensor_epsonG370.o
endif

ifeq ($(MODEL), G370PDF0)
	_OBJ+= sensor_epsonG370.o
endif
ifeq ($(MODEL), G325PDF0)
	_OBJ+= sensor_epsonG325.o
endif
OBJ_SCREEN= main_screen.o $(_OBJ)
OBJ_CSV= main_csvlogger.o $(_OBJ)
OBJ_REG= main_regdump.o $(_OBJ)

%.o: %.c $(DEPS) $(DEPS_UART) $(DEPS_IMUMODEL)
	$(CC) -c -o $@ $< $(CFLAGS) $(LIBS)

screen: $(OBJ_SCREEN)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

csvlogger: $(OBJ_CSV)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

regdump: $(OBJ_REG)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

.PHONY: clean all tar help

clean:
	rm -f *.o *~ core *~
	-rm -f csvlogger screen regdump

all: screen csvlogger regdump

tar:
	-tar cvzf archive.tgz *.c *.h readme.txt Makefile

help:
	@echo "supported make commands are:"
	@echo "\tmake clean"
	@echo "\tmake <targets> MODEL=<model>\n"
	@echo "valid <targets> are: csvlogger screen or regdump\n"
	@echo "valid <models> are:"
	@echo "\tG354(default) G364PDC0 G364PDCA G320 V340 G365PDC0 G365PDF0 G370PDC0 G370PDF0 G325PDF0\n"
	@echo "example:\n\tmake csvlogger MODEL=G364PDC0"
	@echo "\tmake screen (defaults to MODEL=G354)"
