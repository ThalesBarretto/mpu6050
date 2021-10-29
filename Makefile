PROG	=mpu6050
PROGB	=$(PROG)-demo
PROGV	=1

CC	=gcc
CPPFLAGS=
CFLAGS	=-DNDEBUG -O2 -march=native -mtune=native -fPIC -Wall -Wextra -Wpedantic
LIBS	=-lm -li2c -lmtx -lmpu6050

SRC	=src
BLD	=bld
OBJ	=$(BLD)/obj
BIN	=$(BLD)/bin

MOD	=mod
MODULES	=$(wildcard	$(MOD)/*)
SRCS	=$(wildcard	$(SRC)/*.c)
HDRS	=$(wildcard	$(SRC)/*.h)
OBJS	=$(patsubst	$(SRC)/%.c,$(OBJ)/%.o, $(SRCS))
PKGS	=$(patsubst	%,%.tar.gz,$(PROGB))

modules:
	-for i in mod/*; do cd $$i && make all && cd ../../; done

modules_install:
	-for i in mod/*; do cd $$i && make install && cd ../../; done

modules_uninstall:
	-for i in mod/*; do cd $$i && make uninstall && cd ../../; done

modules_clean:
	-for i in mod/*; do cd $$i && make clean && cd ../../; done

$(OBJ):
	mkdir -p $@

$(BIN):
	mkdir -p $@

$(OBJ)/%.o: $(SRC)/%.c | $(OBJ)
	$(CC) $(WFLAGS) $(CFLAGS) $(LIBS) -c $< -o $@


$(BIN)/$(PROGB): $(OBJS) | $(BIN) 
	$(CC) $(WFLAGS) $(CFLAGS) $(LIBS) -o $@ $(OBJS) $(LDFLAGS)

release: CFLAGS=-O2 -DNDEBUG
release: WFLAGS=-Wall
release: clean
release: $(BIN)/$(PROGB)

clean:
	$(RM) -rf $(BLD)

dist: release 
	tar -czvf $@ $(BIN)/$(PROGB)

all:	modules modules_install $(BIN)/$(PROGB) 

# Install/uninstall instructions
INSTALL=install
INSTDIR=/opt
prefix=/usr
INCDIR=$(prefix)/include
BINDIR=$(prefix)/bin
LIBDIR=$(prefix)/lib


install:
	-sudo $(INSTALL) -D --owner=root --group=root $(BIN)/$(PROGB) $(INSTDIR)/$(PROG)/$(PROGB)
	-sudo ln -sf $(INSTDIR)/$(PROG)/$(PROGB) $(BINDIR)/$(PROGB) 

uninstall:
	-sudo rm -rf $(INSTDIR)/$(PROG)
	-sudo rm -f $(BINDIR)/$(PROGB)

remove: uninstall

remove_modules: modules_uninstall

remove_all: remove remove_modules 

clean_modules: modules_clean

clean_all: modules_clean clean

.PHONY: all clean install uninstall modules modules_clean modules_install modules_uninstall 
