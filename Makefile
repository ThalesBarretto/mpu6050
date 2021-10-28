CC	=gcc
CPPFLAGS=
CFLAGS	=-g -Wall -Wextra
LIBS	=-lm -li2c -lmtx -lmpu6050
PROG	=mpu6050
PROGB	=mock
PROGV	=1

SRC	=src
BLD	=bld
OBJ	=$(BLD)/obj
BIN	=$(BLD)/bin

SRCS	=$(wildcard	$(SRC)/*.c)
HDRS	=$(wildcard	$(SRC)/*.h)
OBJS	=$(patsubst	$(SRC)/%.c,$(OBJ)/%.o, $(SRCS))
PKGS	=$(patsubst	%,%.tar.gz,$(PROGB))

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

all:	$(BIN)/$(PROGB)

# Install/uninstall instructions
INSTALL=install
INSTDIR=/opt
prefix=/usr
INCDIR=$(prefix)/include
BINDIR=$(prefix)/bin
LIBDIR=$(prefix)/lib

install:
	sudo $(INSTALL) -D --owner=root --group=root $(BIN)/$(PROGB) $(INSTDIR)/$(PROG)/$(PROGB)
	sudo ln -sf $(INSTDIR)/$(PROG)/$(PROGB) $(BINDIR)/$(PROGB) 

uninstall:
	sudo rm -rf $(INSTDIR)/$(PROG)
	sudo rm -f $(BINDIR)/$(PROGB)

remove: uninstall

.PHONY: all clean install uninstall
