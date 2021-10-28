CC	=gcc
CPPFLAGS=
CFLAGS	=-g -Wall -Wextra
LIBS	=-lm -li2c -lmtx -lmpu6050
PROGS	=mock

SRC	=src
BLD	=bld
OBJ	=$(BLD)/obj
BIN	=$(BLD)/bin

SRCS	=$(wildcard	$(SRC)/*.c)
HDRS	=$(wildcard	$(SRC)/*.h)
OBJS	=$(patsubst	$(SRC)/%.c,$(OBJ)/%.o, $(SRCS))
PKGS	=$(patsubst	%,%.tar.gz,$(PROGS))

$(OBJ):
	mkdir -p $@

$(BIN):
	mkdir -p $@

$(OBJ)/%.o: $(SRC)/%.c | $(OBJ)
	$(CC) $(WFLAGS) $(CFLAGS) $(LIBS) -c $< -o $@


$(BIN)/$(PROGS): $(OBJS) | $(BIN)
	$(CC) $(WFLAGS) $(CFLAGS) $(LIBS) -o $@ $(OBJS) $(LDFLAGS)

release: CFLAGS=-O2 -DNDEBUG
release: WFLAGS=-Wall
release: clean
release: $(BIN)/$(PROGS)

clean:
	$(RM) -rf $(BLD)

dist: release 
	tar -czvf $@ $(BIN)/$(PROGS)

all:	$(BIN)/$(PROGS)

.PHONY: all clean dist pi
