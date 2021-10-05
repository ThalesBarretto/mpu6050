CC	=gcc
CFLAGS	=-g -std=gnu17 -O0 -Wall

SRC	=src
BLD	=bld
OBJ	=$(BLD)/obj
BIN	=$(BLD)/bin


LIBS	=-lm -li2c
SRCS	=$(wildcard	$(SRC)/*.c)
OBJS	=$(patsubst	$(SRC)/%.c,$(OBJ)/%.o, $(SRCS))
PKGS	=$(patsubst	$(BINS),$(BINS).tar.gz,$(BINS))
BINS	=mock

release: CFLAGS=-Wall -O2 -DNDEBUG
release: clean
release: $(BIN)/$(BINS)

$(OBJ):
	mkdir -p $@

$(BIN):
	mkdir -p $@

$(OBJ)/%.o: $(SRC)/%.c $(OBJ)
	$(CC) $(CFLAGS) $(LIBS) -c $< -o $@

$(BIN)/$(BINS): $(OBJS) $(BIN)
	$(CC) $(CFLAGS) $(LIBS) -o $@ $(OBJS) $(LDFLAGS)

$(BLD)/$(PKGS): release 
	tar -czvf $@ $(BIN)/$(BINS)

all:	$(BIN)/$(BINS) 

clean:
	$(RM) -r $(BIN) $(OBJ)

dist:	$(BLD)/$(PKGS)

distclean:
	$(RM) $(BLD)/$(PKGS)

.PHONY: all clean dist distclean
