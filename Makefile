CC	=gcc
CFLAGS	=-std=gnu11 -O2 -Wall

SRC	=src
OBJ	=obj
BIN	=bin
TEST	=$(SRC)/tests

LIBS	= -lm -li2c
SRCS	=$(wildcard	$(SRC)/*.c)
OBJS	=$(patsubst	$(SRC)/*.c, $(OBJ)/*.o, $(SRCS))
TESTS	=$(wildcard	$(TEST)/*.c)
TESTO	=$(patsubst	$(TEST)/*.c, $(OBJ)/*.o, $(TESTS))
TESTB	=test_mtx
BINS	=mock

$(OBJ):
	mkdir -p $@

$(BIN):
	mkdir -p $@

$(OBJ)/%.o: $(SRC)/%.c $(TESTS)/%.c $(OBJS) $(TESTO)
	$(CC) $(CFLAGS) $(LIBS) -c $< -o $@

$(BIN)/$(BINS): $(OBJS) $(OBJ) $(BIN)
	$(CC) $(CFLAGS) $(LIBS) $(OBJS) -o $@ $(LDFLAGS)

all:	$(BIN)/$(BINS) 

clean:
	$(RM) -r $(BIN) $(OBJ) 

.PHONY: all clean
