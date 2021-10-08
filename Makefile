CC	=gcc
CFLAGS	=-Wall -g -std=gnu17 -Og 

SRC	=src
BLD	=bld
OBJ	=$(BLD)/obj
BIN	=$(BLD)/bin
PLAY	="/home/pi/play/mpu6050"

LIBS	=-lm -li2c
SRCS	=$(wildcard	$(SRC)/*.c)
OBJS	=$(patsubst	$(SRC)/%.c,$(OBJ)/%.o, $(SRCS))
PKGS	=$(patsubst	$(BINS),$(BINS).tar.gz,$(BINS))
BINS	=mock

release: CFLAGS=-Wall -O2 -DNDEBUG
release: clean
release: $(BIN)/$(BINS)

crazy: CFLAGS=-Wall -Ofast -ftree-vectorize -march=native -mtune=native -DNDEBUG -DHAVE_INLINE -flto
crazy: clean
crazy: $(BIN)/$(BINS)

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

tags:
	cd $(SRC) && ctags -R 

distclean:
	$(RM) $(BLD)/$(PKGS)

pi:
	scp -r $(SRC) pi:$(PLAY)
	scp  Makefile pi:$(PLAY)
	ssh  pi 'cd $(PLAY) && $(MAKE) dist'

.PHONY: all clean dist distclean pi
