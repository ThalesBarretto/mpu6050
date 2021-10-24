CC	=gcc
CFLAGS	=-g -fPIC
WFLAGS	=-Wall \
	 -Wextra \
	 -Wpedantic \
	 -Wsuggest-attribute=pure \
	 -Wsuggest-attribute=const \
	 -Wsuggest-attribute=noreturn \
	 -Wsuggest-attribute=cold \
	 -Wsuggest-attribute=malloc \
	 -Wsuggest-attribute=format \
	 -Wmissing-noreturn \
	 -Wmissing-format-attribute \
	 -Wtrampolines \
	 -Wno-system-headers \
	 -Wshadow \
	 -Wundef \
	 -Wexpansion-to-defined \
	 -Wunused-macros \
	 -Wbad-function-cast \
	 -Wcast-align \
	 -Wdangling-else \
	 -Wlogical-op \
	 -Wsign-compare \
	 -Wsign-conversion\
	 -Wfloat-conversion \
	 -Wmisleading-indentation \
	 -Wsequence-point \
	 -Wmissing-braces \
	 -Wlogical-not-parentheses 

SRC	=src
BLD	=bld
OBJ	=$(BLD)/obj
BIN	=$(BLD)/bin
PLAY	="/home/pi/play/mpu6050"

LIBS	=-lm -li2c
SRCS	=$(wildcard	$(SRC)/*.c)
HDRS	=$(wildcard	$(SRC)/*.h)
OBJS	=$(patsubst	$(SRC)/%.c,$(OBJ)/%.o, $(SRCS))
PKGS	=$(patsubst	$(BINS),$(BINS).tar.gz,$(BINS))
BINS	=mock


release: CFLAGS=-O2 -DNDEBUG
release: WFLAGS=-Wall
release: clean
release: $(BIN)/$(BINS)

crazy: CFLAGS=-Ofast -ftree-vectorize -march=native -mtune=native -DNDEBUG -DHAVE_INLINE -flto
crazy: clean
crazy: $(BIN)/$(BINS)

$(OBJ):
	mkdir -p $@

$(BIN):
	mkdir -p $@

$(OBJ)/%.o: $(SRC)/%.c | $(OBJ)
	$(CC) $(WFLAGS) $(CFLAGS) $(LIBS) -c $< -o $@

$(BIN)/$(BINS): $(OBJS) | $(BIN)
	$(CC) $(WFLAGS) $(CFLAGS) $(LIBS) -o $@ $(OBJS) $(LDFLAGS)

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
	ssh  pi 'cd $(PLAY) && $(MAKE) release'

.PHONY: all clean dist distclean pi fast
