CC := clang

TARGET := crowd
SRC := crowd-raylib-betterstats5.c

# You can override this when calling make, e.g.:
#   make MACOSX_DEPLOYMENT_TARGET=14.0
MACOSX_DEPLOYMENT_TARGET ?= 14.0
export MACOSX_DEPLOYMENT_TARGET

CFLAGS  := -std=c11 -O2 -Wall -Wextra $(shell pkg-config --cflags raylib)
LDFLAGS := $(shell pkg-config --libs raylib)

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET)

.PHONY: all run clean
