CC := clang
MACOSX_DEPLOYMENT_TARGET ?= 14.0
export MACOSX_DEPLOYMENT_TARGET

CFLAGS  := -std=c11 -O2 -I. -Wall -Wextra
LDFLAGS := $(shell pkg-config --libs raylib)

# Define your targets here
TARGETS := crowd hpa-star

# Source files for each target
crowd_SRC     := crowd-steering/crowd-raylib-betterstats5.c
hpa-star_SRC  := hpa-star-tests/test7.c hpa-star-tests/terrain.c hpa-star-tests/pathfinding.c
all: $(TARGETS)

# Pattern rule: build any target from its corresponding _SRC
$(TARGETS):
	$(CC) $(CFLAGS) -o $@ $($@_SRC) $(LDFLAGS)



clean:
	rm -f $(TARGETS)

.PHONY: all clean run-crowd run-hpa $(TARGETS)
