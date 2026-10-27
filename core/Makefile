# Compila o núcleo para PC (SIL). Em Windows/MSYS2 MINGW64:  make   -> libcubecore.dll
CC      ?= gcc
CFLAGS  ?= -O2 -std=c99 -Wall -Wextra -Wno-unused-parameter -fPIC -Iinclude
SRCS     = src/target_detect.c src/cube_core.c src/cc_proto.c src/moteus_proto.c
UNAME_S := $(shell uname -s 2>/dev/null)
ifneq (,$(findstring MINGW,$(UNAME_S)))
  LIB = libcubecore.dll
else ifneq (,$(findstring MSYS,$(UNAME_S)))
  LIB = libcubecore.dll
else
  LIB = libcubecore.so
endif

ifneq (,$(findstring dll,$(LIB)))
  LDEXTRA = -static-libgcc
endif

all: $(LIB)
$(LIB): $(SRCS) include/*.h
	$(CC) $(CFLAGS) -shared -o $@ $(SRCS) -lm $(LDEXTRA)
clean:
	rm -f libcubecore.so libcubecore.dll
