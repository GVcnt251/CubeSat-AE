# Compila o núcleo para PC (SIL). Em Windows/MSYS2 MINGW64:  make   -> libcubecore.dll
CC      ?= gcc
CPPFLAGS += -Iinclude
CFLAGS  ?= -O2 -std=c99 -Wall -Wextra -Wpedantic
SRCS     = src/target_detect.c src/cube_core.c src/cc_proto.c src/moteus_proto.c
UNAME_S := $(shell uname -s 2>/dev/null)
ifeq ($(OS),Windows_NT)
  LIB = libcubecore.dll
else ifneq (,$(findstring MINGW,$(UNAME_S)))
  LIB = libcubecore.dll
else ifneq (,$(findstring MSYS,$(UNAME_S)))
  LIB = libcubecore.dll
else
  LIB = libcubecore.so
endif

ifneq (,$(findstring dll,$(LIB)))
  LDEXTRA = -static-libgcc
  EXE = .exe
else
  CFLAGS += -fPIC
endif

.PHONY: all test clean
all: $(LIB)
$(LIB): $(SRCS) $(wildcard include/*.h)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -shared -o $@ $(SRCS) -lm $(LDEXTRA) $(LDLIBS)
test_core$(EXE): tests/test_core.c $(SRCS) $(wildcard include/*.h)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ tests/test_core.c $(SRCS) -lm $(LDEXTRA) $(LDLIBS)
test: test_core$(EXE)
	./test_core$(EXE)
clean:
	$(RM) libcubecore.so libcubecore.dll test_core test_core.exe
