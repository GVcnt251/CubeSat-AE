# Standalone 3-axis control; the single-axis controller is not a build input.
CC ?= gcc
PYTHON ?= python
CPPFLAGS += -Iinclude
CFLAGS ?= -O2 -std=c99 -Wall -Wextra -Wpedantic -Werror
SRCS = src/cube_core3.c src/target_detect.c src/cc_proto.c src/cc_proto3.c src/moteus_proto.c
HEADERS = $(wildcard include/*.h)
ifeq ($(OS),Windows_NT)
  LIB = libcubecore3.dll
  EXE = .exe
  LDEXTRA = -static-libgcc
else
  LIB = libcubecore3.so
  CFLAGS += -fPIC
endif
TESTS = test_common$(EXE) test_core3$(EXE) test_core3_more$(EXE)
.PHONY: all test test-sil clean
all: $(LIB)
$(LIB): $(SRCS) $(HEADERS)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -shared -o $@ $(SRCS) -lm $(LDEXTRA) $(LDLIBS)
$(TESTS): %$(EXE): tests/%.c $(SRCS) $(HEADERS)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ $< $(SRCS) -lm $(LDEXTRA) $(LDLIBS)
test: $(LIB) $(TESTS)
	./test_common$(EXE)
	./test_core3$(EXE)
	./test_core3_more$(EXE)
	$(PYTHON) tests/test_standalone.py
test-sil: $(LIB)
	$(PYTHON) tests/test_core3_sil.py
	$(PYTHON) tests/test_proto3_python.py
clean:
	$(RM) libcubecore3.dll libcubecore3.so libcubecore3_build.dll libcubecore3_build.so test_common test_common.exe test_core3 test_core3.exe test_core3_more test_core3_more.exe
