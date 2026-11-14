# Compila o núcleo para PC (SIL). Em Windows/MSYS2 MINGW64:  make   -> libcubecore.dll
CC      ?= gcc
CPPFLAGS += -Iinclude
CFLAGS  ?= -O2 -std=c99 -Wall -Wextra -Wpedantic
SRCS     = src/target_detect.c src/cube_core.c src/cc_proto.c src/moteus_proto.c src/cube_core3.c src/cc_proto3.c
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
test_imu$(EXE): tests/test_imu.c $(SRCS) $(wildcard include/*.h)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ tests/test_imu.c $(SRCS) -lm $(LDEXTRA) $(LDLIBS)
test_review$(EXE): tests/test_review.c $(SRCS) $(wildcard include/*.h)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ tests/test_review.c $(SRCS) -lm $(LDEXTRA) $(LDLIBS)
test_core3$(EXE): tests/test_core3.c $(SRCS) $(wildcard include/*.h)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ tests/test_core3.c $(SRCS) -lm $(LDEXTRA) $(LDLIBS)
test_core3_more$(EXE): tests/test_core3_more.c $(SRCS) $(wildcard include/*.h)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ tests/test_core3_more.c $(SRCS) -lm $(LDEXTRA) $(LDLIBS)
test: test_core3$(EXE) test_core3_more$(EXE) test_core$(EXE) test_imu$(EXE) test_review$(EXE)
	./test_core3$(EXE)
	./test_core3_more$(EXE)
	./test_core$(EXE)
	./test_imu$(EXE)
	./test_review$(EXE)
clean:
	$(RM) libcubecore.so libcubecore.dll test_core test_core.exe test_imu test_imu.exe test_review test_review.exe test_core3 test_core3.exe test_core3_more test_core3_more.exe
