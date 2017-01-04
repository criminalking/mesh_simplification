#OPENCV_LD_FLAGS := $(shell pkg-config --libs opencv)
#OPENCV_C_FLAGS := $(shell pkg-config --cflags opencv)

GXX := clang++
BIN_NAME := main.out
CPP_FILES := $(wildcard src/*.cpp)

build:
	@$(GXX) $(CPP_FILES) -o $(BIN_NAME) -O2

run:
	@./$(BIN_NAME)
