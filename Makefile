# This is a pretty ghetto Makefile

CC=g++
CFLAGS=--std=c++11 -g -fopenmp -Wall
INC=-I ./include \
		-I ./tools
EXTRA_SCRIPTS=`libpng-config --cflags --ldflags`
LDFLAGS=-L ./bin -l Halide $(EXTRA_SCRIPTS) \
				-lpthread \
				-ldl
EXECUTABLE=test
test: ./src/test.cpp
	$(CC) ./src/test.cpp $(CFLAGS) $(INC) $(LDFLAGS) -o test

print-%:
	@echo $* = $($*)

