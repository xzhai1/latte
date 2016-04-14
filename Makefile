# This Makefile's skeleton comes from Prof. Tia Newhall from Swarthmore
# College. My alma mater keeps taking care of me :)

#
# 'make depend' uses makedepend to automatically generate dependencies 
#               (dependencies are added to end of Makefile)
# 'make'        build executable file 'mycc'
# 'make clean'  removes all .o and executable files
#

# define the C compiler to use
CC = g++

# define any compile-time flags
CFLAGS  = --std=c++11 -g -fopenmp -Wall

# define any directories containing header files other than /usr/include
INCLUDES = -I ./include -I ../halide/include -I ../halide/tools

# define library paths in addition to /usr/lib
#   if I wanted to include libraries not in /usr/lib I'd specify
#   their path using -Lpath, something like:
LFLAGS = -L ../halide/bin

# define any libraries to link into executable:
#   if I want to link in libraries (libx.so or libx.a) I use the -llibname 
#   option, something like (this will link in libmylib.so and libm.so:
LIBS = -l Halide -l dl

# From Wikipedia:
# 	"pkg-config is computer program that provides a unified interface for 
# 	 querying installed libraries for the purpose of compiling software from 
# 	 its source code. It allows programmers and installation scripts to work 
# 	 without explicit knowledge of detailed library path information."
# For example, the following command produces:
# 	-pthread -I/usr/local/include  -pthread -L/usr/local/lib -lprotobuf
# 	-lpthread	
EXTRA_SCRIPTS = `pkg-config --cflags --libs protobuf libpng`
 
# define the C source files
SRCS = $(wildcard ./src/*.cc) $(wildcard ./tests/*.cc) 

# define the C object files 
#
# This uses Suffix Replacement within a macro:
#   $(name:string1=string2)
#         For each word in 'name' replace 'string1' with 'string2'
# Below we are replacing the suffix .c of all words in the macro SRCS
# with the .o suffix
#
OBJS = $(SRCS:.cc=.o)

# define the executable file 
MAIN = run_test

#
# The following part of the makefile is generic; it can be used to 
# build any executable just by changing the definitions above and by
# deleting dependencies appended to the file from 'make depend'
#

.PHONY: depend clean

all: $(MAIN)
	@echo Project is compiled! 

$(MAIN): $(OBJS) 
	$(CC) $(CFLAGS) $(INCLUDES) -o $(MAIN) $(OBJS) $(LFLAGS) $(LIBS) $(EXTRA_SCRIPTS)

# this is a suffix replacement rule for building .o's from .c's
# it uses automatic variables $<: the name of the prerequisite of
# the rule(a .c file) and $@: the name of the target of the rule (a .o file) 
# (see the gnu make manual section about automatic variables)
.cc.o:
	$(CC) $(CFLAGS) $(INCLUDES) -c $<  -o $@

print-%:
	@echo $* = $($*)

#clean:
#	$(RM) ./src/*.o  ./tests/*.o *~ 

clean:
	$(RM) $(OBJS) *~ 

depend: $(SRCS)
	makedepend $(INCLUDES) $^

# DO NOT DELETE THIS LINE -- make depend needs it
