#COMPILE="g++ -I./imgui -I./imgui/backends -g -O3 -Wall -Werror *.cpp ./imgui/*.cpp ./imgui/backends/*.cpp -lSDL2 -lSDL2_image -lSDL2_ttf -ldl -o a"
#echo ${COMPILE}
#eval $COMPILE
##./a


BINARY=bin
CODEDIRS=. ./imgui ./imgui/backends
INCDIRS=. ./imgui ./imgui/backends
LIBS = -lSDL2 -lSDL2_image -lSDL2_ttf -ldl
#`sdl2-config --libs`

CC=g++
OPT=-O3
# generate files that encode make rules for the .h dependencies
#DEPFLAGS=-MP -MD
# automatically add the -I onto each include directory
CFLAGS=-Wall -Werror -g $(foreach D,$(INCDIRS),-I$(D)) $(OPT) $(DEPFLAGS)

# for-style iteration (foreach) and regular expression completions (wildcard)
CFILES=$(foreach D,$(CODEDIRS),$(wildcard $(D)/*.cpp))
# regular expression replacement
OBJECTS=$(patsubst %.cpp,%.o,$(CFILES))
DEPFILES=$(patsubst %.cpp,%.d,$(CFILES))

all: $(BINARY)

$(BINARY): $(OBJECTS)
	$(CC) $(CFLAGS) -o $@ $^ $(LIBS)

# only want the .c file dependency here, thus $< instead of $^.
#
%.o:%.cpp
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -rf $(BINARY) $(OBJECTS) $(DEPFILES)
	echo done cleaning
	echo hi hi hi
# shell commands are a set of keystrokes away
distribute: clean
	tar zcvf dist.tgz *

# @ silences the printing of the command
# $(info ...) prints output
#diff:
#	$(info The status of the repository, and the volume of per-file changes:)
#	@git status
#	@git diff --stat

# include the dependencies
-include $(DEPFILES)

# add .PHONY so that the non-targetfile - rules work even if a file with the same name exists.
.PHONY: all clean distribute diff
