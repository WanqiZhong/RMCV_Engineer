# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/minivan/Documents/EngineerLaptop

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/minivan/Documents/EngineerLaptop/build

# Include any dependencies generated for this target.
include data/CMakeFiles/data.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include data/CMakeFiles/data.dir/compiler_depend.make

# Include the progress variables for this target.
include data/CMakeFiles/data.dir/progress.make

# Include the compile flags for this target's objects.
include data/CMakeFiles/data.dir/flags.make

data/CMakeFiles/data.dir/src/data.cpp.o: data/CMakeFiles/data.dir/flags.make
data/CMakeFiles/data.dir/src/data.cpp.o: ../data/src/data.cpp
data/CMakeFiles/data.dir/src/data.cpp.o: data/CMakeFiles/data.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/minivan/Documents/EngineerLaptop/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object data/CMakeFiles/data.dir/src/data.cpp.o"
	cd /home/minivan/Documents/EngineerLaptop/build/data && ccache /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT data/CMakeFiles/data.dir/src/data.cpp.o -MF CMakeFiles/data.dir/src/data.cpp.o.d -o CMakeFiles/data.dir/src/data.cpp.o -c /home/minivan/Documents/EngineerLaptop/data/src/data.cpp

data/CMakeFiles/data.dir/src/data.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/data.dir/src/data.cpp.i"
	cd /home/minivan/Documents/EngineerLaptop/build/data && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/minivan/Documents/EngineerLaptop/data/src/data.cpp > CMakeFiles/data.dir/src/data.cpp.i

data/CMakeFiles/data.dir/src/data.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/data.dir/src/data.cpp.s"
	cd /home/minivan/Documents/EngineerLaptop/build/data && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/minivan/Documents/EngineerLaptop/data/src/data.cpp -o CMakeFiles/data.dir/src/data.cpp.s

# Object files for target data
data_OBJECTS = \
"CMakeFiles/data.dir/src/data.cpp.o"

# External object files for target data
data_EXTERNAL_OBJECTS =

data/libdata.a: data/CMakeFiles/data.dir/src/data.cpp.o
data/libdata.a: data/CMakeFiles/data.dir/build.make
data/libdata.a: data/CMakeFiles/data.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/minivan/Documents/EngineerLaptop/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libdata.a"
	cd /home/minivan/Documents/EngineerLaptop/build/data && $(CMAKE_COMMAND) -P CMakeFiles/data.dir/cmake_clean_target.cmake
	cd /home/minivan/Documents/EngineerLaptop/build/data && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/data.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
data/CMakeFiles/data.dir/build: data/libdata.a
.PHONY : data/CMakeFiles/data.dir/build

data/CMakeFiles/data.dir/clean:
	cd /home/minivan/Documents/EngineerLaptop/build/data && $(CMAKE_COMMAND) -P CMakeFiles/data.dir/cmake_clean.cmake
.PHONY : data/CMakeFiles/data.dir/clean

data/CMakeFiles/data.dir/depend:
	cd /home/minivan/Documents/EngineerLaptop/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/minivan/Documents/EngineerLaptop /home/minivan/Documents/EngineerLaptop/data /home/minivan/Documents/EngineerLaptop/build /home/minivan/Documents/EngineerLaptop/build/data /home/minivan/Documents/EngineerLaptop/build/data/CMakeFiles/data.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : data/CMakeFiles/data.dir/depend

