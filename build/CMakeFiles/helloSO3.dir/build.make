# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
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
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/niu/Documents/code/NumberSystems

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/niu/Documents/code/NumberSystems/build

# Include any dependencies generated for this target.
include CMakeFiles/helloSO3.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/helloSO3.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/helloSO3.dir/flags.make

CMakeFiles/helloSO3.dir/helloSO3.cpp.o: CMakeFiles/helloSO3.dir/flags.make
CMakeFiles/helloSO3.dir/helloSO3.cpp.o: ../helloSO3.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/niu/Documents/code/NumberSystems/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/helloSO3.dir/helloSO3.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/helloSO3.dir/helloSO3.cpp.o -c /home/niu/Documents/code/NumberSystems/helloSO3.cpp

CMakeFiles/helloSO3.dir/helloSO3.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/helloSO3.dir/helloSO3.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/niu/Documents/code/NumberSystems/helloSO3.cpp > CMakeFiles/helloSO3.dir/helloSO3.cpp.i

CMakeFiles/helloSO3.dir/helloSO3.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/helloSO3.dir/helloSO3.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/niu/Documents/code/NumberSystems/helloSO3.cpp -o CMakeFiles/helloSO3.dir/helloSO3.cpp.s

CMakeFiles/helloSO3.dir/helloSO3.cpp.o.requires:

.PHONY : CMakeFiles/helloSO3.dir/helloSO3.cpp.o.requires

CMakeFiles/helloSO3.dir/helloSO3.cpp.o.provides: CMakeFiles/helloSO3.dir/helloSO3.cpp.o.requires
	$(MAKE) -f CMakeFiles/helloSO3.dir/build.make CMakeFiles/helloSO3.dir/helloSO3.cpp.o.provides.build
.PHONY : CMakeFiles/helloSO3.dir/helloSO3.cpp.o.provides

CMakeFiles/helloSO3.dir/helloSO3.cpp.o.provides.build: CMakeFiles/helloSO3.dir/helloSO3.cpp.o


# Object files for target helloSO3
helloSO3_OBJECTS = \
"CMakeFiles/helloSO3.dir/helloSO3.cpp.o"

# External object files for target helloSO3
helloSO3_EXTERNAL_OBJECTS =

helloSO3: CMakeFiles/helloSO3.dir/helloSO3.cpp.o
helloSO3: CMakeFiles/helloSO3.dir/build.make
helloSO3: CMakeFiles/helloSO3.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/niu/Documents/code/NumberSystems/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable helloSO3"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/helloSO3.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/helloSO3.dir/build: helloSO3

.PHONY : CMakeFiles/helloSO3.dir/build

CMakeFiles/helloSO3.dir/requires: CMakeFiles/helloSO3.dir/helloSO3.cpp.o.requires

.PHONY : CMakeFiles/helloSO3.dir/requires

CMakeFiles/helloSO3.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/helloSO3.dir/cmake_clean.cmake
.PHONY : CMakeFiles/helloSO3.dir/clean

CMakeFiles/helloSO3.dir/depend:
	cd /home/niu/Documents/code/NumberSystems/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/niu/Documents/code/NumberSystems /home/niu/Documents/code/NumberSystems /home/niu/Documents/code/NumberSystems/build /home/niu/Documents/code/NumberSystems/build /home/niu/Documents/code/NumberSystems/build/CMakeFiles/helloSO3.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/helloSO3.dir/depend

