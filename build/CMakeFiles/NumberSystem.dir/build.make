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
include CMakeFiles/NumberSystem.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/NumberSystem.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/NumberSystem.dir/flags.make

CMakeFiles/NumberSystem.dir/src/Bezier.cpp.o: CMakeFiles/NumberSystem.dir/flags.make
CMakeFiles/NumberSystem.dir/src/Bezier.cpp.o: ../src/Bezier.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/niu/Documents/code/NumberSystems/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/NumberSystem.dir/src/Bezier.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/NumberSystem.dir/src/Bezier.cpp.o -c /home/niu/Documents/code/NumberSystems/src/Bezier.cpp

CMakeFiles/NumberSystem.dir/src/Bezier.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/NumberSystem.dir/src/Bezier.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/niu/Documents/code/NumberSystems/src/Bezier.cpp > CMakeFiles/NumberSystem.dir/src/Bezier.cpp.i

CMakeFiles/NumberSystem.dir/src/Bezier.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/NumberSystem.dir/src/Bezier.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/niu/Documents/code/NumberSystems/src/Bezier.cpp -o CMakeFiles/NumberSystem.dir/src/Bezier.cpp.s

CMakeFiles/NumberSystem.dir/src/Bezier.cpp.o.requires:

.PHONY : CMakeFiles/NumberSystem.dir/src/Bezier.cpp.o.requires

CMakeFiles/NumberSystem.dir/src/Bezier.cpp.o.provides: CMakeFiles/NumberSystem.dir/src/Bezier.cpp.o.requires
	$(MAKE) -f CMakeFiles/NumberSystem.dir/build.make CMakeFiles/NumberSystem.dir/src/Bezier.cpp.o.provides.build
.PHONY : CMakeFiles/NumberSystem.dir/src/Bezier.cpp.o.provides

CMakeFiles/NumberSystem.dir/src/Bezier.cpp.o.provides.build: CMakeFiles/NumberSystem.dir/src/Bezier.cpp.o


CMakeFiles/NumberSystem.dir/src/Complex.cpp.o: CMakeFiles/NumberSystem.dir/flags.make
CMakeFiles/NumberSystem.dir/src/Complex.cpp.o: ../src/Complex.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/niu/Documents/code/NumberSystems/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/NumberSystem.dir/src/Complex.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/NumberSystem.dir/src/Complex.cpp.o -c /home/niu/Documents/code/NumberSystems/src/Complex.cpp

CMakeFiles/NumberSystem.dir/src/Complex.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/NumberSystem.dir/src/Complex.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/niu/Documents/code/NumberSystems/src/Complex.cpp > CMakeFiles/NumberSystem.dir/src/Complex.cpp.i

CMakeFiles/NumberSystem.dir/src/Complex.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/NumberSystem.dir/src/Complex.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/niu/Documents/code/NumberSystems/src/Complex.cpp -o CMakeFiles/NumberSystem.dir/src/Complex.cpp.s

CMakeFiles/NumberSystem.dir/src/Complex.cpp.o.requires:

.PHONY : CMakeFiles/NumberSystem.dir/src/Complex.cpp.o.requires

CMakeFiles/NumberSystem.dir/src/Complex.cpp.o.provides: CMakeFiles/NumberSystem.dir/src/Complex.cpp.o.requires
	$(MAKE) -f CMakeFiles/NumberSystem.dir/build.make CMakeFiles/NumberSystem.dir/src/Complex.cpp.o.provides.build
.PHONY : CMakeFiles/NumberSystem.dir/src/Complex.cpp.o.provides

CMakeFiles/NumberSystem.dir/src/Complex.cpp.o.provides.build: CMakeFiles/NumberSystem.dir/src/Complex.cpp.o


CMakeFiles/NumberSystem.dir/src/Octonion.cpp.o: CMakeFiles/NumberSystem.dir/flags.make
CMakeFiles/NumberSystem.dir/src/Octonion.cpp.o: ../src/Octonion.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/niu/Documents/code/NumberSystems/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/NumberSystem.dir/src/Octonion.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/NumberSystem.dir/src/Octonion.cpp.o -c /home/niu/Documents/code/NumberSystems/src/Octonion.cpp

CMakeFiles/NumberSystem.dir/src/Octonion.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/NumberSystem.dir/src/Octonion.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/niu/Documents/code/NumberSystems/src/Octonion.cpp > CMakeFiles/NumberSystem.dir/src/Octonion.cpp.i

CMakeFiles/NumberSystem.dir/src/Octonion.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/NumberSystem.dir/src/Octonion.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/niu/Documents/code/NumberSystems/src/Octonion.cpp -o CMakeFiles/NumberSystem.dir/src/Octonion.cpp.s

CMakeFiles/NumberSystem.dir/src/Octonion.cpp.o.requires:

.PHONY : CMakeFiles/NumberSystem.dir/src/Octonion.cpp.o.requires

CMakeFiles/NumberSystem.dir/src/Octonion.cpp.o.provides: CMakeFiles/NumberSystem.dir/src/Octonion.cpp.o.requires
	$(MAKE) -f CMakeFiles/NumberSystem.dir/build.make CMakeFiles/NumberSystem.dir/src/Octonion.cpp.o.provides.build
.PHONY : CMakeFiles/NumberSystem.dir/src/Octonion.cpp.o.provides

CMakeFiles/NumberSystem.dir/src/Octonion.cpp.o.provides.build: CMakeFiles/NumberSystem.dir/src/Octonion.cpp.o


CMakeFiles/NumberSystem.dir/src/Quaternion.cpp.o: CMakeFiles/NumberSystem.dir/flags.make
CMakeFiles/NumberSystem.dir/src/Quaternion.cpp.o: ../src/Quaternion.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/niu/Documents/code/NumberSystems/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/NumberSystem.dir/src/Quaternion.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/NumberSystem.dir/src/Quaternion.cpp.o -c /home/niu/Documents/code/NumberSystems/src/Quaternion.cpp

CMakeFiles/NumberSystem.dir/src/Quaternion.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/NumberSystem.dir/src/Quaternion.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/niu/Documents/code/NumberSystems/src/Quaternion.cpp > CMakeFiles/NumberSystem.dir/src/Quaternion.cpp.i

CMakeFiles/NumberSystem.dir/src/Quaternion.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/NumberSystem.dir/src/Quaternion.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/niu/Documents/code/NumberSystems/src/Quaternion.cpp -o CMakeFiles/NumberSystem.dir/src/Quaternion.cpp.s

CMakeFiles/NumberSystem.dir/src/Quaternion.cpp.o.requires:

.PHONY : CMakeFiles/NumberSystem.dir/src/Quaternion.cpp.o.requires

CMakeFiles/NumberSystem.dir/src/Quaternion.cpp.o.provides: CMakeFiles/NumberSystem.dir/src/Quaternion.cpp.o.requires
	$(MAKE) -f CMakeFiles/NumberSystem.dir/build.make CMakeFiles/NumberSystem.dir/src/Quaternion.cpp.o.provides.build
.PHONY : CMakeFiles/NumberSystem.dir/src/Quaternion.cpp.o.provides

CMakeFiles/NumberSystem.dir/src/Quaternion.cpp.o.provides.build: CMakeFiles/NumberSystem.dir/src/Quaternion.cpp.o


CMakeFiles/NumberSystem.dir/src/Sedenion.cpp.o: CMakeFiles/NumberSystem.dir/flags.make
CMakeFiles/NumberSystem.dir/src/Sedenion.cpp.o: ../src/Sedenion.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/niu/Documents/code/NumberSystems/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/NumberSystem.dir/src/Sedenion.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/NumberSystem.dir/src/Sedenion.cpp.o -c /home/niu/Documents/code/NumberSystems/src/Sedenion.cpp

CMakeFiles/NumberSystem.dir/src/Sedenion.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/NumberSystem.dir/src/Sedenion.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/niu/Documents/code/NumberSystems/src/Sedenion.cpp > CMakeFiles/NumberSystem.dir/src/Sedenion.cpp.i

CMakeFiles/NumberSystem.dir/src/Sedenion.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/NumberSystem.dir/src/Sedenion.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/niu/Documents/code/NumberSystems/src/Sedenion.cpp -o CMakeFiles/NumberSystem.dir/src/Sedenion.cpp.s

CMakeFiles/NumberSystem.dir/src/Sedenion.cpp.o.requires:

.PHONY : CMakeFiles/NumberSystem.dir/src/Sedenion.cpp.o.requires

CMakeFiles/NumberSystem.dir/src/Sedenion.cpp.o.provides: CMakeFiles/NumberSystem.dir/src/Sedenion.cpp.o.requires
	$(MAKE) -f CMakeFiles/NumberSystem.dir/build.make CMakeFiles/NumberSystem.dir/src/Sedenion.cpp.o.provides.build
.PHONY : CMakeFiles/NumberSystem.dir/src/Sedenion.cpp.o.provides

CMakeFiles/NumberSystem.dir/src/Sedenion.cpp.o.provides.build: CMakeFiles/NumberSystem.dir/src/Sedenion.cpp.o


CMakeFiles/NumberSystem.dir/src/Slerp.cpp.o: CMakeFiles/NumberSystem.dir/flags.make
CMakeFiles/NumberSystem.dir/src/Slerp.cpp.o: ../src/Slerp.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/niu/Documents/code/NumberSystems/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/NumberSystem.dir/src/Slerp.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/NumberSystem.dir/src/Slerp.cpp.o -c /home/niu/Documents/code/NumberSystems/src/Slerp.cpp

CMakeFiles/NumberSystem.dir/src/Slerp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/NumberSystem.dir/src/Slerp.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/niu/Documents/code/NumberSystems/src/Slerp.cpp > CMakeFiles/NumberSystem.dir/src/Slerp.cpp.i

CMakeFiles/NumberSystem.dir/src/Slerp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/NumberSystem.dir/src/Slerp.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/niu/Documents/code/NumberSystems/src/Slerp.cpp -o CMakeFiles/NumberSystem.dir/src/Slerp.cpp.s

CMakeFiles/NumberSystem.dir/src/Slerp.cpp.o.requires:

.PHONY : CMakeFiles/NumberSystem.dir/src/Slerp.cpp.o.requires

CMakeFiles/NumberSystem.dir/src/Slerp.cpp.o.provides: CMakeFiles/NumberSystem.dir/src/Slerp.cpp.o.requires
	$(MAKE) -f CMakeFiles/NumberSystem.dir/build.make CMakeFiles/NumberSystem.dir/src/Slerp.cpp.o.provides.build
.PHONY : CMakeFiles/NumberSystem.dir/src/Slerp.cpp.o.provides

CMakeFiles/NumberSystem.dir/src/Slerp.cpp.o.provides.build: CMakeFiles/NumberSystem.dir/src/Slerp.cpp.o


CMakeFiles/NumberSystem.dir/src/Spline.cpp.o: CMakeFiles/NumberSystem.dir/flags.make
CMakeFiles/NumberSystem.dir/src/Spline.cpp.o: ../src/Spline.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/niu/Documents/code/NumberSystems/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/NumberSystem.dir/src/Spline.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/NumberSystem.dir/src/Spline.cpp.o -c /home/niu/Documents/code/NumberSystems/src/Spline.cpp

CMakeFiles/NumberSystem.dir/src/Spline.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/NumberSystem.dir/src/Spline.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/niu/Documents/code/NumberSystems/src/Spline.cpp > CMakeFiles/NumberSystem.dir/src/Spline.cpp.i

CMakeFiles/NumberSystem.dir/src/Spline.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/NumberSystem.dir/src/Spline.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/niu/Documents/code/NumberSystems/src/Spline.cpp -o CMakeFiles/NumberSystem.dir/src/Spline.cpp.s

CMakeFiles/NumberSystem.dir/src/Spline.cpp.o.requires:

.PHONY : CMakeFiles/NumberSystem.dir/src/Spline.cpp.o.requires

CMakeFiles/NumberSystem.dir/src/Spline.cpp.o.provides: CMakeFiles/NumberSystem.dir/src/Spline.cpp.o.requires
	$(MAKE) -f CMakeFiles/NumberSystem.dir/build.make CMakeFiles/NumberSystem.dir/src/Spline.cpp.o.provides.build
.PHONY : CMakeFiles/NumberSystem.dir/src/Spline.cpp.o.provides

CMakeFiles/NumberSystem.dir/src/Spline.cpp.o.provides.build: CMakeFiles/NumberSystem.dir/src/Spline.cpp.o


CMakeFiles/NumberSystem.dir/src/Support.cpp.o: CMakeFiles/NumberSystem.dir/flags.make
CMakeFiles/NumberSystem.dir/src/Support.cpp.o: ../src/Support.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/niu/Documents/code/NumberSystems/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/NumberSystem.dir/src/Support.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/NumberSystem.dir/src/Support.cpp.o -c /home/niu/Documents/code/NumberSystems/src/Support.cpp

CMakeFiles/NumberSystem.dir/src/Support.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/NumberSystem.dir/src/Support.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/niu/Documents/code/NumberSystems/src/Support.cpp > CMakeFiles/NumberSystem.dir/src/Support.cpp.i

CMakeFiles/NumberSystem.dir/src/Support.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/NumberSystem.dir/src/Support.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/niu/Documents/code/NumberSystems/src/Support.cpp -o CMakeFiles/NumberSystem.dir/src/Support.cpp.s

CMakeFiles/NumberSystem.dir/src/Support.cpp.o.requires:

.PHONY : CMakeFiles/NumberSystem.dir/src/Support.cpp.o.requires

CMakeFiles/NumberSystem.dir/src/Support.cpp.o.provides: CMakeFiles/NumberSystem.dir/src/Support.cpp.o.requires
	$(MAKE) -f CMakeFiles/NumberSystem.dir/build.make CMakeFiles/NumberSystem.dir/src/Support.cpp.o.provides.build
.PHONY : CMakeFiles/NumberSystem.dir/src/Support.cpp.o.provides

CMakeFiles/NumberSystem.dir/src/Support.cpp.o.provides.build: CMakeFiles/NumberSystem.dir/src/Support.cpp.o


CMakeFiles/NumberSystem.dir/src/Trigintaduonion.cpp.o: CMakeFiles/NumberSystem.dir/flags.make
CMakeFiles/NumberSystem.dir/src/Trigintaduonion.cpp.o: ../src/Trigintaduonion.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/niu/Documents/code/NumberSystems/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/NumberSystem.dir/src/Trigintaduonion.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-7  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/NumberSystem.dir/src/Trigintaduonion.cpp.o -c /home/niu/Documents/code/NumberSystems/src/Trigintaduonion.cpp

CMakeFiles/NumberSystem.dir/src/Trigintaduonion.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/NumberSystem.dir/src/Trigintaduonion.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/niu/Documents/code/NumberSystems/src/Trigintaduonion.cpp > CMakeFiles/NumberSystem.dir/src/Trigintaduonion.cpp.i

CMakeFiles/NumberSystem.dir/src/Trigintaduonion.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/NumberSystem.dir/src/Trigintaduonion.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-7 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/niu/Documents/code/NumberSystems/src/Trigintaduonion.cpp -o CMakeFiles/NumberSystem.dir/src/Trigintaduonion.cpp.s

CMakeFiles/NumberSystem.dir/src/Trigintaduonion.cpp.o.requires:

.PHONY : CMakeFiles/NumberSystem.dir/src/Trigintaduonion.cpp.o.requires

CMakeFiles/NumberSystem.dir/src/Trigintaduonion.cpp.o.provides: CMakeFiles/NumberSystem.dir/src/Trigintaduonion.cpp.o.requires
	$(MAKE) -f CMakeFiles/NumberSystem.dir/build.make CMakeFiles/NumberSystem.dir/src/Trigintaduonion.cpp.o.provides.build
.PHONY : CMakeFiles/NumberSystem.dir/src/Trigintaduonion.cpp.o.provides

CMakeFiles/NumberSystem.dir/src/Trigintaduonion.cpp.o.provides.build: CMakeFiles/NumberSystem.dir/src/Trigintaduonion.cpp.o


# Object files for target NumberSystem
NumberSystem_OBJECTS = \
"CMakeFiles/NumberSystem.dir/src/Bezier.cpp.o" \
"CMakeFiles/NumberSystem.dir/src/Complex.cpp.o" \
"CMakeFiles/NumberSystem.dir/src/Octonion.cpp.o" \
"CMakeFiles/NumberSystem.dir/src/Quaternion.cpp.o" \
"CMakeFiles/NumberSystem.dir/src/Sedenion.cpp.o" \
"CMakeFiles/NumberSystem.dir/src/Slerp.cpp.o" \
"CMakeFiles/NumberSystem.dir/src/Spline.cpp.o" \
"CMakeFiles/NumberSystem.dir/src/Support.cpp.o" \
"CMakeFiles/NumberSystem.dir/src/Trigintaduonion.cpp.o"

# External object files for target NumberSystem
NumberSystem_EXTERNAL_OBJECTS =

libNumberSystem.a: CMakeFiles/NumberSystem.dir/src/Bezier.cpp.o
libNumberSystem.a: CMakeFiles/NumberSystem.dir/src/Complex.cpp.o
libNumberSystem.a: CMakeFiles/NumberSystem.dir/src/Octonion.cpp.o
libNumberSystem.a: CMakeFiles/NumberSystem.dir/src/Quaternion.cpp.o
libNumberSystem.a: CMakeFiles/NumberSystem.dir/src/Sedenion.cpp.o
libNumberSystem.a: CMakeFiles/NumberSystem.dir/src/Slerp.cpp.o
libNumberSystem.a: CMakeFiles/NumberSystem.dir/src/Spline.cpp.o
libNumberSystem.a: CMakeFiles/NumberSystem.dir/src/Support.cpp.o
libNumberSystem.a: CMakeFiles/NumberSystem.dir/src/Trigintaduonion.cpp.o
libNumberSystem.a: CMakeFiles/NumberSystem.dir/build.make
libNumberSystem.a: CMakeFiles/NumberSystem.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/niu/Documents/code/NumberSystems/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Linking CXX static library libNumberSystem.a"
	$(CMAKE_COMMAND) -P CMakeFiles/NumberSystem.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/NumberSystem.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/NumberSystem.dir/build: libNumberSystem.a

.PHONY : CMakeFiles/NumberSystem.dir/build

CMakeFiles/NumberSystem.dir/requires: CMakeFiles/NumberSystem.dir/src/Bezier.cpp.o.requires
CMakeFiles/NumberSystem.dir/requires: CMakeFiles/NumberSystem.dir/src/Complex.cpp.o.requires
CMakeFiles/NumberSystem.dir/requires: CMakeFiles/NumberSystem.dir/src/Octonion.cpp.o.requires
CMakeFiles/NumberSystem.dir/requires: CMakeFiles/NumberSystem.dir/src/Quaternion.cpp.o.requires
CMakeFiles/NumberSystem.dir/requires: CMakeFiles/NumberSystem.dir/src/Sedenion.cpp.o.requires
CMakeFiles/NumberSystem.dir/requires: CMakeFiles/NumberSystem.dir/src/Slerp.cpp.o.requires
CMakeFiles/NumberSystem.dir/requires: CMakeFiles/NumberSystem.dir/src/Spline.cpp.o.requires
CMakeFiles/NumberSystem.dir/requires: CMakeFiles/NumberSystem.dir/src/Support.cpp.o.requires
CMakeFiles/NumberSystem.dir/requires: CMakeFiles/NumberSystem.dir/src/Trigintaduonion.cpp.o.requires

.PHONY : CMakeFiles/NumberSystem.dir/requires

CMakeFiles/NumberSystem.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/NumberSystem.dir/cmake_clean.cmake
.PHONY : CMakeFiles/NumberSystem.dir/clean

CMakeFiles/NumberSystem.dir/depend:
	cd /home/niu/Documents/code/NumberSystems/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/niu/Documents/code/NumberSystems /home/niu/Documents/code/NumberSystems /home/niu/Documents/code/NumberSystems/build /home/niu/Documents/code/NumberSystems/build /home/niu/Documents/code/NumberSystems/build/CMakeFiles/NumberSystem.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/NumberSystem.dir/depend
