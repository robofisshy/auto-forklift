# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /opt/project/fork

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /opt/project/fork

# Include any dependencies generated for this target.
include CMakeFiles/stardraw_forklift.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/stardraw_forklift.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/stardraw_forklift.dir/flags.make

CMakeFiles/stardraw_forklift.dir/src/main.cpp.o: CMakeFiles/stardraw_forklift.dir/flags.make
CMakeFiles/stardraw_forklift.dir/src/main.cpp.o: src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/opt/project/fork/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/stardraw_forklift.dir/src/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stardraw_forklift.dir/src/main.cpp.o -c /opt/project/fork/src/main.cpp

CMakeFiles/stardraw_forklift.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stardraw_forklift.dir/src/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /opt/project/fork/src/main.cpp > CMakeFiles/stardraw_forklift.dir/src/main.cpp.i

CMakeFiles/stardraw_forklift.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stardraw_forklift.dir/src/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /opt/project/fork/src/main.cpp -o CMakeFiles/stardraw_forklift.dir/src/main.cpp.s

CMakeFiles/stardraw_forklift.dir/src/main.cpp.o.requires:

.PHONY : CMakeFiles/stardraw_forklift.dir/src/main.cpp.o.requires

CMakeFiles/stardraw_forklift.dir/src/main.cpp.o.provides: CMakeFiles/stardraw_forklift.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/stardraw_forklift.dir/build.make CMakeFiles/stardraw_forklift.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/stardraw_forklift.dir/src/main.cpp.o.provides

CMakeFiles/stardraw_forklift.dir/src/main.cpp.o.provides.build: CMakeFiles/stardraw_forklift.dir/src/main.cpp.o


CMakeFiles/stardraw_forklift.dir/src/location/MarkerDetector.cpp.o: CMakeFiles/stardraw_forklift.dir/flags.make
CMakeFiles/stardraw_forklift.dir/src/location/MarkerDetector.cpp.o: src/location/MarkerDetector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/opt/project/fork/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/stardraw_forklift.dir/src/location/MarkerDetector.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stardraw_forklift.dir/src/location/MarkerDetector.cpp.o -c /opt/project/fork/src/location/MarkerDetector.cpp

CMakeFiles/stardraw_forklift.dir/src/location/MarkerDetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stardraw_forklift.dir/src/location/MarkerDetector.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /opt/project/fork/src/location/MarkerDetector.cpp > CMakeFiles/stardraw_forklift.dir/src/location/MarkerDetector.cpp.i

CMakeFiles/stardraw_forklift.dir/src/location/MarkerDetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stardraw_forklift.dir/src/location/MarkerDetector.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /opt/project/fork/src/location/MarkerDetector.cpp -o CMakeFiles/stardraw_forklift.dir/src/location/MarkerDetector.cpp.s

CMakeFiles/stardraw_forklift.dir/src/location/MarkerDetector.cpp.o.requires:

.PHONY : CMakeFiles/stardraw_forklift.dir/src/location/MarkerDetector.cpp.o.requires

CMakeFiles/stardraw_forklift.dir/src/location/MarkerDetector.cpp.o.provides: CMakeFiles/stardraw_forklift.dir/src/location/MarkerDetector.cpp.o.requires
	$(MAKE) -f CMakeFiles/stardraw_forklift.dir/build.make CMakeFiles/stardraw_forklift.dir/src/location/MarkerDetector.cpp.o.provides.build
.PHONY : CMakeFiles/stardraw_forklift.dir/src/location/MarkerDetector.cpp.o.provides

CMakeFiles/stardraw_forklift.dir/src/location/MarkerDetector.cpp.o.provides.build: CMakeFiles/stardraw_forklift.dir/src/location/MarkerDetector.cpp.o


CMakeFiles/stardraw_forklift.dir/src/location/marker.cpp.o: CMakeFiles/stardraw_forklift.dir/flags.make
CMakeFiles/stardraw_forklift.dir/src/location/marker.cpp.o: src/location/marker.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/opt/project/fork/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/stardraw_forklift.dir/src/location/marker.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stardraw_forklift.dir/src/location/marker.cpp.o -c /opt/project/fork/src/location/marker.cpp

CMakeFiles/stardraw_forklift.dir/src/location/marker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stardraw_forklift.dir/src/location/marker.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /opt/project/fork/src/location/marker.cpp > CMakeFiles/stardraw_forklift.dir/src/location/marker.cpp.i

CMakeFiles/stardraw_forklift.dir/src/location/marker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stardraw_forklift.dir/src/location/marker.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /opt/project/fork/src/location/marker.cpp -o CMakeFiles/stardraw_forklift.dir/src/location/marker.cpp.s

CMakeFiles/stardraw_forklift.dir/src/location/marker.cpp.o.requires:

.PHONY : CMakeFiles/stardraw_forklift.dir/src/location/marker.cpp.o.requires

CMakeFiles/stardraw_forklift.dir/src/location/marker.cpp.o.provides: CMakeFiles/stardraw_forklift.dir/src/location/marker.cpp.o.requires
	$(MAKE) -f CMakeFiles/stardraw_forklift.dir/build.make CMakeFiles/stardraw_forklift.dir/src/location/marker.cpp.o.provides.build
.PHONY : CMakeFiles/stardraw_forklift.dir/src/location/marker.cpp.o.provides

CMakeFiles/stardraw_forklift.dir/src/location/marker.cpp.o.provides.build: CMakeFiles/stardraw_forklift.dir/src/location/marker.cpp.o


CMakeFiles/stardraw_forklift.dir/src/location/LocationDetector.cpp.o: CMakeFiles/stardraw_forklift.dir/flags.make
CMakeFiles/stardraw_forklift.dir/src/location/LocationDetector.cpp.o: src/location/LocationDetector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/opt/project/fork/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/stardraw_forklift.dir/src/location/LocationDetector.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stardraw_forklift.dir/src/location/LocationDetector.cpp.o -c /opt/project/fork/src/location/LocationDetector.cpp

CMakeFiles/stardraw_forklift.dir/src/location/LocationDetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stardraw_forklift.dir/src/location/LocationDetector.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /opt/project/fork/src/location/LocationDetector.cpp > CMakeFiles/stardraw_forklift.dir/src/location/LocationDetector.cpp.i

CMakeFiles/stardraw_forklift.dir/src/location/LocationDetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stardraw_forklift.dir/src/location/LocationDetector.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /opt/project/fork/src/location/LocationDetector.cpp -o CMakeFiles/stardraw_forklift.dir/src/location/LocationDetector.cpp.s

CMakeFiles/stardraw_forklift.dir/src/location/LocationDetector.cpp.o.requires:

.PHONY : CMakeFiles/stardraw_forklift.dir/src/location/LocationDetector.cpp.o.requires

CMakeFiles/stardraw_forklift.dir/src/location/LocationDetector.cpp.o.provides: CMakeFiles/stardraw_forklift.dir/src/location/LocationDetector.cpp.o.requires
	$(MAKE) -f CMakeFiles/stardraw_forklift.dir/build.make CMakeFiles/stardraw_forklift.dir/src/location/LocationDetector.cpp.o.provides.build
.PHONY : CMakeFiles/stardraw_forklift.dir/src/location/LocationDetector.cpp.o.provides

CMakeFiles/stardraw_forklift.dir/src/location/LocationDetector.cpp.o.provides.build: CMakeFiles/stardraw_forklift.dir/src/location/LocationDetector.cpp.o


CMakeFiles/stardraw_forklift.dir/src/multi_thread/Multi_thread.cpp.o: CMakeFiles/stardraw_forklift.dir/flags.make
CMakeFiles/stardraw_forklift.dir/src/multi_thread/Multi_thread.cpp.o: src/multi_thread/Multi_thread.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/opt/project/fork/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/stardraw_forklift.dir/src/multi_thread/Multi_thread.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stardraw_forklift.dir/src/multi_thread/Multi_thread.cpp.o -c /opt/project/fork/src/multi_thread/Multi_thread.cpp

CMakeFiles/stardraw_forklift.dir/src/multi_thread/Multi_thread.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stardraw_forklift.dir/src/multi_thread/Multi_thread.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /opt/project/fork/src/multi_thread/Multi_thread.cpp > CMakeFiles/stardraw_forklift.dir/src/multi_thread/Multi_thread.cpp.i

CMakeFiles/stardraw_forklift.dir/src/multi_thread/Multi_thread.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stardraw_forklift.dir/src/multi_thread/Multi_thread.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /opt/project/fork/src/multi_thread/Multi_thread.cpp -o CMakeFiles/stardraw_forklift.dir/src/multi_thread/Multi_thread.cpp.s

CMakeFiles/stardraw_forklift.dir/src/multi_thread/Multi_thread.cpp.o.requires:

.PHONY : CMakeFiles/stardraw_forklift.dir/src/multi_thread/Multi_thread.cpp.o.requires

CMakeFiles/stardraw_forklift.dir/src/multi_thread/Multi_thread.cpp.o.provides: CMakeFiles/stardraw_forklift.dir/src/multi_thread/Multi_thread.cpp.o.requires
	$(MAKE) -f CMakeFiles/stardraw_forklift.dir/build.make CMakeFiles/stardraw_forklift.dir/src/multi_thread/Multi_thread.cpp.o.provides.build
.PHONY : CMakeFiles/stardraw_forklift.dir/src/multi_thread/Multi_thread.cpp.o.provides

CMakeFiles/stardraw_forklift.dir/src/multi_thread/Multi_thread.cpp.o.provides.build: CMakeFiles/stardraw_forklift.dir/src/multi_thread/Multi_thread.cpp.o


CMakeFiles/stardraw_forklift.dir/src/utils/utils.cpp.o: CMakeFiles/stardraw_forklift.dir/flags.make
CMakeFiles/stardraw_forklift.dir/src/utils/utils.cpp.o: src/utils/utils.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/opt/project/fork/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/stardraw_forklift.dir/src/utils/utils.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stardraw_forklift.dir/src/utils/utils.cpp.o -c /opt/project/fork/src/utils/utils.cpp

CMakeFiles/stardraw_forklift.dir/src/utils/utils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stardraw_forklift.dir/src/utils/utils.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /opt/project/fork/src/utils/utils.cpp > CMakeFiles/stardraw_forklift.dir/src/utils/utils.cpp.i

CMakeFiles/stardraw_forklift.dir/src/utils/utils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stardraw_forklift.dir/src/utils/utils.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /opt/project/fork/src/utils/utils.cpp -o CMakeFiles/stardraw_forklift.dir/src/utils/utils.cpp.s

CMakeFiles/stardraw_forklift.dir/src/utils/utils.cpp.o.requires:

.PHONY : CMakeFiles/stardraw_forklift.dir/src/utils/utils.cpp.o.requires

CMakeFiles/stardraw_forklift.dir/src/utils/utils.cpp.o.provides: CMakeFiles/stardraw_forklift.dir/src/utils/utils.cpp.o.requires
	$(MAKE) -f CMakeFiles/stardraw_forklift.dir/build.make CMakeFiles/stardraw_forklift.dir/src/utils/utils.cpp.o.provides.build
.PHONY : CMakeFiles/stardraw_forklift.dir/src/utils/utils.cpp.o.provides

CMakeFiles/stardraw_forklift.dir/src/utils/utils.cpp.o.provides.build: CMakeFiles/stardraw_forklift.dir/src/utils/utils.cpp.o


CMakeFiles/stardraw_forklift.dir/src/motor_control/pathTrack.cpp.o: CMakeFiles/stardraw_forklift.dir/flags.make
CMakeFiles/stardraw_forklift.dir/src/motor_control/pathTrack.cpp.o: src/motor_control/pathTrack.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/opt/project/fork/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/stardraw_forklift.dir/src/motor_control/pathTrack.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stardraw_forklift.dir/src/motor_control/pathTrack.cpp.o -c /opt/project/fork/src/motor_control/pathTrack.cpp

CMakeFiles/stardraw_forklift.dir/src/motor_control/pathTrack.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stardraw_forklift.dir/src/motor_control/pathTrack.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /opt/project/fork/src/motor_control/pathTrack.cpp > CMakeFiles/stardraw_forklift.dir/src/motor_control/pathTrack.cpp.i

CMakeFiles/stardraw_forklift.dir/src/motor_control/pathTrack.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stardraw_forklift.dir/src/motor_control/pathTrack.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /opt/project/fork/src/motor_control/pathTrack.cpp -o CMakeFiles/stardraw_forklift.dir/src/motor_control/pathTrack.cpp.s

CMakeFiles/stardraw_forklift.dir/src/motor_control/pathTrack.cpp.o.requires:

.PHONY : CMakeFiles/stardraw_forklift.dir/src/motor_control/pathTrack.cpp.o.requires

CMakeFiles/stardraw_forklift.dir/src/motor_control/pathTrack.cpp.o.provides: CMakeFiles/stardraw_forklift.dir/src/motor_control/pathTrack.cpp.o.requires
	$(MAKE) -f CMakeFiles/stardraw_forklift.dir/build.make CMakeFiles/stardraw_forklift.dir/src/motor_control/pathTrack.cpp.o.provides.build
.PHONY : CMakeFiles/stardraw_forklift.dir/src/motor_control/pathTrack.cpp.o.provides

CMakeFiles/stardraw_forklift.dir/src/motor_control/pathTrack.cpp.o.provides.build: CMakeFiles/stardraw_forklift.dir/src/motor_control/pathTrack.cpp.o


CMakeFiles/stardraw_forklift.dir/src/motor_control/jetsonGPIO.cpp.o: CMakeFiles/stardraw_forklift.dir/flags.make
CMakeFiles/stardraw_forklift.dir/src/motor_control/jetsonGPIO.cpp.o: src/motor_control/jetsonGPIO.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/opt/project/fork/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/stardraw_forklift.dir/src/motor_control/jetsonGPIO.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stardraw_forklift.dir/src/motor_control/jetsonGPIO.cpp.o -c /opt/project/fork/src/motor_control/jetsonGPIO.cpp

CMakeFiles/stardraw_forklift.dir/src/motor_control/jetsonGPIO.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stardraw_forklift.dir/src/motor_control/jetsonGPIO.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /opt/project/fork/src/motor_control/jetsonGPIO.cpp > CMakeFiles/stardraw_forklift.dir/src/motor_control/jetsonGPIO.cpp.i

CMakeFiles/stardraw_forklift.dir/src/motor_control/jetsonGPIO.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stardraw_forklift.dir/src/motor_control/jetsonGPIO.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /opt/project/fork/src/motor_control/jetsonGPIO.cpp -o CMakeFiles/stardraw_forklift.dir/src/motor_control/jetsonGPIO.cpp.s

CMakeFiles/stardraw_forklift.dir/src/motor_control/jetsonGPIO.cpp.o.requires:

.PHONY : CMakeFiles/stardraw_forklift.dir/src/motor_control/jetsonGPIO.cpp.o.requires

CMakeFiles/stardraw_forklift.dir/src/motor_control/jetsonGPIO.cpp.o.provides: CMakeFiles/stardraw_forklift.dir/src/motor_control/jetsonGPIO.cpp.o.requires
	$(MAKE) -f CMakeFiles/stardraw_forklift.dir/build.make CMakeFiles/stardraw_forklift.dir/src/motor_control/jetsonGPIO.cpp.o.provides.build
.PHONY : CMakeFiles/stardraw_forklift.dir/src/motor_control/jetsonGPIO.cpp.o.provides

CMakeFiles/stardraw_forklift.dir/src/motor_control/jetsonGPIO.cpp.o.provides.build: CMakeFiles/stardraw_forklift.dir/src/motor_control/jetsonGPIO.cpp.o


CMakeFiles/stardraw_forklift.dir/src/motor_control/motorControl.cpp.o: CMakeFiles/stardraw_forklift.dir/flags.make
CMakeFiles/stardraw_forklift.dir/src/motor_control/motorControl.cpp.o: src/motor_control/motorControl.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/opt/project/fork/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/stardraw_forklift.dir/src/motor_control/motorControl.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stardraw_forklift.dir/src/motor_control/motorControl.cpp.o -c /opt/project/fork/src/motor_control/motorControl.cpp

CMakeFiles/stardraw_forklift.dir/src/motor_control/motorControl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stardraw_forklift.dir/src/motor_control/motorControl.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /opt/project/fork/src/motor_control/motorControl.cpp > CMakeFiles/stardraw_forklift.dir/src/motor_control/motorControl.cpp.i

CMakeFiles/stardraw_forklift.dir/src/motor_control/motorControl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stardraw_forklift.dir/src/motor_control/motorControl.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /opt/project/fork/src/motor_control/motorControl.cpp -o CMakeFiles/stardraw_forklift.dir/src/motor_control/motorControl.cpp.s

CMakeFiles/stardraw_forklift.dir/src/motor_control/motorControl.cpp.o.requires:

.PHONY : CMakeFiles/stardraw_forklift.dir/src/motor_control/motorControl.cpp.o.requires

CMakeFiles/stardraw_forklift.dir/src/motor_control/motorControl.cpp.o.provides: CMakeFiles/stardraw_forklift.dir/src/motor_control/motorControl.cpp.o.requires
	$(MAKE) -f CMakeFiles/stardraw_forklift.dir/build.make CMakeFiles/stardraw_forklift.dir/src/motor_control/motorControl.cpp.o.provides.build
.PHONY : CMakeFiles/stardraw_forklift.dir/src/motor_control/motorControl.cpp.o.provides

CMakeFiles/stardraw_forklift.dir/src/motor_control/motorControl.cpp.o.provides.build: CMakeFiles/stardraw_forklift.dir/src/motor_control/motorControl.cpp.o


CMakeFiles/stardraw_forklift.dir/src/detect/detect.cpp.o: CMakeFiles/stardraw_forklift.dir/flags.make
CMakeFiles/stardraw_forklift.dir/src/detect/detect.cpp.o: src/detect/detect.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/opt/project/fork/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/stardraw_forklift.dir/src/detect/detect.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stardraw_forklift.dir/src/detect/detect.cpp.o -c /opt/project/fork/src/detect/detect.cpp

CMakeFiles/stardraw_forklift.dir/src/detect/detect.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stardraw_forklift.dir/src/detect/detect.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /opt/project/fork/src/detect/detect.cpp > CMakeFiles/stardraw_forklift.dir/src/detect/detect.cpp.i

CMakeFiles/stardraw_forklift.dir/src/detect/detect.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stardraw_forklift.dir/src/detect/detect.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /opt/project/fork/src/detect/detect.cpp -o CMakeFiles/stardraw_forklift.dir/src/detect/detect.cpp.s

CMakeFiles/stardraw_forklift.dir/src/detect/detect.cpp.o.requires:

.PHONY : CMakeFiles/stardraw_forklift.dir/src/detect/detect.cpp.o.requires

CMakeFiles/stardraw_forklift.dir/src/detect/detect.cpp.o.provides: CMakeFiles/stardraw_forklift.dir/src/detect/detect.cpp.o.requires
	$(MAKE) -f CMakeFiles/stardraw_forklift.dir/build.make CMakeFiles/stardraw_forklift.dir/src/detect/detect.cpp.o.provides.build
.PHONY : CMakeFiles/stardraw_forklift.dir/src/detect/detect.cpp.o.provides

CMakeFiles/stardraw_forklift.dir/src/detect/detect.cpp.o.provides.build: CMakeFiles/stardraw_forklift.dir/src/detect/detect.cpp.o


CMakeFiles/stardraw_forklift.dir/src/forklift/forklift.cpp.o: CMakeFiles/stardraw_forklift.dir/flags.make
CMakeFiles/stardraw_forklift.dir/src/forklift/forklift.cpp.o: src/forklift/forklift.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/opt/project/fork/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object CMakeFiles/stardraw_forklift.dir/src/forklift/forklift.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stardraw_forklift.dir/src/forklift/forklift.cpp.o -c /opt/project/fork/src/forklift/forklift.cpp

CMakeFiles/stardraw_forklift.dir/src/forklift/forklift.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stardraw_forklift.dir/src/forklift/forklift.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /opt/project/fork/src/forklift/forklift.cpp > CMakeFiles/stardraw_forklift.dir/src/forklift/forklift.cpp.i

CMakeFiles/stardraw_forklift.dir/src/forklift/forklift.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stardraw_forklift.dir/src/forklift/forklift.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /opt/project/fork/src/forklift/forklift.cpp -o CMakeFiles/stardraw_forklift.dir/src/forklift/forklift.cpp.s

CMakeFiles/stardraw_forklift.dir/src/forklift/forklift.cpp.o.requires:

.PHONY : CMakeFiles/stardraw_forklift.dir/src/forklift/forklift.cpp.o.requires

CMakeFiles/stardraw_forklift.dir/src/forklift/forklift.cpp.o.provides: CMakeFiles/stardraw_forklift.dir/src/forklift/forklift.cpp.o.requires
	$(MAKE) -f CMakeFiles/stardraw_forklift.dir/build.make CMakeFiles/stardraw_forklift.dir/src/forklift/forklift.cpp.o.provides.build
.PHONY : CMakeFiles/stardraw_forklift.dir/src/forklift/forklift.cpp.o.provides

CMakeFiles/stardraw_forklift.dir/src/forklift/forklift.cpp.o.provides.build: CMakeFiles/stardraw_forklift.dir/src/forklift/forklift.cpp.o


# Object files for target stardraw_forklift
stardraw_forklift_OBJECTS = \
"CMakeFiles/stardraw_forklift.dir/src/main.cpp.o" \
"CMakeFiles/stardraw_forklift.dir/src/location/MarkerDetector.cpp.o" \
"CMakeFiles/stardraw_forklift.dir/src/location/marker.cpp.o" \
"CMakeFiles/stardraw_forklift.dir/src/location/LocationDetector.cpp.o" \
"CMakeFiles/stardraw_forklift.dir/src/multi_thread/Multi_thread.cpp.o" \
"CMakeFiles/stardraw_forklift.dir/src/utils/utils.cpp.o" \
"CMakeFiles/stardraw_forklift.dir/src/motor_control/pathTrack.cpp.o" \
"CMakeFiles/stardraw_forklift.dir/src/motor_control/jetsonGPIO.cpp.o" \
"CMakeFiles/stardraw_forklift.dir/src/motor_control/motorControl.cpp.o" \
"CMakeFiles/stardraw_forklift.dir/src/detect/detect.cpp.o" \
"CMakeFiles/stardraw_forklift.dir/src/forklift/forklift.cpp.o"

# External object files for target stardraw_forklift
stardraw_forklift_EXTERNAL_OBJECTS =

stardraw_forklift: CMakeFiles/stardraw_forklift.dir/src/main.cpp.o
stardraw_forklift: CMakeFiles/stardraw_forklift.dir/src/location/MarkerDetector.cpp.o
stardraw_forklift: CMakeFiles/stardraw_forklift.dir/src/location/marker.cpp.o
stardraw_forklift: CMakeFiles/stardraw_forklift.dir/src/location/LocationDetector.cpp.o
stardraw_forklift: CMakeFiles/stardraw_forklift.dir/src/multi_thread/Multi_thread.cpp.o
stardraw_forklift: CMakeFiles/stardraw_forklift.dir/src/utils/utils.cpp.o
stardraw_forklift: CMakeFiles/stardraw_forklift.dir/src/motor_control/pathTrack.cpp.o
stardraw_forklift: CMakeFiles/stardraw_forklift.dir/src/motor_control/jetsonGPIO.cpp.o
stardraw_forklift: CMakeFiles/stardraw_forklift.dir/src/motor_control/motorControl.cpp.o
stardraw_forklift: CMakeFiles/stardraw_forklift.dir/src/detect/detect.cpp.o
stardraw_forklift: CMakeFiles/stardraw_forklift.dir/src/forklift/forklift.cpp.o
stardraw_forklift: CMakeFiles/stardraw_forklift.dir/build.make
stardraw_forklift: /usr/local/lib/libopencv_videostab.so.3.1.0
stardraw_forklift: /usr/local/lib/libopencv_superres.so.3.1.0
stardraw_forklift: /usr/local/lib/libopencv_shape.so.3.1.0
stardraw_forklift: /usr/local/lib/libopencv_photo.so.3.1.0
stardraw_forklift: /usr/local/lib/libopencv_cudastereo.so.3.1.0
stardraw_forklift: /usr/local/lib/libopencv_cudaoptflow.so.3.1.0
stardraw_forklift: /usr/local/lib/libopencv_cudaobjdetect.so.3.1.0
stardraw_forklift: /usr/local/lib/libopencv_cudaimgproc.so.3.1.0
stardraw_forklift: /usr/local/lib/libopencv_cudacodec.so.3.1.0
stardraw_forklift: /usr/local/lib/libopencv_cudabgsegm.so.3.1.0
stardraw_forklift: /usr/local/lib/libopencv_cudaarithm.so.3.1.0
stardraw_forklift: /usr/local/lib/libopencv_calib3d.so.3.1.0
stardraw_forklift: /usr/lib/aarch64-linux-gnu/libboost_thread.so
stardraw_forklift: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
stardraw_forklift: /usr/lib/aarch64-linux-gnu/libboost_system.so
stardraw_forklift: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
stardraw_forklift: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
stardraw_forklift: /usr/lib/aarch64-linux-gnu/libpthread.so
stardraw_forklift: /usr/local/lib/libopencv_features2d.so.3.1.0
stardraw_forklift: /usr/local/lib/libopencv_flann.so.3.1.0
stardraw_forklift: /usr/local/lib/libopencv_objdetect.so.3.1.0
stardraw_forklift: /usr/local/lib/libopencv_ml.so.3.1.0
stardraw_forklift: /usr/local/lib/libopencv_highgui.so.3.1.0
stardraw_forklift: /usr/local/lib/libopencv_videoio.so.3.1.0
stardraw_forklift: /usr/local/lib/libopencv_imgcodecs.so.3.1.0
stardraw_forklift: /usr/local/lib/libopencv_cudawarping.so.3.1.0
stardraw_forklift: /usr/local/lib/libopencv_video.so.3.1.0
stardraw_forklift: /usr/local/lib/libopencv_imgproc.so.3.1.0
stardraw_forklift: /usr/local/lib/libopencv_core.so.3.1.0
stardraw_forklift: /usr/local/lib/libopencv_cudev.so.3.1.0
stardraw_forklift: CMakeFiles/stardraw_forklift.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/opt/project/fork/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Linking CXX executable stardraw_forklift"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/stardraw_forklift.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/stardraw_forklift.dir/build: stardraw_forklift

.PHONY : CMakeFiles/stardraw_forklift.dir/build

CMakeFiles/stardraw_forklift.dir/requires: CMakeFiles/stardraw_forklift.dir/src/main.cpp.o.requires
CMakeFiles/stardraw_forklift.dir/requires: CMakeFiles/stardraw_forklift.dir/src/location/MarkerDetector.cpp.o.requires
CMakeFiles/stardraw_forklift.dir/requires: CMakeFiles/stardraw_forklift.dir/src/location/marker.cpp.o.requires
CMakeFiles/stardraw_forklift.dir/requires: CMakeFiles/stardraw_forklift.dir/src/location/LocationDetector.cpp.o.requires
CMakeFiles/stardraw_forklift.dir/requires: CMakeFiles/stardraw_forklift.dir/src/multi_thread/Multi_thread.cpp.o.requires
CMakeFiles/stardraw_forklift.dir/requires: CMakeFiles/stardraw_forklift.dir/src/utils/utils.cpp.o.requires
CMakeFiles/stardraw_forklift.dir/requires: CMakeFiles/stardraw_forklift.dir/src/motor_control/pathTrack.cpp.o.requires
CMakeFiles/stardraw_forklift.dir/requires: CMakeFiles/stardraw_forklift.dir/src/motor_control/jetsonGPIO.cpp.o.requires
CMakeFiles/stardraw_forklift.dir/requires: CMakeFiles/stardraw_forklift.dir/src/motor_control/motorControl.cpp.o.requires
CMakeFiles/stardraw_forklift.dir/requires: CMakeFiles/stardraw_forklift.dir/src/detect/detect.cpp.o.requires
CMakeFiles/stardraw_forklift.dir/requires: CMakeFiles/stardraw_forklift.dir/src/forklift/forklift.cpp.o.requires

.PHONY : CMakeFiles/stardraw_forklift.dir/requires

CMakeFiles/stardraw_forklift.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/stardraw_forklift.dir/cmake_clean.cmake
.PHONY : CMakeFiles/stardraw_forklift.dir/clean

CMakeFiles/stardraw_forklift.dir/depend:
	cd /opt/project/fork && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /opt/project/fork /opt/project/fork /opt/project/fork /opt/project/fork /opt/project/fork/CMakeFiles/stardraw_forklift.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/stardraw_forklift.dir/depend

