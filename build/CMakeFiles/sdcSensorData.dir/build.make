# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.7.0/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.7.0/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/spoderman/Desktop/SDC-Merged

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/spoderman/Desktop/SDC-Merged/build

# Include any dependencies generated for this target.
include CMakeFiles/sdcSensorData.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/sdcSensorData.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sdcSensorData.dir/flags.make

CMakeFiles/sdcSensorData.dir/sdcSensorData.cc.o: CMakeFiles/sdcSensorData.dir/flags.make
CMakeFiles/sdcSensorData.dir/sdcSensorData.cc.o: ../sdcSensorData.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/spoderman/Desktop/SDC-Merged/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/sdcSensorData.dir/sdcSensorData.cc.o"
	/Applications/Xcode-beta.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sdcSensorData.dir/sdcSensorData.cc.o -c /Users/spoderman/Desktop/SDC-Merged/sdcSensorData.cc

CMakeFiles/sdcSensorData.dir/sdcSensorData.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sdcSensorData.dir/sdcSensorData.cc.i"
	/Applications/Xcode-beta.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/spoderman/Desktop/SDC-Merged/sdcSensorData.cc > CMakeFiles/sdcSensorData.dir/sdcSensorData.cc.i

CMakeFiles/sdcSensorData.dir/sdcSensorData.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sdcSensorData.dir/sdcSensorData.cc.s"
	/Applications/Xcode-beta.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/spoderman/Desktop/SDC-Merged/sdcSensorData.cc -o CMakeFiles/sdcSensorData.dir/sdcSensorData.cc.s

CMakeFiles/sdcSensorData.dir/sdcSensorData.cc.o.requires:

.PHONY : CMakeFiles/sdcSensorData.dir/sdcSensorData.cc.o.requires

CMakeFiles/sdcSensorData.dir/sdcSensorData.cc.o.provides: CMakeFiles/sdcSensorData.dir/sdcSensorData.cc.o.requires
	$(MAKE) -f CMakeFiles/sdcSensorData.dir/build.make CMakeFiles/sdcSensorData.dir/sdcSensorData.cc.o.provides.build
.PHONY : CMakeFiles/sdcSensorData.dir/sdcSensorData.cc.o.provides

CMakeFiles/sdcSensorData.dir/sdcSensorData.cc.o.provides.build: CMakeFiles/sdcSensorData.dir/sdcSensorData.cc.o


# Object files for target sdcSensorData
sdcSensorData_OBJECTS = \
"CMakeFiles/sdcSensorData.dir/sdcSensorData.cc.o"

# External object files for target sdcSensorData
sdcSensorData_EXTERNAL_OBJECTS =

libsdcSensorData.dylib: CMakeFiles/sdcSensorData.dir/sdcSensorData.cc.o
libsdcSensorData.dylib: CMakeFiles/sdcSensorData.dir/build.make
libsdcSensorData.dylib: libsdcVisibleObject.dylib
libsdcSensorData.dylib: libsdcLidarRay.dylib
libsdcSensorData.dylib: libsdcLidarSensorInfo.dylib
libsdcSensorData.dylib: libsdcAngle.dylib
libsdcSensorData.dylib: /usr/local/Cellar/gazebo6/HEAD-5ef70d513ac1_2/lib/libgazebo.dylib
libsdcSensorData.dylib: /usr/local/Cellar/gazebo6/HEAD-5ef70d513ac1_2/lib/libgazebo_client.dylib
libsdcSensorData.dylib: /usr/local/Cellar/gazebo6/HEAD-5ef70d513ac1_2/lib/libgazebo_gui_building.dylib
libsdcSensorData.dylib: /usr/local/Cellar/gazebo6/HEAD-5ef70d513ac1_2/lib/libgazebo_gui_viewers.dylib
libsdcSensorData.dylib: /usr/local/Cellar/gazebo6/HEAD-5ef70d513ac1_2/lib/libgazebo_gui_model.dylib
libsdcSensorData.dylib: /usr/local/Cellar/gazebo6/HEAD-5ef70d513ac1_2/lib/libgazebo_gui.dylib
libsdcSensorData.dylib: /usr/local/Cellar/gazebo6/HEAD-5ef70d513ac1_2/lib/libgazebo_sensors.dylib
libsdcSensorData.dylib: /usr/local/Cellar/gazebo6/HEAD-5ef70d513ac1_2/lib/libgazebo_rendering.dylib
libsdcSensorData.dylib: /usr/local/Cellar/gazebo6/HEAD-5ef70d513ac1_2/lib/libgazebo_selection_buffer.dylib
libsdcSensorData.dylib: /usr/local/Cellar/gazebo6/HEAD-5ef70d513ac1_2/lib/libgazebo_physics_bullet.dylib
libsdcSensorData.dylib: /usr/local/Cellar/gazebo6/HEAD-5ef70d513ac1_2/lib/libgazebo_physics_simbody.dylib
libsdcSensorData.dylib: /usr/local/Cellar/gazebo6/HEAD-5ef70d513ac1_2/lib/libgazebo_physics_ode.dylib
libsdcSensorData.dylib: /usr/local/Cellar/gazebo6/HEAD-5ef70d513ac1_2/lib/libgazebo_physics.dylib
libsdcSensorData.dylib: /usr/local/Cellar/gazebo6/HEAD-5ef70d513ac1_2/lib/libgazebo_ode.dylib
libsdcSensorData.dylib: /usr/local/Cellar/gazebo6/HEAD-5ef70d513ac1_2/lib/libgazebo_transport.dylib
libsdcSensorData.dylib: /usr/local/Cellar/gazebo6/HEAD-5ef70d513ac1_2/lib/libgazebo_msgs.dylib
libsdcSensorData.dylib: /usr/local/Cellar/gazebo6/HEAD-5ef70d513ac1_2/lib/libgazebo_util.dylib
libsdcSensorData.dylib: /usr/local/Cellar/gazebo6/HEAD-5ef70d513ac1_2/lib/libgazebo_common.dylib
libsdcSensorData.dylib: /usr/local/Cellar/gazebo6/HEAD-5ef70d513ac1_2/lib/libgazebo_skyx.dylib
libsdcSensorData.dylib: /usr/local/Cellar/gazebo6/HEAD-5ef70d513ac1_2/lib/libgazebo_gimpact.dylib
libsdcSensorData.dylib: /usr/local/Cellar/gazebo6/HEAD-5ef70d513ac1_2/lib/libgazebo_opcode.dylib
libsdcSensorData.dylib: /usr/local/Cellar/gazebo6/HEAD-5ef70d513ac1_2/lib/libgazebo_opende_ou.dylib
libsdcSensorData.dylib: /usr/local/Cellar/gazebo6/HEAD-5ef70d513ac1_2/lib/libgazebo_math.dylib
libsdcSensorData.dylib: /usr/local/Cellar/gazebo6/HEAD-5ef70d513ac1_2/lib/libgazebo_ccd.dylib
libsdcSensorData.dylib: /usr/local/lib/libboost_thread-mt.dylib
libsdcSensorData.dylib: /usr/local/lib/libboost_signals-mt.dylib
libsdcSensorData.dylib: /usr/local/lib/libboost_system-mt.dylib
libsdcSensorData.dylib: /usr/local/lib/libboost_filesystem-mt.dylib
libsdcSensorData.dylib: /usr/local/lib/libboost_program_options-mt.dylib
libsdcSensorData.dylib: /usr/local/lib/libboost_regex-mt.dylib
libsdcSensorData.dylib: /usr/local/lib/libboost_iostreams-mt.dylib
libsdcSensorData.dylib: /usr/local/lib/libboost_date_time-mt.dylib
libsdcSensorData.dylib: /usr/local/lib/libboost_chrono-mt.dylib
libsdcSensorData.dylib: /usr/local/lib/libboost_atomic-mt.dylib
libsdcSensorData.dylib: /usr/local/lib/libprotobuf.dylib
libsdcSensorData.dylib: /usr/local/Cellar/sdformat3/3.7.0_2/lib/libsdformat.dylib
libsdcSensorData.dylib: /usr/local/Cellar/ignition-math2/2.6.0/lib/libignition-math2.dylib
libsdcSensorData.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgreMain.dylib
libsdcSensorData.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgreTerrain.dylib
libsdcSensorData.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgrePaging.dylib
libsdcSensorData.dylib: /usr/local/Cellar/ignition-math2/2.6.0/lib/libignition-math2.dylib
libsdcSensorData.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgreMain.dylib
libsdcSensorData.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgreTerrain.dylib
libsdcSensorData.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgrePaging.dylib
libsdcSensorData.dylib: CMakeFiles/sdcSensorData.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/spoderman/Desktop/SDC-Merged/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libsdcSensorData.dylib"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sdcSensorData.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/sdcSensorData.dir/build: libsdcSensorData.dylib

.PHONY : CMakeFiles/sdcSensorData.dir/build

CMakeFiles/sdcSensorData.dir/requires: CMakeFiles/sdcSensorData.dir/sdcSensorData.cc.o.requires

.PHONY : CMakeFiles/sdcSensorData.dir/requires

CMakeFiles/sdcSensorData.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sdcSensorData.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sdcSensorData.dir/clean

CMakeFiles/sdcSensorData.dir/depend:
	cd /Users/spoderman/Desktop/SDC-Merged/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/spoderman/Desktop/SDC-Merged /Users/spoderman/Desktop/SDC-Merged /Users/spoderman/Desktop/SDC-Merged/build /Users/spoderman/Desktop/SDC-Merged/build /Users/spoderman/Desktop/SDC-Merged/build/CMakeFiles/sdcSensorData.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sdcSensorData.dir/depend

