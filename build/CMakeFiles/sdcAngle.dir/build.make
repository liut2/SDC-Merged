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
CMAKE_SOURCE_DIR = /Users/taoliu/Desktop/SDC-Merged

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/taoliu/Desktop/SDC-Merged/build

# Include any dependencies generated for this target.
include CMakeFiles/sdcAngle.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/sdcAngle.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sdcAngle.dir/flags.make

CMakeFiles/sdcAngle.dir/sdcAngle.cc.o: CMakeFiles/sdcAngle.dir/flags.make
CMakeFiles/sdcAngle.dir/sdcAngle.cc.o: ../sdcAngle.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/taoliu/Desktop/SDC-Merged/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/sdcAngle.dir/sdcAngle.cc.o"
	/Library/Developer/CommandLineTools/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sdcAngle.dir/sdcAngle.cc.o -c /Users/taoliu/Desktop/SDC-Merged/sdcAngle.cc

CMakeFiles/sdcAngle.dir/sdcAngle.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sdcAngle.dir/sdcAngle.cc.i"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/taoliu/Desktop/SDC-Merged/sdcAngle.cc > CMakeFiles/sdcAngle.dir/sdcAngle.cc.i

CMakeFiles/sdcAngle.dir/sdcAngle.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sdcAngle.dir/sdcAngle.cc.s"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/taoliu/Desktop/SDC-Merged/sdcAngle.cc -o CMakeFiles/sdcAngle.dir/sdcAngle.cc.s

CMakeFiles/sdcAngle.dir/sdcAngle.cc.o.requires:

.PHONY : CMakeFiles/sdcAngle.dir/sdcAngle.cc.o.requires

CMakeFiles/sdcAngle.dir/sdcAngle.cc.o.provides: CMakeFiles/sdcAngle.dir/sdcAngle.cc.o.requires
	$(MAKE) -f CMakeFiles/sdcAngle.dir/build.make CMakeFiles/sdcAngle.dir/sdcAngle.cc.o.provides.build
.PHONY : CMakeFiles/sdcAngle.dir/sdcAngle.cc.o.provides

CMakeFiles/sdcAngle.dir/sdcAngle.cc.o.provides.build: CMakeFiles/sdcAngle.dir/sdcAngle.cc.o


# Object files for target sdcAngle
sdcAngle_OBJECTS = \
"CMakeFiles/sdcAngle.dir/sdcAngle.cc.o"

# External object files for target sdcAngle
sdcAngle_EXTERNAL_OBJECTS =

libsdcAngle.dylib: CMakeFiles/sdcAngle.dir/sdcAngle.cc.o
libsdcAngle.dylib: CMakeFiles/sdcAngle.dir/build.make
libsdcAngle.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo.dylib
libsdcAngle.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_client.dylib
libsdcAngle.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_gui_building.dylib
libsdcAngle.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_gui_viewers.dylib
libsdcAngle.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_gui_model.dylib
libsdcAngle.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_gui.dylib
libsdcAngle.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_sensors.dylib
libsdcAngle.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_rendering.dylib
libsdcAngle.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_selection_buffer.dylib
libsdcAngle.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_physics_bullet.dylib
libsdcAngle.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_physics_simbody.dylib
libsdcAngle.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_physics_ode.dylib
libsdcAngle.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_physics.dylib
libsdcAngle.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_ode.dylib
libsdcAngle.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_transport.dylib
libsdcAngle.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_msgs.dylib
libsdcAngle.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_util.dylib
libsdcAngle.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_common.dylib
libsdcAngle.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_skyx.dylib
libsdcAngle.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_gimpact.dylib
libsdcAngle.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_opcode.dylib
libsdcAngle.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_opende_ou.dylib
libsdcAngle.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_math.dylib
libsdcAngle.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_ccd.dylib
libsdcAngle.dylib: /usr/local/lib/libboost_thread-mt.dylib
libsdcAngle.dylib: /usr/local/lib/libboost_signals-mt.dylib
libsdcAngle.dylib: /usr/local/lib/libboost_system-mt.dylib
libsdcAngle.dylib: /usr/local/lib/libboost_filesystem-mt.dylib
libsdcAngle.dylib: /usr/local/lib/libboost_program_options-mt.dylib
libsdcAngle.dylib: /usr/local/lib/libboost_regex-mt.dylib
libsdcAngle.dylib: /usr/local/lib/libboost_iostreams-mt.dylib
libsdcAngle.dylib: /usr/local/lib/libboost_date_time-mt.dylib
libsdcAngle.dylib: /usr/local/lib/libboost_chrono-mt.dylib
libsdcAngle.dylib: /usr/local/lib/libboost_atomic-mt.dylib
libsdcAngle.dylib: /usr/local/lib/libprotobuf.dylib
libsdcAngle.dylib: /usr/local/Cellar/sdformat3/3.7.0_2/lib/libsdformat.dylib
libsdcAngle.dylib: /usr/local/Cellar/ignition-math2/2.6.0/lib/libignition-math2.dylib
libsdcAngle.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgreMain.dylib
libsdcAngle.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgreTerrain.dylib
libsdcAngle.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgrePaging.dylib
libsdcAngle.dylib: /usr/local/Cellar/ignition-math2/2.6.0/lib/libignition-math2.dylib
libsdcAngle.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgreMain.dylib
libsdcAngle.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgreTerrain.dylib
libsdcAngle.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgrePaging.dylib
libsdcAngle.dylib: CMakeFiles/sdcAngle.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/taoliu/Desktop/SDC-Merged/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libsdcAngle.dylib"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sdcAngle.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/sdcAngle.dir/build: libsdcAngle.dylib

.PHONY : CMakeFiles/sdcAngle.dir/build

CMakeFiles/sdcAngle.dir/requires: CMakeFiles/sdcAngle.dir/sdcAngle.cc.o.requires

.PHONY : CMakeFiles/sdcAngle.dir/requires

CMakeFiles/sdcAngle.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sdcAngle.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sdcAngle.dir/clean

CMakeFiles/sdcAngle.dir/depend:
	cd /Users/taoliu/Desktop/SDC-Merged/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/taoliu/Desktop/SDC-Merged /Users/taoliu/Desktop/SDC-Merged /Users/taoliu/Desktop/SDC-Merged/build /Users/taoliu/Desktop/SDC-Merged/build /Users/taoliu/Desktop/SDC-Merged/build/CMakeFiles/sdcAngle.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sdcAngle.dir/depend

