# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.6

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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.6.2/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.6.2/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Accounts/mawbye/Desktop/SDC-Merged

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Accounts/mawbye/Desktop/SDC-Merged/build

# Include any dependencies generated for this target.
include CMakeFiles/sdcManager.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/sdcManager.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sdcManager.dir/flags.make

CMakeFiles/sdcManager.dir/sdcManager.cc.o: CMakeFiles/sdcManager.dir/flags.make
CMakeFiles/sdcManager.dir/sdcManager.cc.o: ../sdcManager.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Accounts/mawbye/Desktop/SDC-Merged/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/sdcManager.dir/sdcManager.cc.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sdcManager.dir/sdcManager.cc.o -c /Accounts/mawbye/Desktop/SDC-Merged/sdcManager.cc

CMakeFiles/sdcManager.dir/sdcManager.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sdcManager.dir/sdcManager.cc.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Accounts/mawbye/Desktop/SDC-Merged/sdcManager.cc > CMakeFiles/sdcManager.dir/sdcManager.cc.i

CMakeFiles/sdcManager.dir/sdcManager.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sdcManager.dir/sdcManager.cc.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Accounts/mawbye/Desktop/SDC-Merged/sdcManager.cc -o CMakeFiles/sdcManager.dir/sdcManager.cc.s

CMakeFiles/sdcManager.dir/sdcManager.cc.o.requires:

.PHONY : CMakeFiles/sdcManager.dir/sdcManager.cc.o.requires

CMakeFiles/sdcManager.dir/sdcManager.cc.o.provides: CMakeFiles/sdcManager.dir/sdcManager.cc.o.requires
	$(MAKE) -f CMakeFiles/sdcManager.dir/build.make CMakeFiles/sdcManager.dir/sdcManager.cc.o.provides.build
.PHONY : CMakeFiles/sdcManager.dir/sdcManager.cc.o.provides

CMakeFiles/sdcManager.dir/sdcManager.cc.o.provides.build: CMakeFiles/sdcManager.dir/sdcManager.cc.o


# Object files for target sdcManager
sdcManager_OBJECTS = \
"CMakeFiles/sdcManager.dir/sdcManager.cc.o"

# External object files for target sdcManager
sdcManager_EXTERNAL_OBJECTS =

libsdcManager.dylib: CMakeFiles/sdcManager.dir/sdcManager.cc.o
libsdcManager.dylib: CMakeFiles/sdcManager.dir/build.make
libsdcManager.dylib: libmanager.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_client.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_gui_building.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_gui_viewers.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_gui_model.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_gui.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_sensors.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_rendering.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_selection_buffer.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_physics_bullet.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_physics_simbody.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_physics_ode.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_physics.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_ode.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_transport.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_msgs.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_util.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_common.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_skyx.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_gimpact.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_opcode.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_opende_ou.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_math.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_ccd.dylib
libsdcManager.dylib: /usr/local/lib/libboost_thread-mt.dylib
libsdcManager.dylib: /usr/local/lib/libboost_signals-mt.dylib
libsdcManager.dylib: /usr/local/lib/libboost_system-mt.dylib
libsdcManager.dylib: /usr/local/lib/libboost_filesystem-mt.dylib
libsdcManager.dylib: /usr/local/lib/libboost_program_options-mt.dylib
libsdcManager.dylib: /usr/local/lib/libboost_regex-mt.dylib
libsdcManager.dylib: /usr/local/lib/libboost_iostreams-mt.dylib
libsdcManager.dylib: /usr/local/lib/libboost_date_time-mt.dylib
libsdcManager.dylib: /usr/local/lib/libboost_chrono-mt.dylib
libsdcManager.dylib: /usr/local/lib/libboost_atomic-mt.dylib
libsdcManager.dylib: /usr/local/lib/libprotobuf.dylib
libsdcManager.dylib: /usr/local/Cellar/sdformat3/3.7.0/lib/libsdformat.dylib
libsdcManager.dylib: /usr/local/Cellar/ignition-math2/2.5.0/lib/libignition-math2.dylib
libsdcManager.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgreMain.dylib
libsdcManager.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgreTerrain.dylib
libsdcManager.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgrePaging.dylib
libsdcManager.dylib: /usr/local/Cellar/ignition-math2/2.5.0/lib/libignition-math2.dylib
libsdcManager.dylib: libinstruction.dylib
libsdcManager.dylib: librequest.dylib
libsdcManager.dylib: libsdcSensorData.dylib
libsdcManager.dylib: libsdcVisibleObject.dylib
libsdcManager.dylib: libsdcLidarRay.dylib
libsdcManager.dylib: libsdcLidarSensorInfo.dylib
libsdcManager.dylib: libsdcAngle.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_client.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_gui_building.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_gui_viewers.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_gui_model.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_gui.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_sensors.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_rendering.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_selection_buffer.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_physics_bullet.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_physics_simbody.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_physics_ode.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_physics.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_ode.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_transport.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_msgs.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_util.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_common.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_skyx.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_gimpact.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_opcode.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_opende_ou.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_math.dylib
libsdcManager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_ccd.dylib
libsdcManager.dylib: /usr/local/lib/libboost_thread-mt.dylib
libsdcManager.dylib: /usr/local/lib/libboost_signals-mt.dylib
libsdcManager.dylib: /usr/local/lib/libboost_system-mt.dylib
libsdcManager.dylib: /usr/local/lib/libboost_filesystem-mt.dylib
libsdcManager.dylib: /usr/local/lib/libboost_program_options-mt.dylib
libsdcManager.dylib: /usr/local/lib/libboost_regex-mt.dylib
libsdcManager.dylib: /usr/local/lib/libboost_iostreams-mt.dylib
libsdcManager.dylib: /usr/local/lib/libboost_date_time-mt.dylib
libsdcManager.dylib: /usr/local/lib/libboost_chrono-mt.dylib
libsdcManager.dylib: /usr/local/lib/libboost_atomic-mt.dylib
libsdcManager.dylib: /usr/local/lib/libprotobuf.dylib
libsdcManager.dylib: /usr/local/Cellar/sdformat3/3.7.0/lib/libsdformat.dylib
libsdcManager.dylib: /usr/local/Cellar/ignition-math2/2.5.0/lib/libignition-math2.dylib
libsdcManager.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgreMain.dylib
libsdcManager.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgreTerrain.dylib
libsdcManager.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgrePaging.dylib
libsdcManager.dylib: /usr/local/Cellar/ignition-math2/2.5.0/lib/libignition-math2.dylib
libsdcManager.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgreMain.dylib
libsdcManager.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgreTerrain.dylib
libsdcManager.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgrePaging.dylib
libsdcManager.dylib: CMakeFiles/sdcManager.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Accounts/mawbye/Desktop/SDC-Merged/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libsdcManager.dylib"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sdcManager.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/sdcManager.dir/build: libsdcManager.dylib

.PHONY : CMakeFiles/sdcManager.dir/build

CMakeFiles/sdcManager.dir/requires: CMakeFiles/sdcManager.dir/sdcManager.cc.o.requires

.PHONY : CMakeFiles/sdcManager.dir/requires

CMakeFiles/sdcManager.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sdcManager.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sdcManager.dir/clean

CMakeFiles/sdcManager.dir/depend:
	cd /Accounts/mawbye/Desktop/SDC-Merged/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Accounts/mawbye/Desktop/SDC-Merged /Accounts/mawbye/Desktop/SDC-Merged /Accounts/mawbye/Desktop/SDC-Merged/build /Accounts/mawbye/Desktop/SDC-Merged/build /Accounts/mawbye/Desktop/SDC-Merged/build/CMakeFiles/sdcManager.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sdcManager.dir/depend

