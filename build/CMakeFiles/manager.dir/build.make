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
include CMakeFiles/manager.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/manager.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/manager.dir/flags.make

CMakeFiles/manager.dir/manager.cc.o: CMakeFiles/manager.dir/flags.make
CMakeFiles/manager.dir/manager.cc.o: ../manager.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Accounts/mawbye/Desktop/SDC-Merged/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/manager.dir/manager.cc.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/manager.dir/manager.cc.o -c /Accounts/mawbye/Desktop/SDC-Merged/manager.cc

CMakeFiles/manager.dir/manager.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/manager.dir/manager.cc.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Accounts/mawbye/Desktop/SDC-Merged/manager.cc > CMakeFiles/manager.dir/manager.cc.i

CMakeFiles/manager.dir/manager.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/manager.dir/manager.cc.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Accounts/mawbye/Desktop/SDC-Merged/manager.cc -o CMakeFiles/manager.dir/manager.cc.s

CMakeFiles/manager.dir/manager.cc.o.requires:

.PHONY : CMakeFiles/manager.dir/manager.cc.o.requires

CMakeFiles/manager.dir/manager.cc.o.provides: CMakeFiles/manager.dir/manager.cc.o.requires
	$(MAKE) -f CMakeFiles/manager.dir/build.make CMakeFiles/manager.dir/manager.cc.o.provides.build
.PHONY : CMakeFiles/manager.dir/manager.cc.o.provides

CMakeFiles/manager.dir/manager.cc.o.provides.build: CMakeFiles/manager.dir/manager.cc.o


# Object files for target manager
manager_OBJECTS = \
"CMakeFiles/manager.dir/manager.cc.o"

# External object files for target manager
manager_EXTERNAL_OBJECTS =

libmanager.dylib: CMakeFiles/manager.dir/manager.cc.o
libmanager.dylib: CMakeFiles/manager.dir/build.make
libmanager.dylib: libinstruction.dylib
libmanager.dylib: librequest.dylib
libmanager.dylib: libsdcSensorData.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_client.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_gui_building.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_gui_viewers.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_gui_model.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_gui.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_sensors.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_rendering.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_selection_buffer.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_physics_bullet.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_physics_simbody.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_physics_ode.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_physics.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_ode.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_transport.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_msgs.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_util.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_common.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_skyx.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_gimpact.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_opcode.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_opende_ou.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_math.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_ccd.dylib
libmanager.dylib: /usr/local/lib/libboost_thread-mt.dylib
libmanager.dylib: /usr/local/lib/libboost_signals-mt.dylib
libmanager.dylib: /usr/local/lib/libboost_system-mt.dylib
libmanager.dylib: /usr/local/lib/libboost_filesystem-mt.dylib
libmanager.dylib: /usr/local/lib/libboost_program_options-mt.dylib
libmanager.dylib: /usr/local/lib/libboost_regex-mt.dylib
libmanager.dylib: /usr/local/lib/libboost_iostreams-mt.dylib
libmanager.dylib: /usr/local/lib/libboost_date_time-mt.dylib
libmanager.dylib: /usr/local/lib/libboost_chrono-mt.dylib
libmanager.dylib: /usr/local/lib/libboost_atomic-mt.dylib
libmanager.dylib: /usr/local/lib/libprotobuf.dylib
libmanager.dylib: /usr/local/Cellar/sdformat3/3.7.0/lib/libsdformat.dylib
libmanager.dylib: /usr/local/Cellar/ignition-math2/2.5.0/lib/libignition-math2.dylib
libmanager.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgreMain.dylib
libmanager.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgreTerrain.dylib
libmanager.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgrePaging.dylib
libmanager.dylib: /usr/local/Cellar/ignition-math2/2.5.0/lib/libignition-math2.dylib
libmanager.dylib: libsdcVisibleObject.dylib
libmanager.dylib: libsdcLidarRay.dylib
libmanager.dylib: libsdcLidarSensorInfo.dylib
libmanager.dylib: libsdcAngle.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_client.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_gui_building.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_gui_viewers.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_gui_model.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_gui.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_sensors.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_rendering.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_selection_buffer.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_physics_bullet.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_physics_simbody.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_physics_ode.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_physics.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_ode.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_transport.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_msgs.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_util.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_common.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_skyx.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_gimpact.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_opcode.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_opende_ou.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_math.dylib
libmanager.dylib: /usr/local/Cellar/gazebo6/6.6.0/lib/libgazebo_ccd.dylib
libmanager.dylib: /usr/local/lib/libboost_thread-mt.dylib
libmanager.dylib: /usr/local/lib/libboost_signals-mt.dylib
libmanager.dylib: /usr/local/lib/libboost_system-mt.dylib
libmanager.dylib: /usr/local/lib/libboost_filesystem-mt.dylib
libmanager.dylib: /usr/local/lib/libboost_program_options-mt.dylib
libmanager.dylib: /usr/local/lib/libboost_regex-mt.dylib
libmanager.dylib: /usr/local/lib/libboost_iostreams-mt.dylib
libmanager.dylib: /usr/local/lib/libboost_date_time-mt.dylib
libmanager.dylib: /usr/local/lib/libboost_chrono-mt.dylib
libmanager.dylib: /usr/local/lib/libboost_atomic-mt.dylib
libmanager.dylib: /usr/local/lib/libprotobuf.dylib
libmanager.dylib: /usr/local/Cellar/sdformat3/3.7.0/lib/libsdformat.dylib
libmanager.dylib: /usr/local/Cellar/ignition-math2/2.5.0/lib/libignition-math2.dylib
libmanager.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgreMain.dylib
libmanager.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgreTerrain.dylib
libmanager.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgrePaging.dylib
libmanager.dylib: /usr/local/Cellar/ignition-math2/2.5.0/lib/libignition-math2.dylib
libmanager.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgreMain.dylib
libmanager.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgreTerrain.dylib
libmanager.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgrePaging.dylib
libmanager.dylib: CMakeFiles/manager.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Accounts/mawbye/Desktop/SDC-Merged/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libmanager.dylib"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/manager.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/manager.dir/build: libmanager.dylib

.PHONY : CMakeFiles/manager.dir/build

CMakeFiles/manager.dir/requires: CMakeFiles/manager.dir/manager.cc.o.requires

.PHONY : CMakeFiles/manager.dir/requires

CMakeFiles/manager.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/manager.dir/cmake_clean.cmake
.PHONY : CMakeFiles/manager.dir/clean

CMakeFiles/manager.dir/depend:
	cd /Accounts/mawbye/Desktop/SDC-Merged/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Accounts/mawbye/Desktop/SDC-Merged /Accounts/mawbye/Desktop/SDC-Merged /Accounts/mawbye/Desktop/SDC-Merged/build /Accounts/mawbye/Desktop/SDC-Merged/build /Accounts/mawbye/Desktop/SDC-Merged/build/CMakeFiles/manager.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/manager.dir/depend

