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
include CMakeFiles/sdcCameraSensor.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/sdcCameraSensor.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sdcCameraSensor.dir/flags.make

CMakeFiles/sdcCameraSensor.dir/sdcCameraSensor.cc.o: CMakeFiles/sdcCameraSensor.dir/flags.make
CMakeFiles/sdcCameraSensor.dir/sdcCameraSensor.cc.o: ../sdcCameraSensor.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/taoliu/Desktop/SDC-Merged/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/sdcCameraSensor.dir/sdcCameraSensor.cc.o"
	/Library/Developer/CommandLineTools/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sdcCameraSensor.dir/sdcCameraSensor.cc.o -c /Users/taoliu/Desktop/SDC-Merged/sdcCameraSensor.cc

CMakeFiles/sdcCameraSensor.dir/sdcCameraSensor.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sdcCameraSensor.dir/sdcCameraSensor.cc.i"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/taoliu/Desktop/SDC-Merged/sdcCameraSensor.cc > CMakeFiles/sdcCameraSensor.dir/sdcCameraSensor.cc.i

CMakeFiles/sdcCameraSensor.dir/sdcCameraSensor.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sdcCameraSensor.dir/sdcCameraSensor.cc.s"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/taoliu/Desktop/SDC-Merged/sdcCameraSensor.cc -o CMakeFiles/sdcCameraSensor.dir/sdcCameraSensor.cc.s

CMakeFiles/sdcCameraSensor.dir/sdcCameraSensor.cc.o.requires:

.PHONY : CMakeFiles/sdcCameraSensor.dir/sdcCameraSensor.cc.o.requires

CMakeFiles/sdcCameraSensor.dir/sdcCameraSensor.cc.o.provides: CMakeFiles/sdcCameraSensor.dir/sdcCameraSensor.cc.o.requires
	$(MAKE) -f CMakeFiles/sdcCameraSensor.dir/build.make CMakeFiles/sdcCameraSensor.dir/sdcCameraSensor.cc.o.provides.build
.PHONY : CMakeFiles/sdcCameraSensor.dir/sdcCameraSensor.cc.o.provides

CMakeFiles/sdcCameraSensor.dir/sdcCameraSensor.cc.o.provides.build: CMakeFiles/sdcCameraSensor.dir/sdcCameraSensor.cc.o


# Object files for target sdcCameraSensor
sdcCameraSensor_OBJECTS = \
"CMakeFiles/sdcCameraSensor.dir/sdcCameraSensor.cc.o"

# External object files for target sdcCameraSensor
sdcCameraSensor_EXTERNAL_OBJECTS =

libsdcCameraSensor.dylib: CMakeFiles/sdcCameraSensor.dir/sdcCameraSensor.cc.o
libsdcCameraSensor.dylib: CMakeFiles/sdcCameraSensor.dir/build.make
libsdcCameraSensor.dylib: libmanager.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_client.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_gui_building.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_gui_viewers.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_gui_model.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_gui.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_sensors.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_rendering.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_selection_buffer.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_physics_bullet.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_physics_simbody.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_physics_ode.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_physics.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_ode.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_transport.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_msgs.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_util.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_common.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_skyx.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_gimpact.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_opcode.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_opende_ou.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_math.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_ccd.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libboost_thread-mt.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libboost_signals-mt.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libboost_system-mt.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libboost_filesystem-mt.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libboost_program_options-mt.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libboost_regex-mt.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libboost_iostreams-mt.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libboost_date_time-mt.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libboost_chrono-mt.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libboost_atomic-mt.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libprotobuf.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/sdformat3/3.7.0_2/lib/libsdformat.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/ignition-math2/2.6.0/lib/libignition-math2.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgreMain.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgreTerrain.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgrePaging.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/ignition-math2/2.6.0/lib/libignition-math2.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libopencv_videostab.2.4.13.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libopencv_ts.a
libsdcCameraSensor.dylib: /usr/local/lib/libopencv_superres.2.4.13.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libopencv_stitching.2.4.13.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libopencv_contrib.2.4.13.dylib
libsdcCameraSensor.dylib: libsdcSensorData.dylib
libsdcCameraSensor.dylib: libsdcVisibleObject.dylib
libsdcCameraSensor.dylib: libsdcLidarRay.dylib
libsdcCameraSensor.dylib: libsdcLidarSensorInfo.dylib
libsdcCameraSensor.dylib: libsdcAngle.dylib
libsdcCameraSensor.dylib: libinstruction.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_client.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_gui_building.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_gui_viewers.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_gui_model.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_gui.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_sensors.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_rendering.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_selection_buffer.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_physics_bullet.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_physics_simbody.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_physics_ode.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_physics.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_ode.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_transport.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_msgs.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_util.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_common.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_skyx.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_gimpact.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_opcode.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_opende_ou.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_math.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/gazebo6/6.6.0_2/lib/libgazebo_ccd.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libboost_thread-mt.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libboost_signals-mt.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libboost_system-mt.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libboost_filesystem-mt.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libboost_program_options-mt.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libboost_regex-mt.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libboost_iostreams-mt.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libboost_date_time-mt.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libboost_chrono-mt.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libboost_atomic-mt.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libprotobuf.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/sdformat3/3.7.0_2/lib/libsdformat.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/ignition-math2/2.6.0/lib/libignition-math2.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgreMain.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgreTerrain.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgrePaging.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/ignition-math2/2.6.0/lib/libignition-math2.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgreMain.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgreTerrain.dylib
libsdcCameraSensor.dylib: /usr/local/Cellar/ogre/1.7.4/lib/libOgrePaging.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libopencv_nonfree.2.4.13.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libopencv_ocl.2.4.13.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libopencv_gpu.2.4.13.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libopencv_photo.2.4.13.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libopencv_objdetect.2.4.13.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libopencv_legacy.2.4.13.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libopencv_video.2.4.13.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libopencv_ml.2.4.13.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libopencv_calib3d.2.4.13.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libopencv_features2d.2.4.13.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libopencv_highgui.2.4.13.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libopencv_imgproc.2.4.13.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libopencv_flann.2.4.13.dylib
libsdcCameraSensor.dylib: /usr/local/lib/libopencv_core.2.4.13.dylib
libsdcCameraSensor.dylib: CMakeFiles/sdcCameraSensor.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/taoliu/Desktop/SDC-Merged/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libsdcCameraSensor.dylib"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sdcCameraSensor.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/sdcCameraSensor.dir/build: libsdcCameraSensor.dylib

.PHONY : CMakeFiles/sdcCameraSensor.dir/build

CMakeFiles/sdcCameraSensor.dir/requires: CMakeFiles/sdcCameraSensor.dir/sdcCameraSensor.cc.o.requires

.PHONY : CMakeFiles/sdcCameraSensor.dir/requires

CMakeFiles/sdcCameraSensor.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sdcCameraSensor.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sdcCameraSensor.dir/clean

CMakeFiles/sdcCameraSensor.dir/depend:
	cd /Users/taoliu/Desktop/SDC-Merged/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/taoliu/Desktop/SDC-Merged /Users/taoliu/Desktop/SDC-Merged /Users/taoliu/Desktop/SDC-Merged/build /Users/taoliu/Desktop/SDC-Merged/build /Users/taoliu/Desktop/SDC-Merged/build/CMakeFiles/sdcCameraSensor.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sdcCameraSensor.dir/depend

