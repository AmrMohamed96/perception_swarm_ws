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
CMAKE_SOURCE_DIR = /home/amr/perception_ws/src/apriltag

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/amr/perception_ws/build/apriltag

# Include any dependencies generated for this target.
include CMakeFiles/opencv_demo.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/opencv_demo.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/opencv_demo.dir/flags.make

CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.o: CMakeFiles/opencv_demo.dir/flags.make
CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.o: /home/amr/perception_ws/src/apriltag/example/opencv_demo.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/amr/perception_ws/build/apriltag/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.o -c /home/amr/perception_ws/src/apriltag/example/opencv_demo.cc

CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/amr/perception_ws/src/apriltag/example/opencv_demo.cc > CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.i

CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/amr/perception_ws/src/apriltag/example/opencv_demo.cc -o CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.s

CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.o.requires:

.PHONY : CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.o.requires

CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.o.provides: CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.o.requires
	$(MAKE) -f CMakeFiles/opencv_demo.dir/build.make CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.o.provides.build
.PHONY : CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.o.provides

CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.o.provides.build: CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.o


# Object files for target opencv_demo
opencv_demo_OBJECTS = \
"CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.o"

# External object files for target opencv_demo
opencv_demo_EXTERNAL_OBJECTS =

opencv_demo: CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.o
opencv_demo: CMakeFiles/opencv_demo.dir/build.make
opencv_demo: lib/libapriltag.so.3.1.0
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stitching3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_superres3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videostab3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_aruco3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bgsegm3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bioinspired3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ccalib3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_cvv3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dpm3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_face3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_fuzzy3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_hdf3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_img_hash3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_line_descriptor3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_optflow3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_reg3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_rgbd3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_saliency3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stereo3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_structured_light3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_surface_matching3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_tracking3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xfeatures2d3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ximgproc3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xobjdetect3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xphoto3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_shape3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_photo3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_datasets3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_plot3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_text3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dnn3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ml3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_video3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_calib3d3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_features2d3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_highgui3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videoio3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_viz3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_phase_unwrapping3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_flann3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_objdetect3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
opencv_demo: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
opencv_demo: CMakeFiles/opencv_demo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/amr/perception_ws/build/apriltag/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable opencv_demo"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/opencv_demo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/opencv_demo.dir/build: opencv_demo

.PHONY : CMakeFiles/opencv_demo.dir/build

CMakeFiles/opencv_demo.dir/requires: CMakeFiles/opencv_demo.dir/example/opencv_demo.cc.o.requires

.PHONY : CMakeFiles/opencv_demo.dir/requires

CMakeFiles/opencv_demo.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/opencv_demo.dir/cmake_clean.cmake
.PHONY : CMakeFiles/opencv_demo.dir/clean

CMakeFiles/opencv_demo.dir/depend:
	cd /home/amr/perception_ws/build/apriltag && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/amr/perception_ws/src/apriltag /home/amr/perception_ws/src/apriltag /home/amr/perception_ws/build/apriltag /home/amr/perception_ws/build/apriltag /home/amr/perception_ws/build/apriltag/CMakeFiles/opencv_demo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/opencv_demo.dir/depend

