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
CMAKE_SOURCE_DIR = /home/sakson/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sakson/catkin_ws/build

# Include any dependencies generated for this target.
include goalline/CMakeFiles/goalline.dir/depend.make

# Include the progress variables for this target.
include goalline/CMakeFiles/goalline.dir/progress.make

# Include the compile flags for this target's objects.
include goalline/CMakeFiles/goalline.dir/flags.make

goalline/CMakeFiles/goalline.dir/src/goline.cpp.o: goalline/CMakeFiles/goalline.dir/flags.make
goalline/CMakeFiles/goalline.dir/src/goline.cpp.o: /home/sakson/catkin_ws/src/goalline/src/goline.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sakson/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object goalline/CMakeFiles/goalline.dir/src/goline.cpp.o"
	cd /home/sakson/catkin_ws/build/goalline && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/goalline.dir/src/goline.cpp.o -c /home/sakson/catkin_ws/src/goalline/src/goline.cpp

goalline/CMakeFiles/goalline.dir/src/goline.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/goalline.dir/src/goline.cpp.i"
	cd /home/sakson/catkin_ws/build/goalline && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sakson/catkin_ws/src/goalline/src/goline.cpp > CMakeFiles/goalline.dir/src/goline.cpp.i

goalline/CMakeFiles/goalline.dir/src/goline.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/goalline.dir/src/goline.cpp.s"
	cd /home/sakson/catkin_ws/build/goalline && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sakson/catkin_ws/src/goalline/src/goline.cpp -o CMakeFiles/goalline.dir/src/goline.cpp.s

goalline/CMakeFiles/goalline.dir/src/goline.cpp.o.requires:

.PHONY : goalline/CMakeFiles/goalline.dir/src/goline.cpp.o.requires

goalline/CMakeFiles/goalline.dir/src/goline.cpp.o.provides: goalline/CMakeFiles/goalline.dir/src/goline.cpp.o.requires
	$(MAKE) -f goalline/CMakeFiles/goalline.dir/build.make goalline/CMakeFiles/goalline.dir/src/goline.cpp.o.provides.build
.PHONY : goalline/CMakeFiles/goalline.dir/src/goline.cpp.o.provides

goalline/CMakeFiles/goalline.dir/src/goline.cpp.o.provides.build: goalline/CMakeFiles/goalline.dir/src/goline.cpp.o


# Object files for target goalline
goalline_OBJECTS = \
"CMakeFiles/goalline.dir/src/goline.cpp.o"

# External object files for target goalline
goalline_EXTERNAL_OBJECTS =

/home/sakson/catkin_ws/devel/lib/goalline/goalline: goalline/CMakeFiles/goalline.dir/src/goline.cpp.o
/home/sakson/catkin_ws/devel/lib/goalline/goalline: goalline/CMakeFiles/goalline.dir/build.make
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /opt/ros/melodic/lib/libactionlib.so
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /opt/ros/melodic/lib/libroscpp.so
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /opt/ros/melodic/lib/librosconsole.so
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/lib/aarch64-linux-gnu/libboost_regex.so
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /opt/ros/melodic/lib/librostime.so
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /opt/ros/melodic/lib/libcpp_common.so
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/lib/aarch64-linux-gnu/libboost_system.so
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/lib/aarch64-linux-gnu/libboost_thread.so
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_gapi.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_stitching.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_alphamat.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_aruco.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_bgsegm.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_bioinspired.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_ccalib.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_dnn_objdetect.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_dnn_superres.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_dpm.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_face.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_freetype.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_fuzzy.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_hdf.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_hfs.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_img_hash.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_intensity_transform.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_line_descriptor.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_mcc.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_quality.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_rapid.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_reg.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_rgbd.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_saliency.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_signal.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_stereo.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_structured_light.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_superres.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_surface_matching.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_tracking.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_videostab.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_viz.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_wechat_qrcode.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_xfeatures2d.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_xobjdetect.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_xphoto.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/lib/aarch64-linux-gnu/libcurl.so
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_shape.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_highgui.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_datasets.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_plot.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_text.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_ml.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_phase_unwrapping.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_optflow.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_ximgproc.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_video.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_videoio.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_imgcodecs.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_objdetect.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_calib3d.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_dnn.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_features2d.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_flann.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_photo.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_imgproc.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: /usr/local/lib/libopencv_core.so.4.10.0
/home/sakson/catkin_ws/devel/lib/goalline/goalline: goalline/CMakeFiles/goalline.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/sakson/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/sakson/catkin_ws/devel/lib/goalline/goalline"
	cd /home/sakson/catkin_ws/build/goalline && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/goalline.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
goalline/CMakeFiles/goalline.dir/build: /home/sakson/catkin_ws/devel/lib/goalline/goalline

.PHONY : goalline/CMakeFiles/goalline.dir/build

goalline/CMakeFiles/goalline.dir/requires: goalline/CMakeFiles/goalline.dir/src/goline.cpp.o.requires

.PHONY : goalline/CMakeFiles/goalline.dir/requires

goalline/CMakeFiles/goalline.dir/clean:
	cd /home/sakson/catkin_ws/build/goalline && $(CMAKE_COMMAND) -P CMakeFiles/goalline.dir/cmake_clean.cmake
.PHONY : goalline/CMakeFiles/goalline.dir/clean

goalline/CMakeFiles/goalline.dir/depend:
	cd /home/sakson/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sakson/catkin_ws/src /home/sakson/catkin_ws/src/goalline /home/sakson/catkin_ws/build /home/sakson/catkin_ws/build/goalline /home/sakson/catkin_ws/build/goalline/CMakeFiles/goalline.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : goalline/CMakeFiles/goalline.dir/depend

