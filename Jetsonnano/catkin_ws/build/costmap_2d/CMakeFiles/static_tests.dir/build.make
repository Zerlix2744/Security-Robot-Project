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
include costmap_2d/CMakeFiles/static_tests.dir/depend.make

# Include the progress variables for this target.
include costmap_2d/CMakeFiles/static_tests.dir/progress.make

# Include the compile flags for this target's objects.
include costmap_2d/CMakeFiles/static_tests.dir/flags.make

costmap_2d/CMakeFiles/static_tests.dir/test/static_tests.cpp.o: costmap_2d/CMakeFiles/static_tests.dir/flags.make
costmap_2d/CMakeFiles/static_tests.dir/test/static_tests.cpp.o: /home/sakson/catkin_ws/src/costmap_2d/test/static_tests.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sakson/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object costmap_2d/CMakeFiles/static_tests.dir/test/static_tests.cpp.o"
	cd /home/sakson/catkin_ws/build/costmap_2d && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/static_tests.dir/test/static_tests.cpp.o -c /home/sakson/catkin_ws/src/costmap_2d/test/static_tests.cpp

costmap_2d/CMakeFiles/static_tests.dir/test/static_tests.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/static_tests.dir/test/static_tests.cpp.i"
	cd /home/sakson/catkin_ws/build/costmap_2d && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sakson/catkin_ws/src/costmap_2d/test/static_tests.cpp > CMakeFiles/static_tests.dir/test/static_tests.cpp.i

costmap_2d/CMakeFiles/static_tests.dir/test/static_tests.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/static_tests.dir/test/static_tests.cpp.s"
	cd /home/sakson/catkin_ws/build/costmap_2d && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sakson/catkin_ws/src/costmap_2d/test/static_tests.cpp -o CMakeFiles/static_tests.dir/test/static_tests.cpp.s

costmap_2d/CMakeFiles/static_tests.dir/test/static_tests.cpp.o.requires:

.PHONY : costmap_2d/CMakeFiles/static_tests.dir/test/static_tests.cpp.o.requires

costmap_2d/CMakeFiles/static_tests.dir/test/static_tests.cpp.o.provides: costmap_2d/CMakeFiles/static_tests.dir/test/static_tests.cpp.o.requires
	$(MAKE) -f costmap_2d/CMakeFiles/static_tests.dir/build.make costmap_2d/CMakeFiles/static_tests.dir/test/static_tests.cpp.o.provides.build
.PHONY : costmap_2d/CMakeFiles/static_tests.dir/test/static_tests.cpp.o.provides

costmap_2d/CMakeFiles/static_tests.dir/test/static_tests.cpp.o.provides.build: costmap_2d/CMakeFiles/static_tests.dir/test/static_tests.cpp.o


# Object files for target static_tests
static_tests_OBJECTS = \
"CMakeFiles/static_tests.dir/test/static_tests.cpp.o"

# External object files for target static_tests
static_tests_EXTERNAL_OBJECTS =

/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: costmap_2d/CMakeFiles/static_tests.dir/test/static_tests.cpp.o
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: costmap_2d/CMakeFiles/static_tests.dir/build.make
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /home/sakson/catkin_ws/devel/lib/liblayers.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: gtest/googlemock/gtest/libgtest.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /home/sakson/catkin_ws/devel/lib/libcostmap_2d.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /usr/lib/aarch64-linux-gnu/libboost_system.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /usr/lib/aarch64-linux-gnu/libboost_thread.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libdynamic_reconfigure_config_init_mutex.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/liblaser_geometry.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libtf.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libclass_loader.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /usr/lib/libPocoFoundation.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /usr/lib/aarch64-linux-gnu/libdl.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libroslib.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/librospack.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /usr/lib/aarch64-linux-gnu/libpython2.7.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /usr/lib/aarch64-linux-gnu/libboost_program_options.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /usr/lib/aarch64-linux-gnu/libtinyxml2.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/liborocos-kdl.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/liborocos-kdl.so.1.4.0
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libtf2_ros.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libactionlib.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libmessage_filters.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libtf2.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libvoxel_grid.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libroscpp.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/librosconsole.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /usr/lib/aarch64-linux-gnu/libboost_regex.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/librostime.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libcpp_common.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /usr/lib/aarch64-linux-gnu/libboost_system.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /usr/lib/aarch64-linux-gnu/libboost_thread.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libdynamic_reconfigure_config_init_mutex.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/liblaser_geometry.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libtf.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libclass_loader.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /usr/lib/libPocoFoundation.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /usr/lib/aarch64-linux-gnu/libdl.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libroslib.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/librospack.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /usr/lib/aarch64-linux-gnu/libpython2.7.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /usr/lib/aarch64-linux-gnu/libboost_program_options.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /usr/lib/aarch64-linux-gnu/libtinyxml2.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/liborocos-kdl.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/liborocos-kdl.so.1.4.0
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libtf2_ros.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libactionlib.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libmessage_filters.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libtf2.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libvoxel_grid.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libroscpp.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/librosconsole.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /usr/lib/aarch64-linux-gnu/libboost_regex.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/librostime.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /opt/ros/melodic/lib/libcpp_common.so
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests: costmap_2d/CMakeFiles/static_tests.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/sakson/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests"
	cd /home/sakson/catkin_ws/build/costmap_2d && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/static_tests.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
costmap_2d/CMakeFiles/static_tests.dir/build: /home/sakson/catkin_ws/devel/lib/costmap_2d/static_tests

.PHONY : costmap_2d/CMakeFiles/static_tests.dir/build

costmap_2d/CMakeFiles/static_tests.dir/requires: costmap_2d/CMakeFiles/static_tests.dir/test/static_tests.cpp.o.requires

.PHONY : costmap_2d/CMakeFiles/static_tests.dir/requires

costmap_2d/CMakeFiles/static_tests.dir/clean:
	cd /home/sakson/catkin_ws/build/costmap_2d && $(CMAKE_COMMAND) -P CMakeFiles/static_tests.dir/cmake_clean.cmake
.PHONY : costmap_2d/CMakeFiles/static_tests.dir/clean

costmap_2d/CMakeFiles/static_tests.dir/depend:
	cd /home/sakson/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sakson/catkin_ws/src /home/sakson/catkin_ws/src/costmap_2d /home/sakson/catkin_ws/build /home/sakson/catkin_ws/build/costmap_2d /home/sakson/catkin_ws/build/costmap_2d/CMakeFiles/static_tests.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : costmap_2d/CMakeFiles/static_tests.dir/depend
