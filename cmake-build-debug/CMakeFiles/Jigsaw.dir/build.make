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
CMAKE_COMMAND = /Applications/CLion.app/Contents/bin/cmake/bin/cmake

# The command to remove a file.
RM = /Applications/CLion.app/Contents/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/summer/ClionProjects/Jigsaw

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/summer/ClionProjects/Jigsaw/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/Jigsaw.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Jigsaw.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Jigsaw.dir/flags.make

CMakeFiles/Jigsaw.dir/main.cpp.o: CMakeFiles/Jigsaw.dir/flags.make
CMakeFiles/Jigsaw.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/summer/ClionProjects/Jigsaw/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Jigsaw.dir/main.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Jigsaw.dir/main.cpp.o -c /Users/summer/ClionProjects/Jigsaw/main.cpp

CMakeFiles/Jigsaw.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Jigsaw.dir/main.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/summer/ClionProjects/Jigsaw/main.cpp > CMakeFiles/Jigsaw.dir/main.cpp.i

CMakeFiles/Jigsaw.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Jigsaw.dir/main.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/summer/ClionProjects/Jigsaw/main.cpp -o CMakeFiles/Jigsaw.dir/main.cpp.s

CMakeFiles/Jigsaw.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/Jigsaw.dir/main.cpp.o.requires

CMakeFiles/Jigsaw.dir/main.cpp.o.provides: CMakeFiles/Jigsaw.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/Jigsaw.dir/build.make CMakeFiles/Jigsaw.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/Jigsaw.dir/main.cpp.o.provides

CMakeFiles/Jigsaw.dir/main.cpp.o.provides.build: CMakeFiles/Jigsaw.dir/main.cpp.o


CMakeFiles/Jigsaw.dir/filter/Point_filter.cpp.o: CMakeFiles/Jigsaw.dir/flags.make
CMakeFiles/Jigsaw.dir/filter/Point_filter.cpp.o: ../filter/Point_filter.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/summer/ClionProjects/Jigsaw/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Jigsaw.dir/filter/Point_filter.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Jigsaw.dir/filter/Point_filter.cpp.o -c /Users/summer/ClionProjects/Jigsaw/filter/Point_filter.cpp

CMakeFiles/Jigsaw.dir/filter/Point_filter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Jigsaw.dir/filter/Point_filter.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/summer/ClionProjects/Jigsaw/filter/Point_filter.cpp > CMakeFiles/Jigsaw.dir/filter/Point_filter.cpp.i

CMakeFiles/Jigsaw.dir/filter/Point_filter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Jigsaw.dir/filter/Point_filter.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/summer/ClionProjects/Jigsaw/filter/Point_filter.cpp -o CMakeFiles/Jigsaw.dir/filter/Point_filter.cpp.s

CMakeFiles/Jigsaw.dir/filter/Point_filter.cpp.o.requires:

.PHONY : CMakeFiles/Jigsaw.dir/filter/Point_filter.cpp.o.requires

CMakeFiles/Jigsaw.dir/filter/Point_filter.cpp.o.provides: CMakeFiles/Jigsaw.dir/filter/Point_filter.cpp.o.requires
	$(MAKE) -f CMakeFiles/Jigsaw.dir/build.make CMakeFiles/Jigsaw.dir/filter/Point_filter.cpp.o.provides.build
.PHONY : CMakeFiles/Jigsaw.dir/filter/Point_filter.cpp.o.provides

CMakeFiles/Jigsaw.dir/filter/Point_filter.cpp.o.provides.build: CMakeFiles/Jigsaw.dir/filter/Point_filter.cpp.o


CMakeFiles/Jigsaw.dir/keypoint/keypoint.cpp.o: CMakeFiles/Jigsaw.dir/flags.make
CMakeFiles/Jigsaw.dir/keypoint/keypoint.cpp.o: ../keypoint/keypoint.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/summer/ClionProjects/Jigsaw/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/Jigsaw.dir/keypoint/keypoint.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Jigsaw.dir/keypoint/keypoint.cpp.o -c /Users/summer/ClionProjects/Jigsaw/keypoint/keypoint.cpp

CMakeFiles/Jigsaw.dir/keypoint/keypoint.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Jigsaw.dir/keypoint/keypoint.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/summer/ClionProjects/Jigsaw/keypoint/keypoint.cpp > CMakeFiles/Jigsaw.dir/keypoint/keypoint.cpp.i

CMakeFiles/Jigsaw.dir/keypoint/keypoint.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Jigsaw.dir/keypoint/keypoint.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/summer/ClionProjects/Jigsaw/keypoint/keypoint.cpp -o CMakeFiles/Jigsaw.dir/keypoint/keypoint.cpp.s

CMakeFiles/Jigsaw.dir/keypoint/keypoint.cpp.o.requires:

.PHONY : CMakeFiles/Jigsaw.dir/keypoint/keypoint.cpp.o.requires

CMakeFiles/Jigsaw.dir/keypoint/keypoint.cpp.o.provides: CMakeFiles/Jigsaw.dir/keypoint/keypoint.cpp.o.requires
	$(MAKE) -f CMakeFiles/Jigsaw.dir/build.make CMakeFiles/Jigsaw.dir/keypoint/keypoint.cpp.o.provides.build
.PHONY : CMakeFiles/Jigsaw.dir/keypoint/keypoint.cpp.o.provides

CMakeFiles/Jigsaw.dir/keypoint/keypoint.cpp.o.provides.build: CMakeFiles/Jigsaw.dir/keypoint/keypoint.cpp.o


CMakeFiles/Jigsaw.dir/featrue_align/FeatrueAlign.cpp.o: CMakeFiles/Jigsaw.dir/flags.make
CMakeFiles/Jigsaw.dir/featrue_align/FeatrueAlign.cpp.o: ../featrue_align/FeatrueAlign.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/summer/ClionProjects/Jigsaw/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/Jigsaw.dir/featrue_align/FeatrueAlign.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Jigsaw.dir/featrue_align/FeatrueAlign.cpp.o -c /Users/summer/ClionProjects/Jigsaw/featrue_align/FeatrueAlign.cpp

CMakeFiles/Jigsaw.dir/featrue_align/FeatrueAlign.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Jigsaw.dir/featrue_align/FeatrueAlign.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/summer/ClionProjects/Jigsaw/featrue_align/FeatrueAlign.cpp > CMakeFiles/Jigsaw.dir/featrue_align/FeatrueAlign.cpp.i

CMakeFiles/Jigsaw.dir/featrue_align/FeatrueAlign.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Jigsaw.dir/featrue_align/FeatrueAlign.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/summer/ClionProjects/Jigsaw/featrue_align/FeatrueAlign.cpp -o CMakeFiles/Jigsaw.dir/featrue_align/FeatrueAlign.cpp.s

CMakeFiles/Jigsaw.dir/featrue_align/FeatrueAlign.cpp.o.requires:

.PHONY : CMakeFiles/Jigsaw.dir/featrue_align/FeatrueAlign.cpp.o.requires

CMakeFiles/Jigsaw.dir/featrue_align/FeatrueAlign.cpp.o.provides: CMakeFiles/Jigsaw.dir/featrue_align/FeatrueAlign.cpp.o.requires
	$(MAKE) -f CMakeFiles/Jigsaw.dir/build.make CMakeFiles/Jigsaw.dir/featrue_align/FeatrueAlign.cpp.o.provides.build
.PHONY : CMakeFiles/Jigsaw.dir/featrue_align/FeatrueAlign.cpp.o.provides

CMakeFiles/Jigsaw.dir/featrue_align/FeatrueAlign.cpp.o.provides.build: CMakeFiles/Jigsaw.dir/featrue_align/FeatrueAlign.cpp.o


CMakeFiles/Jigsaw.dir/io/LoadData.cpp.o: CMakeFiles/Jigsaw.dir/flags.make
CMakeFiles/Jigsaw.dir/io/LoadData.cpp.o: ../io/LoadData.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/summer/ClionProjects/Jigsaw/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/Jigsaw.dir/io/LoadData.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Jigsaw.dir/io/LoadData.cpp.o -c /Users/summer/ClionProjects/Jigsaw/io/LoadData.cpp

CMakeFiles/Jigsaw.dir/io/LoadData.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Jigsaw.dir/io/LoadData.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/summer/ClionProjects/Jigsaw/io/LoadData.cpp > CMakeFiles/Jigsaw.dir/io/LoadData.cpp.i

CMakeFiles/Jigsaw.dir/io/LoadData.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Jigsaw.dir/io/LoadData.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/summer/ClionProjects/Jigsaw/io/LoadData.cpp -o CMakeFiles/Jigsaw.dir/io/LoadData.cpp.s

CMakeFiles/Jigsaw.dir/io/LoadData.cpp.o.requires:

.PHONY : CMakeFiles/Jigsaw.dir/io/LoadData.cpp.o.requires

CMakeFiles/Jigsaw.dir/io/LoadData.cpp.o.provides: CMakeFiles/Jigsaw.dir/io/LoadData.cpp.o.requires
	$(MAKE) -f CMakeFiles/Jigsaw.dir/build.make CMakeFiles/Jigsaw.dir/io/LoadData.cpp.o.provides.build
.PHONY : CMakeFiles/Jigsaw.dir/io/LoadData.cpp.o.provides

CMakeFiles/Jigsaw.dir/io/LoadData.cpp.o.provides.build: CMakeFiles/Jigsaw.dir/io/LoadData.cpp.o


CMakeFiles/Jigsaw.dir/ICP_Refine/ICPRefine.cpp.o: CMakeFiles/Jigsaw.dir/flags.make
CMakeFiles/Jigsaw.dir/ICP_Refine/ICPRefine.cpp.o: ../ICP_Refine/ICPRefine.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/summer/ClionProjects/Jigsaw/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/Jigsaw.dir/ICP_Refine/ICPRefine.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Jigsaw.dir/ICP_Refine/ICPRefine.cpp.o -c /Users/summer/ClionProjects/Jigsaw/ICP_Refine/ICPRefine.cpp

CMakeFiles/Jigsaw.dir/ICP_Refine/ICPRefine.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Jigsaw.dir/ICP_Refine/ICPRefine.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/summer/ClionProjects/Jigsaw/ICP_Refine/ICPRefine.cpp > CMakeFiles/Jigsaw.dir/ICP_Refine/ICPRefine.cpp.i

CMakeFiles/Jigsaw.dir/ICP_Refine/ICPRefine.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Jigsaw.dir/ICP_Refine/ICPRefine.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/summer/ClionProjects/Jigsaw/ICP_Refine/ICPRefine.cpp -o CMakeFiles/Jigsaw.dir/ICP_Refine/ICPRefine.cpp.s

CMakeFiles/Jigsaw.dir/ICP_Refine/ICPRefine.cpp.o.requires:

.PHONY : CMakeFiles/Jigsaw.dir/ICP_Refine/ICPRefine.cpp.o.requires

CMakeFiles/Jigsaw.dir/ICP_Refine/ICPRefine.cpp.o.provides: CMakeFiles/Jigsaw.dir/ICP_Refine/ICPRefine.cpp.o.requires
	$(MAKE) -f CMakeFiles/Jigsaw.dir/build.make CMakeFiles/Jigsaw.dir/ICP_Refine/ICPRefine.cpp.o.provides.build
.PHONY : CMakeFiles/Jigsaw.dir/ICP_Refine/ICPRefine.cpp.o.provides

CMakeFiles/Jigsaw.dir/ICP_Refine/ICPRefine.cpp.o.provides.build: CMakeFiles/Jigsaw.dir/ICP_Refine/ICPRefine.cpp.o


CMakeFiles/Jigsaw.dir/utils/RotatePointCloud.cpp.o: CMakeFiles/Jigsaw.dir/flags.make
CMakeFiles/Jigsaw.dir/utils/RotatePointCloud.cpp.o: ../utils/RotatePointCloud.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/summer/ClionProjects/Jigsaw/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/Jigsaw.dir/utils/RotatePointCloud.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Jigsaw.dir/utils/RotatePointCloud.cpp.o -c /Users/summer/ClionProjects/Jigsaw/utils/RotatePointCloud.cpp

CMakeFiles/Jigsaw.dir/utils/RotatePointCloud.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Jigsaw.dir/utils/RotatePointCloud.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/summer/ClionProjects/Jigsaw/utils/RotatePointCloud.cpp > CMakeFiles/Jigsaw.dir/utils/RotatePointCloud.cpp.i

CMakeFiles/Jigsaw.dir/utils/RotatePointCloud.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Jigsaw.dir/utils/RotatePointCloud.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/summer/ClionProjects/Jigsaw/utils/RotatePointCloud.cpp -o CMakeFiles/Jigsaw.dir/utils/RotatePointCloud.cpp.s

CMakeFiles/Jigsaw.dir/utils/RotatePointCloud.cpp.o.requires:

.PHONY : CMakeFiles/Jigsaw.dir/utils/RotatePointCloud.cpp.o.requires

CMakeFiles/Jigsaw.dir/utils/RotatePointCloud.cpp.o.provides: CMakeFiles/Jigsaw.dir/utils/RotatePointCloud.cpp.o.requires
	$(MAKE) -f CMakeFiles/Jigsaw.dir/build.make CMakeFiles/Jigsaw.dir/utils/RotatePointCloud.cpp.o.provides.build
.PHONY : CMakeFiles/Jigsaw.dir/utils/RotatePointCloud.cpp.o.provides

CMakeFiles/Jigsaw.dir/utils/RotatePointCloud.cpp.o.provides.build: CMakeFiles/Jigsaw.dir/utils/RotatePointCloud.cpp.o


# Object files for target Jigsaw
Jigsaw_OBJECTS = \
"CMakeFiles/Jigsaw.dir/main.cpp.o" \
"CMakeFiles/Jigsaw.dir/filter/Point_filter.cpp.o" \
"CMakeFiles/Jigsaw.dir/keypoint/keypoint.cpp.o" \
"CMakeFiles/Jigsaw.dir/featrue_align/FeatrueAlign.cpp.o" \
"CMakeFiles/Jigsaw.dir/io/LoadData.cpp.o" \
"CMakeFiles/Jigsaw.dir/ICP_Refine/ICPRefine.cpp.o" \
"CMakeFiles/Jigsaw.dir/utils/RotatePointCloud.cpp.o"

# External object files for target Jigsaw
Jigsaw_EXTERNAL_OBJECTS =

Jigsaw: CMakeFiles/Jigsaw.dir/main.cpp.o
Jigsaw: CMakeFiles/Jigsaw.dir/filter/Point_filter.cpp.o
Jigsaw: CMakeFiles/Jigsaw.dir/keypoint/keypoint.cpp.o
Jigsaw: CMakeFiles/Jigsaw.dir/featrue_align/FeatrueAlign.cpp.o
Jigsaw: CMakeFiles/Jigsaw.dir/io/LoadData.cpp.o
Jigsaw: CMakeFiles/Jigsaw.dir/ICP_Refine/ICPRefine.cpp.o
Jigsaw: CMakeFiles/Jigsaw.dir/utils/RotatePointCloud.cpp.o
Jigsaw: CMakeFiles/Jigsaw.dir/build.make
Jigsaw: /usr/local/lib/libboost_system-mt.dylib
Jigsaw: /usr/local/lib/libboost_filesystem-mt.dylib
Jigsaw: /usr/local/lib/libboost_thread-mt.dylib
Jigsaw: /usr/local/lib/libboost_date_time-mt.dylib
Jigsaw: /usr/local/lib/libboost_iostreams-mt.dylib
Jigsaw: /usr/local/lib/libboost_serialization-mt.dylib
Jigsaw: /usr/local/lib/libboost_chrono-mt.dylib
Jigsaw: /usr/local/lib/libpcl_common.dylib
Jigsaw: /usr/local/lib/libpcl_octree.dylib
Jigsaw: /usr/local/Cellar/openni/1.5.7.10/lib/libOpenNI.dylib
Jigsaw: /usr/lib/libz.dylib
Jigsaw: /usr/lib/libexpat.dylib
Jigsaw: /System/Library/Frameworks/Python.framework/Versions/2.7/Python
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkWrappingTools-8.1.a
Jigsaw: /usr/local/lib/libjpeg.dylib
Jigsaw: /usr/local/lib/libpng.dylib
Jigsaw: /usr/local/lib/libtiff.dylib
Jigsaw: /usr/local/lib/libhdf5.dylib
Jigsaw: /usr/local/lib/libsz.dylib
Jigsaw: /usr/lib/libdl.dylib
Jigsaw: /usr/lib/libm.dylib
Jigsaw: /usr/local/lib/libhdf5_hl.dylib
Jigsaw: /usr/local/lib/libnetcdf.dylib
Jigsaw: /usr/lib/libxml2.dylib
Jigsaw: /usr/local/lib/libpcl_io.dylib
Jigsaw: /usr/local/Cellar/flann/1.9.1_3/lib/libflann_cpp_s.a
Jigsaw: /usr/local/lib/libpcl_kdtree.dylib
Jigsaw: /usr/local/lib/libpcl_search.dylib
Jigsaw: /usr/local/lib/libpcl_sample_consensus.dylib
Jigsaw: /usr/local/lib/libpcl_filters.dylib
Jigsaw: /usr/local/lib/libpcl_features.dylib
Jigsaw: /usr/local/lib/libpcl_ml.dylib
Jigsaw: /usr/local/lib/libpcl_segmentation.dylib
Jigsaw: /usr/local/lib/libpcl_visualization.dylib
Jigsaw: /usr/local/lib/libqhull_p.dylib
Jigsaw: /usr/local/lib/libpcl_surface.dylib
Jigsaw: /usr/local/lib/libpcl_registration.dylib
Jigsaw: /usr/local/lib/libpcl_keypoints.dylib
Jigsaw: /usr/local/lib/libpcl_tracking.dylib
Jigsaw: /usr/local/lib/libpcl_recognition.dylib
Jigsaw: /usr/local/lib/libpcl_stereo.dylib
Jigsaw: /usr/local/lib/libpcl_apps.dylib
Jigsaw: /usr/local/lib/libpcl_outofcore.dylib
Jigsaw: /usr/local/lib/libpcl_people.dylib
Jigsaw: /usr/local/lib/libGLEW.dylib
Jigsaw: /usr/local/lib/libpcl_simulation.dylib
Jigsaw: /usr/local/lib/libboost_system-mt.dylib
Jigsaw: /usr/local/lib/libboost_filesystem-mt.dylib
Jigsaw: /usr/local/lib/libboost_thread-mt.dylib
Jigsaw: /usr/local/lib/libboost_date_time-mt.dylib
Jigsaw: /usr/local/lib/libboost_iostreams-mt.dylib
Jigsaw: /usr/local/lib/libboost_serialization-mt.dylib
Jigsaw: /usr/local/lib/libboost_chrono-mt.dylib
Jigsaw: /usr/local/lib/libqhull_p.dylib
Jigsaw: /usr/local/Cellar/openni/1.5.7.10/lib/libOpenNI.dylib
Jigsaw: /usr/local/Cellar/flann/1.9.1_3/lib/libflann_cpp_s.a
Jigsaw: /usr/lib/libz.dylib
Jigsaw: /usr/lib/libexpat.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkDomainsChemistryOpenGL2-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkFiltersFlowPaths-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkFiltersGeneric-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkFiltersHyperTree-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkFiltersParallelImaging-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkFiltersPoints-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkFiltersProgrammable-8.1.1.dylib
Jigsaw: /System/Library/Frameworks/Python.framework/Versions/2.7/Python
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkWrappingTools-8.1.a
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkFiltersPython-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkFiltersSMP-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkFiltersSelection-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkFiltersTexture-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkFiltersTopology-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkFiltersVerdict-8.1.1.dylib
Jigsaw: /usr/local/lib/libjpeg.dylib
Jigsaw: /usr/local/lib/libpng.dylib
Jigsaw: /usr/local/lib/libtiff.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkGeovisCore-8.1.1.dylib
Jigsaw: /usr/local/lib/libhdf5.dylib
Jigsaw: /usr/local/lib/libsz.dylib
Jigsaw: /usr/lib/libdl.dylib
Jigsaw: /usr/lib/libm.dylib
Jigsaw: /usr/local/lib/libhdf5_hl.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkIOAMR-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkIOEnSight-8.1.1.dylib
Jigsaw: /usr/local/lib/libnetcdf.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkIOExodus-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkIOExportOpenGL2-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkIOImport-8.1.1.dylib
Jigsaw: /usr/lib/libxml2.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkIOInfovis-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkIOLSDyna-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkIOMINC-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkIOMovie-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkIOPLY-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkIOParallel-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkIOParallelXML-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkIOSQL-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkIOTecplotTable-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkIOVideo-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkImagingMorphological-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkImagingStatistics-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkImagingStencil-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkInfovisBoostGraphAlgorithms-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkInteractionImage-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkRenderingContextOpenGL2-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkRenderingFreeTypeFontConfig-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkRenderingImage-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkRenderingLOD-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkRenderingVolumeOpenGL2-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkViewsContext2D-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkViewsInfovis-8.1.1.dylib
Jigsaw: /usr/local/lib/libpcl_common.dylib
Jigsaw: /usr/local/lib/libpcl_octree.dylib
Jigsaw: /usr/local/lib/libpcl_io.dylib
Jigsaw: /usr/local/lib/libpcl_kdtree.dylib
Jigsaw: /usr/local/lib/libpcl_search.dylib
Jigsaw: /usr/local/lib/libpcl_sample_consensus.dylib
Jigsaw: /usr/local/lib/libpcl_filters.dylib
Jigsaw: /usr/local/lib/libpcl_features.dylib
Jigsaw: /usr/local/lib/libpcl_ml.dylib
Jigsaw: /usr/local/lib/libpcl_segmentation.dylib
Jigsaw: /usr/local/lib/libpcl_visualization.dylib
Jigsaw: /usr/local/lib/libpcl_surface.dylib
Jigsaw: /usr/local/lib/libpcl_registration.dylib
Jigsaw: /usr/local/lib/libpcl_keypoints.dylib
Jigsaw: /usr/local/lib/libpcl_tracking.dylib
Jigsaw: /usr/local/lib/libpcl_recognition.dylib
Jigsaw: /usr/local/lib/libpcl_stereo.dylib
Jigsaw: /usr/local/lib/libpcl_apps.dylib
Jigsaw: /usr/local/lib/libpcl_outofcore.dylib
Jigsaw: /usr/local/lib/libpcl_people.dylib
Jigsaw: /usr/local/lib/libGLEW.dylib
Jigsaw: /usr/local/lib/libpcl_simulation.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkDomainsChemistry-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkWrappingPython27Core-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkverdict-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkproj4-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkFiltersAMR-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkIOExport-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkRenderingGL2PSOpenGL2-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkgl2ps-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtklibharu-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkoggtheora-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkFiltersParallel-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkexoIIc-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkIOGeometry-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkIONetCDF-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtknetcdfcpp-8.1.1.dylib
Jigsaw: /usr/local/lib/libnetcdf.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkjsoncpp-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkParallelCore-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkIOLegacy-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtksqlite-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkRenderingOpenGL2-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkglew-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkImagingMath-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkChartsCore-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkRenderingContext2D-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkFiltersImaging-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkInfovisLayout-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkInfovisCore-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkViewsCore-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkInteractionWidgets-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkFiltersHybrid-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkImagingGeneral-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkImagingSources-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkFiltersModeling-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkImagingHybrid-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkIOImage-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkDICOMParser-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkmetaio-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkInteractionStyle-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkFiltersExtraction-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkFiltersStatistics-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkImagingFourier-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkalglib-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkRenderingAnnotation-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkImagingColor-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkRenderingVolume-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkImagingCore-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkIOXML-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkIOXMLParser-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkIOCore-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtklz4-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkRenderingLabel-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkRenderingFreeType-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkRenderingCore-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkCommonColor-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkFiltersGeometry-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkFiltersSources-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkFiltersGeneral-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkCommonComputationalGeometry-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkFiltersCore-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkCommonExecutionModel-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkCommonDataModel-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkCommonMisc-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkCommonSystem-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtksys-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkCommonTransforms-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkCommonMath-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkCommonCore-8.1.1.dylib
Jigsaw: /usr/local/Cellar/vtk/8.1.0/lib/libvtkfreetype-8.1.1.dylib
Jigsaw: /usr/lib/libz.dylib
Jigsaw: CMakeFiles/Jigsaw.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/summer/ClionProjects/Jigsaw/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX executable Jigsaw"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Jigsaw.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Jigsaw.dir/build: Jigsaw

.PHONY : CMakeFiles/Jigsaw.dir/build

CMakeFiles/Jigsaw.dir/requires: CMakeFiles/Jigsaw.dir/main.cpp.o.requires
CMakeFiles/Jigsaw.dir/requires: CMakeFiles/Jigsaw.dir/filter/Point_filter.cpp.o.requires
CMakeFiles/Jigsaw.dir/requires: CMakeFiles/Jigsaw.dir/keypoint/keypoint.cpp.o.requires
CMakeFiles/Jigsaw.dir/requires: CMakeFiles/Jigsaw.dir/featrue_align/FeatrueAlign.cpp.o.requires
CMakeFiles/Jigsaw.dir/requires: CMakeFiles/Jigsaw.dir/io/LoadData.cpp.o.requires
CMakeFiles/Jigsaw.dir/requires: CMakeFiles/Jigsaw.dir/ICP_Refine/ICPRefine.cpp.o.requires
CMakeFiles/Jigsaw.dir/requires: CMakeFiles/Jigsaw.dir/utils/RotatePointCloud.cpp.o.requires

.PHONY : CMakeFiles/Jigsaw.dir/requires

CMakeFiles/Jigsaw.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Jigsaw.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Jigsaw.dir/clean

CMakeFiles/Jigsaw.dir/depend:
	cd /Users/summer/ClionProjects/Jigsaw/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/summer/ClionProjects/Jigsaw /Users/summer/ClionProjects/Jigsaw /Users/summer/ClionProjects/Jigsaw/cmake-build-debug /Users/summer/ClionProjects/Jigsaw/cmake-build-debug /Users/summer/ClionProjects/Jigsaw/cmake-build-debug/CMakeFiles/Jigsaw.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Jigsaw.dir/depend

