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
CMAKE_SOURCE_DIR = /home/work/code_repository/pcl_codes/grasping2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/work/code_repository/pcl_codes/grasping2/build

# Include any dependencies generated for this target.
include CMakeFiles/step_1_downsampling_object_cloud.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/step_1_downsampling_object_cloud.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/step_1_downsampling_object_cloud.dir/flags.make

CMakeFiles/step_1_downsampling_object_cloud.dir/step_1_downsampling_object_cloud.cpp.o: CMakeFiles/step_1_downsampling_object_cloud.dir/flags.make
CMakeFiles/step_1_downsampling_object_cloud.dir/step_1_downsampling_object_cloud.cpp.o: ../step_1_downsampling_object_cloud.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/work/code_repository/pcl_codes/grasping2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/step_1_downsampling_object_cloud.dir/step_1_downsampling_object_cloud.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/step_1_downsampling_object_cloud.dir/step_1_downsampling_object_cloud.cpp.o -c /home/work/code_repository/pcl_codes/grasping2/step_1_downsampling_object_cloud.cpp

CMakeFiles/step_1_downsampling_object_cloud.dir/step_1_downsampling_object_cloud.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/step_1_downsampling_object_cloud.dir/step_1_downsampling_object_cloud.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/work/code_repository/pcl_codes/grasping2/step_1_downsampling_object_cloud.cpp > CMakeFiles/step_1_downsampling_object_cloud.dir/step_1_downsampling_object_cloud.cpp.i

CMakeFiles/step_1_downsampling_object_cloud.dir/step_1_downsampling_object_cloud.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/step_1_downsampling_object_cloud.dir/step_1_downsampling_object_cloud.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/work/code_repository/pcl_codes/grasping2/step_1_downsampling_object_cloud.cpp -o CMakeFiles/step_1_downsampling_object_cloud.dir/step_1_downsampling_object_cloud.cpp.s

CMakeFiles/step_1_downsampling_object_cloud.dir/step_1_downsampling_object_cloud.cpp.o.requires:

.PHONY : CMakeFiles/step_1_downsampling_object_cloud.dir/step_1_downsampling_object_cloud.cpp.o.requires

CMakeFiles/step_1_downsampling_object_cloud.dir/step_1_downsampling_object_cloud.cpp.o.provides: CMakeFiles/step_1_downsampling_object_cloud.dir/step_1_downsampling_object_cloud.cpp.o.requires
	$(MAKE) -f CMakeFiles/step_1_downsampling_object_cloud.dir/build.make CMakeFiles/step_1_downsampling_object_cloud.dir/step_1_downsampling_object_cloud.cpp.o.provides.build
.PHONY : CMakeFiles/step_1_downsampling_object_cloud.dir/step_1_downsampling_object_cloud.cpp.o.provides

CMakeFiles/step_1_downsampling_object_cloud.dir/step_1_downsampling_object_cloud.cpp.o.provides.build: CMakeFiles/step_1_downsampling_object_cloud.dir/step_1_downsampling_object_cloud.cpp.o


# Object files for target step_1_downsampling_object_cloud
step_1_downsampling_object_cloud_OBJECTS = \
"CMakeFiles/step_1_downsampling_object_cloud.dir/step_1_downsampling_object_cloud.cpp.o"

# External object files for target step_1_downsampling_object_cloud
step_1_downsampling_object_cloud_EXTERNAL_OBJECTS =

step_1_downsampling_object_cloud: CMakeFiles/step_1_downsampling_object_cloud.dir/step_1_downsampling_object_cloud.cpp.o
step_1_downsampling_object_cloud: CMakeFiles/step_1_downsampling_object_cloud.dir/build.make
step_1_downsampling_object_cloud: /usr/local/lib/libpcl_surface.so
step_1_downsampling_object_cloud: /usr/local/lib/libpcl_keypoints.so
step_1_downsampling_object_cloud: /usr/local/lib/libpcl_outofcore.so
step_1_downsampling_object_cloud: /usr/local/lib/libpcl_stereo.so
step_1_downsampling_object_cloud: /usr/local/lib/libpcl_recognition.so
step_1_downsampling_object_cloud: /usr/local/lib/libpcl_tracking.so
step_1_downsampling_object_cloud: /usr/local/lib/libpcl_people.so
step_1_downsampling_object_cloud: /usr/lib/x86_64-linux-gnu/libboost_system.so
step_1_downsampling_object_cloud: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
step_1_downsampling_object_cloud: /usr/lib/x86_64-linux-gnu/libboost_thread.so
step_1_downsampling_object_cloud: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
step_1_downsampling_object_cloud: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
step_1_downsampling_object_cloud: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
step_1_downsampling_object_cloud: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
step_1_downsampling_object_cloud: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
step_1_downsampling_object_cloud: /usr/lib/x86_64-linux-gnu/libboost_regex.so
step_1_downsampling_object_cloud: /usr/lib/x86_64-linux-gnu/libpthread.so
step_1_downsampling_object_cloud: /usr/lib/x86_64-linux-gnu/libqhull.so
step_1_downsampling_object_cloud: /usr/lib/libOpenNI.so
step_1_downsampling_object_cloud: /usr/lib/libOpenNI2.so
step_1_downsampling_object_cloud: /usr/local/lib/libpcl_registration.so
step_1_downsampling_object_cloud: /usr/local/lib/libpcl_visualization.so
step_1_downsampling_object_cloud: /usr/local/lib/libpcl_io.so
step_1_downsampling_object_cloud: /usr/local/lib/libpcl_segmentation.so
step_1_downsampling_object_cloud: /usr/local/lib/libpcl_features.so
step_1_downsampling_object_cloud: /usr/local/lib/libpcl_filters.so
step_1_downsampling_object_cloud: /usr/local/lib/libpcl_sample_consensus.so
step_1_downsampling_object_cloud: /usr/local/lib/libpcl_search.so
step_1_downsampling_object_cloud: /usr/local/lib/libpcl_kdtree.so
step_1_downsampling_object_cloud: /usr/local/lib/libpcl_octree.so
step_1_downsampling_object_cloud: /usr/local/lib/libvtkIOMovie-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkoggtheora-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkFiltersProgrammable-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkFiltersParallelImaging-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkIOPLY-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkImagingStencil-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkFiltersVerdict-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkverdict-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkRenderingVolumeOpenGL-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkIOParallelXML-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkFiltersTexture-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkFiltersSelection-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkFiltersGeneric-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkIOSQL-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtksqlite-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkImagingStatistics-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkIOImport-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkFiltersSMP-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkIOEnSight-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkIOVideo-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkGeovisCore-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkproj4-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkIOMINC-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkDomainsChemistry-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkRenderingLOD-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkFiltersHyperTree-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkImagingMorphological-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkIOExodus-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkIOExport-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkRenderingGL2PS-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkRenderingContextOpenGL-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkgl2ps-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkImagingMath-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkIOInfovis-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtklibxml2-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkIOAMR-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkFiltersAMR-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkInteractionImage-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkViewsInfovis-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkFiltersImaging-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkInfovisLayout-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkChartsCore-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkInfovisCore-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkRenderingLabel-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkIOParallel-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkFiltersParallel-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkParallelCore-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkIONetCDF-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkjsoncpp-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkexoIIc-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkNetCDF_cxx-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkNetCDF-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkhdf5_hl-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkhdf5-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkIOLSDyna-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkRenderingImage-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkFiltersFlowPaths-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkViewsContext2D-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkRenderingContext2D-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkViewsCore-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkInteractionWidgets-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkImagingGeneral-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkFiltersModeling-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkRenderingVolume-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkFiltersHybrid-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkRenderingAnnotation-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkRenderingFreeType-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkftgl-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkfreetype-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkImagingColor-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkInteractionStyle-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkRenderingLIC-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkIOLegacy-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkImagingSources-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkRenderingOpenGL-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkRenderingCore-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkFiltersGeometry-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkFiltersExtraction-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkFiltersStatistics-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkImagingFourier-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkalglib-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkFiltersSources-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkFiltersGeneral-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkFiltersCore-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkCommonComputationalGeometry-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkCommonColor-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkImagingHybrid-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkImagingCore-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkIOImage-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkDICOMParser-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkmetaio-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkpng-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtktiff-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkjpeg-6.3.so.1
step_1_downsampling_object_cloud: /usr/lib/x86_64-linux-gnu/libGLU.so
step_1_downsampling_object_cloud: /usr/lib/x86_64-linux-gnu/libGL.so
step_1_downsampling_object_cloud: /usr/lib/x86_64-linux-gnu/libSM.so
step_1_downsampling_object_cloud: /usr/lib/x86_64-linux-gnu/libICE.so
step_1_downsampling_object_cloud: /usr/lib/x86_64-linux-gnu/libX11.so
step_1_downsampling_object_cloud: /usr/lib/x86_64-linux-gnu/libXext.so
step_1_downsampling_object_cloud: /usr/lib/x86_64-linux-gnu/libXt.so
step_1_downsampling_object_cloud: /usr/local/lib/libvtkIOXML-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkIOGeometry-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkIOXMLParser-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkIOCore-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkCommonExecutionModel-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkCommonDataModel-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkCommonSystem-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkCommonTransforms-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtksys-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkCommonMisc-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkCommonMath-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkCommonCore-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkzlib-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libvtkexpat-6.3.so.1
step_1_downsampling_object_cloud: /usr/local/lib/libpcl_ml.so
step_1_downsampling_object_cloud: /usr/local/lib/libpcl_common.so
step_1_downsampling_object_cloud: CMakeFiles/step_1_downsampling_object_cloud.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/work/code_repository/pcl_codes/grasping2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable step_1_downsampling_object_cloud"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/step_1_downsampling_object_cloud.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/step_1_downsampling_object_cloud.dir/build: step_1_downsampling_object_cloud

.PHONY : CMakeFiles/step_1_downsampling_object_cloud.dir/build

CMakeFiles/step_1_downsampling_object_cloud.dir/requires: CMakeFiles/step_1_downsampling_object_cloud.dir/step_1_downsampling_object_cloud.cpp.o.requires

.PHONY : CMakeFiles/step_1_downsampling_object_cloud.dir/requires

CMakeFiles/step_1_downsampling_object_cloud.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/step_1_downsampling_object_cloud.dir/cmake_clean.cmake
.PHONY : CMakeFiles/step_1_downsampling_object_cloud.dir/clean

CMakeFiles/step_1_downsampling_object_cloud.dir/depend:
	cd /home/work/code_repository/pcl_codes/grasping2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/work/code_repository/pcl_codes/grasping2 /home/work/code_repository/pcl_codes/grasping2 /home/work/code_repository/pcl_codes/grasping2/build /home/work/code_repository/pcl_codes/grasping2/build /home/work/code_repository/pcl_codes/grasping2/build/CMakeFiles/step_1_downsampling_object_cloud.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/step_1_downsampling_object_cloud.dir/depend

