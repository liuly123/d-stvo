# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.14

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/liuly/stvo

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/liuly/stvo/build

# Utility rule file for stvo_includes.

# Include the progress variables for this target.
include CMakeFiles/stvo_includes.dir/progress.make

CMakeFiles/stvo_includes: ../3rdparty/line_descriptor/include/line_descriptor/descriptor_custom.hpp
CMakeFiles/stvo_includes: ../3rdparty/line_descriptor/include/line_descriptor_custom.hpp
CMakeFiles/stvo_includes: ../3rdparty/line_descriptor/src/bitarray_custom.hpp
CMakeFiles/stvo_includes: ../3rdparty/line_descriptor/src/bitops_custom.hpp
CMakeFiles/stvo_includes: ../3rdparty/line_descriptor/src/precomp_custom.hpp
CMakeFiles/stvo_includes: ../3rdparty/line_descriptor/src/types_custom.hpp
CMakeFiles/stvo_includes: ../include/auxiliar.h
CMakeFiles/stvo_includes: ../include/config.h
CMakeFiles/stvo_includes: ../include/dataset.h
CMakeFiles/stvo_includes: ../include/gridStructure.h
CMakeFiles/stvo_includes: ../include/lineIterator.h
CMakeFiles/stvo_includes: ../include/matching.h
CMakeFiles/stvo_includes: ../include/pinholeStereoCamera.h
CMakeFiles/stvo_includes: ../include/sceneRepresentation.h
CMakeFiles/stvo_includes: ../include/stereoFeatures.h
CMakeFiles/stvo_includes: ../include/stereoFrame.h
CMakeFiles/stvo_includes: ../include/stereoFrameHandler.h
CMakeFiles/stvo_includes: ../include/timer.h


stvo_includes: CMakeFiles/stvo_includes
stvo_includes: CMakeFiles/stvo_includes.dir/build.make

.PHONY : stvo_includes

# Rule to build all files generated by this target.
CMakeFiles/stvo_includes.dir/build: stvo_includes

.PHONY : CMakeFiles/stvo_includes.dir/build

CMakeFiles/stvo_includes.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/stvo_includes.dir/cmake_clean.cmake
.PHONY : CMakeFiles/stvo_includes.dir/clean

CMakeFiles/stvo_includes.dir/depend:
	cd /home/liuly/stvo/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/liuly/stvo /home/liuly/stvo /home/liuly/stvo/build /home/liuly/stvo/build /home/liuly/stvo/build/CMakeFiles/stvo_includes.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/stvo_includes.dir/depend

