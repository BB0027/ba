# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /mnt/e/Poject/guide/My_guide/testmap/backend

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /mnt/e/Poject/guide/My_guide/testmap/backend/out/build/preset1

# Include any dependencies generated for this target.
include CMakeFiles/backend.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/backend.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/backend.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/backend.dir/flags.make

CMakeFiles/backend.dir/graph_model.cpp.o: CMakeFiles/backend.dir/flags.make
CMakeFiles/backend.dir/graph_model.cpp.o: /mnt/e/Poject/guide/My_guide/testmap/backend/graph_model.cpp
CMakeFiles/backend.dir/graph_model.cpp.o: CMakeFiles/backend.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/mnt/e/Poject/guide/My_guide/testmap/backend/out/build/preset1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/backend.dir/graph_model.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/backend.dir/graph_model.cpp.o -MF CMakeFiles/backend.dir/graph_model.cpp.o.d -o CMakeFiles/backend.dir/graph_model.cpp.o -c /mnt/e/Poject/guide/My_guide/testmap/backend/graph_model.cpp

CMakeFiles/backend.dir/graph_model.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/backend.dir/graph_model.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/e/Poject/guide/My_guide/testmap/backend/graph_model.cpp > CMakeFiles/backend.dir/graph_model.cpp.i

CMakeFiles/backend.dir/graph_model.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/backend.dir/graph_model.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/e/Poject/guide/My_guide/testmap/backend/graph_model.cpp -o CMakeFiles/backend.dir/graph_model.cpp.s

CMakeFiles/backend.dir/map_model.cpp.o: CMakeFiles/backend.dir/flags.make
CMakeFiles/backend.dir/map_model.cpp.o: /mnt/e/Poject/guide/My_guide/testmap/backend/map_model.cpp
CMakeFiles/backend.dir/map_model.cpp.o: CMakeFiles/backend.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/mnt/e/Poject/guide/My_guide/testmap/backend/out/build/preset1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/backend.dir/map_model.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/backend.dir/map_model.cpp.o -MF CMakeFiles/backend.dir/map_model.cpp.o.d -o CMakeFiles/backend.dir/map_model.cpp.o -c /mnt/e/Poject/guide/My_guide/testmap/backend/map_model.cpp

CMakeFiles/backend.dir/map_model.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/backend.dir/map_model.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/e/Poject/guide/My_guide/testmap/backend/map_model.cpp > CMakeFiles/backend.dir/map_model.cpp.i

CMakeFiles/backend.dir/map_model.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/backend.dir/map_model.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/e/Poject/guide/My_guide/testmap/backend/map_model.cpp -o CMakeFiles/backend.dir/map_model.cpp.s

CMakeFiles/backend.dir/planner.cpp.o: CMakeFiles/backend.dir/flags.make
CMakeFiles/backend.dir/planner.cpp.o: /mnt/e/Poject/guide/My_guide/testmap/backend/planner.cpp
CMakeFiles/backend.dir/planner.cpp.o: CMakeFiles/backend.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/mnt/e/Poject/guide/My_guide/testmap/backend/out/build/preset1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/backend.dir/planner.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/backend.dir/planner.cpp.o -MF CMakeFiles/backend.dir/planner.cpp.o.d -o CMakeFiles/backend.dir/planner.cpp.o -c /mnt/e/Poject/guide/My_guide/testmap/backend/planner.cpp

CMakeFiles/backend.dir/planner.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/backend.dir/planner.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/e/Poject/guide/My_guide/testmap/backend/planner.cpp > CMakeFiles/backend.dir/planner.cpp.i

CMakeFiles/backend.dir/planner.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/backend.dir/planner.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/e/Poject/guide/My_guide/testmap/backend/planner.cpp -o CMakeFiles/backend.dir/planner.cpp.s

CMakeFiles/backend.dir/test.cpp.o: CMakeFiles/backend.dir/flags.make
CMakeFiles/backend.dir/test.cpp.o: /mnt/e/Poject/guide/My_guide/testmap/backend/test.cpp
CMakeFiles/backend.dir/test.cpp.o: CMakeFiles/backend.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/mnt/e/Poject/guide/My_guide/testmap/backend/out/build/preset1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/backend.dir/test.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/backend.dir/test.cpp.o -MF CMakeFiles/backend.dir/test.cpp.o.d -o CMakeFiles/backend.dir/test.cpp.o -c /mnt/e/Poject/guide/My_guide/testmap/backend/test.cpp

CMakeFiles/backend.dir/test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/backend.dir/test.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/e/Poject/guide/My_guide/testmap/backend/test.cpp > CMakeFiles/backend.dir/test.cpp.i

CMakeFiles/backend.dir/test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/backend.dir/test.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/e/Poject/guide/My_guide/testmap/backend/test.cpp -o CMakeFiles/backend.dir/test.cpp.s

CMakeFiles/backend.dir/tinystr.cpp.o: CMakeFiles/backend.dir/flags.make
CMakeFiles/backend.dir/tinystr.cpp.o: /mnt/e/Poject/guide/My_guide/testmap/backend/tinystr.cpp
CMakeFiles/backend.dir/tinystr.cpp.o: CMakeFiles/backend.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/mnt/e/Poject/guide/My_guide/testmap/backend/out/build/preset1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/backend.dir/tinystr.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/backend.dir/tinystr.cpp.o -MF CMakeFiles/backend.dir/tinystr.cpp.o.d -o CMakeFiles/backend.dir/tinystr.cpp.o -c /mnt/e/Poject/guide/My_guide/testmap/backend/tinystr.cpp

CMakeFiles/backend.dir/tinystr.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/backend.dir/tinystr.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/e/Poject/guide/My_guide/testmap/backend/tinystr.cpp > CMakeFiles/backend.dir/tinystr.cpp.i

CMakeFiles/backend.dir/tinystr.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/backend.dir/tinystr.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/e/Poject/guide/My_guide/testmap/backend/tinystr.cpp -o CMakeFiles/backend.dir/tinystr.cpp.s

CMakeFiles/backend.dir/tinyxml.cpp.o: CMakeFiles/backend.dir/flags.make
CMakeFiles/backend.dir/tinyxml.cpp.o: /mnt/e/Poject/guide/My_guide/testmap/backend/tinyxml.cpp
CMakeFiles/backend.dir/tinyxml.cpp.o: CMakeFiles/backend.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/mnt/e/Poject/guide/My_guide/testmap/backend/out/build/preset1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/backend.dir/tinyxml.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/backend.dir/tinyxml.cpp.o -MF CMakeFiles/backend.dir/tinyxml.cpp.o.d -o CMakeFiles/backend.dir/tinyxml.cpp.o -c /mnt/e/Poject/guide/My_guide/testmap/backend/tinyxml.cpp

CMakeFiles/backend.dir/tinyxml.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/backend.dir/tinyxml.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/e/Poject/guide/My_guide/testmap/backend/tinyxml.cpp > CMakeFiles/backend.dir/tinyxml.cpp.i

CMakeFiles/backend.dir/tinyxml.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/backend.dir/tinyxml.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/e/Poject/guide/My_guide/testmap/backend/tinyxml.cpp -o CMakeFiles/backend.dir/tinyxml.cpp.s

CMakeFiles/backend.dir/tinyxmlerror.cpp.o: CMakeFiles/backend.dir/flags.make
CMakeFiles/backend.dir/tinyxmlerror.cpp.o: /mnt/e/Poject/guide/My_guide/testmap/backend/tinyxmlerror.cpp
CMakeFiles/backend.dir/tinyxmlerror.cpp.o: CMakeFiles/backend.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/mnt/e/Poject/guide/My_guide/testmap/backend/out/build/preset1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/backend.dir/tinyxmlerror.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/backend.dir/tinyxmlerror.cpp.o -MF CMakeFiles/backend.dir/tinyxmlerror.cpp.o.d -o CMakeFiles/backend.dir/tinyxmlerror.cpp.o -c /mnt/e/Poject/guide/My_guide/testmap/backend/tinyxmlerror.cpp

CMakeFiles/backend.dir/tinyxmlerror.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/backend.dir/tinyxmlerror.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/e/Poject/guide/My_guide/testmap/backend/tinyxmlerror.cpp > CMakeFiles/backend.dir/tinyxmlerror.cpp.i

CMakeFiles/backend.dir/tinyxmlerror.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/backend.dir/tinyxmlerror.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/e/Poject/guide/My_guide/testmap/backend/tinyxmlerror.cpp -o CMakeFiles/backend.dir/tinyxmlerror.cpp.s

CMakeFiles/backend.dir/tinyxmlparser.cpp.o: CMakeFiles/backend.dir/flags.make
CMakeFiles/backend.dir/tinyxmlparser.cpp.o: /mnt/e/Poject/guide/My_guide/testmap/backend/tinyxmlparser.cpp
CMakeFiles/backend.dir/tinyxmlparser.cpp.o: CMakeFiles/backend.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/mnt/e/Poject/guide/My_guide/testmap/backend/out/build/preset1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/backend.dir/tinyxmlparser.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/backend.dir/tinyxmlparser.cpp.o -MF CMakeFiles/backend.dir/tinyxmlparser.cpp.o.d -o CMakeFiles/backend.dir/tinyxmlparser.cpp.o -c /mnt/e/Poject/guide/My_guide/testmap/backend/tinyxmlparser.cpp

CMakeFiles/backend.dir/tinyxmlparser.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/backend.dir/tinyxmlparser.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/e/Poject/guide/My_guide/testmap/backend/tinyxmlparser.cpp > CMakeFiles/backend.dir/tinyxmlparser.cpp.i

CMakeFiles/backend.dir/tinyxmlparser.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/backend.dir/tinyxmlparser.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/e/Poject/guide/My_guide/testmap/backend/tinyxmlparser.cpp -o CMakeFiles/backend.dir/tinyxmlparser.cpp.s

# Object files for target backend
backend_OBJECTS = \
"CMakeFiles/backend.dir/graph_model.cpp.o" \
"CMakeFiles/backend.dir/map_model.cpp.o" \
"CMakeFiles/backend.dir/planner.cpp.o" \
"CMakeFiles/backend.dir/test.cpp.o" \
"CMakeFiles/backend.dir/tinystr.cpp.o" \
"CMakeFiles/backend.dir/tinyxml.cpp.o" \
"CMakeFiles/backend.dir/tinyxmlerror.cpp.o" \
"CMakeFiles/backend.dir/tinyxmlparser.cpp.o"

# External object files for target backend
backend_EXTERNAL_OBJECTS =

backend: CMakeFiles/backend.dir/graph_model.cpp.o
backend: CMakeFiles/backend.dir/map_model.cpp.o
backend: CMakeFiles/backend.dir/planner.cpp.o
backend: CMakeFiles/backend.dir/test.cpp.o
backend: CMakeFiles/backend.dir/tinystr.cpp.o
backend: CMakeFiles/backend.dir/tinyxml.cpp.o
backend: CMakeFiles/backend.dir/tinyxmlerror.cpp.o
backend: CMakeFiles/backend.dir/tinyxmlparser.cpp.o
backend: CMakeFiles/backend.dir/build.make
backend: CMakeFiles/backend.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/mnt/e/Poject/guide/My_guide/testmap/backend/out/build/preset1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Linking CXX executable backend"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/backend.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/backend.dir/build: backend
.PHONY : CMakeFiles/backend.dir/build

CMakeFiles/backend.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/backend.dir/cmake_clean.cmake
.PHONY : CMakeFiles/backend.dir/clean

CMakeFiles/backend.dir/depend:
	cd /mnt/e/Poject/guide/My_guide/testmap/backend/out/build/preset1 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /mnt/e/Poject/guide/My_guide/testmap/backend /mnt/e/Poject/guide/My_guide/testmap/backend /mnt/e/Poject/guide/My_guide/testmap/backend/out/build/preset1 /mnt/e/Poject/guide/My_guide/testmap/backend/out/build/preset1 /mnt/e/Poject/guide/My_guide/testmap/backend/out/build/preset1/CMakeFiles/backend.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/backend.dir/depend

