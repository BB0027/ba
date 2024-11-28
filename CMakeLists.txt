cmake_minimum_required(VERSION 3.5.0)
project(backend VERSION 0.1.0 LANGUAGES C CXX)

add_executable(backend graph_model.cpp map_model.cpp planner.cpp test.cpp tinystr.cpp tinyxml.cpp tinyxmlerror.cpp tinyxmlparser.cpp)

find_package(Crow REQUIRED)
target_link_libraries(backend Crow::Crow)

include(CTest)
enable_testing()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
