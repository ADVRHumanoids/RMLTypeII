project(RMLTypeII)
cmake_minimum_required(VERSION 3.0.0)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(MacroInstallLib)

include_directories(include/ReflexxesTypeII/)

file(GLOB ReflexxesTypeII_SRC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    "src/TypeIIRML/*.cpp"
)

add_library(ReflexxesTypeII SHARED ${ReflexxesTypeII_SRC})
# target_include_directories(ReflexxesTypeII PUBLIC include/ReflexxesTypeII)

library_install(ReflexxesTypeII 1 0 0)