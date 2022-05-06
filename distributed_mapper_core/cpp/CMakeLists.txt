cmake_minimum_required(VERSION 3.10)
project(distributed-mapper)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")
#set(CMAKE_CXX_STANDARD 17)

# Add option to enable testing
option(ENABLE_TESTS "Enable tests" ON)

###########################################################################
# Find GTSAM
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})
link_directories(${GTSAM_LIBRARY_DIRS})
message("GTSAM Libraries: ${GTSAM_LIBRARIES}")

# for unittest examples
#set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")
#include(GtsamTesting)

include_directories(src)

#add DCSAM
include_directories(/home/yewei/dcsam/dcsam/include)
add_library(DCSAM STATIC IMPORTED)
set_property(TARGET DCSAM PROPERTY IMPORTED_LOCATION /home/yewei/dcsam/dcsam/build/libdcsam.a)

###########################################################################
# Find Boost
find_package(Boost COMPONENTS serialization filesystem system program_options thread REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_libraries(${Boost_LIBRARIES})
message("Boost Libraries: ${Boost_LIBRARIES}")

###########################################################################
# Distributed Mapper
add_library (distributed_mapper
        src/DistributedMapper.cpp
        src/MultiRobotUtils.cpp
        )

target_link_libraries(distributed_mapper
        ${Boost_LIBRARIES}
        gtsam
        DCSAM)

###########################################################################
# Add example executable
add_executable (runDistributedMapper scripts/runDistributedMapper.cpp)
target_link_libraries (runDistributedMapper distributed_mapper)

###########################################################################
# Tests
#add_subdirectory(tests)

# Include unit tests directory to the project.
if (${ENABLE_TESTS})
    message(STATUS "Testing enabled. Building tests.")
    find_package(GTSAMCMakeTools)
    include(GtsamTesting)
    add_subdirectory(tests)
endif()


###########################################################################
# Install
#file(GLOB headers "src/*.h")
#install(FILES ${headers} DESTINATION include/distributed_mapper/)
#configure_file (${PROJECT_SOURCE_DIR}/DistributedMapperConfig.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/DistributedMapperConfig.cmake @ONLY)
#install(FILES ${CMAKE_CURRENT_BINARY_DIR}/DistributedMapperConfig.cmake DESTINATION share/distributed_mapper)


