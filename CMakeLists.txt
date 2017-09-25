project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

file(GLOB headers "src/*.hpp" "src/*.h")
file(GLOB sources "src/main.cpp" ${headers})

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


add_executable(path_planning ${sources}
        src/Utilities.h
        src/PathPlanner.h
        src/JsonConverters.h
        src/CartesianPoint.h
        src/FrenetPoint.h
        src/Waypoint.h
        src/WaypointsList.h
        src/World.cpp
        src/World.h
        src/Path.h
        src/Car.h
        src/TelemetryData.h
        src/Lane.h
        src/PathPlanner.cpp src/State.h src/Constants.h src/CarState.h src/GapSpeed.h)

target_link_libraries(path_planning z ssl uv uWS)
