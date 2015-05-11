project( CarPlanner )
cmake_minimum_required(VERSION 2.8)

if( NOT CMAKE_BUILD_TYPE )
    set(CMAKE_BUILD_TYPE Debug)
endif()

# Add to module path, so we can find our cmake modules
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)
include(def_library)
include(export_package)

message( STATUS "CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}" )

add_definitions(-DBT_USE_DOUBLE_PRECISION)
add_definitions(-DBT_NO_PROFILE)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall") #-DBOOST_NO_CXX11_NUMERIC_LIMITS")

IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
ENDIF()

# the boost libraries that we need
#set( BOOST_LIBS thread date_time system timer )

find_package( Eigen3 REQUIRED )
FIND_PACKAGE( Pangolin REQUIRED )
find_package( SceneGraph REQUIRED )
find_package( CVars REQUIRED )
find_package( Protobuf REQUIRED )
find_package( Bullet REQUIRED )
#find_package( Boost COMPONENTS ${BOOST_LIBS} REQUIRED )
find_package( Sophus REQUIRED )

set( _LIBRARIES  ${BULLET_LIBRARIES}  ${BOOST_LIBRARIES} ${Sophus_LIBRARIES} ${VRPN_SERVER_LIBRARY}) # ${MVL_LIBRARIES})
set( _INCLUDE_DIRS
        ${CMAKE_BINARY_DIR}
        ${Sophus_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        #${MVL_INCLUDE_DIRS}
        ${CVars_INCLUDE_DIRS}
        ${BULLET_INCLUDE_DIR}
        ${SceneGraph_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        )

include_directories( ${_INCLUDE_DIRS} )


include_directories( ${CAR_PLANNER_INCLUDE_DIRS} )

set( SRCS
        src/apply_velocities_functor.cc
        src/car_controller.cc
        src/vehicle_parameters.cc
        src/bullet/bullet_vehicle.cc
        src/bullet/gl_debug_drawer.cc
        src/bullet/raycast_vehicle.cc
        src/bullet/wheel_info.cc
        src/solvers/bezier_boundary_solver.cc
        src/solvers/car_regressor.cc
        src/solvers/local_planner.cc
        src/utils/gui_exception.cc
   )

set( HDRS
        include/CarPlanner/apply_velocities_functor.h
        include/CarPlanner/boundary_solver.h
        include/CarPlanner/car_controller.h
        include/CarPlanner/control_command.h
        include/CarPlanner/ninjacar.h
        include/CarPlanner/ninjacar_impl.h
        include/CarPlanner/ninjacar_models.h
        include/CarPlanner/rigid_body.h
        include/CarPlanner/vehicle_parameters.h
        include/CarPlanner/vehicle_state.h
        include/CarPlanner/bullet/bullet_vehicle.h
        include/CarPlanner/bullet/bullet_vehicle_parameters.h
        include/CarPlanner/bullet/bullet_vehicle_state.h
        include/CarPlanner/bullet/bullet_world_instance.h
        include/CarPlanner/bullet/car_wheel.h
        include/CarPlanner/bullet/gl_debug_drawer.h
        include/CarPlanner/bullet/raycast_vehicle.h
        include/CarPlanner/bullet/wheel_info.h
        include/CarPlanner/solvers/bezier_boundary_solver.h
        include/CarPlanner/solvers/car_regressor.h
        include/CarPlanner/solvers/local_planner.h
        include/CarPlanner/utils/cvar_helpers.h
        include/CarPlanner/utils/gui_exception.h
        include/CarPlanner/utils/optim.h
        include/CarPlanner/utils/thread_pool.h
        include/CarPlanner/utils/time.h
        include/CarPlanner/utils/vector.h
        )

string( TOLOWER ${PROJECT_NAME} LIBRARY_NAME )

def_library(${LIBRARY_NAME}
  SOURCES ${HDRS} ${SRCS}
  LINK_LIBS ${_LIBRARIES}
  PACKAGE CarPlanner
  )

export_package( CarPlanner
  TARGETS ${LIBRARY_NAME}
  LIBRARY ${LIBRARY_NAME}
  LIBRARIES ${LIBRARY_NAME}
  INCLUDE_DIRS ${_INCLUDE_DIRS}
  )

