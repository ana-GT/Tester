# Tester

cmake_minimum_required( VERSION 2.8 )
project ( Tester )

set (CMAKE_INSTALL_PREFIX /usr/local )
set (CMAKE_CXX_FLAGS "-std=C++0x -g -O0 -Wall -Wextra -Wpointer-arith -fno-default-inline -fno-implicit-inline-templates -DwxUSE_THREADS")

include (${CMAKE_INSTALL_PREFIX}/share/dart/DART_config.cmake)
include (${CMAKE_INSTALL_PREFIX}/share/grip/GRIP_config.cmake)

find_package( PCL 1.1 REQUIRED )

include_directories( /usr/include/eigen3
  ${PCL_INCLUDE_DIRS}
  . )

link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} ) 

message ( STATUS "Creating Makefile for Tester" )
set( CMAKE_CXX_FLAGS "-L /usr/local/lib" )


set( SRC_PLANNERS Planners/RRT/RRT.cpp
  Planners/RRT/PathPlanner.cpp
  kdtree/kdtree.c
  Planners/LJM2_Suite/myFunctions/CheckObject.cpp
  Planners/LJM2_Suite/myFunctions/CheckProcess.cpp
  Planners/LJM2_Suite/LJM2/LJM2.cpp 
  Planners/LJM2_Suite/LJM2/searchFunctionsFree.cpp
  Planners/LJM2_Suite/LJM2/searchFunctionsSurface.cpp
  Planners/JTFollower/JTFollower.cpp
  Planners/JNSFollower/JNSFollower.cpp )

set( HDR_PLANNERS Planners/RRT/RRT.h
  Planners/RRT/PathPlanner.h
  kdtree/kdtree.h
  Planners/LJM2_Suite/myFunctions/CheckObject.h
  Planners/LJM2_Suite/myFunctions/CheckProcess.h
  Planners/LJM2_Suite/LJM2/LJM2.h
  Planners/JTFollower/JTFollower.h
  Planners/JNSFollower/JNSFollower.h )

#set( SRC_LJM2 LJM2/LJM2.cpp 
#  LJM2/searchFunctionsFree.cpp
#  LJM2/searchFunctionsSurface.cpp
#  kdtree/kdtree.c )

#set( HDR_LJM2 LJM2/LJM2.h )

set( SRC_IK Planners/IK/IK.cpp
  Planners/IK/IKSearch.cpp
  Planners/IK/IKSearch_LA.cpp
  Planners/IK/IKSearch_BT2.cpp
  Planners/IK/IKSearch_BT.cpp
  Planners/IK/IKGradient.cpp)

set( HDR_IK Planners/IK/IK.h 
  Planners/IK/IKSearch.h
  Planners/IK/IKGradient.h)

set( SRC_UTILS Planners/IK/BinaryHeap.cpp )
set( HDR_UTILS Planners/IK/BinaryHeap.h )

set( SRC_FILES   TesterTabApp.cpp
  ConfigTab.cpp PlannerTab.cpp TrackerTab.cpp globalStuff.cpp )
			       
set ( HDR_FILES  ConfigTab.h PlannerTab.h TrackerTab.h globalStuff.h )

set( SRC_PLOT Plot/Plot.cpp )
set( HDR_PLOT Plot/Plot.h )

set (wxWidgets_USE_LIBS base core gl)
find_package (wxWidgets)
if (wxWidgets_FOUND)
  include (${wxWidgets_USE_FILE})
  include_directories(${DART_HEADERS} ${GRIP_HEADERS} ${WXWIDGETS_SHARED_INCLUDE_DIRS})
  link_directories(${CMAKE_SOURCE_DIR}/lib ${GRIP_LIBDIR} ${WXWIDGETS_SHARED_LIBRARY_DIRS} )
  
  add_executable ( TesterTab  ${HDR_FILES} ${SRC_FILES} ${SRC_PLANNERS} ${HDR_PLANNERS} ${HDR_IK} ${SRC_IK} ${SRC_PLOT} ${HDR_PLOT} ${SRC_UTILS} ${HDR_UTILS} )
  target_link_libraries( TesterTab ${PCL_LIBRARIES}
    ${GRIP_LIBRARIES} 
    ${DART_LIBRARIES} 
    ${wxWidgets_LIBRARIES} )	

   
else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)
