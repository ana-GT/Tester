# Tester

cmake_minimum_required( VERSION 2.8 )
project ( Tester )

set (CMAKE_INSTALL_PREFIX /usr/local )
set (CMAKE_CXX_FLAGS "-L /usr/local/lib" )

include (${CMAKE_INSTALL_PREFIX}/share/dart/DART_config.cmake)
include (${CMAKE_INSTALL_PREFIX}/share/grip/GRIP_config.cmake)

find_package( PCL 1.1 REQUIRED )

include_directories( /usr/include/eigen3
  ${PCL_INCLUDE_DIRS}
  . )

link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} ) 

message ( STATUS "Creating Makefile for Paper" )

set( SRC_FILES   PaperApp.cpp
  ConfigTab.cpp globalStuff.cpp )
			       
set ( HDR_FILES  ConfigTab.h globalStuff.h )

set (wxWidgets_USE_LIBS base core gl)
find_package (wxWidgets)
if (wxWidgets_FOUND)
  include (${wxWidgets_USE_FILE})
  include_directories(${DART_HEADERS} ${GRIP_HEADERS} ${WXWIDGETS_SHARED_INCLUDE_DIRS})
  link_directories(${CMAKE_SOURCE_DIR}/lib ${GRIP_LIBDIR} ${WXWIDGETS_SHARED_LIBRARY_DIRS} )
    add_executable ( Paper  ${HDR_FILES} ${SRC_FILES} )
  target_link_libraries( Paper ${PCL_LIBRARIES}
    ${GRIP_LIBRARIES} 
    ${DART_LIBRARIES} 
    ${wxWidgets_LIBRARIES} )	

   
else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)
