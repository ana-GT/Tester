cmake_minimum_required (VERSION 2.6)
project (tabRipPlanner)

message (STATUS "Creating Makefile for the tabRipPlanner project")

# This include is used only if you have installed Grip
if(WIN32)
	set (CMAKE_INSTALL_PREFIX "C:/Golems" )
    message (STATUS "Setup Visual Studio Specific Flags")
	set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd /Zi /GL /Gy /W1")
	set (CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD /Zi /GL /Gy /W1")
	set (CMAKE_DEFAULT_STARTUP_PROJECT tabRipPlanner)
else ()
   set (CMAKE_INSTALL_PREFIX /usr/local )
   set (CMAKE_CXX_FLAGS "-g -O0 -Wall -Wextra -Wpointer-arith -fno-default-inline -fno-implicit-inline-templates -DwxUSE_THREADS")
endif()
include (${CMAKE_INSTALL_PREFIX}/share/dart/DART_config.cmake)
include (${CMAKE_INSTALL_PREFIX}/share/grip/GRIP_config.cmake)
# End include


# Search all header and source files
file (GLOB srcs "*.cpp")
file (GLOB hdrs "*.h")
file (GLOB kdtree_srcs kdtree/kdtree.c )

set (wxWidgets_USE_LIBS base core gl)
find_package (wxWidgets)
if (wxWidgets_FOUND)
  include (${wxWidgets_USE_FILE})
  include_directories(${DART_HEADERS} ${GRIP_HEADERS} ${WXWIDGETS_SHARED_INCLUDE_DIRS})
  link_directories(${CMAKE_SOURCE_DIR}/lib ${GRIP_LIBDIR} ${WXWIDGETS_SHARED_LIBRARY_DIRS})

  if(MSVC)
	add_executable (tabRipPlanner WIN32 ${srcs} ${hdrs} ${kdtree_srcs} )
	set_target_properties (tabRipPlanner PROPERTIES LINK_FLAGS "/SUBSYSTEM:WINDOWS")
   else()
    add_executable (tabRipPlanner ${srcs} ${hdrs} ${kdtree_srcs} )
   endif()

   target_link_libraries (tabRipPlanner ${GRIP_LIBRARIES} ${DART_LIBRARIES} ${wxWidgets_LIBRARIES} )
   
else (wxWidgets_FOUND)
  message ("wxWidgets not found!")
endif (wxWidgets_FOUND)






