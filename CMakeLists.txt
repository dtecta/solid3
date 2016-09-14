cmake_minimum_required(VERSION 2.8.8)

project(SOLID)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules;${CMAKE_MODULE_PATH}")

include(GNUInstallDirs)

set(VERSION_MAJOR 3)
set(VERSION_MINOR 5)
set(VERSION_PATCH 8)
set(VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})

set(CMAKE_DEBUG_POSTFIX  "_d")

set(OUTPUT_BINDIR ${PROJECT_BINARY_DIR}/bin)
make_directory(${OUTPUT_BINDIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_BINDIR})

set(OUTPUT_LIBDIR ${PROJECT_BINARY_DIR}/lib)
make_directory(${OUTPUT_LIBDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${OUTPUT_LIBDIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${OUTPUT_LIBDIR})


# Dynamic vs Static Linking
option(DYNAMIC_SOLID "Set to ON to build SOLID for dynamic linking.  Use OFF for static." ON)

if(NOT DYNAMIC_SOLID)
  add_definitions(-DSOLID_STATIC)
endif(NOT DYNAMIC_SOLID)

option(USE_DOUBLES "Use double-precision floating-point numbers." OFF)

if(USE_DOUBLES)
  add_definitions(-DUSE_DOUBLES)
endif(USE_DOUBLES)

include_directories(
	${PROJECT_SOURCE_DIR}/include
)

if (WIN32)
  if (CMAKE_SIZEOF_VOID_P EQUAL 8)
	set(ARCH "x64")		
  else()
    set(ARCH "x86")	
  endif()	
  if (MSVC)
    if (MSVC90)
      set(PLATFORM "msvc90")
    elseif(MSVC10)
      set(PLATFORM "msvc100")
    elseif(MSVC11) 	
      set(PLATFORM "msvc110")
    elseif(MSVC12)
      set(PLATFORM "msvc120")
	elseif(MSVC14)
      set(PLATFORM "msvc140")
    else()
      set(PLATFORM "msvcxxx")
    endif()
  else()
    set(PLATFORM "unknown")
  endif ()

  set(GLUT_ROOT_PATH "${PROJECT_SOURCE_DIR}/../3rdPartyDeps/${ARCH}/${PLATFORM}")
endif(WIN32)


find_package(GLUT)
find_package(OpenGL)


if(MSVC) 
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /W3 /Oi /fp:fast /D_SCL_SECURE_NO_WARNINGS /D_CRT_SECURE_NO_WARNINGS")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /GR- /W3 /Oi /fp:fast /D_SCL_SECURE_NO_WARNINGS /D_CRT_SECURE_NO_WARNINGS")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /GS-") 
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /GS-") 
  set(CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO "${CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO} /INCREMENTAL:NO")
  set(CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO "${CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO} /INCREMENTAL:NO")
  if(CMAKE_CL_64)
  else(CMAKE_CL_64)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /arch:SSE2")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:SSE2")
  endif(CMAKE_CL_64)
endif(MSVC)

if(UNIX)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -ffast-math -msse2 -mfpmath=sse")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-rtti -Wall -ffast-math -msse2 -mfpmath=sse")
  if (DYNAMIC_SOLID)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
  endif(DYNAMIC_SOLID)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lrt")
endif(UNIX)

add_subdirectory(src)
add_subdirectory(examples)
#add_subdirectory(doc)

include(CMakePackageConfigHelpers)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/solid3-config.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/solid3-config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/solid3
)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/solid3-config.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/solid3)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/solid3-config-version.cmake VERSION ${VERSION} COMPATIBILITY ExactVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/solid3-config-version.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/solid3)
install(EXPORT solid3 DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/solid3 NAMESPACE solid3:: FILE solid3-export.cmake)
