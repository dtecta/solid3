add_subdirectory(dynamics)

add_executable(sample sample.cpp)
add_dependencies(sample solid3)
set_target_properties(sample PROPERTIES DEBUG_POSTFIX _d)
target_link_libraries(sample solid3)

set(DEPS dynamics solid3)

if(GLUT_FOUND)

include_directories(
  ${PROJECT_SOURCE_DIR}/examples/dynamics
  ${GLUT_INCLUDE_DIR} 
  ${OPENGL_INCLUDE_DIR}
)

foreach(EXE gldemo mnm physics)
add_executable(${EXE} ${EXE}.cpp)
add_dependencies(${EXE} ${DEPS})
set_target_properties(${EXE} PROPERTIES DEBUG_POSTFIX _d)
target_link_libraries(${EXE} 
  ${DEPS}
  ${GLUT_LIBRARIES}
  ${OPENGL_LIBRARIES}
)
endforeach(EXE)

endif(GLUT_FOUND)
