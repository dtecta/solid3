add_subdirectory(qhull)

if(DYNAMIC_SOLID)
  set(LIBRARY_TYPE "SHARED")
  add_definitions(-DSOLID_DLL_EXPORT)
else(DYNAMIC_SOLID)
  set(LIBRARY_TYPE "STATIC")
  add_definitions(-DSOLID_STATIC)
endif(DYNAMIC_SOLID)

include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/src/broad)
include_directories(${PROJECT_SOURCE_DIR}/src/convex)
include_directories(${PROJECT_SOURCE_DIR}/src/complex)

set(SOLID_PUBLIC_HEADERS 
  ${PROJECT_SOURCE_DIR}/include/SOLID.h
  ${PROJECT_SOURCE_DIR}/include/SOLID_broad.h
  ${PROJECT_SOURCE_DIR}/include/SOLID_types.h
)

add_library(solid3 ${LIBRARY_TYPE}
  ${SOLID_PUBLIC_HEADERS}
  broad/BP_C-api.cpp
  broad/BP_Endpoint.h
  broad/BP_EndpointList.cpp
  broad/BP_EndpointList.h
  broad/BP_Proxy.cpp
  broad/BP_Proxy.h
  broad/BP_ProxyList.h
  broad/BP_Scene.cpp
  broad/BP_Scene.h
  convex/DT_Accuracy.cpp
  convex/DT_Accuracy.h
  convex/DT_Array.h
  convex/DT_Box.cpp
  convex/DT_Box.h
  convex/DT_Cone.cpp
  convex/DT_Cone.h
  convex/DT_Convex.cpp
  convex/DT_Convex.h
  convex/DT_Cylinder.cpp
  convex/DT_Cylinder.h
  convex/DT_GJK.h
  convex/DT_Hull.h
  convex/DT_IndexArray.h
  convex/DT_LineSegment.cpp
  convex/DT_LineSegment.h
  convex/DT_Minkowski.h
  convex/DT_PenDepth.cpp
  convex/DT_PenDepth.h
  convex/DT_Point.cpp
  convex/DT_Point.h
  convex/DT_Polyhedron.cpp
  convex/DT_Polyhedron.h
  convex/DT_Polytope.cpp
  convex/DT_Polytope.h
  convex/DT_Shape.h
  convex/DT_Sphere.cpp
  convex/DT_Sphere.h
  convex/DT_Transform.h
  convex/DT_Triangle.cpp
  convex/DT_Triangle.h
  convex/DT_TriEdge.cpp
  convex/DT_TriEdge.h
  convex/DT_VertexBase.h
  complex/DT_BBoxTree.cpp
  complex/DT_BBoxTree.h
  complex/DT_CBox.h
  complex/DT_Complex.cpp
  complex/DT_Complex.h
  DT_AlgoTable.h
  DT_C-api.cpp
  DT_Encounter.cpp
  DT_Encounter.h
  DT_Object.cpp
  DT_Object.h
  DT_Response.h
  DT_RespTable.cpp
  DT_RespTable.h
  DT_Scene.cpp
  DT_Scene.h
  $<TARGET_OBJECTS:qhull>
)

target_include_directories(solid3 INTERFACE $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>)
set_target_properties(solid3 PROPERTIES VERSION ${VERSION})

install(
    TARGETS solid3
    EXPORT solid3
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT runtime NAMELINK_SKIP
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT runtime
)
if(DYNAMIC_SOLID)
    install(
        TARGETS solid3
        EXPORT solid3
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT development NAMELINK_ONLY
    )
endif()
install(
    FILES ${SOLID_PUBLIC_HEADERS}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/SOLID
    COMPONENT development
)

if(MSVC AND DYNAMIC_SOLID AND ${CMAKE_MAJOR_VERSION} GREATER 2 AND ${CMAKE_MINOR_VERSION} GREATER 0)
    install(FILES $<TARGET_PDB_FILE:solid3> DESTINATION ${CMAKE_INSTALL_BINDIR} CONFIGURATIONS Debug RelWithDebInfo COMPONENT debug)
endif()
