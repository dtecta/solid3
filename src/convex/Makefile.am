noinst_LTLIBRARIES = libconvex.la

libconvex_la_SOURCES = \
	DT_Accuracy.cpp	\
	DT_Accuracy.h \
	DT_Array.h \
	DT_Box.cpp \
	DT_Box.h \
	DT_Cone.cpp \
	DT_Cone.h \
	DT_Convex.cpp \
	DT_Convex.h \
	DT_Cylinder.cpp \
	DT_Cylinder.h \
	DT_GJK.h \
	DT_Hull.h \
	DT_IndexArray.h \
	DT_LineSegment.cpp \
	DT_LineSegment.h \
	DT_Minkowski.h \
	DT_PenDepth.cpp \
	DT_PenDepth.h \
	DT_Point.cpp \
	DT_Point.h \
	DT_Polyhedron.cpp \
	DT_Polyhedron.h \
	DT_Polytope.cpp \
	DT_Polytope.h \
	DT_Shape.h \
	DT_Sphere.cpp \
	DT_Sphere.h \
	DT_Transform.h \
	DT_TriEdge.cpp \
	DT_TriEdge.h \
	DT_Triangle.cpp \
	DT_Triangle.h \
	DT_VertexBase.h

AM_CPPFLAGS = -I$(top_srcdir)/include @DOUBLES_FLAG@ @TRACER_FLAG@
libconvex_la_LIBADD = @QHULL_LIBS@