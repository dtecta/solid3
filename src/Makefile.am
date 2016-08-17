SUBDIRS = broad convex complex qhull 

lib_LTLIBRARIES = libsolid.la

libsolid_la_SOURCES = \
	DT_AlgoTable.h \
	DT_C-api.cpp \
	DT_Encounter.h \
	DT_Object.cpp \
	DT_Object.h \
	DT_RespTable.h \
	DT_Scene.cpp \
	DT_Scene.h \
	DT_Encounter.cpp \
	DT_Response.h \
	DT_RespTable.cpp

libsolid_la_LIBADD = \
	broad/libbroad.la \
	convex/libconvex.la \
	complex/libcomplex.la \
	qhull/libqhull.la	 

AM_CPPFLAGS = -I$(top_srcdir)/include -I$(top_srcdir)/src/convex -I$(top_srcdir)/src/complex @DOUBLES_FLAG@ @TRACER_FLAG@
