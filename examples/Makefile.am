SUBDIRS = dynamics

noinst_PROGRAMS = sample gldemo physics mnm 

sample_SOURCES = sample.cpp
gldemo_SOURCES = gldemo.cpp
physics_SOURCES = physics.cpp
mnm_SOURCES = mnm.cpp

GLLIBS = -lglut -lGLU -lGL -L/usr/X11R6/lib -lXmu -lXi -lX11

sample_LDADD = ../src/libsolid.la  
gldemo_LDADD = ../src/libsolid.la $(GLLIBS)
physics_LDADD = dynamics/libdynamics.la ../src/libsolid.la $(GLLIBS)
mnm_LDADD = dynamics/libdynamics.la ../src/libsolid.la $(GLLIBS)

AM_CPPFLAGS = -I$(top_srcdir)/include -I$(top_srcdir)/examples/dynamics

EXTRA_DIST = README.txt
