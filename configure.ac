#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ(2.57)
AC_INIT(SOLID, 3.5)
AC_CONFIG_SRCDIR([include/SOLID.h])
AC_CONFIG_HEADER([include/config.h])
AM_INIT_AUTOMAKE

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC
AC_PROG_INSTALL
AC_PROG_LIBTOOL

AC_MSG_CHECKING(whether to use double-precision floating-point numbers)
doubles_default="no"
AC_ARG_ENABLE(doubles, [  --enable-doubles=[no/yes] use double-precision floating-point numbers
                       [default=$doubles_default]],, enable_doubles=$doubles_default)
if test "x$enable_doubles" = "xyes"; then
   DOUBLES_FLAG="-DUSE_DOUBLES"
AC_MSG_RESULT(yes)		
else
   DOUBLES_FLAG=""	
AC_MSG_RESULT(no)
fi
AC_SUBST([DOUBLES_FLAG])

AC_MSG_CHECKING(whether to use a rounding error tracer)
tracer_default="no"
AC_ARG_ENABLE(tracer, [  --enable-tracer=[no/yes] use rounding error tracer
                       [default=$tracer_default]],, enable_tracer=$tracer_default)
if test "x$enable_tracer" = "xyes"; then
   TRACER_FLAG="-DUSE_TRACER"
AC_MSG_RESULT(yes)		
else
   TRACER_FLAG=""	
AC_MSG_RESULT(no)
fi
AC_SUBST([TRACER_FLAG])

AC_MSG_CHECKING(whether to enable debugging)
debug_default="no"
AC_ARG_ENABLE(debug, [  --enable-debug=[no/yes] turn on debugging
                       [default=$debug_default]],, enable_debug=$debug_default)
if test "x$enable_debug" = "xyes"; then
   CFLAGS="-g"
   CXXFLAGS="-g"
AC_MSG_RESULT(yes)
else
   CFLAGS="-O2 -fno-gcse -DNDEBUG"	
   CXXFLAGS="-O2 -fno-gcse -DNDEBUG"
AC_MSG_RESULT(no)
fi

AC_CHECK_LIB([glut], [glutInit])

# Checks for libraries.
AC_CHECK_HEADERS(qhull/qhull_a.h)
AC_CHECK_LIB(qhull, qh_qhull, s_have_qhull=yes)
if test "X${s_have_qhull}" = Xyes; then
  QHULL_LIBS="-lqhull"
else
  QHULL_LIBS=""
fi
AC_SUBST([QHULL_LIBS])

# Checks for header files.
AC_HEADER_STDC
AC_CHECK_HEADERS([float.h limits.h stdlib.h string.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_HEADER_STDBOOL
AC_C_CONST
AC_C_INLINE
AC_TYPE_SIZE_T

# Checks for library functions.
AC_CHECK_FUNCS([memset sqrt])

AC_CONFIG_FILES([Makefile
                 doc/Makefile
				 src/broad/Makefile
				 src/convex/Makefile
				 src/complex/Makefile
				 src/qhull/Makefile
                 src/Makefile
				 include/Makefile
				 examples/dynamics/Makefile
				 examples/Makefile])
AC_OUTPUT
