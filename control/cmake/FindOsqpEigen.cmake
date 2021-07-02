#.rst:
# FindOsqpEigen
# -----------
#
# Try to find the OsqpEigen library.
# Once done this will define the following variables::
#
#  OsqpEigen_FOUND         - System has OsqpEigen
#  OsqpEigen_INCLUDE_DIRS  - OsqpEigen include directory
#  OsqpEigen_LIBRARIES     - OsqpEigen libraries


include(FindPackageHandleStandardArgs)

find_path(OsqpEigen_INCLUDEDIR
          NAMES OsqpEigen.hpp
          HINTS "${OsqpEigen_SOURCE_DIR}"
                ENV OsqpEigen_SOURCE_DIR
          PATH_SUFFIXES include)
          
find_library(OsqpEigen_LIB
             NAMES OsqpEigen
             HINTS "${OsqpEigen_BINARY_DIR}"
                   ENV OsqpEigen_BINARY_DIR
             PATH_SUFFIXES lib
                           libs)

set(OsqpEigen_INCLUDE_DIRS /home/mori/Desktop/Projects/solvers/osqp-eigen/build/include/)
set(OsqpEigen_LIBRARIES /home/mori/Desktop/Projects/solvers/osqp-eigen/build/lib/)

find_package_handle_standard_args(OsqpEigen DEFAULT_MSG OsqpEigen_LIBRARIES
                                                      OsqpEigen_INCLUDE_DIRS)
set(OsqpEigen_FOUND ${OsqpEigen_FOUND})