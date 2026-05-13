find_path(G2O_INCLUDE_DIR
  NAMES g2o/core/sparse_optimizer.h
)

find_library(G2O_CORE_LIBRARY
  NAMES g2o_core
)

find_library(G2O_STUFF_LIBRARY
  NAMES g2o_stuff
)

find_library(G2O_SOLVER_EIGEN_LIBRARY
  NAMES g2o_solver_eigen
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(G2O
  REQUIRED_VARS
    G2O_INCLUDE_DIR
    G2O_CORE_LIBRARY
    G2O_STUFF_LIBRARY
    G2O_SOLVER_EIGEN_LIBRARY
)

if(G2O_FOUND AND NOT TARGET G2O::all)
  add_library(G2O::all INTERFACE IMPORTED)
  set_target_properties(G2O::all PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${G2O_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES
      "${G2O_CORE_LIBRARY};${G2O_STUFF_LIBRARY};${G2O_SOLVER_EIGEN_LIBRARY}"
  )
endif()

set(G2O_INCLUDE_DIRS "${G2O_INCLUDE_DIR}")
set(G2O_LIBRARIES
  "${G2O_CORE_LIBRARY};${G2O_STUFF_LIBRARY};${G2O_SOLVER_EIGEN_LIBRARY}"
)
