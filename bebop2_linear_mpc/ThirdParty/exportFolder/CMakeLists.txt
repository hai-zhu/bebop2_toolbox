ADD_LIBRARY(exportFolder STATIC
  acado_auxiliary_functions.c
  acado_auxiliary_functions.h
  acado_integrator.c
  acado_common.h
  acado_qpoases_interface.cpp
  acado_qpoases_interface.hpp
  acado_solver.c
)

TARGET_INCLUDE_DIRECTORIES(exportFolder PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

TARGET_LINK_LIBRARIES(exportFolder qpoases)

TARGET_INCLUDE_DIRECTORIES(exportFolder PUBLIC ../qpoases)

