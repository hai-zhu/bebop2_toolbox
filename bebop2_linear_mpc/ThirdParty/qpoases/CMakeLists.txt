ADD_LIBRARY(qpoases STATIC
  SRC/Bounds.cpp
  SRC/Constraints.cpp
  SRC/CyclingManager.cpp
  SRC/Indexlist.cpp
  SRC/MessageHandling.cpp
  SRC/QProblem.cpp
  SRC/QProblemB.cpp
  SRC/SubjectTo.cpp
  SRC/Utils.cpp
  SRC/EXTRAS/SolutionAnalysis.cpp
)

TARGET_INCLUDE_DIRECTORIES(qpoases PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/INCLUDE/EXTRAS ${CMAKE_CURRENT_SOURCE_DIR}/SRC)

TARGET_LINK_LIBRARIES(qpoases exportFolder)
