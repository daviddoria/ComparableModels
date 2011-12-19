cmake_minimum_required(VERSION 2.6)

PROJECT(ComparableModels)

####################################
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})

######################################
FIND_PACKAGE(VXL REQUIRED)
INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)

#######################################
set(Boost_USE_MULTITHREADED ON)
FIND_PACKAGE(Boost 1.38 COMPONENTS program_options required)

set(PROJECTS /media/portable/Projects/src)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp") #g++

INCLUDE_DIRECTORIES(${INCLUDE_DIRECTORIES}
${PROJECTS}
${Boost_INCLUDE_DIRS}
)

LINK_DIRECTORIES(${LINK_DIRECTORIES} 
${Boost_LIBRARY_DIRS}
)

#############################
	
set(Sources
${PROJECTS}/Geometry/Angles.cpp
${PROJECTS}/Geometry/Color.cpp
${PROJECTS}/Geometry/Edge.cpp
${PROJECTS}/Geometry/Ray.cpp
${PROJECTS}/Geometry/Triangle.cpp
${PROJECTS}/Geometry/Transformation.cpp
${PROJECTS}/Geometry/LineSegment.cpp
${PROJECTS}/Geometry/OrientedPoint.cpp
${PROJECTS}/Geometry/Geometry.cpp
${PROJECTS}/Geometry/Helpers.cpp

${PROJECTS}/ModelFile/ModelFile.cpp

${PROJECTS}/LidarScan/LidarPoint.cpp 
${PROJECTS}/LidarScan/LidarStrip.cpp 
${PROJECTS}/LidarScan/LidarScan.cpp 
${PROJECTS}/LidarScan/LidarScanner.cpp
${PROJECTS}/LidarScan/ScanParamsClass.cpp

${PROJECTS}/ObjViewer/CameraClass.h
${PROJECTS}/VXLHelpers/VXLHelpers.cpp
${PROJECTS}/Integration/integration.cpp

${PROJECTS}/Tools/Tools.cpp

${PROJECTS}/VTKHelpers/Octree.cpp
${PROJECTS}/KDTree/KDTree.cpp

)

#######################
set(Libraries
gomp
vnl vnl_algo vul vgl vgl_algo vil #rrel
${VTK_LIBRARIES}
boost_program_options-mt
)

ADD_LIBRARY(MyLibs ${Sources})
TARGET_LINK_LIBRARIES(MyLibs ${Libraries})

##############################
ADD_EXECUTABLE(Test Test.cpp ComparableModels.cpp)

TARGET_LINK_LIBRARIES(Test MyLibs vnl)
