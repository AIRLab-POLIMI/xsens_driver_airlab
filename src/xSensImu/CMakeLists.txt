cmake_minimum_required(VERSION 2.8.3)
PROJECT(xsens)

#add definitions, compiler switches, etc.
ADD_DEFINITIONS(-Wall)

#list all source files here
#ADD_EXECUTABLE(driver main.cpp interface.cpp)
#find_package(PkgConfig)

find_package(Eigen REQUIRED)
add_definitions (${EIGEN_DEFINITIONS})
include_directories(${EIGEN_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/xsensVendorLib)
add_subdirectory(xsensVendorLib)

add_library(xsens SHARED XsensDriver.cpp)
target_link_libraries(xsens xsensVendorLib) 

#ADD_EXECUTABLE(xsensTest XsensTest.cpp)
#TARGET_LINK_LIBRARIES(xsensTest xsens xsensVendorLib)
#ADD_EXECUTABLE(xsensCalibration XsensCalibration.cpp)
#TARGET_LINK_LIBRARIES(xsensCalibration xsens xsensVendorLib)
 
CONFIGURE_FILE(Doxyfile.in Doxyfile @ONLY)
#ADD_CUSTOM_TARGET(doc doxygen Doxyfile)
INSTALL(TARGETS xsens
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib)
INSTALL(FILES XsensDriver.hpp XsensTypes.hpp DESTINATION include)

CONFIGURE_FILE(xsens.pc.in xsens.pc @ONLY)
INSTALL(FILES ${CMAKE_BINARY_DIR}/xsens.pc DESTINATION lib/pkgconfig)

