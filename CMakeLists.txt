cmake_minimum_required(VERSION 3.6)

project(csrt)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OPENCV_INC "/usr/local/include/opencv2/")
set(OPENCV_LIBS "/usr/local/lib/")
set(LIBS "-lopencv_core -lopencv_videoio -lopencv_imgproc -lopencv_highgui")
set(LIB_COMMON "-L${OPENCV_LIBS} -lgcov")
set(CMAKE_CXX_FLAGS "-march=native -O0 -g3 -Wall -I ${OPENCV_INC}")

file(GLOB SRCS *.cpp)
file(GLOB HEADERS *.hpp)

add_executable(csrt ${SRCS})

target_link_libraries(csrt PRIVATE ${LIB_COMMON} ${LIBS})

install(TARGETS csrt DESTINATION bin)

# # static build of the porject
# cmake_minimum_required(VERSION 3.6)

# project(csrt)

# # Set C++ standard
# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set(OpenCV_SHARED OFF)
# # Find OpenCV package
# set(BUILD_SHARED_LIBS OFF)
# find_package(OpenCV REQUIRED imgproc core)

# # Add executable
# file(GLOB SRCS *.cpp)
# file(GLOB HEADERS *.hpp)
# add_executable(csrt ${SRCS} ${HEADERS})

# # Link OpenCV and ITP libraries statically
# set(OpenCV_STATIC ON)
# set(ITP_LIB "/usr/local/share/OpenCV/3rdparty/lib/libittnotify.a")
# set(OPENCV_LIBS "/usr/local/lib/")
# target_link_libraries(csrt PRIVATE ${OPENCV_LIBS}/libopencv_imgproc.a ${OPENCV_LIBS}/libopencv_core.a ${ITP_LIB} -static -ldl -lz -lpthread)

# # Set compiler flags
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -O0 -g3 -Wall")

# # Install executable
# install(TARGETS csrt DESTINATION bin)

# # Cmake for opencv mobile
# cmake_minimum_required(VERSION 3.6)

# project(csrt)

# # Set C++ standard
# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# # using opencv-mobile
# set(OpenCV_DIR ${CMAKE_SOURCE_DIR}/opencv-mobile-3.4.18-ubuntu-2004/lib/OpenCV/) # Set the OpenCV directory
# find_package(OpenCV REQUIRED imgproc core) # Find OpenCV version 3.4.18

# # Add executable
# file(GLOB SRCS *.cpp)
# file(GLOB HEADERS *.hpp)
# add_executable(csrt ${SRCS} ${HEADERS})

# # Enable OpenMP support and link to the OpenMP library
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
# target_link_libraries(csrt PRIVATE /usr/lib/gcc/x86_64-linux-gnu/9/libgomp.a)

# # Link OpenCV libraries statically
# set(OpenCV_STATIC ON)
# set(OPENCV_LIBS "/home/nile649/Standalone_CSRT/opencv-mobile-3.4.18-ubuntu-2004/lib/")
# target_link_libraries(csrt PRIVATE ${OPENCV_LIBS}/libopencv_imgproc.a ${OPENCV_LIBS}/libopencv_core.a -static -ldl -lpthread)

# # Set compiler flags
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -O0 -g3 -Wall")

# # Install executable
# install(TARGETS csrt DESTINATION bin)
