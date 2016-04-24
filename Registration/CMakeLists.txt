CMAKE_MINIMUM_REQUIRED(VERSION 2.8.4)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

file(GLOB SOURCE_FILES *.cpp)
file(GLOB HEADER_FILES *.h)

# Find OpenCV
find_package(OpenCV REQUIRED )


add_executable(convertCoord ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries( sample_pcTest ${OpenCV_LIBS} )