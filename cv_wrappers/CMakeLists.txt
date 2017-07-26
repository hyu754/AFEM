file(GLOB SOURCE_FILES source/*.cpp)
file(GLOB HEADER_FILES include/*.h)
source_group("Headers" FILES ${HEADER_FILES})
ADD_LIBRARY(CV_CUSTOM_WRAPPER ${SOURCE_FILES} ${HEADER_FILES} )

target_link_libraries( CV_CUSTOM_WRAPPER ${OpenCV_LIBS} ${VTK_LIBRARIES})