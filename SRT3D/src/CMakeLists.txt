cmake_minimum_required(VERSION 3.10)

# Define sources
set(SOURCES common.cpp body.cpp renderer_geometry.cpp renderer.cpp 
    normal_renderer.cpp occlusion_renderer.cpp model.cpp camera.cpp
    loader_camera.cpp viewer.cpp image_viewer.cpp normal_viewer.cpp 
    region_modality.cpp tracker.cpp)
if(USE_AZURE_KINECT)
    list(APPEND SOURCES azure_kinect_camera.cpp)
endif()

# Define libraries
set(LIBRARIES Eigen3::Eigen OpenGL::GL GLEW::GLEW glfw ${OpenCV_LIBRARIES})
if(USE_AZURE_KINECT)
    list(APPEND LIBRARIES k4a::k4a)
endif()
if(UNIX)
    list(APPEND LIBRARIES pthread stdc++fs)
endif()

# Define target
add_library(srt3d STATIC ${SOURCES})
target_link_libraries(srt3d PUBLIC ${LIBRARIES})
