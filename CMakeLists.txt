cmake_minimum_required(VERSION 3.15)
project(LINE_MOD VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#---------------------------------------------
# Source and header files
#---------------------------------------------
file(GLOB_RECURSE SOURCE_FILES ${CMAKE_SOURCE_DIR}/src/*.c ${CMAKE_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE HEADER_FILES ${CMAKE_SOURCE_DIR}/include/*.h ${CMAKE_SOURCE_DIR}/include/*.hpp)

include_directories(${CMAKE_SOURCE_DIR}/include/)
add_definitions(-D GLM_ENABLE_EXPERIMENTAL=TRUE)

#---------------------------------------------
# Dependencies
#---------------------------------------------
find_package(SDL2 REQUIRED)
find_package(glm REQUIRED)
find_package(ASSIMP REQUIRED)
find_package(OpenGL REQUIRED)
find_package(OpenCV REQUIRED)

# GLEW workaround for Homebrew on macOS
# GLEW
set(GLEW_INCLUDE_DIRS /opt/homebrew/include)
set(GLEW_LIBRARIES /opt/homebrew/lib/libGLEW.dylib)
include_directories(${GLEW_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/ext/ntcore)
include_directories(${CMAKE_SOURCE_DIR}/ext/wpi)

# Include directories for found packages
include_directories(
    ${SDL2_INCLUDE_DIRS}
    ${GLM_INCLUDE_DIRS}
    ${ASSIMP_INCLUDE_DIRS}
    ${GLEW_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

#---------------------------------------------
# libfreenect2
#---------------------------------------------
set(FREENECT2_ROOT /opt/homebrew/Cellar/libfreenect2/0.2.1)
find_package(freenect2 REQUIRED PATHS ${FREENECT2_ROOT}/lib/cmake/freenect2)
include_directories(${freenect2_INCLUDE_DIR})

#---------------------------------------------
# Add executables
#---------------------------------------------
add_executable(Detector detector.cpp ${HEADER_FILES} ${SOURCE_FILES})
add_executable(Template_Generator templateGeneration.cpp ${HEADER_FILES} ${SOURCE_FILES})
add_executable(Arcuo_Benchmark arucoBenchmark.cpp ${HEADER_FILES} ${SOURCE_FILES})
add_executable(Main main.cpp ${HEADER_FILES} ${SOURCE_FILES})

#---------------------------------------------
# Link libraries
#---------------------------------------------
foreach(target IN ITEMS Detector Template_Generator Arcuo_Benchmark Main)
    target_link_libraries(${target}
        PRIVATE
            SDL2::SDL2
            SDL2::SDL2main
            glm::glm
            ${ASSIMP_LIBRARIES}
            OpenGL::GL
            ${GLEW_LIBRARIES}
            ${OpenCV_LIBS}
            ${freenect2_LIBRARY}         # main freenect2 library
            /opt/homebrew/lib/libturbojpeg.dylib  # required by freenect2
            /opt/homebrew/lib/libusb-1.0.dylib   # required by freenect2
            ${CMAKE_SOURCE_DIR}/ext/shared/libntcore.dylib
            ${CMAKE_SOURCE_DIR}/ext/shared/libntcorejni.dylib
            ${CMAKE_SOURCE_DIR}/ext/shared/libwpiutil.dylib
            ${CMAKE_SOURCE_DIR}/ext/shared/libwpiutiljni.dylib
            ${CMAKE_SOURCE_DIR}/ext/shared/libwpinet.dylib
            ${CMAKE_SOURCE_DIR}/ext/shared/libwpinetjni.dylib
    )

    target_include_directories(${target}
        PRIVATE
            ${SDL2_INCLUDE_DIRS}
            ${ASSIMP_INCLUDE_DIRS}
            ${GLEW_INCLUDE_DIRS}
            ${OpenCV_INCLUDE_DIRS}
            ${freenect2_INCLUDE_DIR}
            ${CMAKE_SOURCE_DIR}/ext/ntcore
            ${CMAKE_SOURCE_DIR}/ext/wpi
    )
endforeach()

#---------------------------------------------
# Optional: CPack
#---------------------------------------------
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
