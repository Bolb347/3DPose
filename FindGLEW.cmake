# FindGLEW.cmake - helper for projects missing proper find scripts
find_path(GLEW_INCLUDE_DIR GL/glew.h
    PATHS /opt/homebrew/include /usr/include /usr/local/include)

find_library(GLEW_LIBRARY
    NAMES GLEW glew32 glew
    PATHS /opt/homebrew/lib /usr/lib /usr/local/lib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GLEW DEFAULT_MSG
    GLEW_LIBRARY GLEW_INCLUDE_DIR)

mark_as_advanced(GLEW_INCLUDE_DIR GLEW_LIBRARY)
