cmake_minimum_required(VERSION 3.5)

# Check if Doxygen is installed
find_package(Doxygen COMPONENTS dot)

# Generate Doxygen documentation
if (DOXYGEN_FOUND)
    add_custom_target(doc_doxygen ALL
            COMMAND ${DOXYGEN_EXECUTABLE}
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            VERBATIM)
else ()
    message(WARNING "Doxygen not found")
endif ()
