function(ZmpAddSandboxExecutable)
    set(options)
    set(oneValueArgs NAME)
    set(multiValueArgs LIB_DEPENDS RESOURCES SOURCE_FILES)
    cmake_parse_arguments(ZMPSAND
            "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    add_executable(${ZMPSAND_NAME} EXCLUDE_FROM_ALL ${ZMPSAND_NAME}.cpp 
            ${ZMPSAND_SOURCE_FILES})
    set_target_properties(${ZMPSAND_NAME} PROPERTIES
            FOLDER "Sandbox/Zmp")
    target_link_libraries(${ZMPSAND_NAME} ${ZMPSAND_LIB_DEPENDS})
    target_include_directories(${ZMPSAND_NAME} PRIVATE
            ${CMAKE_SOURCE_DIR}/Sandbox/Zmp/shared)
    file(COPY ${ZMPSAND_RESOURCES} DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
endfunction()
