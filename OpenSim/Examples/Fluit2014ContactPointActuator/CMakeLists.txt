# --------------------------------------------------------------------------- #
#        ExampleFluit2014ContactPointActuator — CMakeLists.txt                #
# --------------------------------------------------------------------------- #
# Registers this example with the OpenSim build system using the
# OpenSimAddExampleCXX macro defined in cmake/OpenSimMacros.cmake.
#
# When BUILD_API_EXAMPLES is ON, this example is compiled as a standalone
# executable The data files in the data/ subdirectory are copied to the build
# tree automatically.
#
# The example is also installed to the OpenSim installation directory so
# that users who install a binary distribution can build it independently
# using the installed SampleCMakeLists.txt as a template.
# --------------------------------------------------------------------------- #

set(EXAMPLE_FILES
        subject01_run3.osim
        README.MD)
 
OpenSimAddExampleCXX(
    NAME        exampleFluit2014ContactPointActuator
    SUBDIR      ExampleFluit2014ContactPointActuator
    EXECUTABLES exampleFluit2014ContactPointActuator
    RESOURCES   "${EXAMPLE_FILES}")
 
# Copy data files to the build tree so the executable can find them
# at runtime. Destination matches where the executable is placed.
file(COPY ${EXAMPLE_FILES}
    DESTINATION
    "${CMAKE_BINARY_DIR}/OpenSim/Examples/ExampleFluit2014ContactPointActuator")
 
# Copy Geometry folder separately since it is a directory not a file
file(COPY Geometry
    DESTINATION
    "${CMAKE_BINARY_DIR}/OpenSim/Examples/ExampleFluit2014ContactPointActuator")
