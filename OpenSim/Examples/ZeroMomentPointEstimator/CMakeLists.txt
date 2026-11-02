set(EXAMPLE_ZMP_FILES
        subject01_run3.osim
        subject01_run3_trackingSolution.sto
        Run_3_grf.mot
        README.md)

# Mesh files for visualizing the model, found by OpenSim in a Geometry folder
# next to the model file.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/Geometry")
    list(APPEND EXAMPLE_ZMP_FILES Geometry)
endif()

OpenSimAddExampleCXX(NAME exampleZeroMomentPointEstimator
        SUBDIR ZeroMomentPointEstimator
        RESOURCES "${EXAMPLE_ZMP_FILES}")
