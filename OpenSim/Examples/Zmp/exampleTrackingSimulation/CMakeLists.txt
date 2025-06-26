set(EXAMPLE_TRACKINGSIMULATION_FILES
		Geometry
        subject01.osim
		Run_3_grf.mot
		Run_3_grf.xml
		ikTrackingData.sto
		README.txt)

OpenSimAddExampleCXX(NAME exampleTrackingSimulation SUBDIR Zmp
    EXECUTABLES exampleTrackingSimulation
    RESOURCES "${EXAMPLE_TRACKINGSIMULATION_FILES}")
