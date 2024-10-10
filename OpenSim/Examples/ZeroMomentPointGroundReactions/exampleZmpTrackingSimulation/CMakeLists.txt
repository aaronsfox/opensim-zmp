set(EXAMPLE_ZMPTRACKINGSIMULATION_FILES
		Geometry
        subject01.osim
		Run_3_grf.mot
		Run_3_grf.xml
		ikTrackingData.sto
		README.txt)

OpenSimAddExampleCXX(NAME exampleZmpTrackingSimulation SUBDIR ZeroMomentPointGroundReactions
    EXECUTABLES exampleZmpTrackingSimulation
    RESOURCES "${EXAMPLE_ZMPTRACKINGSIMULATION_FILES}")
