set(EXAMPLE_ZMPFROMMOTION_FILES
		Geometry
        subject01_run3.osim
		subject01_run3_trackingSolution.sto
		Run_3_grf.mot
		README.txt)

OpenSimAddExampleCXX(NAME exampleZmpGroundReactionsFromMotion SUBDIR ZeroMomentPointGroundReactions
    EXECUTABLES exampleZmpGroundReactionsFromMotion
    RESOURCES "${EXAMPLE_ZMPFROMMOTION_FILES}")
