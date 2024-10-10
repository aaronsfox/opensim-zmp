set(EXAMPLE_ZMPFROMSTATE_FILES
		Geometry
        subject01_run3.osim
		subject01_run3_trackingSolution.sto
		Run_3_grf.mot
		README.txt)

OpenSimAddExampleCXX(NAME exampleZmpGroundReactionsFromState SUBDIR ZeroMomentPointGroundReactions
    EXECUTABLES exampleZmpGroundReactionsFromState
    RESOURCES "${EXAMPLE_ZMPFROMSTATE_FILES}")
