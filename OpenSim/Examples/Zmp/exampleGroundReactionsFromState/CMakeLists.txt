set(EXAMPLE_FROMSTATE_FILES
		Geometry
        subject01_run3.osim
		subject01_run3_trackingSolution.sto
		Run_3_grf.mot
		README.txt)

OpenSimAddExampleCXX(NAME exampleGroundReactionsFromState SUBDIR Zmp
    EXECUTABLES exampleGroundReactionsFromState
    RESOURCES "${EXAMPLE_FROMSTATE_FILES}")
