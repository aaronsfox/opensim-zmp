set(EXAMPLE_FROMMOTION_FILES
		Geometry
        subject01_run3.osim
		subject01_run3_trackingSolution.sto
		Run_3_grf.mot
		README.txt)

OpenSimAddExampleCXX(NAME exampleGroundReactionsFromMotion SUBDIR Zmp
    EXECUTABLES exampleGroundReactionsFromMotion
    RESOURCES "${EXAMPLE_FROMMOTION_FILES}")
