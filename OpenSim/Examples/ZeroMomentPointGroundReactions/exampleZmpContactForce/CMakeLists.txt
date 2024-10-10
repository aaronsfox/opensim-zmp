set(EXAMPLE_ZMPCONTACTFORCE_FILES
		Geometry
        subject01_run3.osim
		subject01_run3_trackingSolution.sto
		Run_3_grf.mot
		README.txt)

OpenSimAddExampleCXX(NAME exampleZmpContactForce SUBDIR ZeroMomentPointGroundReactions
    EXECUTABLES exampleZmpContactForce
    RESOURCES "${EXAMPLE_ZMPCONTACTFORCE_FILES}")
