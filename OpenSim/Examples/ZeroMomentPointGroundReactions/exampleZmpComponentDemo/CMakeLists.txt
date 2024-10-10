set(EXAMPLE_ZMPCOMPONENTDEMO_FILES
		Geometry
        subject01_run3.osim
		subject01_run3_trackingSolution.sto
		README.txt)

OpenSimAddExampleCXX(NAME exampleZmpComponentDemo SUBDIR ZeroMomentPointGroundReactions
    EXECUTABLES exampleZmpComponentDemo
    RESOURCES "${EXAMPLE_ZMPCOMPONENTDEMO_FILES}")
