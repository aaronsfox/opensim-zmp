set(EXAMPLE_BASICDEMO_FILES
		Geometry
        subject01_run3.osim
		subject01_run3_trackingSolution.sto
		README.txt)

OpenSimAddExampleCXX(NAME exampleBasicDemo SUBDIR Zmp
    EXECUTABLES exampleBasicDemo
    RESOURCES "${EXAMPLE_BASICDEMO_FILES}")
