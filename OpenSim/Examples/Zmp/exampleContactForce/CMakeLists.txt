set(EXAMPLE_CONTACTFORCE_FILES
		Geometry
        subject01_run3.osim
		subject01_run3_trackingSolution.sto
		Run_3_grf.mot
		README.txt)

OpenSimAddExampleCXX(NAME exampleContactForce SUBDIR Zmp
    EXECUTABLES exampleContactForce
    RESOURCES "${EXAMPLE_CONTACTFORCE_FILES}")
