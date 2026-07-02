
add_subdirectory(Common)
add_subdirectory(Simulation)
add_subdirectory(Actuators)
add_subdirectory(Analyses)
add_subdirectory(ExampleComponents)
add_subdirectory(Tools)
add_subdirectory(Moco)
if (BUILD_API_EXAMPLES)
  add_subdirectory(Examples)
endif()
if (BUILD_TESTING)
	add_subdirectory(Tests)
endif()

#add_subdirectory(Sandbox)

install(FILES OpenSim.h DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/OpenSim")
