set(EXAMPLES_TO_COMPILE exampleZeroMomentPoint1_createAndConnectComponents.cpp
                        exampleZeroMomentPoint2_groundReactions.cpp
)

foreach(EXAMPLE_PROG ${EXAMPLES_TO_COMPILE})
    get_filename_component(EXAMPLE_ROOT ${EXAMPLE_PROG} NAME_WE)
    add_executable(${EXAMPLE_ROOT} ${EXAMPLE_PROG})
    target_link_libraries(${EXAMPLE_ROOT} ${Simbody_LIBRARIES} osimSimulation osimMoco)
    set_target_properties(${EXAMPLE_ROOT} PROPERTIES FOLDER "Examples")
endforeach(EXAMPLE_PROG ${EXAMPLES_TO_COMPILE})
