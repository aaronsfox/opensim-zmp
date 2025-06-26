
file(GLOB INCLUDES *.h)
file(GLOB SOURCES *.cpp)

OpenSimAddLibrary(
    KIT Zmp
    AUTHORS "Aaron Fox"
    LINKLIBS osimCommon osimSimulation osimActuators osimTools
    INCLUDES ${INCLUDES}
    SOURCES ${SOURCES}
    # TESTDIRS Test
    )

