if(NOT CMAKE_CROSSCOMPILING OR NOT CMAKE_C_COMPILER_ID STREQUAL "GNU")
    message(FATAL_ERROR "Embedded smoke requires the GNU ARM cross toolchain")
endif()
target_compile_options(manipulator_kinematics PRIVATE -ffunction-sections -fdata-sections -fstack-usage)
add_executable(manipulator_smoke tests/embedded/smoke.c tests/embedded/startup.c)
set_target_properties(manipulator_smoke PROPERTIES SUFFIX .elf
    C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO)
target_compile_options(manipulator_smoke PRIVATE -ffunction-sections -fdata-sections -fstack-usage)
target_link_options(manipulator_smoke PRIVATE -nostartfiles --specs=nosys.specs
    "-T${CMAKE_CURRENT_SOURCE_DIR}/tests/embedded/mps2_an386.ld"
    "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/manipulator_smoke.map,--gc-sections")
set_property(TARGET manipulator_smoke APPEND PROPERTY LINK_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/embedded/mps2_an386.ld")
target_link_libraries(manipulator_smoke PRIVATE manipulator::kinematics)
