# Compila o MESMO código C que o simulador (../../../core). Nada é copiado.
set(CORE_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../core)
idf_component_register(
    SRCS ${CORE_DIR}/src/cube_core.c ${CORE_DIR}/src/target_detect.c
         ${CORE_DIR}/src/cc_proto.c ${CORE_DIR}/src/moteus_proto.c
    INCLUDE_DIRS ${CORE_DIR}/include)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-parameter)
