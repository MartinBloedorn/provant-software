INCLUDE(CMakeForceCompiler)

# the name of the target operating system
SET(CMAKE_SYSTEM_NAME Generic)

SET(CPREF arm-none-eabi)
SET(CMAKE_C_COMPILER ${CPREF}-gcc)
SET(CMAKE_CXX_COMPILER ${CPREF}-g++)
#SET(AS "arm-none-eabi-gcc -x -assembler-with-cpp")
SET(AS ${CPREF}-as)
SET(AR ${CPREF}-ar)
SET(LD ${CPREF}-ld)
SET(NM ${CPREF}-nm)
SET(OBJCOPY ${CPREF}-objcopy)
SET(OBJDUMP ${CPREF}-objdump)
SET(READELF ${CPREF}-readelf)

CMAKE_FORCE_C_COMPILER(${CPREF}-gcc GNU)
CMAKE_FORCE_CXX_COMPILER(${CPREF}-g++ GNU)
