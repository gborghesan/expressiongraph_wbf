#SET(qpOASES qpOASES-3.0beta)
#INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/external/qpOASES/${qpOASES}/include)

# compile qpOASES libraries
set(qpOASES qpOASES)
FILE(GLOB SRC ${qpOASES}/src/*.cpp )
FILE(GLOB EXTRAS_SRC ${qpOASES}/src/extras/*.cpp )

add_definitions( -D__NO_COPYRIGHT__ )
rosbuild_add_library(qpOASES ${SRC} ${EXTRAS_SRC})

