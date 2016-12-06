SET(LIB_GMOCK_SOURCE_DIR "${LIB_SOURCE_DIR}/gmock")
SET(LIB_GMOCK_SOURCE "${LIB_GMOCK_SOURCE_DIR}/gmock-all.cc")
SET_SOURCE_FILES_PROPERTIES(${LIB_GMOCK_SOURCE} PROPERTIES COMPILE_FLAGS "-w")
ADD_DEFINITIONS(-DGTEST_LANG_CXX11=1)

ADD_LIBRARY(gmock ${LIB_GMOCK_SOURCE} ${LIB_INCLUDE_DIR})

