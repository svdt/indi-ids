cmake_minimum_required(VERSION 3.16)
PROJECT(indi_ids C CXX)

set(INDI_IDS_VERSION_MAJOR 1)
set(INDI_IDS_VERSION_MINOR 0)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake_modules/")
include(GNUInstallDirs)


SET(BIN_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/bin")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall" )
SET(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -DDEBUG_FITS" )
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall" )
SET(CMAKE_C_FLAGS_DEBUG "-O0 -g -DDEBUG_FITS" )

set (CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH})

find_package(uEye-SDK 4.9 REQUIRED)
find_package(CFITSIO REQUIRED)
find_package(INDI REQUIRED)
find_package(ZLIB REQUIRED)
find_package(Threads REQUIRED)
find_package(LibRaw REQUIRED)
find_package(USB1 REQUIRED)


include(CMakeCommon)
include(CheckStructHasMember)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/config.h )
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/indi_ids.xml.cmake ${CMAKE_CURRENT_BINARY_DIR}/indi_ids.xml )

include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR})
include_directories( ${INDI_INCLUDE_DIR})
include_directories( ${uEye-SDK_INCLUDE_DIRS})
include_directories( ${CFITSIO_INCLUDE_DIR})
include_directories( ${LibRaw_INCLUDE_DIR})
include_directories( ${USB1_INCLUDE_DIRS})

########### IDS ###########
set(indiids_SRCS
   ${CMAKE_CURRENT_SOURCE_DIR}/ids_ccd.cpp
   ${CMAKE_CURRENT_SOURCE_DIR}/ids_driver.cpp
   )

IF (UNITY_BUILD)
    ENABLE_UNITY_BUILD(indiids indiids_SRCS 10 cpp)
ENDIF ()

add_executable(indi_ids_ccd ${indiids_SRCS})


message(${uEye-SDK_LIBRARIES})
target_link_libraries(indi_ids_ccd uEye-SDK)
target_link_libraries(indi_ids_ccd ${INDI_LIBRARIES} ${uEye-SDK_LIBRARIES} ${CFITSIO_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${JPEG_LIBRARIES} ${LibRaw_LIBRARIES} ${ZLIB_LIBRARIES})

if (HAVE_WEBSOCKET)
    target_link_libraries(indi_ids_ccd ${Boost_LIBRARIES})
endif()

install(TARGETS indi_ids_ccd RUNTIME DESTINATION bin )

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/indi_ids.xml DESTINATION ${INDI_DATA_DIR})


# Build the camera test application
add_executable(ids_camera_test
    ${CMAKE_CURRENT_SOURCE_DIR}/ids_camera_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ids_driver.cpp
)

target_link_libraries(ids_camera_test
    ${CMAKE_THREAD_LIBS_INIT}
    ${INDI_LIBRARIES}
    ${uEye-SDK_LIBRARIES}
    ${JPEG_LIBRARIES}
    ${LibRaw_LIBRARIES}
    ${ZLIB_LIBRARIES}
)

install(TARGETS ids_camera_test DESTINATION bin)