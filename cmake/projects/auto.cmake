# build the oos_openwsn firmware

add_subdirectory(${CMAKE_SOURCE_DIR}/bsp/boards/${BOARD})
add_subdirectory(${CMAKE_SOURCE_DIR}/drivers)
add_subdirectory(${CMAKE_SOURCE_DIR}/openapps)
add_subdirectory(${CMAKE_SOURCE_DIR}/openweb)
add_subdirectory(${CMAKE_SOURCE_DIR}/openstack)
add_subdirectory(${CMAKE_SOURCE_DIR}/kernel)
# add_subdirectory(${CMAKE_SOURCE_DIR}/projects/common/01bsp_continuously_cal)

file(GLOB SUBDIRECTORIES LIST_DIRECTORIES true "${CMAKE_SOURCE_DIR}/projects/common/*")

foreach(subdirectory ${SUBDIRECTORIES})
    get_filename_component(subdirectory_name ${subdirectory} NAME)
    string(FIND "${subdirectory_name}" "${PROJECT}" STRING_POSITION)
    if(${STRING_POSITION} GREATER_EQUAL 0)
        add_subdirectory(${CMAKE_SOURCE_DIR}/projects/common/${subdirectory_name})
        message(${CMAKE_SOURCE_DIR}/projects/common/${subdirectory_name})
        break()
    endif()
endforeach()
