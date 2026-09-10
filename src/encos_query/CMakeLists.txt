cmake_minimum_required(VERSION 3.16)
project(encos_query C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

include(FetchContent)

# Public, upstream dependency. Keeping it external prevents the private
# supplier distribution from being copied into this repository.
FetchContent_Declare(
    soem
    GIT_REPOSITORY https://github.com/OpenEtherCATsociety/SOEM.git
    GIT_TAG abbf0d42e38d6cfbaa4c1e9e8e07ace651c386fd # v1.4.0
    GIT_SHALLOW TRUE
)
FetchContent_GetProperties(soem)
if(NOT soem_POPULATED)
    FetchContent_Populate(soem)
endif()

set(SOEM_ROOT "${soem_SOURCE_DIR}")
file(GLOB SOEM_SOURCES "${SOEM_ROOT}/soem/*.c")
file(GLOB OSAL_SOURCES "${SOEM_ROOT}/osal/linux/*.c")
file(GLOB OSHW_SOURCES "${SOEM_ROOT}/oshw/linux/*.c")

add_library(encos_soem STATIC ${SOEM_SOURCES} ${OSAL_SOURCES} ${OSHW_SOURCES})
target_include_directories(encos_soem PUBLIC
    "${SOEM_ROOT}/soem"
    "${SOEM_ROOT}/osal"
    "${SOEM_ROOT}/osal/linux"
    "${SOEM_ROOT}/oshw/linux"
)
target_link_libraries(encos_soem PUBLIC pthread rt)

add_library(encos_protocol STATIC encos_protocol.c)
target_include_directories(encos_protocol PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(encos_protocol PUBLIC m)

add_executable(encos_query main.c)
target_compile_options(encos_query PRIVATE -Wall -Wextra -Werror)
target_link_libraries(encos_query PRIVATE encos_protocol encos_soem)

add_executable(encos_motion motion.c)
target_compile_options(encos_motion PRIVATE -Wall -Wextra -Werror)
target_link_libraries(encos_motion PRIVATE encos_protocol encos_soem)

add_executable(encos_assign_id assign_id.c)
target_compile_options(encos_assign_id PRIVATE -Wall -Wextra -Werror)
target_link_libraries(encos_assign_id PRIVATE encos_protocol encos_soem)

enable_testing()
add_executable(encos_protocol_test ../../tests/encos_protocol_test.c)
target_compile_options(encos_protocol_test PRIVATE -Wall -Wextra -Werror)
target_link_libraries(encos_protocol_test PRIVATE encos_protocol)
add_test(NAME encos_protocol_test COMMAND encos_protocol_test)
