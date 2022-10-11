cmake_minimum_required(VERSION 2.8)

project(atc3dglinux)
include(FetchContent)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(LibUSB)
include_directories(${PROJECT_SOURCE_DIR}/include)

# build shared library
add_library(atc3dg SHARED src/atc3dg.cpp)
target_link_libraries(atc3dg ${LIBUSB_LIBRARY})
set_target_properties(atc3dg
	PROPERTIES
	VERSION 0.0.1
	SOVERSION 0.0.1
)


# build example
add_executable(record examples/record.cpp)
target_link_libraries(record atc3dg)
set_target_properties(record PROPERTIES OUTPUT_NAME record)

FetchContent_Declare(
  OpenIGTLink
  GIT_REPOSITORY	https://github.com/openigtlink/OpenIGTLink
  GIT_TAG			d4eaae937b6a28ea2324f90c128800b3ad1cf2b3
)
FetchContent_MakeAvailable(OpenIGTLink)


# build igtlink server
find_package(OpenIGTLink REQUIRED)
include(${OpenIGTLink_USE_FILE})
add_executable(igtlink_server examples/igtlink_server.cpp)
target_link_libraries(igtlink_server atc3dg OpenIGTLink)
set_target_properties(igtlink_server PROPERTIES OUTPUT_NAME igtlink_server)


install(
	TARGETS atc3dg
	DESTINATION /usr/lib
	PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ
)

install(
	FILES include/atc3dg.h
	DESTINATION include
	PERMISSIONS OWNER_READ GROUP_READ WORLD_READ
)

install(
	FILES lib/udev/rules.d/99-libusb.rules
	DESTINATION lib/udev/rules.d
	PERMISSIONS OWNER_READ GROUP_READ WORLD_READ
)

# reload udev rules
install(
	CODE "execute_process(COMMAND udevadm control --reload-rules)"
)
install(
	CODE "execute_process(COMMAND udevadm trigger)"
)
