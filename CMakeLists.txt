cmake_minimum_required(VERSION 3.14)

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


# build application executables
add_executable(record applications/record.cpp)
target_link_libraries(record atc3dg)
set_target_properties(record PROPERTIES OUTPUT_NAME record)

FetchContent_Declare(
  OpenIGTLink
  GIT_REPOSITORY	https://github.com/openigtlink/OpenIGTLink
  # was the most recent commit hash on 11.10.2022
  GIT_TAG			d4eaae937b6a28ea2324f90c128800b3ad1cf2b3
)
FetchContent_MakeAvailable(OpenIGTLink)

FetchContent_Declare(
  cli11
  GIT_REPOSITORY https://github.com/CLIUtils/CLI11
  GIT_TAG        v2.2.0
)
FetchContent_MakeAvailable(cli11)


# build igtlink server
#find_package(IGSIO REQUIRED)
find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})

find_package(OpenIGTLink REQUIRED)
include(${OpenIGTLink_USE_FILE})
include_directories(${cli11_SOURCE_DIR}/include)
add_executable(atcigtlinkserver applications/igtlink_server.cpp)
target_link_libraries(atcigtlinkserver atc3dg OpenIGTLink ${VTK_LIBRARIES})
set_target_properties(atcigtlinkserver PROPERTIES OUTPUT_NAME atcigtlinkserver)


install(
	TARGETS atc3dg
	DESTINATION /usr/lib
	PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ
)

install(
	TARGETS atcigtlinkserver
	DESTINATION /usr/bin
	PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

install(
	FILES include/atc3dg.hpp
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
