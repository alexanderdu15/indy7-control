cmake_minimum_required(VERSION 3.10)
project(indy7_a2r)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Find required packages
find_package(Protobuf CONFIG REQUIRED)
message(STATUS "Using protobuf ${Protobuf_VERSION}")
find_package(gRPC CONFIG REQUIRED)
message(STATUS "Using gRPC ${gRPC_VERSION}")

# Set gRPC and Protobuf variables
set(_PROTOBUF_LIBPROTOBUF protobuf::libprotobuf)
set(_REFLECTION gRPC::grpc++_reflection)
set(_GRPC_GRPCPP gRPC::grpc++)

set(_NEUROMEKA_CPP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/neuromeka-package/cpp/neuromeka_cpp)

include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${gRPC_INCLUDE_DIRS})
include_directories(${_NEUROMEKA_CPP_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

set(INDY_SOURCES
    ${_NEUROMEKA_CPP_DIR}/indydcp3.cpp
    ${_NEUROMEKA_CPP_DIR}/proto/common_msgs.pb.cc
    ${_NEUROMEKA_CPP_DIR}/proto/config_msgs.pb.cc
    ${_NEUROMEKA_CPP_DIR}/proto/control_msgs.pb.cc
    ${_NEUROMEKA_CPP_DIR}/proto/device_msgs.pb.cc
    ${_NEUROMEKA_CPP_DIR}/proto/ethercat_msgs.pb.cc
    ${_NEUROMEKA_CPP_DIR}/proto/moby_msgs.pb.cc
    ${_NEUROMEKA_CPP_DIR}/proto/rtde_msgs.pb.cc
    ${_NEUROMEKA_CPP_DIR}/proto/hri_msgs.pb.cc
    ${_NEUROMEKA_CPP_DIR}/proto/shared_msgs.pb.cc

    ${_NEUROMEKA_CPP_DIR}/proto/common_msgs.grpc.pb.cc
    ${_NEUROMEKA_CPP_DIR}/proto/config_msgs.grpc.pb.cc
    ${_NEUROMEKA_CPP_DIR}/proto/control_msgs.grpc.pb.cc
    ${_NEUROMEKA_CPP_DIR}/proto/device_msgs.grpc.pb.cc
    ${_NEUROMEKA_CPP_DIR}/proto/ethercat_msgs.grpc.pb.cc
    ${_NEUROMEKA_CPP_DIR}/proto/moby_msgs.grpc.pb.cc
    ${_NEUROMEKA_CPP_DIR}/proto/rtde_msgs.grpc.pb.cc
    ${_NEUROMEKA_CPP_DIR}/proto/hri_msgs.grpc.pb.cc
    ${_NEUROMEKA_CPP_DIR}/proto/shared_msgs.grpc.pb.cc

    ${_NEUROMEKA_CPP_DIR}/proto/config.pb.cc
    ${_NEUROMEKA_CPP_DIR}/proto/control.pb.cc
    ${_NEUROMEKA_CPP_DIR}/proto/device.pb.cc
    ${_NEUROMEKA_CPP_DIR}/proto/ethercat.pb.cc
    ${_NEUROMEKA_CPP_DIR}/proto/eyetask.pb.cc
    ${_NEUROMEKA_CPP_DIR}/proto/indyeye.pb.cc
    ${_NEUROMEKA_CPP_DIR}/proto/moby.pb.cc
    ${_NEUROMEKA_CPP_DIR}/proto/rtde.pb.cc
    ${_NEUROMEKA_CPP_DIR}/proto/cri.pb.cc
    ${_NEUROMEKA_CPP_DIR}/proto/hri.pb.cc

    ${_NEUROMEKA_CPP_DIR}/proto/config.grpc.pb.cc
    ${_NEUROMEKA_CPP_DIR}/proto/control.grpc.pb.cc
    ${_NEUROMEKA_CPP_DIR}/proto/device.grpc.pb.cc
    ${_NEUROMEKA_CPP_DIR}/proto/ethercat.grpc.pb.cc
    ${_NEUROMEKA_CPP_DIR}/proto/eyetask.grpc.pb.cc
    ${_NEUROMEKA_CPP_DIR}/proto/indyeye.grpc.pb.cc
    ${_NEUROMEKA_CPP_DIR}/proto/moby.grpc.pb.cc
    ${_NEUROMEKA_CPP_DIR}/proto/rtde.grpc.pb.cc
    ${_NEUROMEKA_CPP_DIR}/proto/cri.grpc.pb.cc
    ${_NEUROMEKA_CPP_DIR}/proto/hri.grpc.pb.cc
)

add_executable(hello_world examples/hello_world.cpp ${INDY_SOURCES})

add_executable(joint_control examples/joint_control.cpp ${INDY_SOURCES})

# Define common libraries
set(COMMON_LIBS
    ${_PROTOBUF_LIBPROTOBUF}
    ${_GRPC_GRPCPP}
    ${_REFLECTION}
)

target_link_libraries(hello_world PRIVATE ${COMMON_LIBS})
target_link_libraries(joint_control PRIVATE ${COMMON_LIBS})