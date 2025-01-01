cmake_minimum_required(VERSION 3.10)

project(grpc_example)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--no-as-needed")
set(CMAKE_PREFIX_PATH "/opt/grpc" ${CMAKE_PREFIX_PATH})
include(../cmake/common.cmake)

# C++17を指定
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_BUILD_TYPE Debug)

# gRPCのパスを設定
set(GRPC_LIBRARIES /opt/grpc/lib)
set(GRPC_INCLUDE_DIRS /opt/grpc/include /opt/grpc-libs/include)

# Protobufライブラリを探す
find_package(Protobuf REQUIRED)
find_package(gRPC REQUIRED)

# ヘッダーとソースのパスを指定
include_directories(${GRPC_INCLUDE_DIRS} 
    /opt/grpc-libs/include/std_msgs/msg
    /opt/grpc-libs/include/builtin_interfaces/msg
    ${Protobuf_INCLUDE_DIRS}
    include)

link_directories(${GRPC_LIBRARIES} /opt/grpc-libs/lib)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 実行ファイルを設定
add_executable(client src/client.cpp)
add_executable(server src/server.cpp)

# gRPCライブラリとビルド済みライブラリをリンク
target_link_libraries(client 
    gRPC::grpc++
    ${Protobuf_LIBRARIES}
    /opt/grpc-libs/lib/libstd_msgs.so
    /opt/grpc-libs/lib/libbuiltin_interfaces.so
    absl::check
    absl::flags
    absl::flags_parse
    absl::log)

target_link_libraries(server 
    gRPC::grpc++
    ${Protobuf_LIBRARIES}
    /opt/grpc-libs/lib/libstd_msgs.so
    /opt/grpc-libs/lib/libbuiltin_interfaces.so
    absl::check
    absl::flags
    absl::flags_parse
    absl::log)
