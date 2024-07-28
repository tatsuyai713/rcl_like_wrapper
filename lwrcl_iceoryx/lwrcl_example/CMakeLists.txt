cmake_minimum_required(VERSION 3.5)

project(IceoryxPubSubExample)
set(CMAKE_PREFIX_PATH "/opt/iceoryx")
find_package(iceoryx_posh REQUIRED)

add_executable(publisher
    src/publisher.cpp
    src/MyData.cpp
)

target_include_directories(publisher PRIVATE
    include
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(publisher
    iceoryx_posh::iceoryx_posh
)

add_executable(subscriber
src/subscriber.cpp
src/MyData.cpp
)

target_include_directories(subscriber PRIVATE
    include
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(subscriber
    iceoryx_posh::iceoryx_posh
)
