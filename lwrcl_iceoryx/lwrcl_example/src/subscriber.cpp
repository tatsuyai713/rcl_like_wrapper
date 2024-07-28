// subscriber.cpp
#include "MyData.hpp"
#include "iceoryx_posh/popo/subscriber.hpp"
#include "iceoryx_posh/runtime/posh_runtime.hpp"
#include <iostream>

int main() {
    constexpr char APP_NAME[] = "iox-subscriber";
    iox::runtime::PoshRuntime::initRuntime(APP_NAME);

    iox::popo::Subscriber<folder1::folder2::MyData::MyData> subscriber({"Example", "MyData", "Instance"});

    while (true) {
        subscriber.take()
            .and_then([&](const auto& sample) {
                auto& data = *sample;
                std::cout << "Received data: id = " << data.id << ", message = " << data.message << std::endl;
            })
            .or_else([&](auto& error) {
                if (error != iox::popo::ChunkReceiveResult::NO_CHUNK_AVAILABLE) {
                    std::cerr << "Failed to take sample, error: " << static_cast<int>(error) << std::endl;
                }
            });

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
