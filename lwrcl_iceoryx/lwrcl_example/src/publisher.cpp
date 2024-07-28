// publisher.cpp
#include "MyData.hpp"
#include "iceoryx_posh/popo/publisher.hpp"
#include "iceoryx_posh/runtime/posh_runtime.hpp"
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    constexpr char APP_NAME[] = "iox-publisher";
    iox::runtime::PoshRuntime::initRuntime(APP_NAME);

    iox::popo::Publisher<folder1::folder2::MyData> publisher({"Example", "MyData", "Instance"});

    folder1::folder2::MyData data;
    data.id = 42;
    data.message = "Hello, Iceoryx!";

    while (true) {
        publisher.loan()
            .and_then([&](auto& sample) {
                *sample = data;
                sample.publish();
            })
            .or_else([&](auto& error) {
                std::cerr << "Failed to loan sample, error: " << static_cast<int>(error) << std::endl;
            });

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
