// MyData.cpp
#include "MyData.hpp"

namespace folder1 {
namespace folder2 {
namespace MyData {

std::vector<uint8_t> MyData::serialize() const {
    std::vector<uint8_t> buffer;

    // シリアライズ
    buffer.push_back(static_cast<uint8_t>((id >> 24) & 0xFF));
    buffer.push_back(static_cast<uint8_t>((id >> 16) & 0xFF));
    buffer.push_back(static_cast<uint8_t>((id >> 8) & 0xFF));
    buffer.push_back(static_cast<uint8_t>(id & 0xFF));

    size_t message_size = message.size();
    buffer.push_back(static_cast<uint8_t>((message_size >> 24) & 0xFF));
    buffer.push_back(static_cast<uint8_t>((message_size >> 16) & 0xFF));
    buffer.push_back(static_cast<uint8_t>((message_size >> 8) & 0xFF));
    buffer.push_back(static_cast<uint8_t>(message_size & 0xFF));
    buffer.insert(buffer.end(), message.begin(), message.end());

    return buffer;
}

MyData MyData::deserialize(const std::vector<uint8_t>& data) {
    MyData myData;
    size_t offset = 0;

    // デシリアライズ
    myData.id = (static_cast<int32_t>(data[offset]) << 24) |
                (static_cast<int32_t>(data[offset + 1]) << 16) |
                (static_cast<int32_t>(data[offset + 2]) << 8) |
                static_cast<int32_t>(data[offset + 3]);
    offset += 4;

    size_t message_size = (static_cast<size_t>(data[offset]) << 24) |
                          (static_cast<size_t>(data[offset + 1]) << 16) |
                          (static_cast<size_t>(data[offset + 2]) << 8) |
                          static_cast<size_t>(data[offset + 3]);
    offset += 4;
    myData.message = std::string(data.begin() + offset, data.begin() + offset + message_size);
    offset += message_size;

    return myData;
}

} // namespace MyData
} // namespace folder2
} // namespace folder1
