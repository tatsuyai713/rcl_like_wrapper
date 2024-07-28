// MyData.hpp
#ifndef MYDATA_HPP
#define MYDATA_HPP

#include <string>
#include <vector>
#include <cstdint>

namespace folder1 {
namespace folder2 {

struct MyData {
    int32_t id;
    std::string message;

    // シリアライズ関数
    std::vector<uint8_t> serialize() const;

    // デシリアライズ関数
    static MyData deserialize(const std::vector<uint8_t>& data);
};

} // namespace folder2
} // namespace folder1

#endif // MYDATA_HPP
