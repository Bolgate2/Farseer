#include <iostream>
#include <nlohmann/json.hpp>
#include <Eigen/Dense>
#include <uuid_v4/uuid_v4.h>
#include "components/temp.hpp"

using json = nlohmann::json;

int main(int argc, char **argv){
    std::cout << "Hello World!" << std::endl;
    // testing imports
    // Eigen
    std::cout << "Eigen test\n" << (Eigen::Vector3<int> {1,2,3}).transpose() << std::endl;
    // uuid
    std::cout << "uuid_v4 test\n";
    UUIDv4::UUIDGenerator<std::mt19937_64> uuidGenerator;
    UUIDv4::UUID uuid = uuidGenerator.getUUID();
    std::cout << uuid << std::endl;
    // json
    json jason = {
        {"name", "jeff"},
        {"jump street", 22}
    };
    std::cout << "json test\n" << jason.dump(2) << std::endl;

    return 0;
}