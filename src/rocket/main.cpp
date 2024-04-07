#include <iostream>
#include <nlohmann/json.hpp>
#include "components/bodyTube.hpp"
using json = nlohmann::json;

int main(int argc, char **argv){
    
    auto toob = Rocket::BodyTube(10, 20, 2, true, "jeff", Eigen::Vector3d{1,2,3});
    //auto toob3 = Rocket::BodyTube::create(json {});
    auto toobJson = toob.toJson();
    auto toob2 = Rocket::BodyTube::create(toobJson);

    std::cout << toob.toJson().dump(2) << std::endl;
    std::cout << "---------------------------------" << std::endl;
    std::cout << toob2->toJson().dump(2) << std::endl;

    return 0;
}