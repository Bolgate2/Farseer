#include <iostream>
#include <nlohmann/json.hpp>
#include "components/bodyTube.hpp"
using json = nlohmann::json;

int main(int argc, char **argv){
    
    auto toob = std::make_shared<Rocket::BodyTube>(10, 20, 2, true, "jeff", Eigen::Vector3d{1,2,3});
    auto toobJson = toob->toJson();
    auto toob2 = Rocket::componentFromJson(toobJson);

    toob->addComponent(toob2.get());

    std::cout << toob->toJson().dump(2) << std::endl;
    std::cout << "---------------------------------" << std::endl;
    std::cout << toob2->toJson().dump(2) << std::endl;
    std::cout << "---------------------------------" << std::endl;

    auto toob3 = Rocket::componentFromJson(toob->toJson());
    std::cout << toob3->toJson().dump(2) << std::endl;
    std::cout << "---------------------------------" << std::endl;
    toob->removeComponent(toob2.get());
    std::cout << toob2->parent() << std::endl;
    std::cout << toob->toJson().dump(2) << std::endl;

    return 0;
}