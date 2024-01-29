#include <iostream>
#include "components/component.hpp"
#include "components/bodyTube/bodyTube.hpp"

int main(int argc, char **argv){

    //auto comp = Rocket::Component<Rocket::AerodynamicCalculator, Rocket::KinematicCalculator>();

    auto comp = Rocket::create<Rocket::DefaultComponent>();
    std::cout << "Created " << comp.name << " with id " << comp.id() << std::endl;
    //auto comp = Rocket::DefaultComponent::create();
    auto cyl = std::make_unique<Rocket::HollowCylinder>(1,1,1);
    auto toob = Rocket::create<Rocket::BodyTube>(std::move(cyl));
    std::cout << "Created " << toob.name << " with id " << toob.id() << std::endl;

    std::cout << comp.name << " " << comp.CnAlpha(0,0) << std::endl;
    std::cout << toob.name << " " << toob.CnAlpha(0,0) << std::endl;
    std::cout << comp.name << " " << comp.mass(0) << std::endl;
    std::cout << toob.name << " " << toob.mass(0) << std::endl;
    return 0;
}