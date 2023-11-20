#include <iostream>
#include <string>
#include <memory>
#include <fmt/core.h>
#include <Eigen/Dense>
#include <thread>

#include "components/rocket.hpp"
#include "components/stage.hpp"

#include "components/internalComponent.hpp"
#include "shapes/primitives/nosecone/noseconeShapeTypes.hpp"
#include "shapes/components/nosecone/nosecone.hpp"
#include "components/bodyTube/bodyTube.hpp"

#include "misc/finish.hpp"
#include "misc/material.hpp"

#include "components/nosecone/nosecone.hpp"
#include "shapes/components/fins/trapezoidalFin.hpp"
#include "components/fins/finSet.hpp"
#include "components/fins/fin.hpp"
#include "components/motor/motor.hpp"
#include "maths.hpp"

#include "RealAtmos.hpp"
#include "simulation.hpp"


#define SMALL 1e-10

void testVectorFunctions(){
    Eigen::Vector3d v1(1,0,0);
    Eigen::Vector3d v2(0,1,0);
    Eigen::Matrix3d m1 {
        {1,2,3},
        {4,5,6},
        {7,8,9}
    };
    // rotation about the z axis by 90 degrees
    double cos_90 = std::cos(M_PI*90/180);
    double sin_90 = std::sin(M_PI*90/180);
    Eigen::Matrix3d rotmat {
        {cos_90,    -sin_90,    0},
        {sin_90,    cos_90,     0},
        {0,         0,          1}
    };
    Eigen::Quaternion<double> q1;
    q1 = Eigen::AngleAxis<double>(90, Eigen::Vector3d(0,0,1));

    std::cout << v1 << "\n";
    std::cout << v1.cross(v2) << "\n";
    std::cout << v1 + v2 << "\n";
    std::cout << (v1 + v2).normalized() << "\n";
    std::cout << m1 << "\n";
    std::cout << m1.transpose() << "\n";
    std::cout << q1 << "\n";
    std::cout << q1*v1 << "\n";
    std::cout << "vector times rotmat\n" << rotmat*v1 << "\n";

    std::cout << "Hello, from Farseer!\n";
    auto regular_paint = Rocket::regular_paint;
    std::cout << regular_paint.name << "\n";
    Eigen::Matrix3d nullMat = Eigen::Matrix3d::Ones() * std::nan("0");
    std::cout << nullMat << "\n";
    std::cout << (nullMat.hasNaN() ? "true" : "false") << "\n";
}

static std::string toString(const Eigen::MatrixXd& mat){
    std::stringstream ss;
    ss << mat;
    return ss.str();
}

void testNoseconeShapes(){
    std::unique_ptr<Shapes::NoseconeShape> daNose = Shapes::NoseconeShapeFactory::create(Shapes::NoseconeShapeTypes::HAACK, 0.0632/2, 0.13, 0.003, 0);

    fmt::print("{:<30} {}\n", "Da nose volume", daNose->volume());
    fmt::print("{:<30} {}\n", "Da nose filled volume", daNose->filledVolume());
    fmt::print("{:<30} {}\n", "Da nose wet area", daNose->wettedArea());
    fmt::print("{:<30} {}\n", "Da nose plan area", daNose->planformArea());
    fmt::print("{:<30} [{}]\n", "Da nose CG", toString(daNose->cm().transpose()));
    fmt::print("{:<30}\n{}\n", "Da nose Inertia", toString(daNose->inertia()));
    fmt::print("{:<30} {}\n", "Da nose avg rad", daNose->averageRadius());
    auto noseBsRad = daNose->bisectedAverageRadius(daNose->length()/2);
    fmt::print("{:<30} {} {} {}\n", "Da nose start mid end rads", daNose->radius(0), daNose->radius(daNose->length()/2), daNose->radius(daNose->length()));
    fmt::print("{:<30} {} {}\n", "Da nose split rads", noseBsRad[0], noseBsRad[1]);

}

double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

std::shared_ptr<Rocket::AeroComponent> createTestRocket(){
    // TOOB PARAMS
    auto toobMat = std::make_unique<Rocket::Material>("Cardboard", 680);
    auto toobFinish = std::make_unique<Rocket::Finish>("Regular Paint", 60/(std::pow(10,6)));
    auto rocketRadius = 0.0632/2;
    auto toobLength = 0.66;
    auto toobThick = 0.0016;

    // NOSE PARAMS
    auto noseConeLength = 0.13;
    auto noseConeThickness = 0.003;
    auto noseConeShape = Shapes::NoseconeShapeTypes::HAACK;
    double noseConeShapeParam = 0;
    auto noseConeMat = std::make_unique<Rocket::Material>("3D PLA", 1250);
    auto noseConeFinish = std::make_unique<Rocket::Finish>("Regular Paint", 60/(std::pow(10,6)));

    // FIN PARAMS
    auto finRootChord = 0.1;
    auto finTipChord = 0.03;
    auto finSweepLength = 0.06;
    auto finHeight = 0.06;
    auto finThickness = 0.003;
    auto finMat = std::make_unique<Rocket::Material>("Plywood", 630);
    auto finFinish = std::make_unique<Rocket::Finish>("Regular Paint", 60/(std::pow(10,6)));
    auto numFins = 4;

    // MOTOR PARAMS
    std::filesystem::path motorPath = std::filesystem::current_path().append("..").append("AeroTech_F27R_L.eng");

    auto rocket = Rocket::Rocket::create("Jeff the Rocket");

    auto stage1 = Rocket::Stage::create(rocket.get());
    
    auto toob = Rocket::BodyTube::create(
        rocketRadius, toobLength, toobThick, std::move(toobMat), std::move(toobFinish), stage1.get(), "toob", Eigen::Vector3d{noseConeLength,0,0}
        );
    
    auto nose = Rocket::Nosecone::create(
        noseConeShape, rocketRadius, noseConeLength, noseConeThickness, noseConeShapeParam, std::move(noseConeMat), std::move(noseConeFinish), stage1.get()
        );

    auto motor = Rocket::Motor::fromFile(motorPath, toob.get());
    auto motorX = noseConeLength+toobLength-motor->shape()->length();
    motor->setPosition({motorX, 0, 0});
    
    auto alpha = 0;
    auto mach = 0.3;

    
    auto finShape = std::make_unique<Shapes::TrapezoidalFinShape>(finRootChord, finTipChord, finHeight, finSweepLength, finThickness);
    auto fin = std::make_unique<Rocket::Fin>(std::move(finShape), std::move(finMat), std::move(finFinish), "Trapezoidal fin");
    auto finSet = Rocket::FinSet::create(std::move(fin), numFins, toob.get(), "Fin Set", Eigen::Vector3d{ noseConeLength+toobLength-finRootChord, 0, 0 });
    
    rocket->printComponentTree();

    return rocket;
}

std::shared_ptr<Rocket::AeroComponent> createBIGTestRocket(){
    // TOOB PARAMS
    auto toobMat = std::make_unique<Rocket::Material>("Cardboard", 680);
    auto toobFinish = std::make_unique<Rocket::Finish>("Regular Paint", 60/(std::pow(10,6)));
    auto rocketRadius = 0.0632/2;
    auto toobLength = 0.35;
    auto toobThick = 0.0016;

    // NOSE PARAMS
    auto noseConeLength = 0.13;
    auto noseConeThickness = 0.003;
    auto noseConeShape = Shapes::NoseconeShapeTypes::HAACK;
    double noseConeShapeParam = 0;
    auto noseConeMat = std::make_unique<Rocket::Material>("3D PLA", 1250);
    auto noseConeFinish = std::make_unique<Rocket::Finish>("Regular Paint", 60/(std::pow(10,6)));

    // FIN PARAMS
    auto finRootChord = 0.1;
    auto finTipChord = 0.03;
    auto finSweepLength = 0.06;
    auto finHeight = 0.06;
    auto finThickness = 0.003;
    auto finMat = std::make_unique<Rocket::Material>("Plywood", 630);
    auto finFinish = std::make_unique<Rocket::Finish>("Regular Paint", 60/(std::pow(10,6)));
    auto numFins = 4;

    // MOTOR PARAMS
    std::filesystem::path motorPath = std::filesystem::current_path().append("..").append("AeroTech_I600R.eng");

    auto rocket = Rocket::Rocket::create("Jeff the Rocket");

    auto stage1 = Rocket::Stage::create(rocket.get());
    
    auto toob = Rocket::BodyTube::create(
        rocketRadius, toobLength, toobThick, std::move(toobMat), std::move(toobFinish), stage1.get(), "toob", Eigen::Vector3d{noseConeLength,0,0}
        );
    
    auto nose = Rocket::Nosecone::create(
        noseConeShape, rocketRadius, noseConeLength, noseConeThickness, noseConeShapeParam, std::move(noseConeMat), std::move(noseConeFinish), stage1.get()
        );

    auto motor = Rocket::Motor::fromFile(motorPath, toob.get());
    auto motorX = noseConeLength+toobLength-motor->shape()->length();
    motor->setPosition({motorX, 0, 0});
    
    auto alpha = 0;
    auto mach = 0.3;

    
    auto finShape = std::make_unique<Shapes::TrapezoidalFinShape>(finRootChord, finTipChord, finHeight, finSweepLength, finThickness);
    auto fin = std::make_unique<Rocket::Fin>(std::move(finShape), std::move(finMat), std::move(finFinish), "Trapezoidal fin");
    auto finSet = Rocket::FinSet::create(std::move(fin), numFins, toob.get(), "Fin Set", Eigen::Vector3d{ noseConeLength+toobLength-finRootChord, 0, 0 });
    
    rocket->printComponentTree();

    /*
    std::cout << finSet->fin()->referenceArea() << std::endl;
    for(float i = 0; i < 2; i+= 0.01){
        std::cout << finSet->fin()->c_n_a(i, 0, 1.4) << ", ";
    }
    std::cout << std::endl;
    */
    return rocket;
}

void testInertia(){
    Eigen::Matrix3d inert1 = Eigen::Matrix3d::Identity();
    fmt::print("Inert 1\n{}\n", toString(inert1));
    auto inert2 = Utils::parallel_axis_transform(inert1, Eigen::Vector3d{1,0,0}, 1);
    fmt::print("Inert 2\n{}\n", toString(inert2));
    auto inert3 = Utils::parallel_axis_transform(inert2, Eigen::Vector3d{1,0,0}, 1, true);
    fmt::print("Inert 3\n{}\n", toString(inert3));
}

void testMotor(){
    std::filesystem::path motorPath = std::filesystem::current_path().append("..").append("AeroTech_F27R_L.eng");
    auto motor = Rocket::Motor::fromFile(motorPath);
    fmt::print("motor mass 0.955 {}\nmotor mass 1     {}\nmotor mass 1.045 {}\n", motor->mass(0.955), motor->mass(1), motor->mass(1.045));
    fmt::print("motor thrust 0: [{}]\nmotor thrust 1: [{}]\n", toString(motor->thrust(0).transpose()), toString(motor->thrust(1).transpose()));
}

void simRocket(Rocket::AeroComponent* rocket, std::filesystem::path destination){
    rocket->setAllCaching(true);

    auto btime = rocket->calculateBurnoutTime();

    auto sim = Sim::Sim::create(rocket, 0.01, destination);
    auto initialConditions = Sim::defaultStateVector();
    // setting initial conds
    auto initialAzimuth = M_PI/6;
    auto initialLaunchAng = M_PI/6;

    initialConditions[Sim::StateMappings::Psi] = initialAzimuth;
    initialConditions[Sim::StateMappings::Theta] = initialLaunchAng;

    auto lastStep = sim->solve(initialConditions);
    fmt::print("last step [{}]\n", toString(lastStep.transpose()));
}

void testFin(){
    // FIN PARAMS
    auto finRootChord = 0.1;
    auto finTipChord = 0.03;
    auto finSweepLength = 0.06;
    auto finHeight = 0.06;
    auto finThickness = 0.003;
    auto finMat = std::make_unique<Rocket::Material>("Plywood", 630);
    auto finFinish = std::make_unique<Rocket::Finish>("Regular Paint", 60/(std::pow(10,6)));
    auto finShape = std::make_unique<Shapes::TrapezoidalFinShape>(finRootChord, finTipChord, finHeight, finSweepLength, finThickness);
    auto fin = std::make_unique<Rocket::Fin>(std::move(finShape), std::move(finMat), std::move(finFinish), "Trapezoidal fin");

    for(double i = 0; i < 3; i+=0.05){
        std::cout << "m " << i << " cpx " << fin->calculateCp(i,0).x() << std::endl;
    }
}

int main(int argc, char** argv){

    auto subsonicRocket = createTestRocket();
    auto supersonicRocket = createBIGTestRocket();

    auto subsonicPath = std::filesystem::current_path().append("..").append("results").append("subsonic");
    auto supersonicPath = std::filesystem::current_path().append("..").append("results").append("supersonic");

    std::thread supersonicThread (simRocket, supersonicRocket.get(), supersonicPath);
    std::thread subThread (simRocket, subsonicRocket.get(), subsonicPath);
    supersonicThread.join();
    subThread.join();

    return 0;
}
