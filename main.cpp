#include <iostream>
#include <string>
#include <cmath>
#include <fmt/core.h>
#include <uuid_v4.h>
#include <Eigen/Dense>
#include <memory>


#include "components/internalComponent.hpp"
#include "shapes/primitives/nosecone/noseconeShapeTypes.hpp"
#include "shapes/components/nosecone/nosecone.hpp"
#include "shapes/primitives/trapezoidalPrism.hpp"
#include "components/bodyTube/bodyTube.hpp"

#include "components/nosecone/nosecone.hpp"
#include "misc/finish.hpp"
#include "misc/material.hpp"

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

int main(int argc, char** argv){
    UUIDv4::UUIDGenerator<std::mt19937_64> idgen;
    auto id = idgen.getUUID();
    auto mat1 = std::make_unique<Rocket::Material>("Cardboard", 680);
    auto fin1 = std::make_unique<Rocket::Finish>("Regular Paint", 60/(std::pow(10,6)));
    auto radius = 0.0632/2;
    auto toob = Rocket::BodyTube::create(radius, 0.66, 0.0016, std::move(mat1), std::move(fin1), nullptr, "toob", Eigen::Vector3d{0.13,0,0});

    fmt::print("{0:<20} {1:<20}\n", "Toob Name ",toob->name);
    fmt::print("Toob mass {0}\n", toob->mass(0));
    fmt::print("Toob inertia\n{0}\n", toString(toob->inertia(0)));
    fmt::print("Toob cm\n{0}\n", toString(toob->cm(0)));
    auto alpha = deg2rad(5);
    fmt::print("toob cp {}\n", toString(toob->cp(0.3,alpha))); // correct
    fmt::print("toob cna {}\n", toob->c_n_a(0.3,alpha)); // correct
    toob->printComponentTree();

    auto int1 = Rocket::InternalComponent::create(toob.get(), "Steve");
    auto int_1_1 = Rocket::InternalComponent::create(int1.get(), "Bob");
    auto int2 = Rocket::InternalComponent::create(toob.get(), "Jim");

    toob->printComponentTree();
    
    int1->setMass(0.002);
    toob->printComponentTree();
    fmt::print("Toob tree height {}\n", toob->height());

    auto trap = Shapes::TrapezoidalPrism(0.1, 0.03, 0.06, 0.06, 0.003);
    fmt::print("{:<30} {}\n", "Trapz volume", trap.volume());
    fmt::print("{:<30} {}\n", "Trapz cm", toString(trap.cm().transpose()));
    fmt::print("{:<30}\n{}\n", "Trapz inertia", toString(trap.inertia()));

    auto mat2 = std::make_unique<Rocket::Material>("Cardboard", 680);
    auto fin2 = std::make_unique<Rocket::Finish>("Regular Paint", 60/(std::pow(10,6)));

    auto nose = Rocket::Nosecone::create(Shapes::NoseconeShapeTypes::HAACK, 0.0632/2, 0.13, 0.003, 0.0, std::move(mat2), std::move(fin2));

    fmt::print("{0:<20} {1:<20}\n", "Toob Name ",nose->name);
    fmt::print("Nose mass {0}\n", nose->mass(0));
    fmt::print("Nose inertia\n{0}\n", toString(nose->inertia(0)));
    fmt::print("Nose planform area {0}\n", nose->planformArea());
    fmt::print("Nose cm\n{0}\n", toString(nose->cm(0)));
    fmt::print("Nose cp {}\n", toString(nose->cp(0.3,alpha))); // correct
    fmt::print("Nose cna {}\n", nose->c_n_a(0.3,alpha)); // correct

    return 0;
}
