#include <iostream>
#include <string>
#include <memory>
#include <fmt/core.h>
#include <Eigen/Dense>

#include "components/stage.hpp"

#include "components/internalComponent.hpp"
#include "shapes/primitives/nosecone/noseconeShapeTypes.hpp"
#include "shapes/components/nosecone/nosecone.hpp"
#include "components/bodyTube/bodyTube.hpp"

#include "components/nosecone/nosecone.hpp"
#include "shapes/components/fins/trapezoidalFin.hpp"
#include "components/fins/finSet.hpp"
#include "components/fins/fin.hpp"
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

    auto toob = Rocket::BodyTube::create(
        rocketRadius, toobLength, toobThick, std::move(toobMat), std::move(toobFinish), nullptr, "toob", Eigen::Vector3d{noseConeLength,0,0}
        );
    
    auto nose = Rocket::Nosecone::create(
        noseConeShape, rocketRadius, noseConeLength, noseConeThickness, noseConeShapeParam, std::move(noseConeMat), std::move(noseConeFinish)
        );

    toob->printComponentTree();
    nose->printComponentTree();

    auto alpha = deg2rad(5);
    auto mach = 0.3;

    fmt::print("{0:<20} {1:<20}\n", "Nose Name ",nose->name);
    fmt::print("Nose mass {0}\n", nose->mass(0));
    fmt::print("Nose inertia\n{0}\n", toString(nose->inertia(0)));
    fmt::print("Nose planform area {0}\n", nose->planformArea());
    fmt::print("Nose cm\n{0}\n", toString(nose->cm(0)));
    fmt::print("Nose cp {}\n", toString(nose->cp(mach,alpha))); // correct
    fmt::print("Nose cna {}\n", nose->c_n_a(mach,alpha)); // correct
    fmt::print("\n");

    fmt::print("{0:<20} {1:<20}\n", "Toob name ",   toob->name);
    fmt::print("{0:<20} {1:<20}\n", "Toob mass",    toob->mass(0));
    fmt::print("{0:<20} {1:<20}\n", "Toob ref area",    toob->referenceArea());
    fmt::print("{0:<20}\n{1}\n",    "Toob inertia", toString(toob->inertia(0)));
    fmt::print("{0:<20} {1}\n",     "Toob cm",      toString(toob->cm(0).transpose()));
    fmt::print("{0:<20} {1}\n",     "Toob cp",      toString(toob->cp(mach,alpha).transpose())); // correct
    fmt::print("{0:<20} {1:<20}\n", "Toob cna",     toob->c_n_a(mach,alpha)); // correct
    fmt::print("\n");

    
    auto finShape = std::make_unique<Shapes::TrapezoidalFinShape>(finRootChord, finTipChord, finHeight, finSweepLength, finThickness);
    auto fin = std::make_unique<Rocket::Fin>(std::move(finShape), std::move(finMat), std::move(finFinish), "Trapezoidal fin");
    auto finSet = Rocket::FinSet::create(std::move(fin), numFins, toob.get(), "Fin Set", Eigen::Vector3d{ noseConeLength+toobLength-finRootChord, 0, 0 });

    toob->printComponentTree();
    fmt::print("\n");

    fmt::print("aero components size {}\n", toob->aeroComponents().size() );
    fmt::print("ext components size {}\n", toob->components().size() );
    fmt::print("aero comp name {}\n", toob->aeroComponents()[0]->name );

    fmt::print("{0:<20} {1:<20}\n", "Fin name ",finSet->fin()->name);
    fmt::print("{0:<20} {1:<20}\n", "Fin mass", finSet->fin()->mass(0));
    fmt::print("{0:<20}\n{1}\n",    "Fin inertia", toString(finSet->fin()->inertia(0)));
    fmt::print("{0:<20} {1}\n",     "Fin cm", toString(finSet->fin()->cm(0).transpose()));
    fmt::print("{0:<20} {1}\n",     "Fin cp", toString(finSet->fin()->cp(mach,alpha).transpose()));
    fmt::print("{0:<20} {1:<20}\n", "Fin cna", finSet->fin()->c_n_a(mach,alpha));
    fmt::print("\n");

    fmt::print("{0:<20} {1:<20}\n", "Fin set name ",finSet->name);
    fmt::print("{0:<20} {1:<20}\n", "Fin set mass", finSet->mass(0));
    fmt::print("{0:<20}\n{1}\n",    "Fin set inertia", toString(finSet->inertia(0)));
    fmt::print("{0:<20} {1}\n",     "Fin set cm", toString(finSet->cm(0).transpose()));
    fmt::print("{0:<20} {1}\n",     "Fin set cp", toString(finSet->cp(mach,alpha).transpose()));
    fmt::print("{0:<20} {1:<20}\n", "Fin set cna", finSet->c_n_a(mach,alpha));
    fmt::print("\n");

    fmt::print("{0:<20} {1:<20}\n", "Toob name ",   toob->name);
    fmt::print("{0:<20} {1:<20}\n", "Toob mass",    toob->mass(0));
    fmt::print("{0:<20}\n{1}\n",    "Toob inertia", toString(toob->inertia(0)));
    fmt::print("{0:<20} {1}\n",     "Toob cm",      toString(toob->cm(0).transpose()));
    fmt::print("{0:<20} {1}\n",     "Toob cp",      toString(toob->cp(mach,alpha).transpose())); // correct
    fmt::print("{0:<20} {1:<20}\n", "Toob cna",     toob->c_n_a(mach,alpha)); // correct
    fmt::print("\n");

    return toob;
}

int main(int argc, char** argv){
    createTestRocket();

    return 0;
}
