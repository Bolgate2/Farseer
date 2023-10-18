#include <iostream>
#include <cmath>
#include <fmt/core.h>
#include <uuid_v4.h>
#include <Eigen/Dense>
#include "components/component.hpp"
#include "components/rocket.hpp"
#include "simulation.hpp"
#include "misc/finish.hpp"
#define SMALL 1e-10


int main(int argc, char** argv){
    UUIDv4::UUIDGenerator<std::mt19937_64> idgen;
    auto id = idgen.getUUID();
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
    return 0;
}
