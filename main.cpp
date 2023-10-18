#include <iostream>
#include "component.hpp"
#include "rocket.hpp"
#include "simulation.hpp"
#include "finish.hpp"
#include "point.hpp"
#include "fmt/core.h"
#include "uuid_v4.h"

int test_point(){
    auto p1 = Point::Point(1,0,0);
    auto p2 = Point::Point(0,1,0);
    std::cout << p1 << "\n";
    std::cout << p1 + p2 << "\n";
    std::cout << p1 - (p1*2) << "\n";
    std::cout << p2 << "\n";
    std::cout << p1.cross(p2) << "\n";
    std::cout << p1[0] << "\n";
    std::cout << Point::Point(1,1,0).mag() << "\n";
    std::cout << (p1 + p2).normalized() << "\n";
    p1 *= 2;
    std::cout << p1 << "\n";
    p1 /= 2;
    std::cout << p1 << "\n";
    auto p3 = p2;
    std::cout << p3 << "\n";
    p3 *=3;
    std::cout << p1 << p2 << p3 << "\n";
    std::cout << -p1 + p2 << "\n";
    std::cout << p3[2] << "\n";
    fmt::print("p1: {}, p2: {}\n", (std::string) p1, (std::string) p2);
    // setter usage
    auto& x = p1.X();
    auto x2 = p1.X();
    x = 2;
    std::cout << x2 << "\n";
    std::cout << p1 << "\n";
    return 0;
}

int main(int argc, char** argv){
    UUIDv4::UUIDGenerator<std::mt19937_64> idgen;
    auto id = idgen.getUUID();
    std::cout << "Hello, from Farseer!\n";
    Component::print_name();
    Simulation::print_name();
    auto regular_paint = Finish::regular_paint;
    std::cout << regular_paint.name << "\n";
    return 0;
}
