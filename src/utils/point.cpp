#include <cmath>
#include <stdexcept>
#include <iostream>
#include <string>
#include "point.hpp"
#include "fmt/core.h"

namespace Point{
    // constructors
    Point::Point(long double x, long double y, long double z): x(x), y(y), z(z){}
    Point::Point(): x(0), y(0), z(0){}
    Point::Point(const Point& point): x(point.x), y(point.y), z(point.z){}

    // operators
    // assignment
    Point& Point::operator=(const Point& point){
        if (this != &point){
            x = point.x;
            y = point.y;
            z = point.z;
        }
        return *this;
    }

    // equality
    bool Point::operator==(const Point& point) const {
        return ( (x==point.x) && (y==point.y) && (z==point.z) );
    }

    bool Point::operator!=(const Point& point) const {
        return !(*this == point);
    }

    // vector addition and subtraction
    Point& Point::operator+=(const Point& point){
        x += point.x;
        y += point.y;
        z += point.z;
        return *this;
    }

    Point operator+(const Point& left, const Point& right){
        return Point(left.x+right.x, left.y+right.y, left.z+right.z);
    }

    Point& Point::operator-=(const Point& point){
        x -= point.x;
        y -= point.y;
        z -= point.z;
        return *this;
    }

    Point operator-(const Point& left, const Point& right){
        return Point(left.x-right.x, left.y-right.y, left.z-right.z);
    }

    // negative
    Point operator-(const Point& point){
        return Point(-point.x, -point.y, -point.z);
    }

    // scalar multiplication
    Point& Point::operator*=(long double factor){
        x *= factor;
        y *= factor;
        z *= factor;
        return *this;
    }

    Point operator*(const Point& point, long double factor){
        return Point(factor*point.x, factor*point.y, factor*point.z);
    }

    Point operator*(long double factor, const Point& point){
        return Point(factor*point.x, factor*point.y, factor*point.z);
    }

    // scalar division
    Point& Point::operator/=(long double factor){
        x /= factor;
        y /= factor;
        z /= factor;
        return *this;
    }

    Point operator/(const Point& point, long double factor){
        return Point(factor/point.x, factor/point.y, factor/point.z);
    }

    // vector multiplication
    Point Point::cross(const Point& point){
        double cross_x = y*point.z - point.y*z;
        double cross_y = z*point.x - point.z*x;
        double cross_z = x*point.y - point.x*y;
        return Point(cross_x, cross_y, cross_z);
    }

    long double Point::dot(const Point& point){
        return x*point.x + y*point.y + z*point.z;
    }

    // magnitude
    long double Point::mag(){
        return std::sqrt( x*x + y*y + z*z );
    }

    // normalized vector
    Point Point::normalized(){
        long double norm = this->mag();
        return Point(x/norm, y/norm, z/norm);
    }

    // making array accessible like a list
    long double Point::operator[](size_t index) const{
        switch(index){
            case 0:
                return x;
            case 1:
                return y;
            case 2:
                return z;
            default:
                throw std::out_of_range("Point index must be between 0 and 2");
        }
    }

    long double& Point::operator[](size_t index){
        switch(index){
            case 0:
                return x;
            case 1:
                return y;
            case 2:
                return z;
            default:
                throw std::out_of_range( fmt::format("Point index must be between 0 and 2, Index entered: {}", index) );
        }
    }

    Point::operator std::string() const
    {
        return fmt::format("({}, {}, {})", x, y, z);
    }

    std::ostream& operator<<( std::ostream& os, const Point& point ){
        os << (std::string) point;
        return os;
    }
}