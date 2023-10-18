#ifndef POINT_H_
#define POINT_H_

#include <cstdlib>
#include <iostream>

namespace Point{
    // these points are really vectors mathematically
    class Point{
        private:
            long double x, y, z;
        public:
            // contructors
            Point();
            Point(long double x, long double y, long double z);
            Point(const Point& point);
            // operators
            // assignment
            Point& operator=(const Point& point);
            // equality
            bool operator==(const Point& point) const;
            bool operator!=(const Point& point) const;
            // vector addition and subtraction
            Point& operator+=(const Point& point);
            friend Point operator+(const Point& left, const Point& right);
            Point& operator-=(const Point& point);
            friend Point operator-(const Point& left, const Point& right);
            // negative
            friend Point operator-(const Point& point);
            // scalar multiplication
            Point& operator*=(long double factor);
            friend Point operator*(const Point& point, long double factor);
            friend Point operator*(long double factor, const Point& point);
            // scalar division
            Point& operator/=(long double factor);
            friend Point operator/(const Point& point, long double factor);
            // vector multiplication
            Point cross(const Point& point);
            long double dot(const Point& point);
            // magnitude
            long double mag();
            // normalized vector
            Point normalized();
            // making array accessible like a list
            long double  operator[](size_t index) const;
            long double& operator[](size_t index);
            // string conversion and printing
            operator std::string() const;
            friend std::ostream& operator<<( std::ostream& os, const Point& point );
            //getters and setters
            long double X() const {return x;}
            long double Y() const {return y;}
            long double Z() const {return z;}
            long double& X() {return x;}
            long double& Y() {return y;}
            long double& Z() {return z;}
    };
}

#endif