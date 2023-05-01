#include "Vector3d.hpp"

#include <cmath>

Vector3d::Vector3d() : x(0), y(0), z(0) {}
Vector3d::Vector3d(double x, double y, double z) : x(x), y(y), z(z) { }

Vector3d Vector3d::zero() {
    return Vector3d();
}

Vector3d Vector3d::operator+(const Vector3d &a) {
    return Vector3d(x + a.x, y + a.y, z + a.z);
}

Vector3d Vector3d::operator-(const Vector3d &a) {
    return Vector3d(x - a.x, y - a.y, z - a.z);
}

Vector3d Vector3d::operator*(const double a) {
    return Vector3d(a * x, a * y, a * z);
}

Vector3d Vector3d::operator/(const double a) {
    return (*this) * (1 / a);
}

bool Vector3d::operator==(const Vector3d& a) {
    return (x == a.x) && (y == a.y) && (z == a.z);
}

double Vector3d::norm() {
    return sqrt(x * x + y * y + z * z);
} 

std::ostream& operator<<(std::ostream &s, const Vector3d &vec) {
    return s << "(" << vec.x << ", " << vec.y << ", " << vec.z << ")";
}