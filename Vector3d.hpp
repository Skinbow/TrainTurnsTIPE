#pragma once

#include <iostream>

struct Vector3d {
    double x, y, z;
    Vector3d();
    Vector3d(double x, double y, double z);

    static Vector3d zero();

    double dot(const Vector3d &a);
    Vector3d cross(const Vector3d &a);

    Vector3d operator+(const Vector3d &a);
    Vector3d operator-(const Vector3d &a);
    Vector3d operator*(const double a);
    Vector3d operator/(const double a);
    bool operator==(const Vector3d& a);
    double norm(); 
};

std::ostream& operator<<(std::ostream &s, const Vector3d &vec);