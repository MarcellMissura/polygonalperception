#ifndef TRANSFORM3D_H
#define TRANSFORM3D_H
#include "globals.h"
#include "util/Vec3.h"

struct TransformParams
{
    double x = 0.0, y = 0.0, z = 0.0;
    double roll = 0.0, pitch = 0.0, yaw = 0.0;

    bool operator!=(const TransformParams &o) const
    {
        return (fabs(x-o.x) > EPSILON
                || fabs(y-o.y) > EPSILON
                || fabs(z-o.z) > EPSILON
                || fabs(roll-o.roll) > EPSILON
                || fabs(pitch-o.pitch) > EPSILON
                || fabs(yaw-o.yaw) > EPSILON);
    }
};

class Transform3D
{
    alignas(32) double M[16] = {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}; // column-major for opengl

public:

    Transform3D();
    ~Transform3D(){}

    Transform3D(const TransformParams& t);
    void operator=(const TransformParams& t);
    void operator=(const double* v);

    double &at(uint row, uint col);
    double at(uint row, uint col) const;
    double& operator()(uint row, uint col);
    double operator()(uint row, uint col) const;

    TransformParams getParams() const;
    void setFromParams(double x, double y, double z, double roll, double pitch, double yaw);
    void setFromParams(const TransformParams &t);
    void setFromGroundPlane(const Vec3& n, const Vec3& p);

    Transform3D operator*(const Transform3D& o) const;
    Vec3 operator*(const Vec3& v) const;

    // OpenGL support.
    operator const double*() const {return M;}
    const double* data() const {return M;}
};

QDebug operator<<(QDebug dbg, const TransformParams &o);


#endif
