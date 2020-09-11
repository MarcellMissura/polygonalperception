#ifndef VEC2_H_
#define VEC2_H_

#include "VecN.h"
#include <QPointF>
#include "globals.h"

class Vec2 : public VecN<2>
{

public:

    Vec2() : VecN<2>() {}
    Vec2(const VecN<2> &o) : VecN<2>(o) {}
    Vec2(const VecN<1> &o, double xo) : VecN<2>(o, xo) {}
    Vec2(double xo, const VecN<1> &o) : VecN<2>(xo, o) {}
    Vec2(const VecN<3> &o) : VecN<2>(o) {}
    Vec2(const double* o) : VecN<2>(o) {}
    Vec2(double xo, double yo, double zo=0, double wo=0, double ao=0, double bo=0, double co=0) : VecN<2>(xo, yo, zo, wo, ao, bo, co) {}
    Vec2(const QPointF &p) : VecN<2>(p.x(), p.y()) {}

    operator QPointF() const {return QPointF(x, y);}
    bool operator <(const Vec2& v) const {return ((x-v.x) < -EPSILON || fabs(x-v.x) < EPSILON && (y-v.y) < -EPSILON);}
    bool operator <=(const Vec2& v) const {return ( (x-v.x) < -EPSILON || fabs(x-v.x) < EPSILON && (y-v.y) < EPSILON);}
    bool operator >(const Vec2& v) const {return !(*this<=v);}
    bool operator >=(const Vec2& v) const {return !(*this<v);}

public:

    void scale(double s)
    {
        VecN<2>::scale(s);
    }

    void scale(double fx, double fy)
    {
        x *= fx;
        y *= fy;
    }

    void translate(double dx, double dy)
    {
        x += dx;
        y += dy;
    }

    void rotate(double angle)
    {
        if (fabs(angle) < EPSILON)
            return;
        double c = cos(angle);
        double s = sin(angle);
        double x_ = x;
        double y_ = y;
        x = x_*c + y_*-s;
        y = x_*s + y_*c;
    }

    // Fast rotate for cases where the sin and cos of the angle are known.
    void rotate(double s, double c)
    {
        if (fabs(s) < EPSILON)
            return;
        double x_ = x;
        double y_ = y;
        x = x_*c + y_*-s;
        y = x_*s + y_*c;
    }

    // Rotates (flips) the vector by PI/2 in clockwise direction. This operation is extra fast.
    // Essentially, this function computes an unnormalized positive normal.
    void flip()
    {
        double x_ = x;
        x = y;
        y = -x_;
    }

    // Returns the angle from this to the given vector in the range -PI to PI.
    double angleTo(const Vec2 &o) const
    {
        double dot = x*o.x + y*o.y; // dot product
        double det = x*o.y - y*o.x; // determinant
        return atan2(det, dot);
    }

    // Returns the angle of this vector with respect to the x axis.
    double angle() const
    {
        return atan2(y, x); // The faster but less precise version of atan2 is used.
    }

    // Returns true if this vector is left of the other vector.
    // In the special case when the vectors are colinear it will return false.
    bool isLeftOf(const Vec2 &o) const
    {
        return x*o.y-y*o.x < 0;
    }

    // Returns true if this vector is right of the other vector.
    // In the special case when the vectors are colinear it will return false.
    bool isRightOf(const Vec2 &o) const
    {
        return x*o.y-y*o.x > 0;
    }

    // Decides if this vector is colinear with the other.
    bool isColinearWith(const Vec2& o) const
    {
        return (fabs(x*o.y-y*o.x) < EPSILON);
    }
};

#endif
