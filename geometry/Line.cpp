#include "Line.h"
#include "util/ColorUtil.h"

// The Line object is a line segment defined by two points (x1,y1) and (x2,y2).
// Public read/write access to these coordinates is granted for simple use.
// The functions of the Line object maintain the invariant that x1 <= x2.
// The Line object implements a fast line intersection method.

Line::Line()
{
    x1 = 0;
    y1 = 0;
    x2 = 0;
    y2 = 0;

    a = 0;
    b = 0;
}

// Creates a Line with the points p1 and p2.
// The points are switched if needed to make sure that p1.x < p2.x.
Line::Line(const Vec2 &p1, const Vec2 &p2)
{
    if (p1.x < p2.x)
    {
        this->x1 = p1.x;
        this->x2 = p2.x;
        this->y1 = p1.y;
        this->y2 = p2.y;
    }
    else
    {
        this->x1 = p2.x;
        this->x2 = p1.x;
        this->y1 = p2.y;
        this->y2 = p1.y;
    }

    if (isVertical())
    {
        if (y1 > y2)
        {
            double tmp = x1;
            this->x1 = x2;
            this->x2 = tmp;
            tmp = y1;
            this->y1 = y2;
            this->y2 = tmp;
        }
        a = 0;
    }
    else
    {
        a = (y2-y1)/(x2-x1);
    }

    b = y1-x1*a;
}

// Creates a Line.
// The points are switched if needed to make sure that p1.x < p2.x.
Line::Line(double x1_, double y1_, double x2_, double y2_)
{
    if (x1_ < x2_)
    {
        this->x1 = x1_;
        this->x2 = x2_;
        this->y1 = y1_;
        this->y2 = y2_;
    }
    else
    {
        this->x1 = x2_;
        this->x2 = x1_;
        this->y1 = y2_;
        this->y2 = y1_;
    }

    if (isVertical())
    {
        if (y1 > y2)
        {
            double tmp = x1;
            this->x1 = x2;
            this->x2 = tmp;
            tmp = y1;
            this->y1 = y2;
            this->y2 = tmp;
        }
        a = 0;
    }
    else
    {
        a = (y2-y1)/(x2-x1);
    }

    b = y1-x1*a;
}

// Sets the coordinates of the line such that in the end x1 < x2.
void Line::set(double x1_, double y1_, double x2_, double y2_)
{
    if (x1_ < x2_)
    {
        this->x1 = x1_;
        this->x2 = x2_;
        this->y1 = y1_;
        this->y2 = y2_;
    }
    else
    {
        this->x1 = x2_;
        this->x2 = x1_;
        this->y1 = y2_;
        this->y2 = y1_;
    }

    if (isVertical())
    {
        if (y1 > y2)
        {
            double tmp = x1;
            this->x1 = x2;
            this->x2 = tmp;
            tmp = y1;
            this->y1 = y2;
            this->y2 = tmp;
        }
        a = 0;
    }
    else
    {
        a = (y2-y1)/(x2-x1);
    }

    b = y1-x1*a;
}

// Sets the coordinates of the line such that in the end x1 < x2.
void Line::set(const Vec2 &p1, const Vec2 &p2)
{
    if (p1.x < p2.x)
    {
        this->x1 = p1.x;
        this->x2 = p2.x;
        this->y1 = p1.y;
        this->y2 = p2.y;
    }
    else
    {
        this->x1 = p2.x;
        this->x2 = p1.x;
        this->y1 = p2.y;
        this->y2 = p1.y;
    }

    if (isVertical())
    {
        if (y1 > y2)
        {
            double tmp = x1;
            this->x1 = x2;
            this->x2 = tmp;
            tmp = y1;
            this->y1 = y2;
            this->y2 = tmp;
        }
        a = 0;
    }
    else
    {
        a = (y2-y1)/(x2-x1);
    }

    b = y1-x1*a;
}

// Returns the Euclidean length of the line.
double Line::length() const
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Returns the angle of the line with respect to the x-axis.
double Line::angle() const
{
    return fatan2((y2-y1),(x2-x1));
}

// The shortest distance between this line and the point p.
// Note that the shortest distance is either the perpendicular of
// the line through point p, or the distance to one of the end points.
double Line::distance(const Vec2 &p) const
{
    //https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment#

    Vec2 v1(x1,y1);
    Vec2 v2(x2,y2);
    double l2 = (v1-v2).norm2();  // avoid a sqrt
    if (l2 < EPSILON)
        return (p-v1).norm();
    double t = max(0.0, min(1.0, (p-v1)*(v2-v1)/l2));
    Vec2 pp = v1 + t*(v2-v1);
    return (p-pp).norm();
}

// Returns the closest point of this line to the given point p.
// Note that the closest point is either one of the end points, or
// the point where the perpendicular of the line through p
// intersects with the line.
Vec2 Line::closestPoint(const Vec2 &p) const
{
    Vec2 v1(x1,y1);
    Vec2 v2(x2,y2);
    double l2 = (v1-v2).norm2();  // avoid a sqrt
    if (l2 < EPSILON)
        return v1;
    double t = max(0.0, min(1.0, (p-v1)*(v2-v1)/l2));
    Vec2 pp = v1 + t*(v2-v1);
    return pp;
}

// Translates the line by (dx,dy).
void Line::translate(double dx, double dy)
{
    x1 += dx;
    y1 += dy;
    x2 += dx;
    y2 += dy;
    b = y1-x1*a;
}

// Translates the line by d.
void Line::translate(const Vec2 &d)
{
    x1 += d.x;
    y1 += d.y;
    x2 += d.x;
    y2 += d.y;
    b = y1-x1*a;
}

// Rotates the line around the origin, i.e., applies a rotation
// by the given angle to both end points.
void Line::rotate(double angle)
{
    if (fabs(angle) < EPSILON)
        return;

    double c = fcos(angle);
    double s = fsin(angle);
    rotate(s,c);
}

// Fast rotate for cases where the sin and cos of the angle are known.
// The line is rotated around the origin of the coordinate system.
void Line::rotate(double s, double c)
{
    if (fabs(s) < EPSILON)
        return;

    double x_ = x1;
    double y_ = y1;

    x1 = x_*c + y_*-s;
    y1 = x_*s + y_*c;

    x_ = x2;
    y_ = y2;

    x2 = x_*c + y_*-s;
    y2 = x_*s + y_*c;

    // Maintain x1 < x2 invariant.
    if (x2 < x1)
    {
        double tmp = x2;
        x2 = x1;
        x1 = tmp;
        tmp = y2;
        y2 = y1;
        y1 = tmp;
    }

    if (isVertical())
    {
        if (y1 > y2)
        {
            double tmp = x1;
            this->x1 = x2;
            this->x2 = tmp;
            tmp = y1;
            this->y1 = y2;
            this->y2 = tmp;
        }
        a = 0;
    }
    else
    {
        a = (y2-y1)/(x2-x1);
    }

    b = y1-x1*a;
}

// Scales the line by multiplying its end points with s.
void Line::scale(double s)
{
    x1 *= s;
    y1 *= s;
    x2 *= s;
    y2 *= s;
    b = y1-x1*a;
}

// Returns the left endpoint of this line.
Vec2 Line::p1() const
{
    return Vec2(x1, y1);
}

// Returns the right endpoint of this line.
Vec2 Line::p2() const
{
    return Vec2(x2, y2);
}

// Determines if this and the given line segment l intersect.
bool Line::intersects(const Line &l) const
{
    // The sign of this epsilon determines if the line intersection test
    // is loose or tight. When the epsilon is positive, the intersection
    // test is loose in a way that a line passing through the end point
    // of another line is not considered as intersecting. When the epsilon
    // is negative, the intersection test is tight and any kind of a
    // slightest touch between two lines counts as an intersection.
    double epsilon = -EPSILON;

    if (fabs(a-l.a) < epsilon) // lines are parallel
        return false;

    if (x2-x1 < EPSILON) // this line is vertical
    {
        double y = l.b+l.a*x1;
        return (x1 > l.x1+epsilon && x1 < l.x2-epsilon && y > qMin(y1, y2)+epsilon && y < qMax(y1, y2)-epsilon);
    }

    if (l.x2-l.x1 < EPSILON) // other line is vertical
    {
        double y = b+l.x1*a;
        return (l.x1 > x1+epsilon && l.x1 < x2-epsilon && y > qMin(l.y1, l.y2)+epsilon && y < qMax(l.y1, l.y2)-epsilon);
    }

    double x = (l.b-b)/(a-l.a);
    return (x > x1+epsilon && x < x2-epsilon && x > l.x1+epsilon && x < l.x2-epsilon);
}

// Decides if v1 and v2 are on the same side of this line or not.
bool Line::sameSide(const Vec2 &v1, const Vec2 &v2) const
{
    if (x2-x1 < EPSILON) // this line is vertical
        return (v1.x-x1)*(v2.x-x1) > 0;
    return (v1.y-a*v1.x-b)*(v2.y-a*v2.x-b) > 0;
}

// Returns true when the line is vertical.
bool Line::isVertical() const
{
    return (x2-x1 < EPSILON);
}

// Returns true when the line is horizontal.
bool Line::isHorizontal() const
{
    return (fabs(y2-y1) < EPSILON);
}

// Draws the line with a QPainter.
void Line::draw(QPainter *painter)
{
    if (p1() == p2())
        return;
    QLineF l(x1,y1,x2,y2);
    painter->drawLine(l);
}

// Writes the line into a data stream.
void Line::streamOut(QDataStream &out) const
{
    out << x1;
    out << y1;
    out << x2;
    out << y2;
}

// Reads the line from a data stream.
void Line::streamIn(QDataStream &in)
{
    in >> x1;
    in >> y1;
    in >> x2;
    in >> y2;

    if (isVertical())
        a = 0;
    else
        a = (y2-y1)/(x2-x1);
    b = y1-x1*a;
}

QDataStream& operator<<(QDataStream& out, const Line &o)
{
    o.streamOut(out);
    return out;
}

QDataStream& operator>>(QDataStream& in, Line &o)
{
    o.streamIn(in);
    return in;
}

QDebug operator<<(QDebug dbg, const Line &o)
{
    dbg.setAutoInsertSpaces(false);
    dbg << "[" << o.x1 << ", " << o.y1 << "] to [" << o.x2 << ", " << o.y2 <<"] ";
    dbg.setAutoInsertSpaces(true);
    return dbg;
}
