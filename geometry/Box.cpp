#include "Box.h"
#include "util/ColorUtil.h"
#include "blackboard/Config.h"

// The Box is an axis aligned rectangle typically used as a bounding box
// to accelerate collision check routines. Its (x,y) position defines the
// world coordinates of a reference point of the box while t,l,b,r define
// the distance of the top, left, bottom, and right border with respect to
// the reference point. This representation makes it easy to compute the
// Minkowski sum of two boxes, which is needed to compute the bounding box
// of the swept volume of a moving polygon. Most importantly, Box implements
// intersect() methods for a number of geometric primitives for extra fast
// but approximate collision detection.

Box::Box()
{
    x = 0;
    y = 0;
    t = 0;
    l = 0;
    b = 0;
    r = 0;
}

// Sets the bounding box by providing the world coordinates of the top, left,
// bottom, and right borders.
void Box::set(double t, double l, double b, double r)
{
    // The reference point is set to be the bottom left corner.
    this->x = l;
    this->y = b;
    this->t = t-b;
    this->l = 0;
    this->b = 0;
    this->r = r-l;
}

// Sets the bounding box by providing the world coordinates of the reference
// point and the distances to the top, left, bottom, right borders relative
// to the reference point.
void Box::set(double x, double y, double t, double l, double b, double r)
{
    this->x = x;
    this->y = y;
    this->t = t;
    this->l = l;
    this->b = b;
    this->r = r;
}

// Returns the world x coordinate of the left margin.
double Box::left() const
{
    return x+l;
}

// Returns the world x coordinate of the right margin.
double Box::right() const
{
    return x+r;
}

// Returns the world y coordinate of the top margin.
double Box::top() const
{
    return y+t;
}

// Returns the world y coordinate of the bottom margin.
double Box::bottom() const
{
    return y+b;
}

// Returns the world coordinates of the top left corner.
Vec2 Box::topLeft() const
{
    return Vec2(left(), top());
}

// Returns the world coordinates of the top right corner.
Vec2 Box::topRight() const
{
    return Vec2(right(), top());
}

// Returns the world coordinates of the bottom left corner.
Vec2 Box::bottomLeft() const
{
    return Vec2(left(), bottom());
}

// Returns the world coordinates of the bottom right corner.
Vec2 Box::bottomRight() const
{
    return Vec2(right(), bottom());
}

// Sets the world (x,y) position of the box.
void Box::setPos(double x, double y)
{
    this->x = x;
    this->y = y;
}

// Sets the world (x,y) position of the box.
void Box::setPos(const Vec2& p)
{
    this->x = p.x;
    this->y = p.y;
}

// Returns the world (x,y) position of the box.
Vec2 Box::pos() const
{
    return Vec2(x, y);
}

// Translates the box by (dx,dy).
void Box::translate(double dx, double dy)
{
    setPos(this->x+dx, this->y+dy);
}

// Translates the box by (dx,dy).
void Box::translate(const Vec2 &d)
{
    setPos(this->x+d.x, this->y+d.y);
}

// Computes the Minkowski sum of this box and the other Box o.
void Box::operator+=(const Box &o)
{
    t += o.t;
    l += o.l;
    b += o.b;
    r += o.r;
}

// Computes the Minkowski sum of this box and the other Box o.
Box Box::operator+(const Box &o) const
{
    Box box = *this;
    box += o;
    return box;
}

// Returns true if the box intersects with (contains) the point p.
bool Box::intersects(const Vec2 &p) const
{
    return (p.y <= top() && p.x >= left() && p.y >= bottom() && p.x <= right());
}

// Returns true if the box o intersects with this one.
bool Box::intersects(const Box &o) const
{
    return !(o.bottom() > top() || o.right() < left() || o.top() < bottom() || o.left() > right());
}

// Returns true if the bounding box of the line l intersects this box.
bool Box::intersects(const Line &l) const
{
    if (l.x1 > right() || l.x2 < left())
        return false;
    if (l.y1 < l.y2)
    {
        if (l.y1 > top() || l.y2 < bottom())
            return false;
    }
    else
    {
        if (l.y2 > top() || l.y1 < bottom())
            return false;
    }

    return true;
}

// Draws the box on a QPainter.
void Box::draw(QPainter *painter) const
{
    QPainterPath pp;
    pp.moveTo(left(), top());
    pp.lineTo(right(), top());
    pp.lineTo(right(), bottom());
    pp.lineTo(left(), bottom());
    pp.lineTo(left(), top());

    painter->save();
    painter->setPen(colorUtil.pen);
    painter->drawPath(pp);
    painter->restore();
}

void Box::streamOut(QDataStream& out) const
{
    out << x;
    out << y;
    out << t;
    out << l;
    out << b;
    out << r;
}

void Box::streamIn(QDataStream& in)
{
    in >> x;
    in >> y;
    in >> t;
    in >> l;
    in >> b;
    in >> r;
}

QDebug operator<<(QDebug dbg, const Box &o)
{
    dbg << "tl:" << o.topLeft() << "br:" << o.bottomRight() << "pos:" << o.pos();
    return dbg;
}
