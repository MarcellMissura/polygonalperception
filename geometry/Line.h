#ifndef LINE_H_
#define LINE_H_
#include <QPainter>
#include "util/Vec2.h"

// The Line object is a line segment defined by two points (x1,y1) and (x2,y2).
// Public read/write access to these coordinates is granted for simple use.
// The Line implements a fast line intersection method.

class Line
{
public:
    double x1,y1,x2,y2; // end point coordinates
    double a,b; // slope and offset for fast line intersection

    Line();
    Line(const Vec2& p1, const Vec2& p2);
    Line(double x1, double y1, double x2, double y2);

    void set(double x1, double y1, double x2, double y2);
    void set(const Vec2& p1, const Vec2& p2);

    Vec2 p1() const;
    Vec2 p2() const;

    bool intersects(const Line& l) const;
    double length() const;
    double angle() const;
    double distance(const Vec2& p) const;
    Vec2 closestPoint(const Vec2& p) const;

    void translate(double dx, double dy);
    void translate(const Vec2& d);
    void rotate(double angle);
    void rotate(double s, double c);
    void scale(double s);

    bool sameSide(const Vec2& v1, const Vec2& v2) const;
    bool isVertical() const;
    bool isHorizontal() const;

    // Comparison operator for queing.
    bool operator() (const Line* l1, const Line* l2) {return (l1->length() >= l2->length());}
    bool operator() (const Line& l1, const Line& l2) {return (l1.length() >= l2.length());}

    void draw(QPainter* painter);
    virtual void streamOut(QDataStream& out) const;
    virtual void streamIn(QDataStream& in);
};

QDebug operator<<(QDebug dbg, const Line &o);
QDataStream& operator<<(QDataStream& out, const Line &o);
QDataStream& operator>>(QDataStream& in, Line &o);

#endif // Line_H
