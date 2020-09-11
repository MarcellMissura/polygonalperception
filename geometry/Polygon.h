#ifndef POLYGON_H_
#define POLYGON_H_
#include <QPainter>
#include "Line.h"
#include "util/Vec2.h"
#include "util/LinkedList.h"
#include "geometry/Box.h"

class Polygon
{
public:

    // These variables contain the transformation of the polygon,
    // which is first a translation by (x,y) and then a rotation by theta.
    // Whenever the transformation is changed, transform() will have to be
    // called to upate the vertices.
    double x,y,theta;

protected:

    LinkedList<Vec2> vertices; // The corners of the polygon.

protected:
    mutable char convexityFlag;
    mutable bool boundingBoxValid;
    mutable Box aabb;

public:

    Polygon();
    Polygon(double x, double y, double w, double h);
    ~Polygon();

    void clear();
    int size() const;
    QPolygonF polygon() const;

    Vec2 pos() const;
    void setPos(const Vec2& p);
    void setPos(double x, double y);
    double rotation() const;
    void setRotation(double a);
    void translate(double dx, double dy);
    void translate(const Vec2& d);
    void rotate(double a);
    void transform();
    void untransform();

    void grow(double delta);
    void scale(double sx, double sy);

    void reverseOrder();

    Vec2 centroid() const;
    LinkedList<Line> edges() const;
    LinkedList<Vec2> getVertices() const;
    void setVertices(const LinkedList<Vec2>&v);
    ListIterator<Vec2> vertexIterator() const;
    virtual const Box& boundingBox() const;
    Polygon convexHull() const;
    Polygon nonConvexHull() const;
    LinkedList<Polygon> triangulate() const;

    double diameter() const;
    double area() const;
    double distance(const Vec2& p) const;
    Vec2 closestPoint(const Vec2& p) const;

    bool isConvex() const;

    QRectF boundingRect() const;
    QPainterPath shape() const;

    virtual bool intersects(const Polygon &p) const;
    virtual bool intersects(Line l) const;
    virtual bool intersects(Vec2 v) const;

    virtual void draw(QPainter* painter) const;
    virtual void draw() const;
    virtual void streamOut(QDataStream& out) const;
    virtual void streamIn(QDataStream& in);

    void addVertex(const Vec2 &p);
    Polygon& operator<<(const Vec2 &p);
    void removeVertex(const Vec2& p);
};

QDebug operator<<(QDebug dbg, const Polygon &o);
QDataStream& operator<<(QDataStream& out, const Polygon &o);
QDataStream& operator>>(QDataStream& in, Polygon &o);


#endif // POLYGON_H_
