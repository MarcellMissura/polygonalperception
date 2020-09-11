#include "Polygon.h"
#include "globals.h"
#include "blackboard/Config.h"
#include "blackboard/State.h"
#include "Box.h"
#include "util/ColorUtil.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <GL/glu.h>

// The Polygon class is a general purpose polygon that consists of a number
// of vertices and a 2D transform given by a translation (x,y) and a rotation
// (theta). The order of the transformation is first translation, then rotation.
//
// Polygons must have at least 3 vertices. The vertices are stored in a LinkedList
// and must be specified in a counter clockwise order. The LinkedList has an
// advantage in terms of memory management in exchange for not having random
// access to the vertices, but typically the vertices are accessed in a sequence
// (looped over) and random access is not really needed.
//
// The Polygon class provides means to query and manipulate its transformation
// relative to the world using pos(), rotation(), translate(), and rotate().
// We call a polygon that has a nonzero transformation untransformed in the
// sense that its vertices are given relative to the frame that is specified by
// the transformation. When the transformation is zero, the vertices are
// automatically given in world coordinates and we call the polygon transformed.
// You may want to call the transform() method to "consume" the transformation
// and update the vertices of the polygon to world coordinates. You can then
// untransform() a polygon again by settings its centroid as the reference frame.

Polygon::Polygon()
{
    boundingBoxValid = false;
    convexityFlag = -1;
    setPos(0, 0);
    setRotation(0);
}

// Box constructor for convenience.
Polygon::Polygon(double x, double y, double w, double h)
{
    boundingBoxValid = false;
    convexityFlag = 1;
    setPos(x, y);
    setRotation(0);

    vertices << Vec2(-w, h);
    vertices << Vec2(-w, -h);
    vertices << Vec2(w, -h);
    vertices << Vec2(w, h);
}

Polygon::~Polygon()
{

}

// Discards all vertices and resets the transformation to 0.
// It is useful for the recycling of Polygon objects.
void Polygon::clear()
{
    vertices.clear();
    x = 0;
    y = 0;
    theta = 0;
    boundingBoxValid = false;
    convexityFlag = -1;
}

// Returns the number of vertices of this polygon.
int Polygon::size() const
{
    return vertices.size();
}

// Returns the vertices as a QPolygonF. This is used for Qt compatibility.
// The QPolygonF is given in local coordinates (untransformed).
QPolygonF Polygon::polygon() const
{
    QPolygonF pol;
    ListIterator<Vec2> it = vertices.begin();
    while (it.hasNext())
        pol << it.next();
    return pol;
}

// Sets the (x,y) position of the polygon.
void Polygon::setPos(double x, double y)
{
    this->x = x;
    this->y = y;
    boundingBoxValid = false;
}

// Sets the (x,y) position of the polygon.
void Polygon::setPos(const Vec2& p)
{
    this->x = p.x;
    this->y = p.y;
    boundingBoxValid = false;
}

// Returns the (x,y) position of the polygon.
Vec2 Polygon::pos() const
{
    return Vec2(x, y);
}

// Translates the polygon by (dx,dy).
void Polygon::translate(double dx, double dy)
{
    setPos(this->x+dx, this->y+dy);
}

// Translates the polygon by d.
void Polygon::translate(const Vec2 &d)
{
    setPos(this->x+d.x, this->y+d.y);
}

// Rotates the polygon counter clockwise by the angle a given in radians.
void Polygon::rotate(double a)
{
    setRotation(theta+a);
}

// Returns the rotation theta of the polygon.
double Polygon::rotation() const
{
    return theta;
}

// Sets the rotation theta of the polygon.
void Polygon::setRotation(double a)
{
    theta = a;
    boundingBoxValid = false;
}

// Grows (or shrinks) the polygon by delta. Delta is the distance by how many meters the
// vertices are pushed outwards along the half angle between the two neighbouring edges.
// If delta is negative, the vertices are pulled invards and the polygon is shrunk.
// The vertices must be given in a counter clockwise order such that the outside of the
// polygon is always to the right. The polygon doesn't have to be convex and it can be
// in an utransformed state. Be aware that for non-convex polygons this operation may
// result in self-intersections.
void Polygon::grow(double delta)
{
    // Compute an offset to each vertex fisrt.
    int s = size();
    Vec2 offset[size()];
    int i = 0;
    ListIterator<Vec2> it = vertices.begin();
    while (it.hasNext())
    {
        Vec2 v1 = it.peekCur() - it.peekPrev();
        Vec2 v2 = it.peekNext() - it.peekCur();
        it.next();

        v1.normalize();
        v2.normalize();
        Vec2 v3 = (v1-v2)/2;
        v3.normalize();
        offset[i] = sgn(v1.x*v2.y-v1.y*v2.x)*delta*v3;
        i++;
    }

    // Apply the offsets to the vertices.
    it = vertices.begin();
    i = 0;
    while (it.hasNext())
    {
        Vec2& v = it.next();
        v += offset[i];
        i++;
    }

    boundingBoxValid = false;
}

// Scales (multiplies) the polygon vertices by the factors sx and sy.
// When the polygon is in an untransformed state and the center is
// say the centroid of the polygon, this scaling has a growing effect.
// If the polygon is transformed, the result might not be what you
// expect.
void Polygon::scale(double sx, double sy)
{
    ListIterator<Vec2> it = vertices.begin();
    while (it.hasNext())
    {
        Vec2& v = it.next();
        v.x *= sx;
        v.y *= sy;
    }

    boundingBoxValid = false;
}

// Reverses the order of the vertices. This is sometimes needed to restore the CCW order.
void Polygon::reverseOrder()
{
    vertices.reverse();
}

// Computes the centroid of the polygon.
// The centroid is given in world coordinates.
Vec2 Polygon::centroid() const
{
    Vec2 c;
    ListIterator<Vec2> it = vertices.begin();
    while (it.hasNext())
    {
        Vec2& v = it.next();
        c += v;
    }
    c /= size();
    c += pos();
    return c;
}

// Returns the edges of the polygon as a list of lines.
// The order of the returned edges is the same as the order of the vertices (CCW).
// The edges will be given in local coordinates (untransformed).
LinkedList<Line> Polygon::edges() const
{
    LinkedList<Line> edges;
    ListIterator<Vec2> it = vertices.begin();
    while (it.hasNext())
    {
        edges << Line(it.peekCur(), it.peekNext());
        it.next();
    }
    return edges;
}

// Returns a LinkedList of the vertices (corners) of the polygon.
// The vertices will be given in local coordinates (untransformed).
// The returned list is a copy of the vertices. Do with it whatever you want.
LinkedList<Vec2> Polygon::getVertices() const
{
    return vertices;
}

// Sets (overwrites) the vertices of the polygon.
void Polygon::setVertices(const LinkedList<Vec2> &v)
{
    clear();
    vertices = v;
    boundingBoxValid = false;
    convexityFlag = -1;
}

// Returns the bounding box of the polygon in world coordinates.
const Box &Polygon::boundingBox() const
{
    if (boundingBoxValid)
        return aabb;
    boundingBoxValid = true;

    double c = 1;
    double s = 0;
    if (theta > EPSILON || theta < EPSILON)
    {
        c = fcos(theta);
        s = fsin(theta);
    }

    ListIterator<Vec2> it = vertices.begin();
    Vec2 v = it.next();
    if (theta > EPSILON || theta < EPSILON)
        v.rotate(s, c);
    v += pos();
    double left = v.x;
    double right = v.x;
    double top = v.y;
    double bottom = v.y;
    while (it.hasNext())
    {
        Vec2 v = it.next();

        if (theta > EPSILON || theta < EPSILON)
            v.rotate(s, c);
        v += pos();

        left = min(v.x, left);
        right = max(v.x, right);
        top = max(v.y, top);
        bottom = min(v.y, bottom);
    }

    aabb.set(top, left, bottom, right);
    return aabb;
}

// Computes and returns the convex hull of this polygon. The
// points do not have to be in a counter clockwise order to
// compute the convex hull, but the returned convex hull is a
// valid polygon in ccw order. The convex hull is given in local
// coordinates (untransformed), and it has the same transform as
// this polygon.
Polygon Polygon::convexHull() const
{
    if (vertices.isEmpty())
        return Polygon();

    std::vector<cv::Point2f> pol;
    ListIterator<Vec2> it = vertices.begin();
    while (it.hasNext())
    {
        Vec2& v = it.next();
        pol.push_back(cv::Point2f(v.x, v.y));
    }

    std::vector<cv::Point2f> chPoints;
    cv::convexHull(pol, chPoints);

    Polygon ch;
    for (int i = 0; i < chPoints.size(); i++)
        ch << Vec2(chPoints[i].x, chPoints[i].y);
    ch.setPos(pos());
    ch.setRotation(rotation());

    return ch;
}

// Computes and returns the non-convex hull of this polygon.
// The points do not have to be in a counter clockwise order to
// compute the non-convex hull, but the returned non-convex hull
// is a valid polygon in ccw order. The non-convex hull is given
// in local coordinates (untransformed), and it has the same
// transform as this polygon.
Polygon Polygon::nonConvexHull() const
{
    if (vertices.isEmpty())
        return Polygon();

    // Find the leftmost and rightmost points in x direction.
    ListIterator<Vec2> it = vertexIterator();
    Vec2 min = it.next();
    Vec2 max = min;
    while (it.hasNext())
    {
        Vec2& v = it.next();
        if (v.x < min.x)
            min = v;
        if (v.x > max.x)
            max = v;
    }

    // The max-min vector will divide the set of vertices into an upper and lower half.
    Vec2 divider = max-min;

    // Sort the vertices by their x coordinate.
    LinkedList<Vec2> V = getVertices();
    V.sort();

    // Iterate through the sorted points and add them to the upper or lower chain.
    double thresh = PI2;
    LinkedList<Vec2> upper;
    LinkedList<Vec2> lower;
    while (V.hasNext())
    {
        Vec2& v = V.next();
        if (v.isLeftOf(divider))
        {
            Vec2 v1 = upper.end().prev();
            Vec2 v2 = upper.last();
            Vec2 v3 = v;
            if ((v2-v1).angleTo(v3-v2) > thresh)
                upper.pop();
            upper << v;
        }
        else
        {
            Vec2 v1 = lower.end().prev();
            Vec2 v2 = lower.last();
            Vec2 v3 = v;
            if ((v2-v1).angleTo(v3-v2) < -thresh)
                lower.pop();
            lower << v;
        }
    }

    upper.reverse();

    Polygon p;
    while (upper.hasNext())
        p << upper.next();
    while (lower.hasNext())
        p << lower.next();
    p.setPos(pos());
    p.setRotation(rotation());
    return p;
}

// Computes a triangulation of the polygon and returns the triangles as a LinkedList
// of Polygons. The triangulation is computed with the ear clipping method.
LinkedList<Polygon> Polygon::triangulate() const
{
    LinkedList<Polygon> triangles;
    LinkedList<Vec2> V = getVertices();

    while (V.size() > 3)
    {
        qDebug() << "V:" << V;

        ListIterator<Vec2> it = V.begin();
        bool earFound = false;
        while (!earFound && it.hasNext())
        {
            Vec2& v1 = it.peekPrev();
            Vec2& v2 = it.peekCur();
            Vec2& v3 = it.peekNext();
            qDebug() << V.size() << "v1:" << v1 << v2 << v3 << (v2.x-v1.x)*(v3.y-v2.y)-(v2.y-v1.y)*(v3.x-v2.x);

            if ((v2.x-v1.x)*(v3.y-v2.y)-(v2.y-v1.y)*(v3.x-v2.x) > 0) // Left turn test
            {
                earFound = true;
            }
            else
            {
                it.next();
            }
        }

        if (!earFound)
            qDebug() << "No ear found in" << V;

        Vec2& v1 = it.peekPrev();
        Vec2& v2 = it.peekCur();
        Vec2& v3 = it.peekNext();
        Polygon t;
        t << v1;
        t << v2;
        t << v3;
        triangles << t;
        qDebug() << "remove" << v2 << V.size();
        V.remove(v2);
    }

    Polygon last;
    last.setVertices(V);
    triangles << last;

    return triangles;
}

// Returns the diameter of the polygon.
// The diameter of a polygon is the largest distance between any pair of vertices.
double Polygon::diameter() const
{
    double d = 0;
    ListIterator<Vec2> it = vertices.begin();
    while (it.hasNext())
    {
        Vec2& v1 = it.next();
        ListIterator<Vec2> it2 = it;
        while (it2.hasNext())
        {
            Vec2& v2 = it2.next();
            d = qMax(d, (v1-v2).norm());
        }
    }
    return d;
}

// Returns the area of the polygon.
double Polygon::area() const
{
    // Untested code.
    double a = 0;
    ListIterator<Vec2> it = vertexIterator();
    do
    {
        Vec2& v1 = it.peekCur();
        Vec2& v2 = it.peekNext();
        a += v1.x*v2.y-v2.x*v1.y;
        it.next();
    } while (it.hasNext());
    return 0.5*a;
}

// Computes the shortest distance between this polygon and the point p.
double Polygon::distance(const Vec2 &p) const
{
    double minDist = std::numeric_limits<double>::max();
    Line l;
    ListIterator<Vec2> it = vertices.begin();
    while (it.hasNext())
    {
        l.set(it.peekCur(), it.peekNext());
        minDist = min(l.distance(p), minDist);
        it.next();
    }

    return minDist;
}

// Returns the closest point of the polygon to the given point p.
Vec2 Polygon::closestPoint(const Vec2 &p) const
{
    double minDist = std::numeric_limits<double>::max();
    Line l;
    Vec2 cp,pp;
    ListIterator<Vec2> it = vertices.begin();
    while (it.hasNext())
    {
        l.set(it.peekCur(), it.peekNext());
        pp = l.closestPoint(p);
        double n = (pp-p).norm2();
        if (n < minDist)
        {
            cp = pp;
            minDist = n;
        }
        it.next();
    }

    return cp;
}

// Returns true if the polygon is convex.
// The vertices have to be in counterclockwise order, otherwise the test result is undefined.
bool Polygon::isConvex() const
{
    if (convexityFlag == -1)
    {
        convexityFlag = 1;
        ListIterator<Vec2> it = vertices.begin();
        while (it.hasNext())
        {
            Vec2& v1 = it.peekPrev();
            Vec2& v2 = it.peekCur();
            Vec2& v3 = it.peekNext();

            if ((v2.x-v1.x)*(v3.y-v2.y)-(v2.y-v1.y)*(v3.x-v2.x) < 0) // Right turn test.
            {
                convexityFlag = 0;
                break;
            }

            it.next();
        }
    }

    return convexityFlag;
}

// Returns true if the polygon p intersects with this one.
// This is the most generic polygon vs polygon intersection test that
// detects overlaps, touches, and containment. It's based on the SAT
// algorithm and it's only applicable to convex polygons.
bool Polygon::intersects(const Polygon &p) const
{
    //qDebug() << "Polygon::intersects():";
    //qDebug() << "this:" << *this;
    //qDebug() << "with:" << p;

    if (!isConvex())
        qDebug() << "Polygon intersection test called for a nonconvex polygon.";

    // Bounding box check.
    boundingBox();
    if (!aabb.intersects(p.boundingBox()))
        return false;

    // This is an implementation of the SAT algorithm as described here:
    // http://www.dyn4j.org/2010/01/sat/
    // The concept is that if two convex polygons do not intersect, there
    // is a separating line between them, and one of the edges of either
    // polygon must be such a separating line. So the algorithm tests for
    // every edge if one of the polygons lies entirely on the one side and
    // the other polygon on the other. If such an edge is found, the
    // polygons do not intersect. If no edge can be found, the polygons
    // intersect. "Left of" and "right of" tests are performed using the
    // scalar product of the left normal of an edge and the vertex to test.
    // I deviate from the algorithm shown on the website in a way that
    // instead of projecting both polygons on every edge normal and trying
    // to find the separation, I test for every edge only the vertexes of
    // the *other* polygon. The polygon the edge was taken from always lies
    // entirely on the positive side of the edge. If all vertexes of the
    // other polygon lie on the negative side of the edge, we know there is
    // no intersection. This way I might miss an edge whose normal could be
    // a separating axis, even if both polygons lie on the same side of the
    // edge, and so I have to look at more edges than the standard SAT, but
    // I save computation time by computing the cross product only for the
    // vertices of one polygon, and quickly discarding an edge as soon as
    // a cross product evaluates positive. Not sure which way it's faster,
    // but this way is easier to code.


    // The polygons have to be in a transformed state for the SAT.
    Polygon source = *this;
    source.transform();
    Polygon target = p;
    target.transform();

    // Test the edges of the source against the points of the target.
    ListIterator<Vec2> it = source.vertexIterator();
    while (it.hasNext())
    {
        Vec2& v1 = it.peekCur();
        Vec2& v2 = it.peekNext();
        it.next();

        bool allPointsAreRightOf = true;
        ListIterator<Vec2> it2 = target.vertexIterator();
        while (it2.hasNext())
        {
            Vec2& p = it2.next();

            if ((v2.x-v1.x)*(p.y-v1.y)-(v2.y-v1.y)*(p.x-v1.x) >= 0) // right of test with scalar product
            {
                allPointsAreRightOf = false;
                break; // next source edge
            }
        }

        if (allPointsAreRightOf)
            return false;
    }

    // Test the edges of the target against the points of the source.
    it = target.vertexIterator();
    while (it.hasNext())
    {
        Vec2& v1 = it.peekCur();
        Vec2& v2 = it.peekNext();
        it.next();

        bool allPointsAreRightOf = true;
        ListIterator<Vec2> it2 = source.vertexIterator();
        while (it2.hasNext())
        {
            Vec2& p = it2.next();

            if ((v2.x-v1.x)*(p.y-v1.y)-(v2.y-v1.y)*(p.x-v1.x) >= 0) // right of test with scalar product
            {
                allPointsAreRightOf = false;
                break; // next target edge
            }
        }

        if (allPointsAreRightOf)
            return false;
    }

    return true;
}

// Returns true if the line l intersects with any edge of this polygon.
// This is an edge intersection test only that will not detect containment.
// There are no requirements on the polygon. It does not have to be convex.
// The vertex order is not relevant, and it can be transformed or untransformed.
bool Polygon::intersects(Line l) const
{
    //qDebug() << "-------- Collision Checking Line" << l << "with:" << *this;

    // Bounding box check.
//    boundingBox();
//    if (!aabb.intersects(l))
//        return false;

    l.translate(-pos());
    l.rotate(-theta);

    // full edge check.
    Line edge;
    ListIterator<Vec2> it = vertices.begin();
    while (it.hasNext())
    {
        Vec2& v1 = it.peekCur();
        Vec2& v2 = it.peekNext();
        it.next();

        edge.set(v1.x, v1.y, v2.x, v2.y);
        //qDebug() << edge.intersects(l) << edge;
        if (edge.intersects(l))
            return true;
    }

    return false;
}

// Returns true if the polygon contains the point. It will return true for a point
// lying exactly on the boundary of the polygon. The point is given in world coordinates,
// but the polygon does not have to be transformed. There is a speed up if the polygon
// is convex and given in counter clockwise order.
bool Polygon::intersects(Vec2 v) const
{
    // Bounding box check.
    boundingBox();
    if (!aabb.intersects(v))
        return false;

    // Transform the point to local coordinates.
    v -= pos();
    v.rotate(-theta);

    if (isConvex())
    {
        // Point intersection with a convex polygon is implemented using
        // a scalar product test with every edge. The point has to be to the left of
        // every edge in order to be contained in a counter clockwise convex polygon.
        // http://totologic.blogspot.de/2014/01/accurate-point-in-triangle-test.html
        // As soon as one edge is found the point lies on the right of, the algorithm
        // can abort and report that there is no collision.

        ListIterator<Vec2> it = vertices.begin();
        while (it.hasNext())
        {
            Vec2& v1 = it.peekCur();
            Vec2& v2 = it.peekNext();
            it.next();

            if ((v2.y-v1.y)*(v.x-v1.x)+(-v2.x+v1.x)*(v.y-v1.y) > 0) // Strict rightof test.
                return false;
        }

        return true;
    }
    else
    {
        // A point intersection test with a non-convex polygon is just as easy.
        // Take a random half-line starting at the point and intersect it with every edge
        // of the polygon. Is the number of intersections odd, then the point lies inside
        // the polygon. Otherwise it lies outside.

        // Create a vertical half line starting at the point v.
        Line vertical;
        vertical.set(v.x, v.y, v.x, std::numeric_limits<double>::max());

        // Count the number of intersections.
        Line l;
        int ctr = 0;
        ListIterator<Vec2> it = vertices.begin();
        while (it.hasNext())
        {
            Vec2& v1 = it.peekCur();
            Vec2& v2 = it.peekNext();
            l.set(v1, v2);
            if (l.intersects(vertical)) // strict
                ctr++;
            it.next();
        }

        return (ctr%2);
    }

    return true;
}

// Draws the polygon on a QPainter.
void Polygon::draw(QPainter *painter) const
{
    painter->save();
    painter->translate(x, y);
    painter->rotate(180*theta/PI);
    painter->drawPath(shape());
    painter->restore();
}

// Draws the polygon in an OpenGL context.
void Polygon::draw() const
{
    glBegin(GL_LINE_LOOP);
    ListIterator<Vec2> it2 = vertexIterator();
    while (it2.hasNext())
    {
        Vec2& v = it2.next();
        glVertex3f(v.x, v.y, 0.005);
    }
    glEnd();
}

// This method is used by the Qt graphics view framework.
QRectF Polygon::boundingRect() const
{
    return polygon().boundingRect();
}

// Returns the shape of the polygon as a QPainterPath in local coordinates.
// shape() is only used for easy drawing with QPainter.
QPainterPath Polygon::shape() const
{
    QPainterPath pp;
    pp.moveTo(vertices.first());
    ListIterator<Vec2> it = vertices.begin();
    it.next();
    while (it.hasNext())
    {
        Vec2& v = it.next();
        pp.lineTo(v);
    }
    pp.lineTo(vertices.first());
    return pp;
}

// Appends a vertex to the polygon.
void Polygon::addVertex(const Vec2 &p)
{
    vertices << p;
    boundingBoxValid = false;
    convexityFlag = -1;
    return;
}

// Appends a vertex to the polygon.
Polygon& Polygon::operator<<(const Vec2 &p)
{
    vertices << p;
    boundingBoxValid = false;
    convexityFlag = -1;
    return *this;
}

// Removes the given vertex from the polygon, if it exists.
void Polygon::removeVertex(const Vec2 &p)
{
    vertices.remove(p);
    boundingBoxValid = false;
}

// Returns an iterator that can be used to conveniently cycle through the corners of the polygon.
ListIterator<Vec2> Polygon::vertexIterator() const
{
    return vertices.begin();
}

// Consumes the current transformation in a way that it transforms all
// vertices to world coordinates and then resets the transformation to zero.
void Polygon::transform()
{
    if (x == 0 && y == 0 && theta == 0) // little speedup
        return;

    double c = fcos(theta);
    double s = fsin(theta);

    ListIterator<Vec2> it = vertices.begin();
    while (it.hasNext())
    {
        Vec2& v = it.next();
        v.rotate(s, c);
        v += pos();
    }

    setPos(0, 0);
    setRotation(0);
}

// Untransforms the polygon in a way that the centroid becomes the transformation
// and the vertices are expressed with respect to the centroid.
void Polygon::untransform()
{
    Vec2 c = centroid();
    ListIterator<Vec2> it = vertices.begin();
    while (it.hasNext())
    {
        Vec2& v = it.next();
        v -= c;
    }

    setPos(c);
}

// Writes the polygon into a data stream.
void Polygon::streamOut(QDataStream &out) const
{
    out << x;
    out << y;
    out << theta;
    out << vertices;
}

// Reads the polygon from a data stream.
void Polygon::streamIn(QDataStream &in)
{
    in >> x;
    in >> y;
    in >> theta;
    in >> vertices;
    boundingBoxValid = false;
    convexityFlag = -1;
}

QDataStream& operator<<(QDataStream& out, const Polygon &o)
{
    o.streamOut(out);
    return out;
}

QDataStream& operator>>(QDataStream& in, Polygon &o)
{
    o.streamIn(in);
    return in;
}

QDebug operator<<(QDebug dbg, const Polygon &o)
{
    if (dbg.autoInsertSpaces())
        dbg << "pos:" << o.pos() << "angle:" << o.rotation() << "points:" << o.getVertices();
    else
        dbg << "pos: " << o.pos() << "angle: " << o.rotation() << "points: " << o.getVertices();
    return dbg;
}
