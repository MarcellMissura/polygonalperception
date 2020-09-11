#ifndef GRIDMODEL_H_
#define GRIDMODEL_H_
#include <QDebug>
#include "util/Vector.h"
#include "util/Vec2.h"
#include "util/Vec2u.h"
#include "util/Vec2i.h"
#include "learner/Grid.h"
#include "geometry/Polygon.h"
#include "opencv2/imgproc/imgproc.hpp"

class GridModel : public Grid
{
    cv::Mat M;
    uchar maxv;

public:

    GridModel();
    ~GridModel(){}
    GridModel(const GridModel &o);  // copy constructor
    GridModel& operator=(const GridModel &o);

    void init();
    void clear();

    uint getWidth() const;
    uint getHeight() const;

    void setMaxV(uchar m);

    void setBorder(uchar val);
    void dilate(double radius);
    void blur(double radius);
    void canny();

    uchar valueAt(const Vec2& x) const;
    uchar valueAt(const Vec2u &idx) const;
    uchar valueAt(uint i, uint j) const;
    void setAt(const Vec2& x, uchar v);
    void setAt(const Vec2u& idx, uchar v);
    void setAt(uint i, uint j, uchar v);

    const uchar* data() const;
    const uchar* row(const int &r) const;

    void extractPolygons();

    bool isOccupied(const Vec2& x) const;
    bool isOccupied(const Vec2u& idx) const;

    bool hasLineOfSight(const Vec2u& cellIdxA, const Vec2u& cellIdxB) const;

    void draw(QPainter* painter) const;
    void draw() const;

    void streamOut(QDataStream& out) const;
    void streamIn(QDataStream& in);
};

QDebug operator<<(QDebug dbg, const GridModel &w);
QDataStream& operator<<(QDataStream& out, const GridModel &o);
QDataStream& operator>>(QDataStream& in, GridModel &o);

#endif
