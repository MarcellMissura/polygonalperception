#include "GridModel.h"
#include "blackboard/Config.h"
#include "blackboard/State.h"
#include "geometry/Box.h"
#include "util/Logger.h"
#include <iostream>
#include "util/ColorUtil.h"
#include "GL/gl.h"
#include "blackboard/Command.h"

// The grid model is a 2D grid representation of the world. The cell size is
// typically 0.05 cm.

// The GridModel inherits from the Grid class and offers an interface to set up
// a grid structure by defining the number of dimensions, min and max boundaries
// along each dimension, and number of cells in each dimension. It provides
// functions to map continuous valued points into the grid by computing their
// grid coordinates, and to convert between the multi-dimensional cell index and
// a one dimensional cell index (flat index).

// In the heart of the GridModel is an opencv matrix M of type uchar (1 channel).
// Each item of the matrix (each cell in a floor grid) can have one of 256 values.
// The methods of the GridModel class revolve around this matrix such as init()
// -ializing it, setting and reading values of cells with setAt() and valueAt(),
// and checking for occupancy with the isOccupied() method.

// Furthermore, the GridModel offers a way to compute polygonal obstacles from
// the grid with the extractPolygons() function using a contour detection algorithm.

// And last but not least, we can interpret the grid to be an image and use image
// filtering techinques for example to dilate() the map - this is a great way of
// expanding arbitrary obstacles by the robot size - and to blur() the map, which
// smoothes it for the DWA.

// And the GridModel can draw() itself on a QPainter and in an OpenGL environment.

GridModel::GridModel()
{
    maxv = 255;
}

// Copy constructor.
GridModel::GridModel(const GridModel &o) : Grid(o)
{
    *this = o;
}

// Assignment operator.
GridModel& GridModel::operator=(const GridModel &o)
{
    if (this == &o)
        return *this;

    Grid::operator=(o);
    M = o.M.clone();
    maxv = o.maxv;

    return *this;
}

// The init methods sets up the grid structure (min, max, number of cells) and computes the
// raster of the grid coordinates. The parameters are computed using the config.
// This is where the data matrix M is initialized.
void GridModel::init()
{
    // Set up the grid structure.
    setDim(2);
    setN(Vec2u(config.gridSize, 100.0)); // Set the number of nodes per dimension.
    setMin(Vec2(0, -config.gridY)); // Set the minimum values per dimension.
    setMax(Vec2(config.gridX, config.gridY)); // Set the maximum values per dimension.
    rasterize(); // Compute the grid representation.

    // Set up the openCV matrix M, the data structure of the grid.
    // Pay attention that the size parameter is in cols,rows order.
    // x is the forward direction of the robot
    Vec2u n = getN();
    M = cv::Mat::zeros(cv::Size(n.x, n.y), cv::DataType<uchar>::type);
}

// Resets all grid values to zero.
void GridModel::clear()
{
    M = cv::Scalar(0);
}

// Returns the width of the grid (number of cells).
uint GridModel::getWidth() const
{
    return N[0];
}

// Returns the height of the grid (number of cells).
uint GridModel::getHeight() const
{
    return N[1];
}

// Sets the upper limit of the height map (only for visualization).
void GridModel::setMaxV(uchar m)
{
    maxv = m;
}

// Sets all border cells to a value of val.
// This is typically used to mark the border cells as "occupied" for
// cell creeper algorithms like A*.
void GridModel::setBorder(uchar val)
{
    Vec2u N = getN();
    for (int i = 0; i < N[0]; i++)
    {
        setAt(Vec2u(i, 0), val);
        setAt(Vec2u(i, N[1]-1), val);
    }
    for (int j = 0; j < N[1]; j++)
    {
        setAt(Vec2u(0, j), val);
        setAt(Vec2u(N[0]-1, j), val);
    }
}

// Returns true if the cell that the point x is in is occupied.
// All nonzero values are considered to be occupied. This has an effect on the
// grid based path planners A* and LazyThetaA*.
bool GridModel::isOccupied(const Vec2 &x) const
{
    return (valueAt(x) > 0);
}

// Returns true if the cell with the two dimensional index idx is occupied.
// All nonzero values are considered to be occupied. This has an effect on the
// grid based path planners A* and LazyThetaA*.
bool GridModel::isOccupied(const Vec2u &idx) const
{
    return (valueAt(idx) > 0);
}

// Returns true if the line between cell A and cell B does not come across an occupied cell.
// The implementation is based on the Bresenham algorithm. https://de.wikipedia.org/wiki/Bresenham-Algorithmus
// It uses isOccupied() to decide if a cell is free or blocked.
bool GridModel::hasLineOfSight(const Vec2u& cellIdxA, const Vec2u& cellIdxB) const
{
    Vec2u p;
    int t, dx, dy, incx, incy, pdx, pdy, ddx, ddy, deltaslowdirection, deltafastdirection, err;

    int xstart = cellIdxA.x;
    int xend = cellIdxB.x;
    int ystart = cellIdxA.y;
    int yend = cellIdxB.y;

    /* Entfernung in beiden Dimensionen berechnen */
    dx = xend - xstart;
    dy = yend - ystart;

    /* Vorzeichen des Inkrements bestimmen */
    incx = sgn0(dx);
    incy = sgn0(dy);
    if(dx<0) dx = -dx;
    if(dy<0) dy = -dy;

    /* feststellen, welche Entfernung größer ist */
    if (dx>dy)
    {
        /* x ist schnelle Richtung */
        pdx=incx; pdy=0;    /* pd. ist Parallelschritt */
        ddx=incx; ddy=incy; /* dd. ist Diagonalschritt */
        deltaslowdirection =dy;   deltafastdirection =dx;   /* Delta in langsamer Richtung, Delta in schneller Richtung */
    }
    else
    {
        /* y ist schnelle Richtung */
        pdx=0;    pdy=incy; /* pd. ist Parallelschritt */
        ddx=incx; ddy=incy; /* dd. ist Diagonalschritt */
        deltaslowdirection =dx;   deltafastdirection =dy;   /* Delta in langsamer Richtung, Delta in schneller Richtung */
    }

    /* Initialisierungen vor Schleifenbeginn */
    p.x = xstart;
    p.y = ystart;
    err = deltafastdirection/2;

    if (isOccupied(p))
        return false;

    /* Pixel berechnen */
    for(t=0; t<deltafastdirection; ++t) /* t zaehlt die Pixel, deltafastdirection ist Anzahl der Schritte */
    {
        /* Aktualisierung Fehlerterm */
        err -= deltaslowdirection;
        if(err<0)
        {
            /* Fehlerterm wieder positiv (>=0) machen */
            err += deltafastdirection;
            /* Schritt in langsame Richtung, Diagonalschritt */
            p.x += ddx;
            p.y += ddy;
        }
        else
        {
            /* Schritt in schnelle Richtung, Parallelschritt */
            p.x += pdx;
            p.y += pdy;
        }

        if (isOccupied(p))
            return false;
    }

    return true;
}

// Applies a dilate operation by radius to the occupancy grid.
// This is great to expand the obstacles by the roboter size,
// especially because it easily deals with non-convexity and
// overlaps after the dilate.
void GridModel::dilate(double radius)
{
    Vec2 stride = getStride();
    radius = qMax(stride.x, radius);
    cv::Mat mask = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2*radius/stride.x, 2*radius/stride.y));
    cv::dilate(M, M, mask);
}

// Applies a blur operation by radius to the occupancy grid.
// This is useful to smoothen the map for DWA.
void GridModel::blur(double radius)
{
    Vec2 stride = getStride();
    cv::blur(M, M, cv::Size(2*radius/stride.x, 2*radius/stride.y));
}

// Applies a Canny edge filter to the occupancy grid.
// This has not been useful so far.
void GridModel::canny()
{
    cv::Canny(M, M, 0, 1);
}

// Converts the grid to a polygonal representation and writes them into state.polygons.
// The polygons represent a segmentation of the grid.
// The returned polygons are in transformed state, i.e. their transformation is 0.
// The polygons are non-convex and disjunct.
// The internal algorithm segments the grid by means of contour detection.
// The edge of the segments is then simplified with the Douglas Peucker algorithm.
void GridModel::extractPolygons()
{
    // Segmentation by contour detection.
    cv::Mat M2 = M.clone(); // findContours changes the matrix
    std::vector<std::vector<cv::Point>> segmentsAsContour;
    cv::findContours(M2, segmentsAsContour, /*hierachy,*/ cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Douglas Peucker
    std::vector<std::vector<cv::Point>> segmentsAsPolygonDP;
    for (int i = 0; i < segmentsAsContour.size(); i++)
    {
        if (segmentsAsContour[i].size() >= config.minimumSegmentSize)
        {
            std::vector<cv::Point> segmentPoints;
            cv::approxPolyDP(segmentsAsContour[i], segmentPoints, config.douglasPeuckerEpsilon, true);
            segmentsAsPolygonDP.push_back(segmentPoints);
        }
    }

    // Split segments (polygons) that contain loops.
    for (int i = 0; i < segmentsAsPolygonDP.size(); i++)
    {
        for (int j = 0; j < segmentsAsPolygonDP[i].size(); j++)
        {
            for (int k = j+1; k < segmentsAsPolygonDP[i].size(); k++)
            {
                if (segmentsAsPolygonDP[i][j] == segmentsAsPolygonDP[i][k]) // Loop detected from j to k.
                {
                    //qDebug() << "Segment" << i << "contains a loop from" << j << "to" << k;

                    // We split out a new segment for the loop from j to k-1.
                    // The segment is pushed so that it will still be checked.
                    if (k - j > 2)
                    {
                        std::vector<cv::Point> newSegment;
                        for (int n = j; n < k; n++)
                            newSegment.push_back(segmentsAsPolygonDP[i][n]);
                        segmentsAsPolygonDP.push_back(newSegment);
                    }

                    // And we erase the loop from the current segment so that we are left with
                    // corners from 0 to j-1 and from k to end and then we continue checking.
                    segmentsAsPolygonDP[i].erase(segmentsAsPolygonDP[i].begin()+j, segmentsAsPolygonDP[i].begin()+k);
                }
            }
        }
    }

    // Convert the Douglas Peucker segments to Polygon objects.
    // The DP segments come in pixel coordinates and they need to be transformed
    // into world coordinates using the grid layout parameters.
    Vec2 stride = getStride();
    state.polygons.clear();
    for (int i = 0; i < segmentsAsPolygonDP.size(); i++)
    {
        Polygon pol;
        for (int j = 0; j < segmentsAsPolygonDP[i].size(); j++)
            pol << Vec2(segmentsAsPolygonDP[i][j].x, segmentsAsPolygonDP[i][j].y);
        pol.scale(stride.x, stride.y);
        pol.translate(getMin());
        pol.transform();
        pol.reverseOrder();
        state.polygons << pol;
    }

    state.numPolygons = state.polygons.size();
    state.numVertices = 0;
    for (int i = 0; i < state.polygons.size(); i++)
        state.numVertices += state.polygons[i].size();
}

// Evaluates the GridModel at point x using the output value of the cell that contains x.
uchar GridModel::valueAt(const Vec2 &x) const
{
    Vec2u idx = getNodeIndex(x);
    return M.at<uchar>(idx.y, idx.x);
}

// Evaluates the GridModel at the grid cell specified by the index idx.
uchar GridModel::valueAt(const Vec2u& idx) const
{
    return M.at<uchar>(idx.y, idx.x);
}

// Returns the value of cell i,j (row,column).
uchar GridModel::valueAt(uint i, uint j) const
{
    return M.at<uchar>(i, j);
}

// Sets the grid cell that contains x to value v.
void GridModel::setAt(const Vec2 &x, uchar v)
{
    Vec2u idx = getNodeIndex(x);
    M.at<uchar>(idx.y, idx.x) = v;
}

// Sets the grid cell specified by the index idx to value v.
void GridModel::setAt(const Vec2u &idx, uchar v)
{
    M.at<uchar>(idx.y, idx.x) = v;
}

// Sets the grid cell at index i,j (row,column) to value v.
void GridModel::setAt(uint i, uint j, uchar v)
{
    M.at<uchar>(i, j) = v;
}

const uchar *GridModel::data() const
{
    return (uchar*)M.data;
}

const uchar *GridModel::row(const int &r) const
{
    return (uchar*)M.ptr<uchar>(r);
}

// Draws the occupancy grid on a QPainter.
void GridModel::draw(QPainter *painter) const
{
    //drawGridNodes(painter);
    //drawOutputHeatMap(painter, 0, 1, 0.5);

    double min = 0;
    double max = 255;
    double opacity = 0.5;

    Vec2u n = getN();
    Vec2 stride = getStride();

    painter->save();
    painter->setOpacity(opacity);
    painter->setPen(Qt::NoPen);
    for (uint j = 0; j < n.y; j++)
    {
        for (uint i = 0; i < n.x; i++)
        {
            Vec2u idx(i,j);
            Vec2 c = getNodeCoordinates(idx);
            QRectF q(c.x, c.y, stride.x, stride.y);
            q.translate(-0.5*stride.x, -0.5*stride.y);
            painter->setBrush(colorUtil.getHeatMapColor(valueAt(idx), min, max));
            painter->drawRect(q);
        }
    }

    painter->restore();
}

// OpenGL drawing code.
void GridModel::draw() const
{
    Vec2u n = getN();
    Vec2 stride = getStride();
    Vec2 min = getMin();
    Vec2 max = getMax();

    glPushMatrix();
    glTranslated(-0.5*stride.x, 0.5*stride.y,0);

    // Only the outer borders.
    glLineWidth(2);
    glColor3f(0.65, 0.65, 0.65);
    glBegin( GL_LINES );
    glVertex3f(min.x, min.y, 0.001);
    glVertex3f(max.x, min.y, 0.001);
    glVertex3f(min.x, min.y, 0.001);
    glVertex3f(min.x, max.y, 0.001);
    glVertex3f(max.x, max.y, 0.001);
    glVertex3f(min.x, max.y, 0.001);
    glVertex3f(max.x, max.y, 0.001);
    glVertex3f(max.x, min.y, 0.001);
    glEnd();

    // The grid cells.
    if (true)
    {
        glBegin( GL_QUADS );
        for (uint j = 0; j < n.y; j++)
        {
            for (uint i = 0; i < n.x; i++)
            {
                double x = raster[0][i];
                double y = raster[1][j];
                double w = stride.x;
                double h = stride.y;

                uchar v = valueAt(Vec2u(i,j));
                QColor c = colorUtil.getHeightMapColor(v, 0, config.levelCount);

                if (v > 0)
                {
                    glColor3f(c.redF(), c.greenF(), c.blueF());
                    glVertex3f(x, y, 0);
                    glVertex3f(x+w, y, 0);
                    glVertex3f(x+w, y-h, 0);
                    glVertex3f(x, y-h, 0);
                }
            }
        }
        glEnd();
    }

    // The segment borders.
    if (true)
    {
        std::vector<std::vector<cv::Point>> segmentsAsContour;
        cv::Mat M2 = M.clone(); // findContours changes the matrix
        cv::findContours(M2, segmentsAsContour, /*hierachy,*/ cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

        glPushMatrix();
        glTranslated(0,0,0.0005);
        glBegin( GL_QUADS );
        for (int i = 0; i < segmentsAsContour.size(); i++)
        {
            for (int j = 0; j < segmentsAsContour[i].size(); j++)
            {
                double x = raster[0][segmentsAsContour[i][j].x];
                double y = raster[1][segmentsAsContour[i][j].y];
                double w = stride.x;
                double h = stride.y;

                uchar v = valueAt(Vec2u(segmentsAsContour[i][j].x,segmentsAsContour[i][j].y));
                QColor c = colorUtil.getHeightMapColor(v-20, 0, 255);

                glColor3f(c.redF(), c.greenF(), c.blueF());
                glVertex3f(x, y, 0);
                glVertex3f(x+w, y, 0);
                glVertex3f(x+w, y-h, 0);
                glVertex3f(x, y-h, 0);

            }
        }
        glEnd();
        glPopMatrix();
    }


    // All lines
    if(false)
    {
        glLineWidth(1);
        glColor3f(0.65, 0.65, 0.65);
        glBegin( GL_LINES );
        for (uint j = 0; j < n.y; j++)
        {
            double y = raster[1][j];
            glVertex3f(min.x, y, 0.001);
            glVertex3f(max.x, y, 0.001);
        }
        for (uint i = 0; i < n.x; i++)
        {
            double x = raster[0][i];
            glVertex3f(x, min.y, 0.001);
            glVertex3f(x, max.y, 0.001);
        }
        glEnd();
    }

    glPopMatrix();
}

QDebug operator<<(QDebug dbg, const GridModel &w)
{

}

void GridModel::streamOut(QDataStream& out) const
{

}

void GridModel::streamIn(QDataStream& in)
{

}

QDataStream& operator<<(QDataStream& out, const GridModel &o)
{
    o.streamOut(out);
    return out;
}

QDataStream& operator>>(QDataStream& in, GridModel &o)
{
    o.streamIn(in);
    return in;
}
