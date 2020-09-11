#include "PCA.h"
#include <QColor>
#include "blackboard/Config.h"
#include "util/GLlib.h"
#include "util/ColorUtil.h"
#include <GL/glu.h>


// Boring constructor.
PCA::PCA()
{
    loadedPoints = 0;
}

// Clears all data points and resets the PCA search to a blank state.
void PCA::reset()
{
    loadedPoints = 0;
    data.clear();
    normal.set(0);
    mean.set(0);
}

// Returns the number of points in the data set.
int PCA::getLoadedPointCount() const
{
    return loadedPoints;
}

// Initializes the PCA.
// This method should be called after all data points have been added.
void PCA::init()
{
    if (loadedPoints < 3)
    {
        qDebug() << "PCA init(): not enough points to determine the plane.";
        return;
    }

    // Build X and Y.
    using namespace arma;
    Mat<double> X;
    X.set_size(data.size(), 3);
    mean.set(0);
    for (int i = 0; i < data.size(); i++)
    {
        X(i,0) = data[i].x;
        X(i,1) = data[i].y;
        X(i,2) = data[i].z;
        mean += data[i];
    }
    mean /= data.size();

    arma::mat score;
    arma::vec values;

    princomp(coeff, score, values, X);
    //coeff.print();
    //values.print("Values:");

    //Vec3 normal1 = Vec3(coeff(0,0), coeff(0,1), coeff(0,2));
    //Vec3 normal2 = Vec3(coeff(1,0), coeff(1,1), coeff(1,2));
    //qDebug() << normal1.crossed(normal2).normalized();
    normal = Vec3(coeff(2,0), coeff(2,1), coeff(2,2));
    if (normal.z < 0)
        normal = -normal;
}

// Adds a single point to the data point set.
// Do this first and then call init() to perform the PCA.
// You can add more points after calling init(), but call init() again.
void PCA::addDataPoint(const Vec3& p)
{
    data.push_back(p);
    loadedPoints++;
}

// Evaluates the linear regression at point p.
double PCA::evaluateAt(const Vec2& p) const
{
    if (normal.isNull())
        return 0;
    return (mean*normal-p.x*normal.x-p.y*normal.y)/normal.z;
}

// Returns the normal to the regressed plane.
Vec3 PCA::getNormal() const
{
    return normal;
}

// Returns the mean of the given points.
Vec3 PCA::getMean() const
{
    return mean;
}

// Prints all the loaded points.
void PCA::print() const
{
    qDebug() << data;
}

// OpenGL drawing code that draws the location of the stored data points.
void PCA::draw(uint sampleFactor) const
{
    glPushMatrix();
    glPointSize(3);
    glColor3f(0,0,0);
    glBegin(GL_POINTS);
    sampleFactor = qMax((uint)1, sampleFactor);
    for (int k=0; k < loadedPoints; k=k+sampleFactor)
        glVertex3dv(data.at(k));
    glEnd();
    glPopMatrix();
}

QDebug operator<<(QDebug dbg, const PCA &o)
{
    o.print();
}
