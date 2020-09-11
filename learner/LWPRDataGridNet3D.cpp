#include "LWPRDataGridNet3D.h"
#include "learner/LWPR3D.h"
#include "framework/Config.h"
#include "util/Statistics.h"
#include <QGLViewer/vec.h>
using namespace qglviewer;

LWPRDataGridNet3D::LWPRDataGridNet3D()
{
    lwpr = 0;
}

// Computes the GPR using all data in the grid.
void LWPRDataGridNet3D::train()
{
    lwpr = new LWPR_Object(DIM, 1);
    doubleVec norm_in(DIM);
    for (int d = 0; d < DIM; d++)
        norm_in[d] = (max[d]-min[d]);
    lwpr->normIn(norm_in);
    lwpr->setInitD(config.lwprKernelSize);
    lwpr->updateD(config.lwprKernelSizeAdaptation);
    lwpr->setInitAlpha(config.lwprInitAlpha);
    //lwpr->penalty(gamma);

    // Prepare the data in a list.
    QList< NVec<3> > X;
    QList<double> Y;
    for (int i = 0; i < this->nodeCount; i++)
    {
        NVec<4> data = bin[i];
        NVec<3> v;
        v.x = data[0];
        v.y = data[1];
        v.z = data[2];
        double y = data[3];
        X << v;
        Y << y;
    }

    // Present the data in random order in multiple training sweeps.
    for (int i = 0; i < config.lwprTrainingIterations; i++)
    {
        // Scramble the order of the training data.
        NVec<3> v;
        double y;
        for (int i = 0; i < X.size(); i++)
        {
            int j = Statistics::uniformSample(0, X.size());

            v = X[i];
            X[i] = X[j];
            X[j] = v;

            y = Y[i];
            Y[i] = Y[j];
            Y[j] = y;
        }

        // Present the data.
        double mse = 0;
        for (int i = 0; i < X.size(); i++)
        {
            doubleVec _x(DIM);
            for (int d = 0; d < DIM; d++)
                _x[d] = X[i][d];
            doubleVec _y(1);
            _y[0] = y;
            doubleVec y_ = lwpr->update(_x, _y);
            mse += (Y[i]-y_[0])*(Y[i]-y_[0]);

            if (i % 100 == 0)
            {
                int rfs = lwpr->numRFS(0);
                qDebug() << i << "RF:" << rfs << "alpha" << config.lwprInitAlpha << "mse:" << mse/(i+1);
            }
        }
    }


    // For every node in the grid...
    for (int n = 0; n < this->nodeCount; n++)
    {
        // Query the LWPR at the node location.
        NVec<DIM> x = Grid<DIM>::getNodeCoordinates(n);
        doubleVec _x(DIM);
        doubleVec c(1);
        for (int d = 0; d < DIM; d++)
            _x[d] = x[d];
        doubleVec o = lwpr->predict(_x, c);

        // Set the output value.
        setNodeOutput(n, o[0], 1.1-c[0]);

        if (n % 100 == 0)
            qDebug() << n << "of" << this->nodeCount << "mean:" << o[0] << "conf:" << c[0];
    }
}


// Draw the receptive fields.
void LWPRDataGridNet3D::drawReceptiveFields()
{
    if (lwpr == 0)
        return;

    glPushMatrix();
    doubleVec n = lwpr->normIn();
    //glScaled(n[0], n[1], n[2]);
    int rfs = lwpr->numRFS(0);
    for (int i = 0; i < rfs; i++)
    {
        LWPR_ReceptiveFieldObject rf = lwpr->getRF(0, i);
        doubleVec center = rf.center();
        doubleVec slope = rf.slope();
        std::vector<doubleVec> D = rf.D();

        // Scale the center from the unit space to the input space.
        center[0] = center[0]*n[0];
        center[1] = center[1]*n[1];
        center[2] = center[2]*n[2];

        Vec3f x = Vec3f(n[0], 0.0, slope[0]).normalized()/sqrt(D[0][0]);
        Vec3f y = Vec3f(0.0, n[1], slope[1]).normalized()/sqrt(D[1][1]);
        Vec3f z = Vec3f(0.0, -slope[2], n[2]).normalized()/sqrt(D[2][2]);

        glPointSize(5);
        glBegin(GL_POINTS);
        glColor3f(0.6, 0.0, 0.0);
        glVertex3f(center[0], center[1], center[2]);
        glEnd();

        glLineWidth(2);
        glBegin(GL_LINES);
        glColor3f(1.0, 0.0, 0.0);
        glVertex3f(center[0], center[1], center[2]);
        glVertex3f(center[0]+x.x, center[1]+x.y, center[2]+x.z);
        glColor3f(0.0, 1.0, 0.0);
        glVertex3f(center[0], center[1], center[2]);
        glVertex3f(center[0]+y.x, center[1]+y.y, center[2]+y.z);
        glColor3f(0.0, 0.0, 1.0);
        glVertex3f(center[0], center[1], center[2]);
        glVertex3f(center[0]+z.x, center[1]+z.y, center[2]+z.z);
        glEnd();
    }
    glPopMatrix();
}


