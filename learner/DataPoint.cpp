#include "DataPoint.h"
#include "globals.h"
#include "pml/poc.h"
#include "framework/Config.h"
#include "util/Logger.h"
#include "util/ColorUtil.h"
#include "util/GLlib.h"
#include "util/Vec2f.h"

DataPoint::DataPoint()
{
    id = 0;
    d = 0;
    d2 = 0;
    outOfBounds = false;
}

DataPoint::DataPoint(NVec<3> z)
{
    id = 0;
    d = 0;
    d2 = 0;
    outOfBounds = false;

    x = convertZtoX(z);
    y = convertZtoY(z);
    this->z = z;
}

DataPoint::DataPoint(double a, double b, double T)
{
    id = 0;
    d = 0;
    d2 = 0;
    outOfBounds = false;

    z = NVec<3>(a,b,T);
    x = convertZtoX(z);
    y = convertZtoY(z);
}

// Computes a prediction for the state vector xx.
// The prediction computes an estimation of the controls for xx (written into z2)
// and an error bound expressed in seconds (written into d).
void DataPoint::predict(NVec<3> xx)
{
    // Control estimate.
    NVec<3> dz = Jinv*(xx-x);
    z2 = z+dz;
    z2.z = fabs(z.z+sgn(z.z)*dz.z);
    y2 = convertZtoY(z2);

    // Prediction error estimate.
    d = errorRate*dz.absed();

    // Check for out of bounds prediction.
    outOfBounds = false;
    if (z2.x < 0 || z2.x > 1 || z2.y < 0 || z2.y > 1 || z2.z < 0)
    {
//        qDebug() << "DataPoint::predict(): Out of bounds prediction detected for kernel ("
//                 << id << "z:" << z << ") and x:" << xx
//                 << "which evaluates to z2:" << z2 << "and d:" << d;
        outOfBounds = true;
    }
}

// Returns true if the kernel data have been computed.
bool DataPoint::isKnown()
{
    return (fabs(Jinv(0,1)) > 0 || fabs(Jinv(1,0)) > 0);
}

// Returns true if the poc state is withing the boundaries of the configured set of initial states.
bool DataPoint::isInBounds()
{
    return (fabs(x.x) < config.initPhi
            && fabs(x.y) < config.initVphi);
            //&& fabs(x.z) < config.initVx);
}

// Computes the Jacobian and the error rate for the kernel.
void DataPoint::computeKernelData()
{
    Jinv = computeJacobian().inverse();
    //H = computeReducedHessian();
    errorRate = computeErrorRateT();
}

// Computes the Jacobian matrix of the a,b,T representaion at point x,y.
NMat<3,3> DataPoint::computeJacobian()
{
    double a = z.x;
    double b = z.y;
    double T = z.z;

    double dtap = qMin(config.baseSize, 1-a);
    double dtam = qMin(config.baseSize, a);
    double dtbp = qMin(config.baseSize, 1-b);
    double dtbm = qMin(config.baseSize, b);
    double dttp = config.baseSize;
    double dttm = qMin(config.baseSize, fabs(T));

    // Compute six points around the center.
    NVec<3> x1 = convertZtoX(a+dtap, b, T); // expensive
    NVec<3> x2 = convertZtoX(a-dtam, b, T); // expensive
    NVec<3> x3 = convertZtoX(a, b+dtbp, T); // expensive
    NVec<3> x4 = convertZtoX(a, b-dtbm, T); // expensive
    NVec<3> x5 = convertZtoX(a, b, T+sgn(T)*dttp); // expensive
    NVec<3> x6 = convertZtoX(a, b, T-sgn(T)*dttm); // expensive

    NVec<3> dqda = (x1-x2)/(dtap+dtam);
    NVec<3> dqdb = (x3-x4)/(dtbp+dtbm);
    NVec<3> dqdT = (x5-x6)/(dttp+dttm);

    NMat<3,3> J;
    J(0,0) = dqda.x; J(0,1) = dqdb.x; J(0,2) = dqdT.x;
    J(1,0) = dqda.y; J(1,1) = dqdb.y; J(1,2) = dqdT.y;
    J(2,0) = dqda.z; J(2,1) = dqdb.z; J(2,2) = dqdT.z;

    //qDebug() << "Jacobian:";
    //qDebug() << J;

    return J;
}

// Computes a reduced Hessian matrix consisting only of the unmixed second derivatives.
NMat<3,3> DataPoint::computeReducedHessian()
{
    double a = z.x;
    double b = z.y;
    double T = z.z;

    double dtap = qMin(config.baseSize, 1-a);
    double dtam = qMin(config.baseSize, a);
    double dtbp = qMin(config.baseSize, 1-b);
    double dtbm = qMin(config.baseSize, b);
    double dttp = config.baseSize;
    double dttm = qMin(config.baseSize, fabs(T));

    // Compute six points around the center.
    NVec<3> x1 = convertZtoX(a+dtap, b, T); // expensive
    NVec<3> x2 = convertZtoX(a-dtam, b, T); // expensive
    NVec<3> x3 = convertZtoX(a, b+dtbp, T); // expensive
    NVec<3> x4 = convertZtoX(a, b-dtbm, T); // expensive
    NVec<3> x5 = convertZtoX(a, b, T+sgn(T)*dttp); // expensive
    NVec<3> x6 = convertZtoX(a, b, T-sgn(T)*dttm); // expensive

    NVec<3> dqda = (x1+x2-2*x)/((dtap+dtam)*(dtap+dtam));
    NVec<3> dqdb = (x3+x4-2*x)/((dtbp+dtbm)*(dtap+dtam));
    NVec<3> dqdT = (x5+x6-2*x)/((dttp+dttm)*(dtap+dtam));

    NMat<3,3> H;
    H(0,0) = dqda.x; H(0,1) = dqdb.x; H(0,2) = dqdT.x;
    H(1,0) = dqda.y; H(1,1) = dqdb.y; H(1,2) = dqdT.y;
    H(2,0) = dqda.z; H(2,1) = dqdb.z; H(2,2) = dqdT.z;
    H.transpose();

/*
    qDebug() << "Hessian:";
    qDebug() << H;

    // Test direction a:
    qDebug() << "Testing a ------------";
    NMat<3,3> J = computeJacobian();
    for (int i = 0; i < 10; i++)
    {
        NVec<3> dap(a+i*dtap, b, T);
        NVec<3> dam(a-i*dtap, b, T);
        NVec<3> x1 = convertZtoX(dap); // expensive
        NVec<3> x2 = convertZtoX(dam); // expensive
        NVec<3> x3 = x+J*NVec

        qDebug() << i << "+ gt:" << x1;
    }
*/
    return H;
}

// Computes the error rates for the control parameters a,b,T.
// It assumes the Jacobian has already been computed!
NVec<3> DataPoint::computeErrorRate()
{
    double a = z.x;
    double b = z.y;
    double T = z.z;

    qDebug() << "a -------------";

//    for (double dt = 0; dt <= 0.05; dt += 0.001)
//    {
//        NVec<3> x1 = convertZtoX(a+dt, b, T)-x; // expensive
//        NVec<3> x2 = convertZtoX(a-dt, b, T)-x; // expensive
//        double dt1 = (Jinv*x1).z;
//        double dt2 = (Jinv*x2).z;
//        qDebug() << dt << dt1 << dt2 << qMax(fabs(dt1), fabs(dt2));
//    }

    // Determine the error rate in the "a" direction with the Illinois method.
    double abound = (a > 0.5) ?  -a : (1.0-a);
    double ak = 0;
    double bk = 1;
    double fak = -config.errorBound;
    NVec<3> deltaZp = NVec<3>(qBound(0.0, a+bk*abound, 1.0), b, T);
    NVec<3> x1 = convertZtoX(deltaZp)-x; // expensive
    NVec<3> deltaZm = NVec<3>(qBound(0.0, a-bk*abound, 1.0), b, T);
    NVec<3> x2 = convertZtoX(deltaZm)-x; // expensive
    double fbk = qMax((deltaZp-(Jinv*x1)).norm(), (deltaZm-(Jinv*x2)).norm())-config.errorBound;
    double ck = bk;
    double fck = fbk;
    int counter = 0;
    qDebug() << "expanding a" << counter << "ak:" << ak << fak << "bk:" << bk << fbk << "ck:" << ck << fck << "at da:" << ck*abound;
    if (fck > 0)
    {
        while (fabs(fck) > EPSILON && counter < 12)
        {
            ck = ak - fak*(bk-ak)/(fbk-fak);
            deltaZp = NVec<3>(qBound(0.0, a+ck*abound, 1.0), b, T);
            x1 = convertZtoX(deltaZp)-x; // expensive
            deltaZm = NVec<3>(qBound(0.0, a-ck*abound, 1.0), b, T);
            x2 = convertZtoX(deltaZm)-x; // expensive
            fck = qMax((deltaZp-(Jinv*x1)).norm(), (deltaZm-(Jinv*x2)).norm())-config.errorBound;

            if (fck*fbk < 0)
            {
                ak = bk;
                fak = fbk;
                bk = ck;
                fbk = fck;
            }
            else
            {
                fak = 0.5*fak;
                bk = ck;
                fbk = fck;
            }

            counter++;
            qDebug() << "expanding a" << counter << "ak:" << ak << fak << "bk:" << bk << fbk << "ck:" << ck << fck << "at da:" << ck*abound;
        }

        if (fabs(fck) > 1.0E-3)
            qDebug() << "a error rate determination did not finish well with residual error" << fabs(fck) << "for (a,b,T):" << z;
    }

    errorRate.x = fabs((fck+config.errorBound)/(ck*abound));



    qDebug() << "b -------------";

//    for (double dt = 0; dt <= 0.1; dt += 0.001)
//    {
//        NVec<3> x1 = convertZtoX(a, b+dt, T)-x; // expensive
//        NVec<3> x2 = convertZtoX(a, b-dt, T)-x; // expensive
//        double dt1 = (Jinv*x1).z;
//        double dt2 = (Jinv*x2).z;
//        qDebug() << dt << dt1 << dt2 << qMax(fabs(dt1), fabs(dt2));
//    }

    // Determine the error rate in the "b" direction with the Illinois method.
    double bbound = (b > 0.5) ?  -b : (1.0-b);
    ak = 0;
    bk = 1;
    fak = -config.errorBound;
    deltaZp = NVec<3>(a, qBound(0.0, b+bk*bbound, 1.0), T);
    x1 = convertZtoX(deltaZp)-x; // expensive
    deltaZm = NVec<3>(a, qBound(0.0, b-bk*bbound, 1.0), T);
    x2 = convertZtoX(deltaZm)-x; // expensive
    fbk = qMax((deltaZp-(Jinv*x1)).norm(), (deltaZm-(Jinv*x2)).norm())-config.errorBound;

    ck = bk;
    fck = fbk;
    counter = 0;
    qDebug() << "expanding b" << counter << "ak:" << ak << fak << "bk:" << bk << fbk << "ck:" << ck << fck << "at db:" << ck*bbound;
    if (fck > 0)
    {
        while (fabs(fck) > EPSILON && counter < 12)
        {
            ck = ak - fak*(bk-ak)/(fbk-fak);
            deltaZp = NVec<3>(a, qBound(0.0, b+ck*bbound, 1.0), T);
            x1 = convertZtoX(deltaZp)-x; // expensive
            deltaZm = NVec<3>(a, qBound(0.0, b-ck*bbound, 1.0), T);
            x2 = convertZtoX(deltaZm)-x; // expensive
            fck = qMax((deltaZp-(Jinv*x1)).norm(), (deltaZm-(Jinv*x2)).norm())-config.errorBound;

            if (fck*fbk < 0)
            {
                ak = bk;
                fak = fbk;
                bk = ck;
                fbk = fck;
            }
            else
            {
                fak = 0.5*fak;
                bk = ck;
                fbk = fck;
            }

            counter++;
            qDebug() << "expanding b" << counter << "ak:" << ak << fak << "bk:" << bk << fbk << "ck:" << ck << fck << "at db:" << ck*bbound;
        }

        if (fabs(fck) > 1.0E-3)
            qDebug() << "b error rate determination did not finish well with residual error" << fabs(fck) << "for (a,b,T):" << z;

    }

    errorRate.y = fabs((fck+config.errorBound)/(ck*bbound));



    qDebug() << "T -------------";

//    for (double dt = 0; dt <= 0.100; dt += 0.01)
//    {
//        x1 = convertZtoX(a, b, T+sgn(T)*dt)-x; // expensive
//        x2 = convertZtoX(a, b, T-sgn(T)*qMin(fabs(T),dt))-x; // expensive
//        double dt1 = (Jinv*x1).z;
//        double dt2 = (Jinv*x2).z;
//        qDebug() << dt << dt1 << dt2 << qMax(fabs(dt1-dt), fabs(dt2+dt));
//    }

    // Determine the error rate in the "T" direction with the Illinois method.
    double tbound = sgn(T)*(0.04+0.2*fabs(T));
    ak = 0;
    bk = 1;
    fak = -config.errorBound;
    deltaZp = NVec<3>(a, b, T+bk*tbound);
    x1 = convertZtoX(deltaZp)-x; // expensive
    deltaZm = NVec<3>(a, b, T-qMin(bk,T/tbound)*tbound);
    x2 = convertZtoX(deltaZm)-x; // expensive
    fbk = qMax((deltaZp-(Jinv*x1)).norm(), (deltaZm-(Jinv*x2)).norm())-config.errorBound;
    ck = bk;
    fck = fbk;
    counter = 0;
    qDebug() << "expanding T" << counter << "ak:" << ak << fak << "bk:" << bk << fbk << "ck:" << ck << fck << "at dt:" << ck*tbound;
    if (fck > 0)
    {
        while (fabs(fck) > EPSILON && counter < 12)
        {
            ck = ak - fak*(bk-ak)/(fbk-fak);
            deltaZp = NVec<3>(a, b, T+ck*tbound);
            x1 = convertZtoX(deltaZp)-x; // expensive
            deltaZm = NVec<3>(a, b, T-qMin(ck,T/tbound)*tbound);
            x2 = convertZtoX(deltaZm)-x; // expensive
            fck = qMax((deltaZp-(Jinv*x1)).norm(), (deltaZm-(Jinv*x2)).norm())-config.errorBound;

            if (fck*fbk < 0)
            {
                ak = bk;
                fak = fbk;
                bk = ck;
                fbk = fck;
            }
            else
            {
                fak = 0.5*fak;
                bk = ck;
                fbk = fck;
            }

            counter++;
            qDebug() << "expanding T" << counter << "ak:" << ak << fak << "bk:" << bk << fbk << "ck:" << ck << fck << "at dt:" << ck*tbound;
        }

        if (fabs(fck) > 1.0E-3)
        {
            qDebug() << "DataPoint::computeErrorRate(): T error rate determination did not finish well with residual error" << fabs(fck) << "for id:" << id << "(a,b,T):" << z;
        }
        else
        {
            Logger logger("data/T.log",true);
            logger << T << (ck*tbound);
            logger++;
        }

    }
    else
    {
        qDebug() << "DataPoint::computeErrorRate(): T boundary reached before error was maximized for (a,b,T):" << z;
    }

    errorRate.z = fabs((fck+config.errorBound)/(ck*tbound));

    qDebug() << "errorRate:" << errorRate;
    return errorRate;
}


// Computes the error rates for the control parameters a,b,T.
// It assumes the Jacobian has already been computed!
NVec<3> DataPoint::computeErrorRateT()
{
    // Convert y to Tab.
    double a = z.x;
    double b = z.y;
    double T = z.z;

//    qDebug() << "a -------------";

//    for (double dt = 0; dt <= 0.05; dt += 0.001)
//    {
//        NVec<3> x1 = convertZtoX(a+dt, b, T)-x; // expensive
//        NVec<3> x2 = convertZtoX(a-dt, b, T)-x; // expensive
//        double dt1 = (Jinv*x1).z;
//        double dt2 = (Jinv*x2).z;
//        qDebug() << dt << dt1 << dt2 << qMax(fabs(dt1), fabs(dt2));
//    }

    // Determine the error rate in the "a" direction with the Illinois method.
    double abound = (a > 0.5) ?  -a : (1.0-a);
    double ak = 0;
    double bk = 1;
    double fak = -config.errorBound;
    NVec<3> x1 = convertZtoX(qBound(0.0, a+bk*abound, 1.0), b, T)-x; // expensive
    NVec<3> x2 = convertZtoX(qBound(0.0, a-bk*abound, 1.0), b, T)-x; // expensive
    double fbk = qMax(fabs((Jinv*x1).z), fabs((Jinv*x2).z))-config.errorBound;
    double ck = bk;
    double fck = fbk;
    int counter = 0;
//    qDebug() << "expanding a" << counter << "ak:" << ak << fak << "bk:" << bk << fbk << "ck:" << ck << fck << "at da:" << ck*abound;
    if (fck > 0)
    {
        while (fabs(fck) > EPSILON && counter < 12)
        {
            ck = ak - fak*(bk-ak)/(fbk-fak);
            x1 = convertZtoX(qBound(0.0, a+ck*abound, 1.0), b, T)-x; // expensive
            x2 = convertZtoX(qBound(0.0, a-ck*abound, 1.0), b, T)-x; // expensive
            fck = qMax(fabs((Jinv*x1).z), fabs((Jinv*x2).z))-config.errorBound;

            if (fck*fbk < 0)
            {
                ak = bk;
                fak = fbk;
                bk = ck;
                fbk = fck;
            }
            else
            {
                fak = 0.5*fak;
                bk = ck;
                fbk = fck;
            }

            counter++;
//            qDebug() << "expanding a" << counter << "ak:" << ak << fak << "bk:" << bk << fbk << "ck:" << ck << fck << "at da:" << ck*abound;
        }

        if (fabs(fck) > 1.0E-3)
            qDebug() << "a error rate determination did not finish well with residual error" << fabs(fck) << "for (a,b,T):" << z;
    }

    errorRate.x = fabs((fck+config.errorBound)/(ck*abound));



//    qDebug() << "b -------------";

//    for (double dt = 0; dt <= 0.1; dt += 0.001)
//    {
//        NVec<3> x1 = convertZtoX(a, b+dt, T)-x; // expensive
//        NVec<3> x2 = convertZtoX(a, b-dt, T)-x; // expensive
//        double dt1 = (Jinv*x1).z;
//        double dt2 = (Jinv*x2).z;
//        qDebug() << dt << dt1 << dt2 << qMax(fabs(dt1), fabs(dt2));
//    }

    // Determine the error rate in the "b" direction with the Illinois method.
    double bbound = (b > 0.5) ?  -b : (1.0-b);
    ak = 0;
    bk = 1;
    fak = -config.errorBound;
    x1 = convertZtoX(a, qBound(0.0, b+bk*bbound, 1.0), T)-x; // expensive
    x2 = convertZtoX(a, qBound(0.0, b-bk*bbound, 1.0), T)-x; // expensive
    fbk = qMax(fabs((Jinv*x1).z), fabs((Jinv*x2).z))-config.errorBound;
    ck = bk;
    fck = fbk;
    counter = 0;
//    qDebug() << "expanding b" << counter << "ak:" << ak << fak << "bk:" << bk << fbk << "ck:" << ck << fck << "at db:" << ck*bbound;
    if (fck > 0)
    {
        while (fabs(fck) > EPSILON && counter < 12)
        {
            ck = ak - fak*(bk-ak)/(fbk-fak);
            x1 = convertZtoX(a, qBound(0.0, b+ck*bbound, 1.0), T)-x; // expensive
            x2 = convertZtoX(a, qBound(0.0, b-ck*bbound, 1.0), T)-x; // expensive
            fck = qMax(fabs((Jinv*x1).z), fabs((Jinv*x2).z))-config.errorBound;

            if (fck*fbk < 0)
            {
                ak = bk;
                fak = fbk;
                bk = ck;
                fbk = fck;
            }
            else
            {
                fak = 0.5*fak;
                bk = ck;
                fbk = fck;
            }

            counter++;
//            qDebug() << "expanding b" << counter << "ak:" << ak << fak << "bk:" << bk << fbk << "ck:" << ck << fck << "at db:" << ck*bbound;
        }

        if (fabs(fck) > 1.0E-3)
            qDebug() << "b error rate determination did not finish well with residual error" << fabs(fck) << "for (a,b,T):" << z;

    }

    errorRate.y = fabs((fck+config.errorBound)/(ck*bbound));



//    qDebug() << "T -------------";

//    for (double dt = 0; dt <= 0.100; dt += 0.01)
//    {
//        x1 = convertZtoX(a, b, T+sgn(T)*dt)-x; // expensive
//        x2 = convertZtoX(a, b, T-sgn(T)*qMin(fabs(T),dt))-x; // expensive
//        double dt1 = (Jinv*x1).z;
//        double dt2 = (Jinv*x2).z;
//        qDebug() << dt << dt1 << dt2 << qMax(fabs(dt1-dt), fabs(dt2+dt));
//    }

    // Determine the error rate in the "T" direction with the Illinois method.
    double tbound = sgn(T)*(0.04+0.2*fabs(T));
    ak = 0;
    bk = 1;
    fak = -config.errorBound;
    x1 = convertZtoX(a, b, T+bk*tbound)-x; // expensive
    x2 = convertZtoX(a, b, T-qMin(bk,T/tbound)*tbound)-x; // expensive
    double dt1 = (Jinv*x1).z;
    double dt2 = (Jinv*x2).z;
    fbk = qMax(fabs(dt1-fabs(bk*tbound)), fabs(dt2+fabs(qMin(bk,T/tbound)*tbound)))-config.errorBound;
    ck = bk;
    fck = fbk;
    counter = 0;
//    qDebug() << "expanding T" << counter << "ak:" << ak << fak << "bk:" << bk << fbk << "ck:" << ck << fck << "at dt:" << ck*tbound;
    if (fck > 0)
    {
        while (fabs(fck) > EPSILON && counter < 12)
        {
            ck = ak - fak*(bk-ak)/(fbk-fak);
            x1 = convertZtoX(a, b, T+ck*tbound)-x; // expensive
            x2 = convertZtoX(a, b, T-qMin(ck,T/tbound)*tbound)-x; // expensive
            dt1 = (Jinv*x1).z;
            dt2 = (Jinv*x2).z;
            fck = qMax(fabs(dt1-fabs(ck*tbound)), fabs(dt2+fabs(qMin(ck,T/tbound)*tbound)))-config.errorBound;

            if (fck*fbk < 0)
            {
                ak = bk;
                fak = fbk;
                bk = ck;
                fbk = fck;
            }
            else
            {
                fak = 0.5*fak;
                bk = ck;
                fbk = fck;
            }

            counter++;
//            qDebug() << "expanding T" << counter << "ak:" << ak << fak << "bk:" << bk << fbk << "ck:" << ck << fck << "at dt:" << ck*tbound;
        }

        if (fabs(fck) > 1.0E-3)
        {
            qDebug() << "DataPoint::computeErrorRate(): T error rate determination did not finish well with residual error" << fabs(fck) << "for id:" << id << "(a,b,T):" << z;
        }
        else
        {
            Logger logger("data/T.log",true);
            logger << T << (ck*tbound);
            logger++;
        }

    }
    else
    {
        qDebug() << "DataPoint::computeErrorRate(): T boundary reached before error was maximized for (a,b,T):" << z;
    }

    errorRate.z = fabs((fck+config.errorBound)/(ck*tbound));

//    qDebug() << "error:" << errorRate;
    return errorRate;
}

// Returns a list of neighbors in a,b,T space at the boundary points of this kernel,
// where boundary points are combinations of the extreme values of the a,b,T axes
// where the error reaches the critical value.
// This is a very expensive routine that involves up to 14 simulation runs.
QList<DataPoint> DataPoint::enumerateNeighborhood()
{
    double a = z.x;
    double b = z.y;
    double T = z.z;

    double da = config.errorBound/errorRate.x;
    double db = config.errorBound/errorRate.y;
    double dT = sgn(T)*config.errorBound/errorRate.z;

    QList<DataPoint> list;

    if (T*(T-dT) > 0 )
    {
        list << DataPoint(a, b, T-dT);

        if (b+db < 1)
            list << DataPoint(a, b+db, T-dT);

        if (b-db > 0)
            list << DataPoint(a, b-db, T-dT);

        if (a+da < 1)
        {
            list << DataPoint(a+da, b, T-dT);
            if (b+db < 1)
                list << DataPoint(a+da, b+db, T-dT);
            if (b-db > 0)
                list << DataPoint(a+da, b-db, T-dT);
        }

        if (a-da > 0)
        {
                list << DataPoint(a-da, b, T-dT);
            if (b+db < 1)
                list << DataPoint(a-da, b+db, T-dT);
            if (b-db > 0)
                list << DataPoint(a-da, b-db, T-dT);
        }

    }


    if (b+db < 1)
        list << DataPoint(a, b+db, T);

    if (b-db > 0)
        list << DataPoint(a, b-db, T);

    if (a+da < 1)
    {
        list << DataPoint(a+da, b, T);
        if (b+db < 1)
            list << DataPoint(a+da, b+db, T);
        if (b-db > 0)
            list << DataPoint(a+da, b-db, T);
    }

    if (a-da > 0)
    {
        list << DataPoint(a-da, b, T);
        if (b+db < 1)
            list << DataPoint(a-da, b+db, T);
        if (b-db > 0)
            list << DataPoint(a-da, b-db, T);
    }

    list << DataPoint(a, b, T+dT);

    if (b+db < 1)
        list << DataPoint(a, b+db, T+dT);

    if (b-db > 0)
        list << DataPoint(a, b-db, T+dT);

    if (a+da < 1)
    {
        list << DataPoint(a+da, b, T+dT);
        if (b+db < 1)
            list << DataPoint(a+da, b+db, T+dT);
        if (b-db > 0)
            list << DataPoint(a+da, b-db, T+dT);
    }

    if (a-da > 0)
    {
        list << DataPoint(a-da, b, T+dT);
        if (b+db < 1)
            list << DataPoint(a-da, b+db, T+dT);
        if (b-db > 0)
            list << DataPoint(a-da, b-db, T+dT);
    }

    return list;
}

// Computes the acceleration A to be used to implement the first increment of y.
double DataPoint::computeAfromY(const NVec<3> y)
{
    double ax = 0;
    double t = config.systemIterationTime;
    double v = 0;

    if (fabs(y.z) >= t)
    {
        ax = sgn(y.z)*config.A;
    }
    else
    {
        v += y.z*config.A;
        t -= fabs(y.z);

        if (fabs(y.y) >= t)
        {
            v += sgn(y.y)*t*config.A;
            ax = v/config.systemIterationTime;
        }
        else
        {
            v += y.y*config.A;
            t -= fabs(y.y);

            if (fabs(y.x) >= t)
            {
                v += sgn(y.x)*t*config.A;
                ax = v/config.systemIterationTime;
            }
            else
            {
                v += y.x*config.A;
                t -= fabs(y.x);
                ax = v/config.systemIterationTime;
            }
        }
    }

    return ax;
}

// Uses physical simulation to convert a control vector y to phase space x.
NVec<3> DataPoint::convertYtoX(const NVec<3> y)
{
    Poc poc;
    poc.reset();
    poc.simpredict3bangs(y.x, y.y, y.z);
    NVec<3> x(poc.phi, -poc.vphi, -poc.vx);
    return x;
}

// Converts a Tab control vector to bang times t1,t2,t3.
NVec<3> DataPoint::convertZtoY(const NVec<3> z)
{
    return convertZtoY(z.x, z.y, z.z);
}

// Converts a Tab control vector to bang times t1,t2,t3.
NVec<3> DataPoint::convertZtoY(double a, double b, double T)
{
    NVec<3> y;
    y.x = sgn(T)*(1.0-a)*b*fabs(T);
    y.y = -sgn(T)*a*fabs(T);
    y.z = sgn(T)*(1.0-a)*(1.0-b)*fabs(T);
    return y;
}

// Converts a Tab control vector to spherical coordinates.
NVec<3> DataPoint::convertZtoC(const NVec<3> z)
{
    return convertZtoC(z.x, z.y, z.z);
}

// Converts a Tab control vector to spherical coordinates.
NVec<3> DataPoint::convertZtoC(double a, double b, double T)
{
    NVec<3> c;
    c.x = fabs(T)*sin(-PI+a*PII)*cos(-PI+b*PII);
    c.y = fabs(T)*sin(-PI+a*PII)*sin(-PI+b*PII);
    c.z = fabs(T)*cos(-PI+a*PII);
    return c;
}

// Converts bang times t1,t2,t3 to a Tab control vector.
NVec<3> DataPoint::convertYtoZ(const NVec<3> y)
{
    double a = 1.0;
    double b = 0;
    double T = y.norm1();

    if (fabs(fabs(y.y)-T) > EPSILON)
    {
        a = fabs(y.y)/T;
        b = fabs(y.x)/((1.0-a)*T);
    }

    NVec<3> z;
    z.x = a;
    z.y = b;
    z.z = -sgn(y.y)*T;

    return z;
}

// Converts a Tab control vector z to a phase space coordinate x.
NVec<3> DataPoint::convertZtoX(double a, double b, double T)
{
    NVec<3> y = convertZtoY(a, b, T);
    NVec<3> x = convertYtoX(y);
    return x;
}

// Converts a Tab control vector z to a phase space coordinate x.
NVec<3> DataPoint::convertZtoX(const NVec<3> z)
{
    NVec<3> y = convertZtoY(z);
    NVec<3> x = convertYtoX(y);
    return x;
}

// OpenGL drawing code.
void DataPoint::draw(bool showData, bool showKernels)
{
    QColor color;
    double colorMin = 0;
    double colorMax = config.colorContrast;

    glPushMatrix();
    color = colorUtil.mapColor(z.z, colorMin, colorMax);
    glColor3f(color.redF(), color.greenF(), color.blueF());

    if (showData)
    {
        glPointSize(3);
        glBegin(GL_POINTS);
        glVertex3dv(x);
        glEnd();
    }

    if (showKernels && isKnown())
    {
        int slices = 32;

        double alpha = config.errorBound/errorRate.x;
        double beta = config.errorBound/errorRate.y;
        double gamma = config.errorBound/errorRate.z;

        glTranslated(x[0], x[1], x[2]); // translate
        glMultMatrixd(Jinv.inverse());
        glPushMatrix();
        glScaled(alpha, beta, gamma);
        GLlib::drawFrame(1.0);
        glPopMatrix();

        QList<Vec2f> list;
        for (int ii = 0; ii <= slices; ii++)
        {
            Vec2f v(0,1);
            v.rotate(ii*PII/slices);
            v.x *= alpha;
            v.y *= beta;

            if (z.x+v.x > 1)
                v *= (1.0-z.x)/v.x;
            if (z.x+v.x < 0)
                v *= (0.0-z.x)/v.x;
            if (z.y+v.y > 1)
                v *= (1.0-z.y)/v.y;
            if (z.y+v.y < 0)
                v *= (0.0-z.y)/v.y;

            list << v;
        }

        glBegin(GL_TRIANGLE_FAN);
        glColor4f(color.red(), color.green(), color.blue(), 0.8);
        glVertex3f(0, 0, 0);
        for (int i = 0; i < list.size(); i++)
            glVertex3f(list[i].x, list[i].y, 0);
        glEnd();

        glBegin(GL_LINE_STRIP);
        glColor3f(0,0,0);
        for (int i = 0; i < list.size(); i++)
            glVertex3f(list[i].x, list[i].y, 0);
        glEnd();
    }

    glPopMatrix();
}

QTextStream& operator<<(QTextStream& out, const DataPoint &p)
{
    out << "[id:" << p.id
        << " z:" << p.z
        << " x:" << p.x << "]";
    return out;
}

QDataStream& operator<<(QDataStream& out, const DataPoint &p)
{
    out << p.id;
    out << p.x;
    out << p.z;
    out << p.Jinv;
    out << p.H;
    out << p.errorRate;
    return out;
}

QDataStream& operator>>(QDataStream& in, DataPoint &p)
{
    in >> p.id;
    in >> p.x;
    in >> p.z;
    in >> p.Jinv;
    in >> p.H;
    in >> p.errorRate;
    return in;
}

QDebug operator<<(QDebug dbg, const DataPoint &p)
{
    //bool sp = dbg.autoInsertSpaces();
    dbg.nospace();
    dbg << "[id:" << p.id
        //<< " x:" << p.x
        << " y:" << p.convertZtoY(p.z)
        << " y2:" << p.y2
        << " z:" << p.z
        << " z2:" << p.z2
        << " d:" << p.d
        << " " << p.outOfBounds << "]";
    dbg.setAutoInsertSpaces(true);
    return dbg;
}
