#ifndef DATAPOINT_H_
#define DATAPOINT_H_
#include "util/VecN.h"
#include "util/NMat.h"

struct DataPoint
{
    uint id;
    NVec<3> x; // State
    NVec<3> z; // Ctrl in abT
    NVec<3> y; // Ctrl in t1,t2,t3
    NMat<3,3> Jinv;
    NMat<3,3> H;
    NVec<3> errorRate;

    double d; // Distance
    NVec<3> z2; // Extrapolated abT control.
    NVec<3> y2; // Extrapolated t1,t2,t3 control.
    bool outOfBounds;
    double d2;

    DataPoint();
    DataPoint(NVec<3> z);
    DataPoint(double a, double b, double T);

    void computeKernelData();
    NMat<3,3> computeJacobian();
    NMat<3,3> computeReducedHessian();
    NVec<3> computeErrorRate();
    NVec<3> computeErrorRateT();
    bool isKnown();
    bool isInBounds();

    void predict(NVec<3> x);

    QList<DataPoint> enumerateNeighborhood();

    static double computeAfromY(const NVec<3> y);

    static NVec<3> convertZtoC(double a, double b, double T);
    static NVec<3> convertZtoC(const NVec<3> z);
    static NVec<3> convertZtoY(const NVec<3> z);
    static NVec<3> convertZtoY(double a, double b, double T);
    static NVec<3> convertZtoX(const NVec<3> z);
    static NVec<3> convertZtoX(double a, double b, double T);
    static NVec<3> convertYtoZ(const NVec<3> y);
    static NVec<3> convertYtoX(const NVec<3> y);

    void draw(bool showData, bool showKernels);

    DataPoint operator-() const
    {
        DataPoint dp = *this;
        dp.x = -x;
        dp.z.z = -z.z;
        dp.Jinv = -Jinv;
        return dp;
    }

    bool operator<(const DataPoint& dp) const
    {
        return z.z < dp.z.z;
    }
    bool operator<=(const DataPoint& dp) const
    {
        return z.z <= dp.z.z;
    }
    bool operator>(const DataPoint& dp) const
    {
        return z.z > dp.z.z;
    }
    bool operator>=(const DataPoint& dp) const
    {
        return z.z >= dp.z.z;
    }
};

QTextStream& operator<<(QTextStream& out, const DataPoint &p);
QDataStream& operator<<(QDataStream& out, const DataPoint &p);
QDataStream& operator>>(QDataStream& in, DataPoint &p);
QDebug operator<<(QDebug dbg, const DataPoint &p);

#endif /* DATAPOINT_H_ */
