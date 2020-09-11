#include "Transform3D.h"
#include <immintrin.h>

// This is a 3D transformation implementation that integrates well
// into this framework. This Transform3D class exposes functionalities
// to set the transformation from xyzrpy parameters and to decompose
// the transform into these parameters. The parameters are defined
// in the TransformParams struct. Rotations are assumed to occur in
// rpy order. Internally, a plain array holds the matrix in column-major order.

Transform3D::Transform3D()
{

}

Transform3D::Transform3D(const TransformParams& t)
{
    *this = t;
}

// TransformParams to Transform3D assignment operator.
void Transform3D::operator=(const TransformParams& t)
{
    setFromParams(t);
}

// For importing raw buffers. The buffer is copied. The data is expected to be in row major order.
void Transform3D::operator=(const double* v)
{
    at(0,0) = v[0];
    at(0,1) = v[1];
    at(0,2) = v[2];
    at(0,3) = v[3];
    at(1,0) = v[4];
    at(1,1) = v[5];
    at(1,2) = v[6];
    at(1,3) = v[7];
    at(2,0) = v[8];
    at(2,1) = v[9];
    at(2,2) = v[10];
    at(2,3) = v[11];
    at(3,0) = v[12];
    at(3,1) = v[13];
    at(3,2) = v[14];
    at(3,3) = v[15];
}

// Returns a mutable reference to the element of the transformation matrix at position (row, column).
double &Transform3D::at(uint row, uint col)
{
    //return M(row, col);
    return M[row+col*4];
}

// Returns the element of the transformation matrix at position (row, column).
double Transform3D::at(uint row, uint col) const
{
    //return M(row,col);
    return M[row+col*4];
}

// Returns a mutable reference to the element of the transformation matrix at position (row, column).
double &Transform3D::operator()(uint row, uint col)
{
    //return M(row,col);
    return M[row+col*4];
}

// Returns the element of the transformation matrix at position (row, column).
double Transform3D::operator()(uint row, uint col) const
{
    //return M(row,col);
    return M[row+col*4];
}

// Computes a transformation from the xyzrpy parameters given in t.
void Transform3D::setFromParams(const TransformParams& t)
{
    setFromParams(t.x, t.y, t.z, t.roll, t.pitch, t.yaw);
}

// Computes a transformation from the xyzrpy parameters.
void Transform3D::setFromParams(double x, double y, double z, double roll, double pitch, double yaw)
{
    // Following http://planning.cs.uiuc.edu/node102.html#eqn:yprmat

    double cy = cos(yaw); // alpha
    double cp = cos(pitch); // beta
    double cr = cos(roll); // gamma
    double sy = sin(yaw);
    double sp = sin(pitch);
    double sr = sin(roll);

    at(0,0) = cy*cp;
    at(0,1) = cy*sp*sr-sy*cr;
    at(0,2) = cy*sp*cr+sy*sr;
    at(0,3) = x;
    at(1,0) = sy*cp;
    at(1,1) = sy*sp*sr+cy*cr;
    at(1,2) = sy*sp*cr-cy*sr;
    at(1,3) = y;
    at(2,0) = -sp;
    at(2,1) = cp*sr;
    at(2,2) = cp*cr;
    at(2,3) = z;
    at(3,0) = 0;
    at(3,1) = 0;
    at(3,2) = 0;
    at(3,3) = 1;
}

// Computes a transformation given a plane defined by a normal n
// and a point p. The normal must be normalized! The computed
// transform is what is needed to transform the camera so that
// it would see the given plane correctly as ground plane.
// Only roll, pitch, and z (the z intercept) are computed.
// x,y, and yaw remain zero.
void Transform3D::setFromGroundPlane(const Vec3 &n, const Vec3 &p)
{
    Vec3 up(0,0,1);
    Vec3 axis = n^up;
    axis.normalize();
    double angle = n.angleTo(up);
    double z = (n*-p);

    double c = cos(angle);
    double s = sin(angle);
    double t = 1.0 - c;

    at(0,0) = c + axis.x*axis.x*t;
    at(1,1) = c + axis.y*axis.y*t;
    at(2,2) = c + axis.z*axis.z*t;

    double tmp1 = axis.x*axis.y*t;
    double tmp2 = axis.z*s;
    at(1,0) = tmp1 + tmp2;
    at(0,1) = tmp1 - tmp2;
    tmp1 = axis.x*axis.z*t;
    tmp2 = axis.y*s;
    at(2,0) = tmp1 - tmp2;
    at(0,2) = tmp1 + tmp2;
    tmp1 = axis.y*axis.z*t;
    tmp2 = axis.x*s;
    at(2,1) = tmp1 + tmp2;
    at(1,2) = tmp1 - tmp2;
    at(0,3) = 0;
    at(1,3) = 0;
    at(2,3) = z;
    at(3,0) = 0;
    at(3,1) = 0;
    at(3,2) = 0;
    at(3,3) = 1;
}

// Decomposes the transformation into xyzrpy parameters.
TransformParams Transform3D::getParams() const
{
    // Following the formulas at http://planning.cs.uiuc.edu/node103.html

    TransformParams t;
    t.x = at(0,3);
    t.y = at(1,3);
    t.z = at(2,3);

    double sq = sqrt(at(0,0)*at(0,0)+at(1,0)*at(1,0));
    t.yaw = atan2(at(1,0),at(0,0)); // alpha
    t.pitch = atan2(-at(2,0),sq); // beta
    t.roll = atan2(at(2,1),at(2,2)); // gamma

    return t;
}

// Concatenates two transforms.
Transform3D Transform3D::operator*(const Transform3D &o) const
{
    Transform3D TT;

    for (uint i = 0; i < 4; i++)
    {
        for (uint j = 0; j < 4; j++)
        {
            double sum = 0;
            for (uint k = 0; k < 4; k++)
            {
                sum += at(i,k)*o.at(k,j);
            }
            TT.at(i,j) = sum;
        }
    }

    return TT;
}

// Applies the transform to the vector v.
Vec3 Transform3D::operator*(const Vec3 &v) const
{
    Vec3 out;

    out.x = M[0]*v.x + M[4]*v.y + M[8]*v.z + M[12];
    out.y = M[1]*v.x + M[5]*v.y + M[9]*v.z + M[13];
    out.z = M[2]*v.x + M[6]*v.y + M[10]*v.z + M[14];

/*
    __m256d mx1 = _mm256_set1_pd(v.x);
    __m256d mx2 = _mm256_set1_pd(v.y);
    __m256d mx3 = _mm256_set1_pd(v.z);

    __m256d col1 = _mm256_load_pd(&M[0]);
    __m256d col2 = _mm256_load_pd(&M[4]);
    __m256d col3 = _mm256_load_pd(&M[8]);
    __m256d col4 = _mm256_load_pd(&M[12]);

    __m256d p1 = _mm256_mul_pd(mx1, col1);
    __m256d p2 = _mm256_mul_pd(mx2, col2);
    __m256d p3 = _mm256_mul_pd(mx3, col3);

    __m256d s1 = _mm256_add_pd(p1, p2);
    __m256d s2 = _mm256_add_pd(p3, col4);
    __m256d s3 = _mm256_add_pd(s1, s2);

    _mm256_store_pd(out.data(), s3);
*/
    return out;
}

QDebug operator<<(QDebug dbg, const TransformParams &o)
{
    dbg << "r:" << o.roll << "p:" << o.pitch << "y:" << o.yaw << "x:" << o.x << "y:" << o.y << "z:" << o.z;
    return dbg;
}

