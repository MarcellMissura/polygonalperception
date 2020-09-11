#ifndef NMAT
#define NMAT
#include "NVec.h"
#include <math.h>
#include <QDebug>
#include <QDataStream>
#include <armadillo>

// The NMat is a generic matrix template where the number of dimensions N (rows), M (cols)
// is specified as a template parameter. For example, NMat<3,3> m; creates a 3x3 matrix.
// Values are stored in column major order.

template <int N=3,int M=3>
class NMat
{
    double v[N*M];

public:

    NMat()
	{
        memset(v, 0, N*M*sizeof(double));
        for (int i=0; i<qMin(N,M); i++)
            v[i+i*N] = 1.0;
	}

    NMat(const NMat<N,M> &o)
	{
		if (this != &o)
            memcpy(v, &o, N*M*sizeof(double));
	}

    NMat(double xo)
    {
        memset(v, xo, N*M*sizeof(double));
    }

    NMat(const double* xo)
    {
        memcpy(v, xo, N*M*sizeof(double));
    }

    NMat& operator=(const NMat<N,M> &o)
	{
		if (this != &o)
            memcpy(v, &o, N*M*sizeof(double));
		return *this;
	}

    NMat& operator=(double xo)
    {
        memset(v, xo, N*M*sizeof(double));
        return *this;
    }

    NMat& operator+=(const NMat<N,M> &o)
	{
        const double* vo = o.data();
        for (int k = 0; k < N*M; k++)
            v[k] += vo[k];
		return *this;
	}

    NMat& operator-=(const NMat<N,M> &o)
	{
        const double* vo = o.data();
        for (int k = 0; k < N*M; k++)
            v[k] -= vo[k];
		return *this;
	}

    NMat& operator+=(double o)
	{
        for (int k = 0; k < N*M; k++)
            v[k] += o;
		return *this;
	}

    NMat& operator-=(double o)
	{
        for (int k = 0; k < N*M; k++)
            v[k] -= o;
		return *this;
	}

    NMat& operator*=(double s)
	{
        for (int k = 0; k < N*M; k++)
            v[k] *= s;
		return *this;
	}

    NMat& operator/=(double s)
	{
        for (int k = 0; k < N*M; k++)
            v[k] /= s;
		return *this;
	}

    NMat operator+(const NMat<N,M> &o) const
	{
        NMat<N,M> c = *this;
		c += o;
		return c;
	}

    NMat operator-(const NMat<N,M> &o) const
	{
        NMat<N,M> c = *this;
		c -= o;
		return c;
	}

    NMat operator+(double o) const
	{
        NMat<N,M> c = *this;
		c += o;
		return c;
	}

    NMat operator-(double o) const
	{
        NMat<N,M> c = *this;
		c -= o;
		return c;
	}

    NMat operator-() const
    {
        NMat<N,M> c = *this;
        c *= -1;
        return c;
    }

    NVec<N> operator*(const NVec<M>& o) const
    {
        NVec<N> w;
        for (int i=0; i<N; i++)
            for (int j=0; j<M; j++)
                w[i] += v[i+j*N]*o[j];
        return w;
    }

    NMat operator*(double s) const
	{
        NMat<N,M> c = *this;
		c *= s;
		return c;
	}

    NMat operator/(double s) const
	{
        NMat<N,M> c = *this;
		c /= s;
		return c;
	}

    bool operator==(const NMat<N,M> &o) const
	{
        for (int i = 0; i < N*M; i++)
			if (v[i] != o[i])
				return false;
		return true;
	}

    bool operator!=(const NMat<N,M> &o) const
	{
		return !(*this == o);
	}

    double& operator() (int row, int col)
    {
        return v[row+col*N];
    }

    double operator() (int row, int col) const
    {
        return v[row+col*N];
    }

    // Returns the column vector at index col.
    NVec<N> operator[](int col) const
    {
        NVec<N> w;
        for (int i=0; i<N; i++)
            w[i] = v[i+col*N];
        return w;
    }

    void eye()
    {
        memset(v, 0, N*M*sizeof(double));
        for (int i=0; i<qMin(N,M); i++)
            v[i+i*N] = 1.0;
    }

    void zero()
    {
        memset(v, 0, N*M*sizeof(double));
    }

    int rows() const
	{
		return N;
	}

    int cols() const
    {
        return M;
    }

    // Euklidean norm.
    double norm() const
	{
		double sum = 0;
        for (int i = 0; i < N*M; i++)
			sum += v[i]*v[i];
		return sqrt(sum);
	}

    // Squared Euklidean norm (a tiny bit faster).
    double norm2() const
	{
		double sum = 0;
        for (int i = 0; i < N*M; i++)
			sum += v[i]*v[i];
		return sum;
	}

    // Manhattan (L1) norm.
    double norm1() const
    {
        double sum = 0;
        for (int i = 0; i < N*M; i++)
            sum += fabs(v[i]);
        return sum;
    }

    // Fills the matrix with random values between 0 and 1.0.
    void randomize()
	{
        for (int i = 0; i < N*M; i++)
			v[i] = (double)qrand()/RAND_MAX;
	}

    NMat<N,M> inverse()
    {
        arma::Mat<double> T(v, N, M, false);
        arma::Mat<double> Tinv = arma::inv(T);
        NMat<N,M> T2(Tinv.memptr());
        return T2;
    }

    void transpose()
    {
        double temp;
        for (int i=1; i<N; i++)
        {
            for (int j=0; j<i; j++)
            {
                temp = v[i+j*N];
                v[i+j*N] = v[j+i*N];
                v[j+i*N] = temp;
            }
        }
    }

    const double* data() const
    {
        return v;
    }

    // OpenGL support.
    operator const double*() const
    {
        static double m[16] = { 1,0,0,0,
                                0,1,0,0,
                                0,0,1,0,
                                0,0,0,1 };

        m[0]=v[0+0*N]; m[1]=v[1+0*N]; m[2]=v[2+0*N];
        m[4]=v[0+1*N]; m[5]=v[1+1*N]; m[6]=v[2+1*N];
        m[8]=v[0+2*N]; m[9]=v[1+2*N]; m[10]=v[2+2*N];

        return m;
    }
};

template <int N, int M>
NMat<N> operator*(double s, const NMat<N,M> &o)
{
    return NMat<N>(o) * s;
}

template <int N, int M>
QDebug operator<<(QDebug dbg, const NMat<N,M> &o)
{ 
    dbg.nospace() << "[" << o(0,0);
    for (int j=1; j<M; j++)
        dbg << ", " << o(0,j);
    dbg << "]\n";

    for (int i=1; i<N; i++)
    {
        dbg << "[" << o(i,0);
        for (int j=1; j<M; j++)
            dbg << ", " << o(i,j);
        dbg << "]\n";
    }

    return dbg.space();
}

template <int N, int M>
QDataStream& operator<<(QDataStream& out, const NMat<N,M> &o)
{
    const double* v = o.data();
    for (int i = 0; i < N*M; i++)
        out << v[i];
    return out;
}

template <int N, int M>
QDataStream& operator>>(QDataStream& in, NMat<N,M> &o)
{
    for (int j=0; j<M; j++)
        for (int i=0; i<N; i++)
            in >> o(i,j);
    return in;
}

#endif //NMAT
