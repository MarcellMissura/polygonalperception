#ifndef MATN
#define MATN
#include "VecN.h"
#include <math.h>
#include <QDebug>
#include <QDataStream>

// The MatN is a generic matrix template where the number of dimensions N (rows), M (cols)
// is specified as a template parameter. For example, MatN<3,3> m; creates a 3x3 matrix.
// Values are stored in column major order.

template <int N=3,int M=3>
class MatN
{
    double v[N*M];

public:

    MatN()
    {
        memset(v, 0, N*M*sizeof(double));
        for (int i=0; i<qMin(N,M); i++)
            v[i+i*N] = 1.0;
    }

    MatN(const MatN<N,M> &o)
	{
		if (this != &o)
            memcpy(v, &o, N*M*sizeof(double));
	}

    MatN(double xo)
    {
        memset(v, xo, N*M*sizeof(double));
    }

    MatN(const double* xo)
    {
        memcpy(v, xo, N*M*sizeof(double));
    }

    MatN& operator=(const MatN<N,M> &o)
	{
		if (this != &o)
            memcpy(v, &o, N*M*sizeof(double));
		return *this;
	}

    MatN& operator=(double xo)
    {
        memset(v, xo, N*M*sizeof(double));
        return *this;
    }

    MatN& operator+=(const MatN<N,M> &o)
	{
        const double* vo = o.data();
        for (int k = 0; k < N*M; k++)
            v[k] += vo[k];
		return *this;
	}

    MatN& operator-=(const MatN<N,M> &o)
	{
        const double* vo = o.data();
        for (int k = 0; k < N*M; k++)
            v[k] -= vo[k];
		return *this;
	}

    MatN& operator+=(double o)
	{
        for (int k = 0; k < N*M; k++)
            v[k] += o;
		return *this;
	}

    MatN& operator-=(double o)
	{
        for (int k = 0; k < N*M; k++)
            v[k] -= o;
		return *this;
	}

    MatN& operator*=(double s)
	{
        for (int k = 0; k < N*M; k++)
            v[k] *= s;
		return *this;
	}

    MatN& operator/=(double s)
	{
        for (int k = 0; k < N*M; k++)
            v[k] /= s;
		return *this;
	}

    MatN operator+(const MatN<N,M> &o) const
	{
        MatN<N,M> c = *this;
		c += o;
		return c;
	}

    MatN operator-(const MatN<N,M> &o) const
	{
        MatN<N,M> c = *this;
		c -= o;
		return c;
	}

    MatN operator+(double o) const
	{
        MatN<N,M> c = *this;
		c += o;
		return c;
	}

    MatN operator-(double o) const
	{
        MatN<N,M> c = *this;
		c -= o;
		return c;
	}

    MatN operator-() const
    {
        MatN<N,M> c = *this;
        c *= -1;
        return c;
    }

    VecN<N> operator*(const VecN<M>& o) const
    {
        VecN<N> w;
        for (int i=0; i<N; i++)
            for (int j=0; j<M; j++)
                w[i] += v[i+j*N]*o[j];
        return w;
    }

    MatN operator*(double s) const
	{
        MatN<N,M> c = *this;
		c *= s;
		return c;
	}

    MatN operator/(double s) const
	{
        MatN<N,M> c = *this;
		c /= s;
		return c;
	}

    bool operator==(const MatN<N,M> &o) const
	{
        for (int i = 0; i < N*M; i++)
			if (v[i] != o[i])
				return false;
		return true;
	}

    bool operator!=(const MatN<N,M> &o) const
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
    VecN<N> operator[](int col) const
    {
        VecN<N> w;
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

    MatN<N,M> inverse()
    {
        return *this;
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
MatN<N> operator*(double s, const MatN<N,M> &o)
{
    return MatN<N>(o) * s;
}

template <int N, int M>
QDebug operator<<(QDebug dbg, const MatN<N,M> &o)
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
QDataStream& operator<<(QDataStream& out, const MatN<N,M> &o)
{
    const double* v = o.data();
    for (int i = 0; i < N*M; i++)
        out << v[i];
    return out;
}

template <int N, int M>
QDataStream& operator>>(QDataStream& in, MatN<N,M> &o)
{
    for (int j=0; j<M; j++)
        for (int i=0; i<N; i++)
            in >> o(i,j);
    return in;
}

#endif //MatN
