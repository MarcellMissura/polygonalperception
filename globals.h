#ifndef GLOBS_H_
#define GLOBS_H_
#include <math.h>

const int NAO_PORT = 9559;

const double G = 9.81;
const double PI = 3.1415926535897932384626433832795;
const double PII = 2.0*PI;
const double PI2 = 1.5707963267948965579989817342721;
const double PI4 = 0.78539816339744830961566084581988;
const double SPI = 1.7724538509055160272981674833411; // sqrt of pi
const double RAD_TO_DEG =  180.0 / PI;
const double DEG_TO_RAD =  PI / 180.0;
const double EPSILON = 1.0E-6;
const int IMAGE_WIDTH = 640;
const int IMAGE_HEIGHT = 480;
const int CAMERA_SAMPLE_READ_WAIT_TIMEOUT = 2000; // in ms
const int CAMERA_FPS = 30;
const double CAMERA_OPENING_X = 60;
const double CAMERA_OPENING_Y = 46;

const int NUMBER_OF_POINTS = IMAGE_HEIGHT*IMAGE_WIDTH;


// Min max bound trio.
template <typename T>
inline const T &min(const T &a, const T &b) { return (a < b) ? a : b; }
template <typename T>
inline const T &max(const T &a, const T &b) { return (a < b) ? b : a; }
template <typename T>
inline const T &bound(const T &l, const T &val, const T &u){ return max(l, min(u, val)); }

// Returns the sign of the argument. +1 if the argument is positive,
// -1 if the argument is negative, and 0 if the argument is zero.
template <typename T>
inline int sgn0(const T a) { return (fabs(a) < EPSILON ? 0 : a < 0 ? -1 : 1); }

// Returns the sign of the argument. +1 if the argument is positive,
// -1 if the argument is negative.
template <typename T>
inline int sgn(const T a) { return (a < 0 ? -1 : 1); }

// Maps the argument to an angle expressed in radians between -PI and PI.
inline double picut(double x) { return  x < 0 ? fmod(x-PI, PII)+PI : fmod(x+PI, PII)-PI;}

// A possibly faster version of picut, but the runtime grows with the input.
inline double fpicut(double x)
{
    while (x > PI) x -= PII;
    while (x < -PI) x += PII;
    return x;
}

// An even faster picut version that assumes the angle to be given in [-PII, PII].
inline double ffpicut(double x) { return  x > PI ? x-PII : x < -PI ? x+PII : x;}


// Maps the argument to an angle expressed in radians between 0 and 2*PI.
inline double pi2cut(double x) { return fmod(fabs(x), PII);}

// A better version of modulo (%) that wraps rather than retuning a number < 0.
inline int mod(int x, int y) { return x >= 0 ? x%y : (x%y)+y;}

// This is a 20x faster sine implementation with just a tiny error.
inline double fsin(double x)
{
    // This algorithm is taken from http://mooooo.ooo/chebyshev-sine-approximation/
    // "Approximating sin(x) to 5 ULP with Chebyshev polynomials" by Collin Wallace

    x = fpicut(x);

    static double coeffs[6] = {
        -0.10132118,
         0.0066208798,        // x^3
        -0.00017350505,       // x^5
         0.0000025222919,     // x^7
        -0.000000023317787,   // x^9
         0.00000000013291342, // x^11
    };

    double pi_major = 3.1415927;
    double pi_minor = -0.00000008742278;
    double x2 = x*x;
    double p11 = coeffs[5];
    double p9  = p11*x2 + coeffs[4];
    double p7  = p9*x2  + coeffs[3];
    double p5  = p7*x2  + coeffs[2];
    double p3  = p5*x2  + coeffs[1];
    double p1  = p3*x2  + coeffs[0];
    return (x - pi_major - pi_minor) * (x + pi_major + pi_minor) * p1 * x;
}

// This is a 20x faster cosine implementation with just a tiny error.
inline double fcos(double x)
{
    return fsin(x+PI2);
}

// A twice as fast version of atan2 but with a small error.
inline double fatan2(double y, double x)
{
    // This formula is taken from http://www-labs.iro.umontreal.ca/~mignotte/IFT2425/Documents/EfficientApproximationArctgFunction.pdf
    // Sreeraman Rajan et al., Efficient Approximations for the Arctangent Function

    // Special cases.
    if (x == 0 && y == 0)
        return 0;
    if (y == 0)
    {
        if (x > 0)
            return 0;
        return PI;
    }
    if (x == 0)
        return sgn(y)*PI2;

    // Octants 1 and 8.
    if (x > 0 && fabs(y) <= x)
    {
        double x_ = y/x;
        return x_*(1.0584 - sgn(y)*0.273*x_);
    }

    // Octants 2 and 3.
    if (y > 0 && y >= fabs(x))
    {
        double y_ = x/y;
        return PI2-y_*(1.0584-sgn(x)*0.273*y_);
    }

    // Octants 4 and 5.
    if (x < 0 && fabs(y) <= -x)
    {
        double x_ = y/x;
        return sgn(y)*PI+x_*(1.0584 + sgn(y)*0.273*x_);
    }

    // Octants 6 and 7.
    if (y < 0 && fabs(x) <= -y)
    {
        double y_ = x/y;
        return -PI2-y_*(1.0584+sgn(x)*0.273*y_);
    }

    return 0;
}

#endif /* GLOBS_H_ */
