#ifndef SHOW_MATH_H_
#define SHOW_MATH_H_

#include <math.h>
#define MAT_SIZE 6

float getQuinticPolynomial(const float x, const float* line, const int order);

float getCubicPolynomial(const float x, const float* line, const int order);

int combination(const int n, const int m);

void bezierPoint(const float tau,
                 const int n,
                 const float points[][2],
                 float* x,
                 float* y);

void bezierDerivative(const float tau,
                      const int n,
                      const float points[][2],
                      float* dx,
                      float* dy);

void bezierSecDerivative(const float tau,
                         const int n,
                         const float points[][2],
                         float* ddx,
                         float* ddy);

void gaussianElimination(float a[MAT_SIZE][MAT_SIZE + 1]);

void quinticPolyFit(const float T,
                    const float s0,
                    const float v0,
                    const float a0,
                    const float s1,
                    const float v1,
                    const float a1,
                    float coeffi[MAT_SIZE]);

#endif  // SHOW_MATH_H_
