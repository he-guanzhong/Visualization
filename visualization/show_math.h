#ifndef SHOW_MATH_H_
#define SHOW_MATH_H_

#include <math.h>
#define MAT_SIZE 6

int combination(int n, int k);
void bezierPoint(float tau, int n, float points[][2], float* x, float* y);
void bezierDerivative(float tau,
                      int n,
                      float points[][2],
                      float* dx,
                      float* dy);
void bezierSecDerivative(float tau,
                         int n,
                         float points[][2],
                         float* ddx,
                         float* ddy);

void gaussianElimination(float a[MAT_SIZE][MAT_SIZE + 1]);
void quinticPolyFit(float T,
                    float s0,
                    float v0,
                    float a0,
                    float s1,
                    float v1,
                    float a1,
                    float coeffi[MAT_SIZE]);

#endif  // SHOW_MATH_H_