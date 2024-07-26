#include "visualization/show_math.h"

// float fit_coeffi[6];

int combination(int n, int m) {
  long long numerator = 1;
  int denominator = m;
  for (int i = 1; i <= m; i++) {
    numerator *= (n - i + 1);
    while (denominator != 0 && numerator % denominator == 0) {
      numerator /= denominator;
      denominator--;
    }
  }
  return numerator;
}

void bezierPoint(float tau, int n, float points[][2], float* x, float* y) {
  *x = 0, *y = 0;
  if (n < 1)
    return;
  float t = tau / (float)n;
  for (int i = 0; i <= n; i++) {
    *x += combination(n, i) * pow(1 - t, n - i) * pow(t, i) * points[i][0];
    *y += combination(n, i) * pow(1 - t, n - i) * pow(t, i) * points[i][1];
  }
}
void bezierDerivative(float tau,
                      int n,
                      float points[][2],
                      float* dx,
                      float* dy) {
  *dx = 0, *dy = 0;
  if (n < 2)
    return;
  float t = tau / (float)n;
  for (int i = 0; i < n; i++) {
    *dx += n * combination(n - 1, i) * pow(1 - t, n - 1 - i) * pow(t, i) *
           (points[i + 1][0] - points[i][0]);
    *dy += n * combination(n - 1, i) * pow(1 - t, n - 1 - i) * pow(t, i) *
           (points[i + 1][1] - points[i][1]);
  }
  // printf("t = %.2f, dx = %.2f, dy = %.2f\n", t, *dx, *dy);
}
void bezierSecDerivative(float tau,
                         int n,
                         float points[][2],
                         float* ddx,
                         float* ddy) {
  *ddx = 0, *ddy = 0;
  if (n < 3)
    return;
  float t = tau / (float)n;
  for (int i = 0; i < n - 1; i++) {
    float dx = n * (points[i + 2][0] - 2 * points[i + 1][0] + points[i][0]);

    float dy = n * (points[i + 2][1] - 2 * points[i + 1][1] + points[i][1]);
    *ddx += n * (n - 1) * combination(n - 2, i) * pow(1 - t, n - 2 - i) *
            pow(t, i) * dx;
    *ddy += n * (n - 1) * combination(n - 2, i) * pow(1 - t, n - 2 - i) *
            pow(t, i) * dy;
  }
  // printf("t = %.2f, ddx = %.6f, ddy = %.6f\n", t, *ddx, *ddy);
}

void gaussianElimination(float a[MAT_SIZE][MAT_SIZE + 1]) {
  int n = MAT_SIZE;
  for (int i = 0; i < n; i++) {
    // find largest absolute value of a[i][i] as main element
    int maxElem = i;
    for (int k = i + 1; k < n; k++) {
      if (fabs(a[k][i]) > fabs(a[maxElem][i]))
        maxElem = k;
    }
    // swap row of main element to first row
    if (maxElem != i) {
      for (int j = i; j <= n; j++) {
        float tmp = a[i][j];
        a[i][j] = a[maxElem][j];
        a[maxElem][j] = tmp;
      }
    }
    // simplify elements of [i+1,n) to 0. Upper triangular matrix converting
    for (int k = i + 1; k < n; k++) {
      float factor = a[k][i] / a[i][i];
      for (int j = i; j <= n; j++) {
        a[k][j] -= factor * a[i][j];
      }
    }
  }
  // regression process
  for (int i = n - 1; i >= 0; i--) {
    a[i][n] /= a[i][i];
    for (int k = i - 1; k >= 0; k--) {
      a[k][n] -= a[k][i] * a[i][n];
    }
  }
}

void quinticPolyFit(float T,
                    float s0,
                    float v0,
                    float a0,
                    float s1,
                    float v1,
                    float a1,
                    float coeffi[MAT_SIZE]) {
// fifth degree polynominal fitting, set augmented matrix
// s = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
#if MAT_SIZE == 6
  float mat_A[MAT_SIZE][MAT_SIZE + 1] = {
      {1, 0, 0, 0, 0, 0, s0},
      {0, 1, 0, 0, 0, 0, v0},
      {0, 0, 2, 0, 0, 0, a0},
      {1, T, T * T, T * T * T, T * T * T * T, T * T * T * T * T, s1},
      {0, 1, 2 * T, 3 * T * T, 4 * T * T * T, 5 * T * T * T * T, v1},
      {0, 0, 2, 6 * T, 12 * T * T, 20 * T * T * T, a1}};
#elif MAT_SIZE == 4
  float mat_A[MAT_SIZE][MAT_SIZE + 1] = {
      {1, 0, 0, 0, s0},
      {0, 1, 0, 0, v0},
      {1, T, T * T, T * T * T, s1},
      {0, 1, 2 * T, 3 * T * T, v1},
  };
#endif
  gaussianElimination(mat_A);
  for (int i = 0; i < MAT_SIZE; i++) {
    coeffi[i] = mat_A[i][MAT_SIZE];
  }
  /*   printf("ST coeffi: ");
    for (int i = 0; i < MAT_SIZE; i++) {
      printf("a[%d] = %.2f\t", i, mat_A[i][MAT_SIZE]);
    }
    printf("\n"); */
  return;
}