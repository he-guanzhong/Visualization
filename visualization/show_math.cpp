#include "visualization/show_math.h"

float getQuinticPolynomial(const float x, const float* line, const int order) {
  switch (order) {
    case 0:
      return line[0] + line[1] * x + line[2] * x * x + line[3] * x * x * x +
             line[4] * x * x * x * x + line[5] * x * x * x * x * x;
    case 1:
      return line[1] + 2.0f * line[2] * x + 3.0f * line[3] * x * x +
             4.0f * line[4] * x * x * x + 5.0f * line[5] * x * x * x * x;
    case 2:
      return 2.0f * line[2] + 6.0f * line[3] * x + 12.0f * line[4] * x * x +
             20.0f * line[5] * x * x * x;
    default:
      return 0;
  }
}

float getCubicPolynomial(const float x, const float* line, const int order) {
  switch (order) {
    case 0:
      return line[0] + line[1] * x + line[2] * x * x + line[3] * x * x * x;
    case 1:
      return line[1] + 2.0f * line[2] * x + 3.0f * line[3] * x * x;
    case 2:
      return 2.0f * line[2] + 6.0f * line[3] * x;
    case 3:
      return 6.0f * line[3];
    default:
      return 0;
  }
}

int combination(const int n, const int m) {
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

void bezierPoint(const float tau,
                 const int n,
                 const float points[][2],
                 float* x,
                 float* y) {
  *x = 0, *y = 0;
  if (n < 1 || n > 100)
    return;
  const float t = tau / (float)n;
  for (int i = 0; i <= n; i++) {
    *x += combination(n, i) * pow(1 - t, n - i) * pow(t, i) * points[i][0];
    *y += combination(n, i) * pow(1 - t, n - i) * pow(t, i) * points[i][1];
  }
}

void bezierDerivative(const float tau,
                      const int n,
                      const float points[][2],
                      float* dx,
                      float* dy) {
  *dx = 0, *dy = 0;
  if (n < 2 || n > 100)
    return;
  const float t = tau / (float)n;
  for (int i = 0; i < n; i++) {
    *dx += n * combination(n - 1, i) * pow(1 - t, n - 1 - i) * pow(t, i) *
           (points[i + 1][0] - points[i][0]);
    *dy += n * combination(n - 1, i) * pow(1 - t, n - 1 - i) * pow(t, i) *
           (points[i + 1][1] - points[i][1]);
  }
  // printf("t = %.2f, dx = %.2f, dy = %.2f\n", t, *dx, *dy);
}

void bezierSecDerivative(const float tau,
                         const int n,
                         const float points[][2],
                         float* ddx,
                         float* ddy) {
  *ddx = 0, *ddy = 0;
  if (n < 3 || n > 100)
    return;
  const float t = tau / (float)n;
  for (int i = 0; i < n - 1; i++) {
    const float dx =
        n * (points[i + 2][0] - 2 * points[i + 1][0] + points[i][0]);
    const float dy =
        n * (points[i + 2][1] - 2 * points[i + 1][1] + points[i][1]);
    *ddx += n * (n - 1) * combination(n - 2, i) * pow(1 - t, n - 2 - i) *
            pow(t, i) * dx;
    *ddy += n * (n - 1) * combination(n - 2, i) * pow(1 - t, n - 2 - i) *
            pow(t, i) * dy;
  }
  // printf("t = %.2f, ddx = %.6f, ddy = %.6f\n", t, *ddx, *ddy);
}

void gaussianElimination(float a[MAT_SIZE][MAT_SIZE + 1]) {
  const int n = MAT_SIZE;
  for (int i = 0; i < n; i++) {
    // find largest absolute value of a[i][i] as main element
    int maxElem = i;
    for (int k = i + 1; k < n; k++) {
      if (fabsf(a[k][i]) > fabsf(a[maxElem][i]))
        maxElem = k;
    }
    // swap row of main element to first row
    if (maxElem != i) {
      for (int j = i; j <= n; j++) {
        const float tmp = a[i][j];
        a[i][j] = a[maxElem][j];
        a[maxElem][j] = tmp;
      }
    }
    // simplify elements of [i+1,n) to 0. Upper triangular matrix converting
    for (int k = i + 1; k < n; k++) {
      const float factor = a[k][i] / a[i][i];
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

void quinticPolyFit(const float T,
                    const float s0,
                    const float v0,
                    const float a0,
                    const float s1,
                    const float v1,
                    const float a1,
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
