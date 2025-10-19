#include "MyBno_filters.h"

// --- Filtro EMA ---
void filterEMA(float ax, float ay, float az, float alpha,
               float &axf, float &ayf, float &azf)
{
#if EMA_FILTER_ENABLED
  axf = alpha * ax + (1.0f - alpha) * axf;
  ayf = alpha * ay + (1.0f - alpha) * ayf;
  azf = alpha * az + (1.0f - alpha) * azf;
#else
  axf = ax;
  ayf = ay;
  azf = az;
#endif
}

// --- Filtro de Mediana Genérico ---
float medianN(float *buf, int N)
{
  float tmp[N];
  memcpy(tmp, buf, N * sizeof(float));

  for (int i = 0; i < N - 1; i++) {
    for (int j = i + 1; j < N; j++) {
      if (tmp[j] < tmp[i]) {
        float t = tmp[i];
        tmp[i] = tmp[j];
        tmp[j] = t;
      }
    }
  }
  return tmp[N / 2];
}

// --- Filtro de Mediana 3 eixos ---
void filterMedianN(float ax, float ay, float az,
                   float &axf, float &ayf, float &azf)
{
#if MEDIAN_FILTER_ENABLED
  static float bx[MEDIAN_WINDOW_SIZE] = {0};
  static float by[MEDIAN_WINDOW_SIZE] = {0};
  static float bz[MEDIAN_WINDOW_SIZE] = {0};
  static int idx = 0;

  bx[idx] = ax;
  by[idx] = ay;
  bz[idx] = az;
  idx = (idx + 1) % MEDIAN_WINDOW_SIZE;

  axf = medianN(bx, MEDIAN_WINDOW_SIZE);
  ayf = medianN(by, MEDIAN_WINDOW_SIZE);
  azf = medianN(bz, MEDIAN_WINDOW_SIZE);
#else
  axf = ax;
  ayf = ay;
  azf = az;
#endif
}