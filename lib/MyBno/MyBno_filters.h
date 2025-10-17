#pragma once
#include <Arduino.h>
#include "MyBno_config.h"

#ifndef __MYBNO_FILTERS_H__
#define __MYBNO_FILTERS_H__

// --- Filtro EMA ---
// alpha em [0..1]. Quanto menor, mais suave (mais filtragem).
// Escolha tipicamente entre 0.05 e 0.2 para ruído moderado.
void filterEMA(float ax, float ay, float az, float alpha,
               float &axf, float &ayf, float &azf);

// --- Filtro de Mediana Genérico ---
// usado dentro da mediana de N janelas
float medianN(float *buf, int N);

// --- Filtro de Mediana N janelas ---
void filterMedianN(float ax, float ay, float az,
                   float &axf, float &ayf, float &azf);

#endif