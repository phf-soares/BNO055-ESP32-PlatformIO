#pragma once
#include "MyBno_filters.h"

#ifndef __MYBNO_CONFIG_H__
#define __MYBNO_CONFIG_H__

// ======== CONFIGURAÇÕES GLOBAIS ========

// --- Acelerômetro ---
#define BNO055_ACCELRANGE_2G  0x00
#define BNO055_ACCELRANGE_4G  0x01
#define BNO055_ACCELRANGE_8G  0x02
#define BNO055_ACCELRANGE_16G 0x03

#ifndef ACC_RANGE
  #define ACC_RANGE BNO055_ACCELRANGE_2G
#endif

// --- UART Monitor ---
#ifndef BAUD_RATE
  #define BAUD_RATE 1600000
#endif

// --- Filtros ---
#ifndef EMA_FILTER_ENABLED
  #define EMA_FILTER_ENABLED 1
#endif
#ifndef EMA_ALPHA
  #define EMA_ALPHA 0.1f
#endif

#ifndef MEDIAN_FILTER_ENABLED
  #define MEDIAN_FILTER_ENABLED 1
#endif
#ifndef MEDIAN_WINDOW_SIZE
  #define MEDIAN_WINDOW_SIZE 3
#endif

// 1 = EMA → Mediana | 0 = Mediana → EMA
#ifndef FILTER_ORDER_EMA_FIRST
  #define FILTER_ORDER_EMA_FIRST 0
#endif

#endif