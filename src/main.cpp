#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"

#define BNO055_ACCELRANGE_2G  0x00
#define BNO055_ACCELRANGE_4G  0x01
#define BNO055_ACCELRANGE_8G  0x02
#define BNO055_ACCELRANGE_16G 0x03
#define BAUD_RATE 1600000

const uint8_t INTERRUPT_PIN = 2;

void interrupt_callback(void);
volatile bool interrupt;

Adafruit_BNO055 bno = Adafruit_BNO055();
imu::Vector<3> acc;

template<typename T>
T median3(T a, T b, T c) {
  if (a > b) std::swap(a, b);
  if (b > c) std::swap(b, c);
  if (a > b) std::swap(a, b);
  return b;
}

static float ax1=0, ax2=0, ay1=0, ay2=0, az1=0, az2=0;

void filterMedian3(float ax, float ay, float az,
                   float &axm, float &aym, float &azm) {
  axm = median3(ax, ax1, ax2);
  aym = median3(ay, ay1, ay2);
  azm = median3(az, az1, az2);
  ax2 = ax1; ax1 = ax;
  ay2 = ay1; ay1 = ay;
  az2 = az1; az1 = az;
}

static bool ema_init = false;
static float ax_f=0, ay_f=0, az_f=0;
float ax, ay, az, axf, ayf, azf;
float axff, ayff, azff;

void filterEMA(float ax, float ay, float az, float alpha,
               float &ax_out, float &ay_out, float &az_out) {
  if (!ema_init) { ax_f = ax; ay_f = ay; az_f = az; ema_init = true; }
  ax_f += alpha * (ax - ax_f);
  ay_f += alpha * (ay - ay_f);
  az_f += alpha * (az - az_f);
  ax_out = ax_f; ay_out = ay_f; az_out = az_f;
}

void setup(void)
{
  Serial.begin(BAUD_RATE);     
  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  if(!bno.setAccRange( ACC_RANGE ))
  {
    Serial.print("Ooops, changing Acc range seems to be refused!");    
  }      
  interrupt = false;
  pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interrupt_callback, RISING);   
  bno.enableAnyMotion(0, 0);  
  bno.enableInterruptsOnXYZ(ENABLE, ENABLE, ENABLE);
  bno.setExtCrystalUse(true);    
}

void loop(void)
{
  if (interrupt) {
    acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    ax = acc.x(); ay = acc.y(); az = acc.z();
    filterEMA(ax,ay,az, 0.1f, axf,ayf,azf);
    filterMedian3(axf,ayf,azf, axff,ayff,azff);      
    Serial.printf("%.2f,%.2f,%.2f\n", axff, ayff, azff);    
  }
  else
  {
    bno.resetInterrupts();
    interrupt = false;
  }
}

void interrupt_callback(void)
{
  interrupt = true;
}