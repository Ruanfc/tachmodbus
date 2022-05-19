/*
** Inverter.h - Bliblioteca para lidar com o inversor e seu respectivo sensor
*hall
*/

#ifndef Inverter_h
#define Inverter_h
#include <Arduino.h>
#include <ArduinoModbus.h>
//#include <Wire.h>


class Inverter {
public:
  // void init(float tempKp, float tempKi);
  Inverter(int id, char *name, uint8_t hall_pin);
  void init();
  // float calcrps();
  float calcrps();
  float calcrps_now();
  void printinfo(); // remover argumentos
  void isr_handler();
  void atuador();
  float bound(float x, float x_min, float x_max);

  int getState();
  void updateSpeed(unsigned long ctime);
  float Inverter::getSpeed();
  float Inverter::get_set_speed();
  void setSpeed(float new_speed);
  int getHallPin();
  void checkStop(unsigned long ctime);

  void writeToMotor(float value);
  void reset();
  int getId();

  //float output;

private:
  int _id; // Id do inversor na rede modbus
  uint8_t _hall_pin;
  uint8_t _currentValue;
  char *_name;
  float set_speed = 0;
  float speed = 0;
  volatile int state = 0;
  volatile unsigned long start = 200, end_time = 0;
  // Controle de velocidade
  // PID
  unsigned long currentTime, elapsedTime, previousTime = 0;
  float error, lastError = 0, cumError, rateError;
  float output;
  //Falta ajustar parâmetros
  float Kp = 1, Ki = 0, Kd = 0;
  // Limites
  float set_acc = 0;
  float  acc = 0;
  float prev_speed = 0;
  float ACC_MAX = 600;
  float ACC_MIN = 60;
  float MAX_SPEED = 600;
  float STEP_SIZE = 0.1;

};
#endif
