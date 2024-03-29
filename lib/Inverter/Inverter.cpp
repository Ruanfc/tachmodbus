#include "Inverter.h"

Inverter::Inverter(int id, char *name, uint8_t hall_pin) {
  _id = id;
  _name = name;
  _hall_pin = hall_pin;
}

// void Inverter::init(float tempKp, float tempKi) {
void Inverter::init() {
  pinMode(this->_hall_pin, INPUT);
  while (!ModbusRTUClient.holdingRegisterWrite(_id, 8, 2)) {delay(100);}
  ModbusRTUClient.setTimeout(50); //unsigned long (25 milissegundos)
}

float Inverter::calcrps() {
  float time_passed = (float(this->end_time - this->start) / 1000000.0);
  float frequency = bound(1 / time_passed, 0, 600);
  return frequency;
}

void Inverter::printinfo() {
  Serial.print(_name);
  Serial.print(": ");
  Serial.print(speed);
  Serial.println(" rps");
}

void Inverter::isr_handler() {
  // IRAM_ATTR void isr_rotor(void) {
  start = end_time;
  end_time = micros();
  state = HIGH;
}

void Inverter::atuador() {
  // O atuador deve funcionar conforme um controlador PID.
  // set point = set_speed
  // current value = speed
  // out value to inverter = output

  // Primeiro determina o tempo
  currentTime = micros();
  if (write_success) // medida para evitar instabilidade
    {
      elapsedTime = (currentTime - previousTime);
      //Cálcular erro
      error = set_speed - speed;
      //integral
      cumError += error * elapsedTime/1000000;
      //derivative
      rateError += (error - lastError)/(elapsedTime);
      //Output value
      this->output = Kp * error + Ki * cumError + Kd * rateError;
      //Housekeeping for next iteration
      lastError = error;
    }
  previousTime = currentTime;

  output = bound(output, 0, 40);
  Serial.print("\\");
  Serial.print(output);
  Serial.print("\\");

  //Agora jogamos o valor de saída para a serial
  write_success = writeToMotor(output);
}

int Inverter::writeToMotor(float value)
  {
    //Olhar manual específico do fre-700, na página 241
    //or bitwise on register 40009 to forward rotate motor
        //ModbusRTUClient.registerMaskWrite(_id,8, ~0x2, 0x2);
    //ModbusRTUClient.holdingRegisterWrite(_id, 8, 2);
    //set register 40014 to control running frequency
    //ModbusRTUClient.holdingRegisterWrite(_id, 13, (int) value);
    return ModbusRTUClient.holdingRegisterWrite(_id, 13, (int)100*value);
  }

float Inverter::bound(float x, float x_min, float x_max) {
  if (x < x_min) {
    x = x_min;
  }
  if (x > x_max) {
    x = x_max;
  }
  return x;
}

int Inverter::getState() { return this->state; }

void Inverter::updateSpeed(unsigned long ctime) {
  if (this->state) {
    if (ctime > this->end_time/1000 + 1000)
    {
      this->speed = 0;
    }
    else
    {
      this->speed = calcrps();
    }
    this->state = LOW;
  }
}

void Inverter::setSpeed(float new_speed) { set_speed = new_speed; }
float Inverter::getSpeed() { return this->speed; }
float Inverter::get_set_speed() { return this->set_speed; }

int Inverter::getHallPin() { return _hall_pin; }

void Inverter::checkStop(unsigned long ctime)
{
  if (ctime > this->end_time + 1000000)
  {
    this->state = LOW;
    this->speed = 0;
  }
}

int Inverter::getId() {return _id;}
