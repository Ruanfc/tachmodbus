#include "Inverter.h"
#include <Arduino.h>
#include <ArduinoModbus.h>

//RS485Class RS485_(Serial1, 18, A6, A5);
//RS485Class RS485_(Serial2, 16, A6, A5);
RS485Class RS485_(Serial1, 18, A9, A8); //Troca o serial por serial1
Inverter impeller(1, "Impeller", 3);
Inverter rotor(2, "Rotor", 2);

#define SERIAL_PRINT_INTERVAL 100
#define ATUADOR_INTERVAL 100
unsigned long current_millis = 0, previous_millis = 0, previous_millis_atuador= 0;

String inputStr = "";
int range = 10;


void impeller_isr() { impeller.isr_handler(); }

void rotor_isr() { rotor.isr_handler(); }

void setup() {
  Serial.begin(115200);
  ModbusRTUClient.begin(RS485_, 9600, SERIAL_8E1);
  //ModbusRTUClient.begin(115200, SERIAL_8E1);
  impeller.init();
  rotor.init();
  attachInterrupt(digitalPinToInterrupt(impeller.getHallPin()), impeller_isr,
                  RISING);
  attachInterrupt(digitalPinToInterrupt(rotor.getHallPin()), rotor_isr, RISING);
}
void loop() {
  // Atualiza valores de velocidades (apenas quando houver interrupção)

  current_millis = millis();
  impeller.updateSpeed(current_millis);
  rotor.updateSpeed(current_millis);
  //elapsed_millis = current_millis - previous_millis;
  // Espera pelo menos 1 milisegundo para cada atuação
  if (current_millis > previous_millis_atuador + ATUADOR_INTERVAL)
    {
      impeller.atuador();
      rotor.atuador();
      previous_millis_atuador = current_millis;
    }

  // Altera os motores em um intervalo fixo de 100ms
  if (current_millis > previous_millis + SERIAL_PRINT_INTERVAL) {
   Serial.print(impeller.getSpeed());
   Serial.print(";");
   //impeller.writeToMotor(impeller.output);
   Serial.print(rotor.getSpeed());
   Serial.print(";");
   //rotor.writeToMotor(rotor.output);
   Serial.print(impeller.get_set_speed());
   Serial.print(";");
   Serial.print(rotor.get_set_speed());
   Serial.println();
    previous_millis = current_millis;
  }
  /* // Checagem de parada do inversor */
  /* impeller.checkStop(current_millis); */
  /* rotor.checkStop(current_millis); */
}

// Esta função entra em ação todo fim de loop
void serialEvent() {
  if (Serial.available()) {
    inputStr = Serial.readString();
    //int len = sizeof(inputStr);

    // Remove b' vindo da aplicação em python
    inputStr.replace("b'","");
    
    // Dá split na string recebida
    int dotcommaIndex = inputStr.indexOf(";");
    if (dotcommaIndex > 0) {
      String impellerStr = inputStr.substring(0, dotcommaIndex);
      String rotorStr = inputStr.substring(dotcommaIndex + 1);

      impeller.setSpeed(impellerStr.toFloat());
      rotor.setSpeed(rotorStr.toFloat());
    }
  }
} // Fim SerialEvent
