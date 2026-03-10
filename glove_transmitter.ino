#include <Arduino.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

#define SERVO_COUNT 5

int servoPins[SERVO_COUNT] = {13, 12, 14, 27, 26};

int pwmChannels[SERVO_COUNT] = {0,1,2,3,4};

#define PWM_FREQ 50
#define PWM_RES 16

uint32_t angleToDuty(int angle)
{
  float pulseWidthMs = 1.0 + (angle / 180.0);
  float dutyCycle = (pulseWidthMs / 20.0) * 65535;

  return (uint32_t)dutyCycle;
}

void setup()
{
  Serial.begin(115200);

  SerialBT.begin("Rayhan_RobotHand");

  Serial.println("Robot Hand Bluetooth Receiver Ready");

  for (int i = 0; i < SERVO_COUNT; i++)
  {
    ledcSetup(pwmChannels[i], PWM_FREQ, PWM_RES);
    ledcAttachPin(servoPins[i], pwmChannels[i]);
  }
}

void loop()
{
  if (SerialBT.available())
  {
    String line = SerialBT.readStringUntil('\n');

    Serial.print("Received: ");
    Serial.println(line);

    int angles[SERVO_COUNT];
    int index = 0;

    char *token = strtok((char*)line.c_str(), ",");

    while (token != NULL && index < SERVO_COUNT)
    {
      angles[index] = atoi(token);
      index++;
      token = strtok(NULL, ",");
    }

    for (int i = 0; i < SERVO_COUNT; i++)
    {
      angles[i] = constrain(angles[i], 0, 180);
      uint32_t duty = angleToDuty(angles[i]);
      ledcWrite(pwmChannels[i], duty);
    }
  }
}