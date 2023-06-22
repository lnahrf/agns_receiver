#include <Arduino.h>
#include "receiver.h"

void setup()
{
  Serial.begin(9600);

  RECEIVER::initReceiver();

  std::string mac = "MAC: " + RECEIVER::getMACAddress();
  Serial.println(mac.c_str());
};

void loop()
{
  RECEIVER::messageHandler();
  RECEIVER::stateTimeHandler();
  delay(100);
};
