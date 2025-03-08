#include <Arduino.h>
#include <KB_music.h>

#ifndef I2C_ADDR
#define I2C_ADDR 0x69
#endif

#include <puppybot.h>
#include "PuppyBotIMU.h"
#include "PuppyBotTurnPID.h"


typedef int Number;
typedef int Boolean;

${EXTINC}
${VARIABLE}
${FUNCTION}

void setup() {
  
  puppybot_setup();
  Serial.begin(115200);
  Serial.print("Runing.....");
  ${SETUP_CODE}
  ${BLOCKSETUP}
}

void loop() {
  ${LOOP_CODE}
  ${LOOP_EXT_CODE}
  
}
