//*********************************************************************************
//**
//** Project.........: Magnetic Transmitting Loop Antenna Controller
//**
//**
//** Copyright (C) 2015  Loftur E. Jonasson  (tf3lj [at] arrl [dot] net)
//**
//** This program is free software: you can redistribute it and/or modify
//** it under the terms of the GNU General Public License as published by
//** the Free Software Foundation, either version 3 of the License, or
//** (at your option) any later version.
//**
//** This program is distributed in the hope that it will be useful,
//** but WITHOUT ANY WARRANTY; without even the implied warranty of
//** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//** GNU General Public License for more details.
//**
//** You should have received a copy of the GNU General Public License
//** along with this program.  If not, see <http://www.gnu.org/licenses/>.
//**
//** Platform........: Teensy 3.1 & 3.2 (http://www.pjrc.com)
//**                   (It may be possible to adapt this code to other
//**                    Arduino compatible platforms, however this will
//**                    require extensive rewriting of some portions of
//**                    the code)
//**
//** Initial version.: 0.00, 2012-10-20  Loftur Jonasson, TF3LJ / VE2LJX
//**                   (pre-alpha version)
//**
//**
//*********************************************************************************

//
//---------------------------------------------------------------------------------
// Routines to control TMC2209 stepper drivers via UART
//---------------------------------------------------------------------------------
//

#include <Arduino.h>
#include "ML.h"
#include <TMCStepper.h>

#if STEPSTICKS == 2 && TMCUART // This entire file is irrelevant if these are not set.

/**
 * Author Teemu Mäntykallio
 *
 * Plot TMC2130 or TMC2660 motor load using the stallGuard value.
 * You can finetune the reading by changing the STALL_VALUE.
 * This will let you control at which load the value will read 0
 * and the stall flag will be triggered. This will also set pin DIAG1 high.
 * A higher STALL_VALUE will make the reading less sensitive and
 * a lower STALL_VALUE will make it more sensitive.
 *
 * You can control the rotation speed with
 * 0 Stop
 * 1 Resume
 * + Speed up
 * - Slow down
 */

#define MAX_SPEED        40 // In timer value
#define MIN_SPEED      1000

#define STALL_VALUE     100 // [0..255]

#define SERIAL_PORT Serial2
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2

#define R_SENSE 0.11f // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);

using namespace TMC2208_n;

// Using direct register manipulation can reach faster stepping times
#define STEP_PORT     PORTF // Match with STEP_PIN
#define STEP_BIT_POS      0 // Match with STEP_PIN

ISR(TIMER1_COMPA_vect){
  //STEP_PORT ^= 1 << STEP_BIT_POS;
  digitalWrite(STEP_PIN, !digitalRead(STEP_PIN));
}

void setup() {
  Serial.begin(250000);         // Init serial port and set baudrate
  while(!Serial);               // Wait for serial port to connect
  Serial.println("\nStart...");

  SERIAL_PORT.begin(115200);
  driver.beginSerial(115200);

  pinMode(drv8825_enable, OUTPUT);
  pinMode(drv8825_step, OUTPUT);
  pinMode(drv8825_dir, OUTPUT);
  digitalWrite(drv8825_enable, LOW);

  driver.begin();
  driver.toff(4);
  driver.blank_time(24);
  driver.rms_current(400); // mA
  driver.microsteps(16);
  driver.TCOOLTHRS(0xFFFFF); // 20bit max
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.SGTHRS(STALL_VALUE);

  // Set stepper interrupt
  {
    cli();//stop interrupts
    TCCR1A = 0;// set entire TCCR1A register to 0
    TCCR1B = 0;// same for TCCR1B
    TCNT1  = 0;//initialize counter value to 0
    OCR1A = 256;// = (16*10^6) / (1*1024) - 1 (must be <65536)
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS11 bits for 8 prescaler
    TCCR1B |= (1 << CS11);// | (1 << CS10);
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);
    sei();//allow interrupts
  }
}

void loop() {
  static uint32_t last_time=0;
  uint32_t ms = millis();

  while(Serial.available() > 0) {
    int8_t read_byte = Serial.read();
    if (read_byte == '0')      { TIMSK1 &= ~(1 << OCIE1A); digitalWrite( drv8825_enable, HIGH ); }
    else if (read_byte == '1') { TIMSK1 |=  (1 << OCIE1A); digitalWrite( drv8825_enable,  LOW ); }
    else if (read_byte == '+') { if (OCR1A > MAX_SPEED) OCR1A -= 20; }
    else if (read_byte == '-') { if (OCR1A < MIN_SPEED) OCR1A += 20; }
  }

  if((ms-last_time) > 100) { //run every 0.1s
    last_time = ms;

    Serial.print("0 ");
    Serial.print(driver.SG_RESULT(), DEC);
    Serial.print(" ");
    Serial.println(driver.cs2rms(driver.cs_actual()), DEC);
  }
}

#endif
