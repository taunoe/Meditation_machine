/*
 File:    main.cpp
 Project: Meditation Machine
 Started: 10.10.2024
 Edited:  22.10.2024

 Copyright Tauno Erik & TSENTER 2024
*/
#include <Arduino.h>
#include "Unistep2.h"        // https://github.com/reven/Unistep2
#include "Radar_MR24HPC1.h"  // https://github.com/taunoe/mr24hpc1-radar

// Radar RX -> Pico GPIO0
// Radar TX -> Pico GPIO1
Radar_MR24HPC1 radar = Radar_MR24HPC1(&Serial1);

// Radar settings
const int RADAR_INTERVAL =  300;  // ms 300
const int MOTION_ENERGY_THRESHOLD = 150;  // 15
const int STATIC_ENERGY_THRESHOLD = 120;  // 120
const int STATIC_DISTANSE_THRESHOLD = 200;  // 200 cm


// Stepper Motor
const int STEPS_PER_REV = 4096;
const int STEP_DELAY = 900;  // in micros

#define STOP               0
#define CLOCKWISE          1
#define COUNTER_CLOCKWISE -1

// Stepper Motor 1 Pins
// Left Gears
const int M1_IN1 = 2;  // Pico GPIO2
const int M1_IN2 = 3;
const int M1_IN3 = 4;
const int M1_IN4 = 5;

Unistep2 left_gears(M1_IN1, M1_IN2, M1_IN3, M1_IN4, STEPS_PER_REV, STEP_DELAY);

// Stepper Motor 2 Pins
// Center disk
const int M2_IN1 = 6;  // Pico GPIO6
const int M2_IN2 = 7;
const int M2_IN3 = 8;
const int M2_IN4 = 9;

Unistep2 center_disk(M2_IN1, M2_IN2, M2_IN3, M2_IN4, STEPS_PER_REV, STEP_DELAY);

// Stepper Motor 3 Pins
// Right Gears
const int M3_IN1 = 12;  // Pico GPIO10
const int M3_IN2 = 13;
const int M3_IN3 = 14;
const int M3_IN4 = 15;

Unistep2 right_gears(M3_IN1, M3_IN2, M3_IN3, M3_IN4, STEPS_PER_REV, STEP_DELAY);


// Timing
uint64_t prev_millis = 0;
bool ask_radar = false;

/*******************************************/




/*****************************************
 * Core 0 setup
 *****************************************/
void setup() {
  Serial.begin(115200);   // Serial print
  Serial1.begin(115200);  // Radar

  while (!Serial1) {
      Serial.println("Radar disconnected");
      delay(500);
  }
  Serial.println("Radar ready");
  //radar.set_mode(SIMPLE);
  radar.set_mode(ADVANCED);
  // radar.set_static_limit(RANGE_300_CM);

}


/*****************************************
 * Core 1 setup
 *****************************************/
/*
void setup1() {
}
*/


/*****************************************
 * Core 0 loop
 *****************************************/
void loop() {
  uint64_t current_millis = millis();
  static int motion_energy = 0;
  static int static_energy = 0;
  static int static_distance = 0;
  static int motion_distance = 0;

  //radar.run(VERBAL);
  radar.run(NONVERBAL);

  center_disk.run();
  left_gears.run();
  right_gears.run();

  center_disk.move(COUNTER_CLOCKWISE);
  left_gears.move(COUNTER_CLOCKWISE);
  right_gears.move(COUNTER_CLOCKWISE);

  if ((current_millis - prev_millis) >= RADAR_INTERVAL) {
    ask_radar = true;
    prev_millis = current_millis;
  }

  if (ask_radar) {
    ask_radar = false;

    motion_energy = radar.get_motion_energy();
    static_energy = radar.get_static_energy();
    static_distance = radar.get_static_distance();
    motion_distance = radar.get_motion_distance();

    Serial.print("Liikuv ");
    Serial.print(motion_energy);
    Serial.print(" \td: ");
    Serial.print(motion_distance);
    Serial.print("\t\tSeisev ");
    Serial.print(static_energy);
    Serial.print(" \td: ");
    Serial.print(static_distance);
    Serial.print("cm");
    Serial.println();
  }

  //
  if (static_energy > STATIC_ENERGY_THRESHOLD) {
    if (motion_energy < MOTION_ENERGY_THRESHOLD) {
      // Serial.println("__ 2 Liigu");
      if (static_distance <= STATIC_DISTANSE_THRESHOLD) {
        // Serial.println("__ 3 Liigu");
        //left_gears.move(CLOCKWISE);
        //right_gears.move(COUNTER_CLOCKWISE);
      }
    }
  } else {
    //left_gears.move(STOP);
    //right_gears.move(STOP);
  }

}

/*****************************************
 * Core 1 loop
 *****************************************/
/*
void loop1() {
}
*/

