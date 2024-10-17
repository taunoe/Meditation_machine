/*
 File:    main.cpp
 Project: Meditation Machine
 Started: 10.10.2024
 Edited:  17.10.2024

 Copyright Tauno Erik & TSENTER 2024
*/

#include <pico/multicore.h>
#include <Arduino.h>
#include "Unistep2.h"        // https://github.com/reven/Unistep2
#include "Radar_MR24HPC1.h"  // https://github.com/taunoe/mr24hpc1-radar

// Radar RX -> Pico GPIO0
// Radar TX -> Pico GPIO1
Radar_MR24HPC1 radar = Radar_MR24HPC1(&Serial1);

int heartbeat = 0;

// Radar settings
const int RADAR_INTERVAL =  500;  // ms 300
const int MOTION_ENERGY_THRESHOLD = 15;
const int STATIC_ENERGY_THRESHOLD = 121;
const int STATIC_DISTANSE_THRESHOLD = 200;  // cm


// Stepper Motor
const int STEPS_PER_REV = 4096;
const int STEP_DELAY = 900;      // in micros

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
const int M3_IN1 = 10;  // Pico GPIO10
const int M3_IN2 = 11;
const int M3_IN3 = 12;
const int M3_IN4 = 13;

Unistep2 right_gears(M3_IN1, M3_IN2, M3_IN3, M3_IN4, STEPS_PER_REV, STEP_DELAY);


// Timing
uint64_t prev_millis = 0;
bool ask_radar = false;


void core1_task() {
  //static int motor_1_direction = 0;  // +1, -1 or 0
  //static int center_disk_direction = CLOCKWISE;

  while (true) {
    center_disk.run();
    center_disk.move(CLOCKWISE);
  }
}


void setup() {
  // Serial print
  Serial.begin(115200);
  // Radar
  Serial1.begin(115200);

  while (!Serial1) {
      Serial.println("Radar disconnected");
      delay(500);
  }
  Serial.println("Radar ready");
  //radar.set_mode(SIMPLE);
  radar.set_mode(ADVANCED);
  // radar.set_static_limit(RANGE_300_CM);

  // Launch core1_task on Core 1
  multicore_launch_core1(core1_task);
}


void loop() {
  //static int motor_3_direction = 0;
  uint64_t current_millis = millis();

  //radar.run(VERBAL);
  radar.run(NONVERBAL);


  if ((current_millis - prev_millis) >= RADAR_INTERVAL) {
    ask_radar = true;
    prev_millis = current_millis;
  }

  if (ask_radar) {
    ask_radar = false;

    int motion_energy = radar.get_motion_energy();
    int static_energy = radar.get_static_energy();
    int static_distance = radar.get_static_distance();
    int motion_distance = radar.get_motion_distance();

    // Serial.print(counter);
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

    //
    if (static_energy > STATIC_ENERGY_THRESHOLD) {
      if (motion_energy < MOTION_ENERGY_THRESHOLD) {
        if (static_distance <= STATIC_DISTANSE_THRESHOLD) {
          // Move
          left_gears.run();
          left_gears.move(CLOCKWISE);

          right_gears.run();
          right_gears.move(COUNTER_CLOCKWISE);
        }
      }
    } else {
      left_gears.run();
          left_gears.move(STOP);
          
          right_gears.run();
          right_gears.move(STOP);
      
    }
  }
}
