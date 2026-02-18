#include <AFMotor.h>

#define NUM_ROTATIONS 2
#define SWITCH_PIN A5

#define FULL_ROTATION 200

#define INIT_SPEED 40
#define SPEED_MIN 20
#define SPEED_MAX 150


AF_Stepper stepper(FULL_ROTATION, 2);
int motor_speed = INIT_SPEED;
int temp_speed;

enum state_t{IDLE, WAITING, ROTATING};
state_t state = IDLE;

bool switch_pressed(){
    return analogRead(SWITCH_PIN) > 1000;
}

void setup() {
  Serial.begin(9600);
  stepper.setSpeed(motor_speed);
}

void loop() {

  if(Serial.available()){
      temp_speed = Serial.parseInt();
      if(temp_speed >= SPEED_MIN && temp_speed <= SPEED_MAX){
          motor_speed = temp_speed;
          stepper.setSpeed(motor_speed);
          Serial.print("Speed set to: ");
          Serial.println(motor_speed);
       }
    }

  delay(100);
  if(state == IDLE){
    if(switch_pressed()){
      state = WAITING;
      Serial.println("Going into WAITING state.");
    }
  }
  else if(state == WAITING){
    if(!switch_pressed()){
      state = ROTATING;
      Serial.println("Going into ROTATING state.");
    }
  }
  else if(state == ROTATING){

    delay(1000);

    stepper.step(NUM_ROTATIONS*FULL_ROTATION, FORWARD, DOUBLE);

    state = IDLE;

    Serial.println("Going into IDLE state.");
    stepper.release();
  }

}
