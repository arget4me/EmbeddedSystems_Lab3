
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <semphr.h>

#include <IRremote.h>

/* ----------------------------------------------
                 MOTOR
   ---------------------------------------------- */
#define ENGINE_LEFT_BACKW 10
#define ENGINE_LEFT_FORW 9
#define ENGINE_LEFT 5

#define ENGINE_RIGHT_BACKW 8
#define ENGINE_RIGHT_FORW 7
#define ENGINE_RIGHT 6
#define MINIMUM_ENGINE_SPEED 100
#define MEDIUM_ENGINE_SPEED 200
#define MAX_ENGINE_SPEED 255

#define TURN_SPEED_LEFT 0.0
#define TURN_SPEED_RIGHT 0.0

/* ----------------------------------------------
                 DISTANCE SENSOR
   ---------------------------------------------- */
#define ECHO_PIN 12
#define TRIG_PIN 11
#define DISTANCE_LED A5
#define SPEED_OF_SOUND 340.0
#define MIN_DISTANCE 20 //cm
#define pulseTimeToDistanceCM(x) ((SPEED_OF_SOUND * (((double)x / 2.0) / 1000000.0)) * 100.0)


/* ----------------------------------------------
                 IR SENSOR
   ---------------------------------------------- */
#define IR_PIN A0
#define IR_POWER 0x00FF629D
#define IR_A 0x00FF22DD
#define IR_B 0x00FF02FD
#define IR_C 0x00FFC23D
#define IR_UP 0x00FF9867
#define IR_DOWN 0x00FF38C7
#define IR_LEFT 0x00FF30CF
#define IR_RIGHT 0x00FF7A85
#define IR_SELECT 0x00FF18E7

//Alternative IR codes
#define IR_POWER_ALT 0x00511DBB
#define IR_A_ALT 0x52A3D41F
#define IR_B_ALT 0xD7E84B1B
#define IR_C_ALT 0x20FE4DBB
#define IR_UP_ALT 0x97483BFB
#define IR_DOWN_ALT 0x488F3CBB
#define IR_LEFT_ALT 0x9716BE3F
#define IR_RIGHT_ALT 0x6182021B
#define IR_SELECT_ALT 0x3D9AE3F7

/* ----------------------------------------------
                 Global Variables
   ---------------------------------------------- */
QueueHandle_t motorQueue = NULL;
QueueHandle_t directionQueue = NULL;

bool globalObstacleAhead = false;

SemaphoreHandle_t obstacleMutex = NULL;


/* ---------------------------------------------- */


void taskDistance(void *param) {
  float distance = 0.0; //current distance in cm
  for (;;) {
    
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    unsigned long distanceTime = pulseIn(ECHO_PIN, HIGH);
    distance = pulseTimeToDistanceCM(distanceTime);
    //Serial.print(distance); Serial.println("cm");
    
    if (distance < MIN_DISTANCE){
      xSemaphoreTake(obstacleMutex, portMAX_DELAY);
      globalObstacleAhead = true;
      xSemaphoreGive(obstacleMutex);
      digitalWrite(DISTANCE_LED, HIGH);
      //@NOTE: If an obstacle is found. then there is less need to keep looking for an obstacle.
      vTaskDelay(pdMS_TO_TICKS(500));
    }else{
      digitalWrite(DISTANCE_LED, LOW);
      xSemaphoreTake(obstacleMutex, portMAX_DELAY);
      globalObstacleAhead = false;
      xSemaphoreGive(obstacleMutex);
     }
      
    vTaskDelay(pdMS_TO_TICKS(10));

  }
  vTaskDelete(NULL);
}




void taskIRReceiver(void* param) {
  IRrecv receiver(IR_PIN);
  receiver.enableIRIn();
  decode_results results;

  int dir = 0;
  int motorSpeed = MAX_ENGINE_SPEED;
  int latestCommand = 0; //checks if the last send command is a motor speed or direction command
  for (;;) {
    int oldMotorSpeed = motorSpeed;
    bool runMotor = false; //Determenes if the motor should continue running or not
    if (receiver.decode(&results)) {
      switch (results.value) {
        case IR_A_ALT:Serial.println("ALT");
        case IR_A:
          {
            motorSpeed = MINIMUM_ENGINE_SPEED;
            latestCommand = motorSpeed;
          } break;
        case IR_B_ALT:Serial.println("ALT");
        case IR_B:
          {
            motorSpeed = MEDIUM_ENGINE_SPEED;
            latestCommand = motorSpeed;
          } break;
        case IR_C_ALT:Serial.println("ALT");
        case IR_C:
          {
            motorSpeed = MAX_ENGINE_SPEED;
            latestCommand = motorSpeed;
          } break;
        case IR_UP_ALT:Serial.println("ALT");
        case IR_UP:
          {
            runMotor = true;
            dir = 1;
            latestCommand = dir;
            
          } break;
          
        case IR_DOWN_ALT:Serial.println("ALT");
        case IR_DOWN:
          {
            runMotor = true;
            dir = 2;
            latestCommand = dir;
          } break;
        case IR_LEFT_ALT:Serial.println("ALT");
        case IR_LEFT:
          {
            runMotor = true;
            dir = 3;
            latestCommand = dir;
          } break;
        case IR_RIGHT_ALT:Serial.println("ALT");
        case IR_RIGHT:
          {
            runMotor = true;
            dir = 4;
            latestCommand = dir;
          } break;
        case IR_POWER_ALT:Serial.println("ALT");
        case IR_POWER:
          {
            runMotor = true;
            dir = 0;
            latestCommand = dir;
          } break;
        case IR_SELECT_ALT:Serial.println("ALT");
        case IR_SELECT:
        {
        }break;
        case 0xFFFFFFFF:
        {
          //Serial.println("FFFFFFFF : Repeat code");
          if (latestCommand == motorSpeed){
            runMotor = false;
          }else {
            runMotor = true;
          }
        }break;
        //
        default:
          {
            dir = 0;
            //Serial.println(results.value, HEX);
          } break;

      }
      
      if(oldMotorSpeed != motorSpeed){
        xQueueOverwrite(motorQueue, &motorSpeed);
      }
      if(runMotor){
        xQueueOverwrite(directionQueue, &dir);
      }
      receiver.resume();
      //Lowest priority task. No delay needed.
      //vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
  
  vTaskDelete(NULL);
}


void taskRunMotor(void* param) {
  int motorSpeed = MAX_ENGINE_SPEED;
  int dir = 1;
  float leftWheel = 1.0;
  float rightWheel = 1.0;
  
  analogWrite(ENGINE_LEFT, motorSpeed * leftWheel);
  analogWrite(ENGINE_RIGHT, motorSpeed * rightWheel);
  
  for (;;) {
    bool newInstruction = false;
    if(xQueueReceive(motorQueue, &motorSpeed, 0) == pdTRUE){
      continue;
    }
    else{
      if (xQueueReceive(directionQueue, &dir, 0) == pdTRUE) {
        newInstruction = true;
        switch(dir){
          case 0:
          {
            leftWheel = 0.0;
            rightWheel = 0.0;
          }break;
          case 1:
          {
            
            leftWheel = 1.0;
            rightWheel = 1.0;
            digitalWrite(ENGINE_LEFT_FORW, HIGH);
            digitalWrite(ENGINE_RIGHT_FORW, HIGH);
            digitalWrite(ENGINE_LEFT_BACKW, LOW);
            digitalWrite(ENGINE_RIGHT_BACKW, LOW);
          }break;
          case 2:
          {
            leftWheel = 1.0;
            rightWheel = 1.0;
            digitalWrite(ENGINE_LEFT_FORW, LOW);
            digitalWrite(ENGINE_RIGHT_FORW, LOW);
            digitalWrite(ENGINE_LEFT_BACKW, HIGH);
            digitalWrite(ENGINE_RIGHT_BACKW, HIGH);
          }break;
          case 3:
          {
            leftWheel = TURN_SPEED_LEFT; 
            rightWheel = 1.0;
          }break;
          case 4:
          { 
            leftWheel = 1.0;
            rightWheel = TURN_SPEED_RIGHT; 
          }break;
        }
      }
      if(!newInstruction ){
        analogWrite(ENGINE_LEFT, 0);
        analogWrite(ENGINE_RIGHT, 0);
        
      }else{
        xSemaphoreTake(obstacleMutex, portMAX_DELAY);
        bool obstacleAhead = globalObstacleAhead;
        xSemaphoreGive(obstacleMutex);
        if(!obstacleAhead || dir == 2){      
          analogWrite(ENGINE_LEFT, motorSpeed * leftWheel);
          analogWrite(ENGINE_RIGHT, motorSpeed * rightWheel);
        }else {
          analogWrite(ENGINE_LEFT, 0);
          analogWrite(ENGINE_RIGHT, 0);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  vTaskDelete(NULL);
}


void setup() {
  Serial.begin(9600);
  // Distance Sensor
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(DISTANCE_LED, OUTPUT); 
  
  // IR Sensor
  pinMode(IR_PIN, INPUT);
  
  // Motor
  pinMode(ENGINE_LEFT, OUTPUT);
  pinMode(ENGINE_LEFT_FORW, OUTPUT);
  pinMode(ENGINE_LEFT_BACKW, OUTPUT);
  pinMode(ENGINE_RIGHT, OUTPUT);
  pinMode(ENGINE_RIGHT_FORW, OUTPUT);
  pinMode(ENGINE_RIGHT_BACKW, OUTPUT);

  obstacleMutex = xSemaphoreCreateMutex();
  motorQueue = xQueueCreate(1, sizeof(int) );
  directionQueue = xQueueCreate(1, sizeof(int) );
  if (motorQueue != NULL && directionQueue != NULL && obstacleMutex != NULL) {   
    xTaskCreate(taskDistance, "Distance Sensor", 128, NULL, 2, NULL);
    xTaskCreate(taskIRReceiver, "IR Sensor", 128, NULL, 0, NULL);
    xTaskCreate(taskRunMotor, "Motor", 128, NULL, 1, NULL);
  }
}

void loop() {}
