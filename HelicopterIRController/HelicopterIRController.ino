#include <boarddefs.h>
#include <IRremote.h>
#include <IRremoteInt.h>
#include <ir_Lego_PF_BitStreamEncoder.h>

#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <semphr.h>



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

#define TURN_SPEED_LEFT 0.6
#define TURN_SPEED_RIGHT 0.6

/* ----------------------------------------------
                 DISTANCE SENSOR
   ---------------------------------------------- */
#define ECHO_PIN 12
#define TRIG_PIN 11
#define DISTANCE_LED A5
#define SPEED_OF_SOUND 340.0
#define MIN_DISTANCE 20 //cm
#define pulseTimeToDistanceCM(x) ((SPEED_OF_SOUND * (((double)x / 2.0) / 1000000.0)) * 100.0)


/* ---------------------------------      
        SilverlitIRController       
   --------------------------------- */

//#define DUMP_CONTROLLER_BINARY
//#define LOG_CONTROLLER
#define IR_PIN A0
#define IR_MIN_MAX_BORDER 800
#define IR_VALUE_TO_BIT(x) ((x * USECPERTICK) > IR_MIN_MAX_BORDER)
#define SILVERLIT_HEADER_CODE 0x0E
#define SILVERLIT_MESSAGE_SIZE 42
struct SilverlitIRController {
  unsigned char header;   //6bit
  unsigned char channel;  //2bit
  unsigned char throttle; //4bit
  unsigned char lrTrim;   //2bit
  unsigned char yaw;      //3bit
  unsigned char fire;     //2bit
  unsigned char checkSum; //2bit
};

/* ----------------------------------------------
                 Global Variables
   ---------------------------------------------- */
QueueHandle_t controllerQueue = NULL;

bool globalObstacleAhead = false;

SemaphoreHandle_t obstacleMutex = NULL;


/* ---------------------------------------------- */




#ifdef DUMP_CONTROLLER_BINARY
void dumpSilverlitIRBinaryValue(decode_results *results)
{
  int parts[] = {1, 7, 11, 19, 23, 29, 33, 37, 39, 43};
  int partsIndex = 0;
  for (int i = 1;  i < results->rawlen;  i++){
    if(partsIndex < sizeof(parts)){
      if(i == parts[partsIndex]){
        Serial.print(" ");
        partsIndex++;
      }
    }
    
    Serial.print(IR_VALUE_TO_BIT(results->rawbuf[i]));
  }Serial.println("");
}
#endif

#ifdef LOG_CONTROLLER
void printSilverlitIRController(const SilverlitIRController& controller){
  Serial.println("Silverlit IR Controller - - - - - - - - -");
  //Serial.print("  Header: "); Serial.println(controller.header);
  Serial.print("  Channel: "); Serial.println(controller.channel);
  Serial.print("  Throttle: "); Serial.println(controller.throttle);
  Serial.print("  Trim: "); Serial.println(controller.lrTrim);
  Serial.print("  Yaw: "); Serial.println(controller.yaw);
  Serial.print("  Fire: "); Serial.println(controller.fire);
  //Serial.print("  Checksum: "); Serial.println(controller.checkSum);
  Serial.println("- - - - - - - - - - - - - - - - - - - - -\n");
}
#endif

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
      analogWrite(ENGINE_LEFT, 0);
      analogWrite(ENGINE_RIGHT, 0);
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


unsigned char parseIRValue(int* rawBuffer, int stepLength, int numBits){
  unsigned char result = 0;

  for(int i = 0; i < numBits; i++){
    int currentValue = rawBuffer[i * stepLength];
    result = result << 1;
    result += IR_VALUE_TO_BIT(currentValue);
  }
  return result;
}

SilverlitIRController parseSilverlitIRController(decode_results *results){
  if(results->rawlen < SILVERLIT_MESSAGE_SIZE){
    //Serial.print("!- MESSAGE TO SHORT!"); Serial.println(results->rawlen);
    return {};
  }
  SilverlitIRController controller = {};
  controller.header = parseIRValue(results->rawbuf + 1, 1, 6);
  if(controller.header != SILVERLIT_HEADER_CODE){
    //Serial.print("!- WRONG HEADER: "); Serial.println(controller.header);
    return {};
  }
  
  controller.channel = parseIRValue(results->rawbuf + 7, 2, 2);
  controller.throttle = parseIRValue(results->rawbuf + 11, 2, 4);
  controller.lrTrim = parseIRValue(results->rawbuf + 19, 2, 2);
  controller.yaw = parseIRValue(results->rawbuf + 23, 2, 3);
  controller.fire = parseIRValue(results->rawbuf + 33, 2, 2);
  controller.checkSum = parseIRValue(results->rawbuf + 39, 2, 2);
  
  return controller;
}



void taskRunMotor(void* param) {
  int motorSpeed = 0;
  SilverlitIRController controller = {};
  float leftWheel = 1.0;
  float rightWheel = 1.0;
  int channel = 1;
  
  analogWrite(ENGINE_LEFT, motorSpeed * leftWheel);
  analogWrite(ENGINE_RIGHT, motorSpeed * rightWheel);
  digitalWrite(ENGINE_LEFT_FORW, HIGH);
  digitalWrite(ENGINE_RIGHT_FORW, HIGH);
  digitalWrite(ENGINE_LEFT_BACKW, LOW);
  digitalWrite(ENGINE_RIGHT_BACKW, LOW);
  
  for (;;) {
    bool newInstruction = false;
    if (xQueueReceive(controllerQueue, &controller, portMAX_DELAY) == pdTRUE) {
      newInstruction = true;
      /*if(throttle > 0)
      motorSpeed = (MAX_ENGINE_SPEED - MIN_ENGINE_SPEED) * (throttle/14) + MIN_ENGINE_SPEED;
      else
      motorSpeed = 0;
      */
      if(channel != controller.channel){
        channel = controller.channel;
        if(channel == 1) {
          digitalWrite(ENGINE_LEFT_FORW, HIGH);
          digitalWrite(ENGINE_RIGHT_FORW, HIGH);
          digitalWrite(ENGINE_LEFT_BACKW, LOW);
          digitalWrite(ENGINE_RIGHT_BACKW, LOW);
        }else {
          digitalWrite(ENGINE_LEFT_FORW, LOW);
          digitalWrite(ENGINE_RIGHT_FORW, LOW);
          digitalWrite(ENGINE_LEFT_BACKW, HIGH);
          digitalWrite(ENGINE_RIGHT_BACKW, HIGH);
        }
      }
      
      motorSpeed = (int) (MAX_ENGINE_SPEED * ((float)controller.throttle / 14.0));
      
      leftWheel = 1.0;
      rightWheel = 1.0;
      if(controller.yaw > 0){
        if(controller.yaw < 4) {
          //leftWheel = TURN_SPEED_LEFT * ((3-controller.yaw) / 3);
          leftWheel = 1.0 - ((float)controller.yaw / 3);
        }else {
          rightWheel = 1.0 - ((float)(controller.yaw - 3) / 3);
        }
      }
     
      xSemaphoreTake(obstacleMutex, portMAX_DELAY);
      bool obstacleAhead = globalObstacleAhead;
      xSemaphoreGive(obstacleMutex);
      if(!obstacleAhead || channel != 1){      
        analogWrite(ENGINE_LEFT, motorSpeed * leftWheel);
        analogWrite(ENGINE_RIGHT, motorSpeed * rightWheel);
      }else {
        analogWrite(ENGINE_LEFT, 0);
        analogWrite(ENGINE_RIGHT, 0);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  vTaskDelete(NULL);
}

void taskReadIRController(){
  IRrecv irrecv(IR_PIN);
  decode_results results;
  
  irrecv.enableIRIn(); // Start the receiver
  #ifdef LOG_CONTROLLER
  char printCounter = 0;
  #endif
  for(;;){
    if (irrecv.decode(&results)) {
      #ifdef DUMP_CONTROLLER_BINARY
      dumpSilverlitIRBinaryValue(&results);
      #endif
      SilverlitIRController controller = parseSilverlitIRController(&results);
      #ifdef LOG_CONTROLLER
      if(printCounter++ % 5 == 0){
        printSilverlitIRController(controller);
      }
      #endif
      //send controller to motor.
      xQueueOverwrite(controllerQueue, &controller);
      
      irrecv.resume(); // Continue receiving
    }
    //vTaskDelay(10);
  }
  vTaskDelete(NULL);
}

void setup() {
  Serial.begin(9600);
  // Distance Sensor
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(DISTANCE_LED, OUTPUT); 

  // Motor
  pinMode(ENGINE_LEFT, OUTPUT);
  pinMode(ENGINE_LEFT_FORW, OUTPUT);
  pinMode(ENGINE_LEFT_BACKW, OUTPUT);
  pinMode(ENGINE_RIGHT, OUTPUT);
  pinMode(ENGINE_RIGHT_FORW, OUTPUT);
  pinMode(ENGINE_RIGHT_BACKW, OUTPUT);

  obstacleMutex = xSemaphoreCreateMutex();
  controllerQueue = xQueueCreate(1, sizeof(SilverlitIRController) );
  if (controllerQueue != NULL && obstacleMutex != NULL) {   
    xTaskCreate(taskDistance, "Distance Sensor", 128, NULL, 2, NULL);
    xTaskCreate(taskReadIRController, "Read IR Controller", 128, NULL, 0, NULL);
    xTaskCreate(taskRunMotor, "Motor", 128, NULL, 1, NULL);
  }
}

void loop() {}
