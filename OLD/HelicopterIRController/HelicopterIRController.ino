#include <boarddefs.h>
#include <IRremote.h>
#include <IRremoteInt.h>
#include <ir_Lego_PF_BitStreamEncoder.h>

#include <Arduino_FreeRTOS.h>

/* ---------------------------------      
        SilverlitIRController       
   --------------------------------- */

//#define DUMP_CONTROLLER_BINARY
#define LOG_CONTROLLER
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





void taskReadIRController(){
  IRrecv irrecv(IR_PIN); // Receive on pin 11
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
      
      irrecv.resume(); // Continue receiving
    }
    vTaskDelay(10);
  }
  vTaskDelete(NULL);
}

void setup() {
  Serial.begin(9600);
  xTaskCreate(taskReadIRController, "Read IR Controller", 256, NULL, 0, NULL);
}

void loop() {}
