#include <boarddefs.h>
#include <IRremote.h>
#include <IRremoteInt.h>
#include <ir_Lego_PF_BitStreamEncoder.h>

#include <Arduino_FreeRTOS.h>

void taskReadIRController(){
  IRrecv irrecv(11); // Receive on pin 11
  decode_results results;
  
  irrecv.enableIRIn(); // Start the receiver
  for(;;){
    if (irrecv.decode(&results)) {
      Serial.println(results.value, HEX);
      irrecv.resume(); // Continue receiving
    }
    //vTaskDelay(100);
  }
  vTaskDelete(NULL);
}

void setup() {
  Serial.begin(9600);
  xTaskCreate(taskReadIRController, "Read IR Controller", 128, NULL, 0, NULL);
}

void loop() {
 
}
