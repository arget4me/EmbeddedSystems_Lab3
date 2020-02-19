#include <boarddefs.h>
#include <IRremote.h>
#include <IRremoteInt.h>
#include <ir_Lego_PF_BitStreamEncoder.h>



// Using the D6 pin on the Wemos D1 mini
uint16_t RECV_PIN = A0;

IRrecv irrecv(RECV_PIN);

decode_results results;

void setup()
{
  Serial.begin(9600);
  irrecv.enableIRIn();  // Start the receiver
  Serial.println("Start");
}

String GetIRBinaryValue(decode_results *results, unsigned int level)
{
  String result = "";
  for (int i = 0;  i < results->rawlen - 2;  i++)
  {
    if (results->rawbuf[i] > level)
    {
      result += "1";
    }
    else
    {
      result += "0";
    }
  }
  return result;
}

void DumpIRBinaryValue(decode_results *results, unsigned int level)
{
  int c = 0;
  for (int i = 0;  i < results->rawlen;  i++)
  {
    if (level == 0)
    {
      Serial.print(String(results->rawbuf[i]) + " ");
    }
    else if (results->rawbuf[i] > level)
    {
      Serial.print("1");
    }
    else
    {
      Serial.print("0");
    }
    c++;
    if (c > 9)
    {
      c = 0;
      Serial.print(" ");
    }
  }
}

long ParseIRValueDebug(decode_results *results, unsigned int minMaxBorder, int indexes[], int indexesSize)
{
  long result = 0;
  Serial.print("#");
  for (int i = 0; i < indexesSize;  i++)
  {
    if (results->rawbuf[indexes[i]] > minMaxBorder)
    {
      Serial.print("1");
    }
    else
    {
      Serial.print("0");
    }

    //Serial.print(String(results->rawbuf[i]) + ".");
    int t = results->rawbuf[indexes[i]];
    result <<= 1;
    result += t > minMaxBorder;
  }
  Serial.print(">" + String(result) + "#");
  return result;
}

long ParseIRValue(decode_results *results, unsigned int minMaxBorder,
                  int indexes[], int indexesSize)
{
  long result = 0;

  for (int i = 0; i < indexesSize;  i++)
  {
    int t = results->rawbuf[indexes[i]];
    result <<= 1;//shifts the bits to the left in result.
    result += t > minMaxBorder;
  }
  return result;
}

void loop()
{
  if (irrecv.decode(&results))
  {
    DumpIRBinaryValue(&results, 16);
    /*
    Serial.println("");
    Serial.println("Controlls - - - - - - - - -");
    {
      Serial.print("  Channel: ");
      int channelIndexes[] = {7, 9};
      int channel = ParseIRValue(&results, 16, channelIndexes, 2);
      Serial.println(String(channel) + "|");
    }
    {
      Serial.print("  Throttle: ");
      int throttleIndexes[] = {11, 13, 15, 17};
      int throttle = ParseIRValue(&results, 16, throttleIndexes, 4);
      Serial.println(String(throttle) + "|");
    }
    {
      Serial.print("  Trim: ");
      int lrTrimIndexes[] = {19, 21};
      int lrTrim = ParseIRValue(&results, 16, lrTrimIndexes, 2);
      Serial.println(String(lrTrim) + "|");
    }
    {
      Serial.print("  Yaw: ");
      int yawIndexes[] = {23, 25, 27};
      int yaw = ParseIRValue(&results, 16, yawIndexes, 3);
      Serial.println(String(yaw) + "|");
    }
    {
      Serial.print("  Fire: ");
      int fireIndexes[] = {33, 35};
      int fire = ParseIRValue(&results, 16, fireIndexes, 2);
      Serial.println(String(fire) + "|");
    }
    {
      Serial.print("  Checksum: ");
      int checkIndexes[] = {37, 39};
      int checkSum = ParseIRValue(&results, 16, checkIndexes, 2);
      Serial.println(String(checkSum) + "|");
    }
    
    Serial.println("- - - - - - - - - - - - - -");
    */
    Serial.println("");
    
    /* Picoo Z */
    /*
      Serial.print(" - ");
      int throttleIndexes[] = {15, 17, 19, 21};
      int throttle = ParseIRValue(&results, 16, throttleIndexes, 4);
      Serial.print(String(throttle));

      Serial.print(" - ");
      int dirIndexes[] = {31, 33, 35};
      int dir = ParseIRValue(&results, 16, dirIndexes, 3);
      Serial.print(String(dir));

      Serial.print(" - ");
      int chnlIndexes[] = {11, 13};
      int chnl = ParseIRValue(&results, 16, chnlIndexes, 2);
      Serial.print(String(chnl));

      Serial.print(" - ");
      int trmIndexes[] = {23, 25, 27, 29};
      int trm = ParseIRValue(&results, 16, trmIndexes, 4);
      Serial.print(String(trm));
    */

    /* Dauphin */
    /*Serial.print(" - ");
      int chnlA[] = {3};
      int chnl = ParseIRValue(&results, 16, chnlA, 1);
      Serial.print(String(chnl));

      Serial.print(" - ");
      int leftThrA[] = {9, 11, 13, 15, 17};
      int leftThr = ParseIRValue(&results, 16, leftThrA, 5);
      Serial.print(String(leftThr));

      Serial.print(" - ");
      int rightDirA[] = {19, 21, 23, 25, 27};
      int rightDir = ParseIRValue(&results, 16, rightDirA, 5);
      Serial.print(String(rightDir));

      Serial.print(" - ");
      int leftDirA[] = {29, 31, 33, 35, 37};
      int leftDir = ParseIRValue(&results, 16, leftDirA, 5);
      Serial.print(String(leftDir));

      Serial.print(" - ");
      int rightThrA[] = {39, 41, 43, 45, 47};
      int rightThr = ParseIRValue(&results, 16, rightThrA, 5);
      Serial.print(String(rightThr));

      Serial.print(" - ");
      int lightA[] = {49};
      int light = ParseIRValue(&results, 16, lightA, 1);
      Serial.print(String(light));
    */
    irrecv.resume();  // Receive the next value
  }
  delay(100);
}
