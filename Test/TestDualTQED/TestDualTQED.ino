/*
* August 2013
*
* Kristof Robot
*
* Program to test TQED encoder counter accuracy
*
* It polls 2 TQED encoder counters over (fast) I2C, 
* compares the counter difference with expected difference,
* and writes results to Serial
*
* Schematic:
*  Hook up the encoder inputs of two TQED encoder counters to a frequency generator,
*  and connect their SDA and SCL ports to an Arduino Uno.
*  Use 4.7K Ohm pull up resistors on SDA and SCL lines.
*
* Results:  
*  With a 4883Hz quadrature signal (e.g. generated by QuadratureSimulator.ino), 
*  and a polling delay of 20ms (i.e. about 50Hz polling rate),
*  i.e. expected rate of ticks is 390.
*  We find indeed an average rate of about 390, which is maintained consistently with an accuracy of <2 ticks 
*  Of course this requires careful tuning of the polling loop timing (using DELAY_* values).
*  Below timings worked for me, but you might need to re-tune, 
*  especially if you print out different/other output via Serial.
*
*/

#include <Wire.h>
#include <TQED.h>

TQED qed(0x36);
TQED qed2(0x38);
long prevCount=0L;
long currentCount=0L;
long prevCount2=0L;
long currentCount2=0L;
long count=0L;
long errorCount=0L;
long errorCount2=0L;
int extraDelay=0;

const int EXPECTED_TICKS=390; //fill in expected value here
const int ACCURACY=2; //fill in expected accuracy here
const int STATUS_FREQ=100; //how often should we print results to serial?
const int DELAY_BETWEEN_POLLS=20; //how long should we wait between polling (in ms);
const int DELAY_LOOP_US=715; //delay that loop itself takes (in us), to be subtracted from above delay to get a precise timing
const int DELAY_STATUS=200; //extra delay when status message is printed

#define TWI_FREQ_FAST 400000L  //I2C bus speed, default is 100kHz, this value allows speeds up to 400Khz

void setup()
{ 
  //set fast I2C bus speed
  TWBR = ((16000000L / TWI_FREQ_FAST) - 16) / 2;
  
  Serial.begin(115200);  // start serial for output
  Serial.println("Dual TQED Counter Test");
  Serial.println("Prints out 'OK' if accuracy is within ");
  Serial.print(ACCURACY);
  Serial.print(" ticks of ");
  Serial.print(EXPECTED_TICKS);
  Serial.print(" expected ticks per interval of ");
  Serial.print(DELAY_BETWEEN_POLLS);
  Serial.println(" ms.");
  Serial.println("Otherwise it prints out number of errors, detected tick rate, and resets error counts.");
  qed.resetCount();
  qed2.resetCount();
  errorCount=-1; //init at -1, as first round will always give error
  errorCount2=-1;
}


void loop()
{
  //Get both counter values
  currentCount=qed.getCount();
  currentCount2=qed2.getCount();

  //check accuracy
  if (abs(currentCount-prevCount-EXPECTED_TICKS) > ACCURACY) {
      errorCount++;
  }
   
  if (abs(currentCount2-prevCount2-EXPECTED_TICKS) > ACCURACY) {
      errorCount2++;
  }
 
  count++;
  if (count%STATUS_FREQ == 0) {
    if ((errorCount+errorCount2)==0 ) {
      Serial.println("OK"); //2 errors is normal because of startup
      extraDelay=DELAY_STATUS;
    } else {
      Serial.print(errorCount);
      Serial.print(" ");
      Serial.println(errorCount2);
      Serial.print(currentCount-prevCount);
      Serial.print(" ");
      Serial.println(currentCount2-prevCount2);
      
      //reset error counts
      errorCount=-1;
      errorCount2=-1;
    }
  }
  
  prevCount=currentCount;
  prevCount2=currentCount2;
  
  delay(DELAY_BETWEEN_POLLS-1);
  delayMicroseconds(1000-DELAY_LOOP_US-extraDelay); //account for status reporting
}
