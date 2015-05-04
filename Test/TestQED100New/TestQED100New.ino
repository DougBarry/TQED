#include <Wire.h>
#include <TQED.h>"
#include "../TinyQEDI2CCMD.h"

TQED qed(0x39);

void setup()
{ 
  Serial.begin(115200);  // start serial for output
  Serial.println("TQED Test");
  delay(1000);
  Serial.println("Attempt fetching own address test");
  uint8_t testaddr = 0;
  testaddr = qed.getAddress();
  Serial.print("Rcvd: ");
  Serial.println(testaddr);
  delay(1000);
  Serial.println("Attempt setting count to center");
  qed.centerCount();
  delay(1000);
  Serial.println("Attempt read counter");
  Serial.println(qed.getCount());
  delay(1000);
  Serial.println("Attempt setting count to zero");
  qed.resetCount();
  delay(1000);
  Serial.println("Attempt read counter");
  Serial.println(qed.getCount());
  delay(1000);
  Serial.println("Attmpt read counter min");
  Serial.println(qed.getCounterMin());
  delay(1000);
  Serial.println("Attmpt read counter max");
  Serial.println(qed.getCounterMax());
  delay(1000);
  Serial.println("Attmpt read counter current pos");
  Serial.println(qed.getCount());
}

void ChangeAddress()
{
  Serial.println("Changing address to 0x40");
  qed.setAddress(0x40);
}

void loop()
{
  //Display the counter value
  Serial.println(qed.getCount());
  
  delay(100);
}
