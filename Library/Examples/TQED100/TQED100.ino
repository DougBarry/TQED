#include <Wire.h>
#include <TQED.h>

TQED qed(0x36);

void setup()
{ 
  Serial.begin(115200);  // start serial for output
  Serial.println("TQED Test");
}

void ChangeAddress()
{
  Serial.println("Changing address to 0x40");
  qed.setAddress(0x40);
}

void Center()
{
  qed.centerCount();
}

void Reset()
{
  qed.resetCount();
}

void loop()
{
  //Display the counter value
  Serial.println(qed.getCount());
  
  delay(100);
}
