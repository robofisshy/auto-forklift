#include <Wire.h>
int DANGER=4;
int WARRING_1=5;
int WARRING_2=6;
int CASE_1=7;
int CASE_2=8;
String str="";

struct Security{
  char radar_forward;
  char radar_backward;
  char cl_switch;
}secu;


void setup()
{
  Wire.begin(0x04);   			//4号I2C设备
  Wire.onRequest(requestEvent); 
  Wire.onReceive(receiveEvent);
  Serial.begin(9600);
  pinMode(DANGER,INPUT);
  pinMode(WARRING_1,INPUT);
  pinMode(WARRING_2,INPUT);
  pinMode(CASE_1,OUTPUT);
  pinMode(CASE_2,OUTPUT);
  secu.radar_forward='n';
  secu.radar_backward='n';
  secu.cl_switch='n';
}
 
void loop()
{
  if((digitalRead(DANGER)==LOW)&&(digitalRead(WARRING_1)==LOW))
  {
    secu.radar_forward='s';        
  }
  if((digitalRead(DANGER)==LOW)&&(digitalRead(WARRING_1)==HIGH))
  {
    secu.radar_forward='w';
  }
  if((digitalRead(DANGER)==HIGH)&&(digitalRead(WARRING_1)==HIGH))
  {
    secu.radar_forward='d';
  }
}
 

void receiveEvent(int howMany)
{
  str="";
  Serial.println("hello_r");
 while(Wire.available()>0)
 {
  str+=char(Wire.read());
 }
}

void requestEvent()
{
  Serial.println("hello_w");
  char retState[sizeof(secu)];
  memset(retState,'n',sizeof(secu));
  memcpy(retState,&secu,sizeof(secu));
 Serial.println(str);
 if(str=="security_state")
 {
   Serial.println(secu.radar_forward);
  Wire.write((uint8_t *)retState,sizeof(secu));
 }
}

