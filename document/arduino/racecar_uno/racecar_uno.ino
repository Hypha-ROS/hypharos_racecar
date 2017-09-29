#include <Servo.h> 
Servo servo;  // create servo object to control a servo 
Servo motor;
int out_1 ; //getValue func should be removed (TBD) 
int out_2 ;
String check=0 ;
//int out_4=0;
int led = 13;
int reversing_flag=0;
int servovalue=0;
int motorvalue=0;

unsigned long time=0;
unsigned long time2;
unsigned long time3;
void setup() 
{ 
 Serial.begin(57600);
  while (!Serial) {
  }
  Serial.setTimeout(20); // remember to set timeout, default is 1000ms
  servo.attach(9);  // attaches the servo on pin 9 to the servo object 
  motor.attach(3);  // attaches the ESC   on pin 3 to the servo object 
  servo.write(90);
  motor.writeMicroseconds(1500);
  pinMode(led, OUTPUT);
  digitalWrite(led,LOW);
  digitalWrite(led,HIGH);
} 

void loop() {
    if (Serial.available()) {
      String commad_string = Serial.readString(); // alternative: read() or readBytes()
      out_1 = getValue(commad_string, ',', 0).toInt(); //getValue func should be removed (TBD) 
      out_2 = getValue(commad_string, ',', 1).toInt();
      check= getValue(commad_string, ',', 2);
      if(check.toInt() != (commad_string.length()-2-check.length()) )
      {
        //WL_ref = 0.0;
        //WR_ref = 0.0;
        return;
      } 
      
      
     // int out_4 = getValue(commad_string, ',', 3).toInt();
       if(out_1==0 || out_2==0) 
       {
         servovalue=90;
         motorvalue=1520;
       }
       else if  (out_2>1650 || out_2<1430)
       {
         servovalue=90;
         motorvalue=1520;
       }
       else if  (out_2>1550   &&  reversing_flag==0)
       {
         servo.write(out_1);
         
         motor.writeMicroseconds(1520);
         time3=millis();
         while ((millis()-time3)<30)
         {
          // Serial.println("re2");
          motor.writeMicroseconds(1520);
         }
         
         
         motor.writeMicroseconds(1600);
         time3=millis();
         while ((millis()-time3)<30)
         {
           //digitalWrite(led,LOW);
           motor.writeMicroseconds(1600);
         }
         
         motor.writeMicroseconds(1520);
         time3=millis();
         while ((millis()-time3)<50)
         {
          // Serial.println("re2");
          motor.writeMicroseconds(1520);
         }
         //digitalWrite(led,LOW);
         //servo.write(out_1);
         //motor.writeMicroseconds(out_2);
         servovalue=out_1;
         motorvalue=out_2;
         reversing_flag=1;
         //Serial.println("re_fin");
         
       }
       else if (out_2>=1500   &&  reversing_flag==1)
       {
         servovalue=out_1;
         motorvalue=out_2;
       }
       else 
       {
         servovalue=out_1;
         motorvalue=out_2;
       reversing_flag=0;
       }
       time=millis();
      //Serial.println(out_1);
    }

    if (reversing_flag==1)
    {
      digitalWrite(led,LOW);
    }
    else
    {
      digitalWrite(led,HIGH);
    }
    time2=millis();
    if ((time2-time)>2000){
    servovalue=90;
    motorvalue=1520;
    //Serial.println(servovalue);
    } 
    servo.write(servovalue);
    motor.writeMicroseconds(motorvalue);
}
    

    
    
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}
