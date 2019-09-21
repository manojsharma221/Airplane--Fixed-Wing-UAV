#include <Wire.h>
#include <Servo.h>
//#include<EEPROM.h>
          //RFID
Servo esc;
Servo pitch_servo;
Servo yaw_servo;
Servo roll_servoL;
Servo roll_servoR;

unsigned long previousMillis = 0;
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
const uint64_t pipeIn = 0xE8E8F0F0E1LL;     //Remember that this code is the same as in the transmitter

RF24 radio(9, 10); 

struct MyData {
byte throttle;      //We define each byte of data input, in this case just 3 channels
byte pitch;
byte roll;
byte yaw;
byte TPU;
byte TPD;
byte TRR;
byte TRL;
};
MyData data;

void resetData()
{
//We define the inicial value of each data input
//3 potenciometers will be in the middle position so 127 is the middle from 254
data.throttle = 0;
data.pitch = 130;
data.roll = 130;
data.yaw=130;
data.TPU=0;
data.TPD=0;
data.TRR=0;
data.TRL=0;
}






/*MPU-6050 gives you 16 bits data so you have to create some 16int constants
 * to store the data for accelerations and gyro*/

int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
 

float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];



float elapsedTime, time, timePrev;
float rad_to_deg = 180/3.141592654;

float errorpitch,errorroll,pwmpitch, pwmroll,PIDpitch,PIDroll, previous_errorpitch,previous_errorroll;
float pidpitch_p=0;
float pidpitch_i=0;
float pidpitch_d=0;

float pidroll_p=0;
float pidroll_i=0;
float pidroll_d=0;

/////////////////PID CONSTANTS/////////////////
double kp=1.5;//3.55
double ki=0;//0.003
double kd=0;;//2.05
///////////////////////////////////////////////

int throttle,yaw;
double pitch;
double roll;
/* TRIM CONSTANTS*/
float tpu,tpd,trr,trl,pitchTrim,rollTrim;
float desired_angle = 0; //This is the angle in which we whant the
                         //balance to stay steady



void setup() {
  Wire.begin(); //begin the wire comunication
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(250000);
  esc.attach(2);
  roll_servoL.attach(3);
  roll_servoR.attach(4);
   pitch_servo.attach(5);
   yaw_servo.attach(6);
  esc.writeMicroseconds(1000);
  delay(2000);
   //pitchTrim=EEPROM.read(2);
   //yawTrim=EEPROM.read(100);
 resetData();
radio.begin();
radio.setAutoAck(false);

radio.setDataRate(RF24_250KBPS);
radio.openReadingPipe(1,pipeIn);
//we start the radio comunication
radio.startListening();
  time = millis(); //Start counting time in milliseconds
}//end of setup void

unsigned long lastRecvTime = 0;

void recvData()
{
while ( radio.available() ) {
radio.read(&data, sizeof(MyData));
lastRecvTime = millis();
}
}


void loop() {
  recvData();

/////////////////////////////I M U/////////////////////////////////////
      timePrev = time;  // the previous time is stored before the actual time read
    time = millis();  // actual time read
    elapsedTime = (time - timePrev) / 1000; 

    if ( time - lastRecvTime > 1000 ) {
// Signal lost?
resetData();
}

//Here we check if we've lost signal, if we did we reset the values 

throttle = map(data.throttle, 0, 255, 1000, 2000);     //PWM value on digital pin D2
pitch = map(data.pitch,    0, 255, 30, 140);     //PWM value on digital pin D3
roll = map(data.roll,     0, 255, 30, 140);     //PWM value on digital pin D5
yaw = map(data.yaw,     0, 255, 30, 140);
tpu=data.TPU;
tpd=data.TPD;
trr=data.TRR;
trl=data.TRL;

/*TRIMMING*/
if (tpu==1)
{
  pitchTrim=pitchTrim+0.1;
  if(pitchTrim>50)
  {
    pitchTrim=50;
  }
 // EEPROM.write(2,pitchTrim);
}
if(tpd==1)
{
  pitchTrim=pitchTrim-0.1;
 if(pitchTrim<-50)
 {
  pitchTrim=-50;
 }
 // EEPROM.write(2,pitchTrim);
}
if (tpu==0||tpd==0)
{
  pitchTrim=pitchTrim+0;
}

if(trr==1)
{
  rollTrim=rollTrim+0.1;
  if(rollTrim>50)
  {
    rollTrim=50;
  }
 // EEPROM.write(100,yawTrim);
}
if(trl==1)
{
  rollTrim=rollTrim-0.1;
  if(rollTrim<-50)
  {
    rollTrim=-50;
  }
//  EEPROM.write(100,yawTrim);
}
if(trr==0||trl==0)
{
  rollTrim=rollTrim+0;
}

//Serial.print(pitchTrim);Serial.print("    ");Serial.println(rollTrim);
  
  /*The tiemStep is the time that elapsed since the previous loop. 
   * This is the value that we will use in the formulas as "elapsedTime" 
   * in seconds. We work in ms so we haveto divide the value by 1000 
   to obtain seconds*/

  /*Reed the values that the accelerometre gives.
   * We know that the slave adress for this IMU is 0x68 in
   * hexadecimal. For that in the RequestFrom and the 
   * begin functions we have to put this value.*/
   
     Wire.beginTransmission(0x68);
     Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
     Wire.endTransmission(false);
     Wire.requestFrom(0x68,6,true); 
   
   /*We have asked for the 0x3B register. The IMU will send a brust of register.
    * The amount of register to read is specify in the requestFrom function.
    * In this case we request 6 registers. Each value of acceleration is made out of
    * two 8bits registers, low values and high values. For that we request the 6 of them  
    * and just make then sum of each pair. For that we shift to the left the high values 
    * register (<<) and make an or (|) operation to add the low values.*/
    
     Acc_rawX=Wire.read()<<8|Wire.read(); //each value needs two registres
     Acc_rawY=Wire.read()<<8|Wire.read();
     Acc_rawZ=Wire.read()<<8|Wire.read();

 
    /*///This is the part where you need to calculate the angles using Euler equations///*/
    
    /* - Now, to obtain the values of acceleration in "g" units we first have to divide the raw   
     * values that we have just read by 16384.0 because that is the value that the MPU6050 
     * datasheet gives us.*/
    /* - Next we have to calculate the radian to degree value by dividing 180º by the PI number
    * which is 3.141592654 and store this value in the rad_to_deg variable. In order to not have
    * to calculate this value in each loop we have done that just once before the setup void.
    */

    /* Now we can apply the Euler formula. The atan will calculate the arctangent. The
     *  pow(a,b) will elevate the a value to the b power. And finnaly sqrt function
     *  will calculate the rooth square.*/
     /*---X---*/
     Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
     /*---Y---*/
     Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
 
   /*Now we read the Gyro data in the same way as the Acc data. The adress for the
    * gyro data starts at 0x43. We can see this adresses if we look at the register map
    * of the MPU6050. In this case we request just 4 values. W don¡t want the gyro for 
    * the Z axis (YAW).*/
    
   Wire.beginTransmission(0x68);
   Wire.write(0x43); //Gyro data first adress
   Wire.endTransmission(false);
   Wire.requestFrom(0x68,4,true); //Just 4 registers
   
   Gyr_rawX=Wire.read()<<8|Wire.read(); //Once again we shif and sum
   Gyr_rawY=Wire.read()<<8|Wire.read();
 
   /*Now in order to obtain the gyro data in degrees/seconda we have to divide first
   the raw value by 131 because that's the value that the datasheet gives us*/

   /*---X---*/
   Gyro_angle[0] = Gyr_rawX/131.0; 
   /*---Y---*/
   Gyro_angle[1] = Gyr_rawY/131.0;

   /*Now in order to obtain degrees we have to multiply the degree/seconds
   *value by the elapsedTime.*/
   /*Finnaly we can apply the final filter where we add the acceleration
   *part that afects the angles and ofcourse multiply by 0.98 */

   /*---X axis angle---*/
   Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
   /*---Y axis angle---*/
   Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];
   
   /*Now we have our angles in degree and values from -100º to 100º aprox*/
    //Serial.println(Total_angle[0])  ;   Serial.println(Total_angle[1]);

   
  
/*///////////////////////////P I D///////////////////////////////////*/
/*Remember that for the balance we will use just one axis. I've choose the x angle
to implement the PID with. That means that the x axis of the IMU has to be paralel to
the balance*/

/*First calculate the error between the desired angle and 
*the real measured angle*/
errorpitch = Total_angle[1] - desired_angle;
errorroll = Total_angle[0] - desired_angle;

    
/*Next the proportional value of the PID is just a proportional constant
*multiplied by the error*/

pidpitch_p = kp*errorpitch;
pidroll_p=kp*errorroll;
/*The integral part should only act if we are close to the
desired position but we want to fine tune the error. That's
why I've made a if operation for an error between -2 and 2 degree.
To integrate we just sum the previous integral value with the
error multiplied by  the integral constant. This will integrate (increase)
the value each loop till we reach the 0 point*/
if(-3 <errorpitch <3)
{
  pidpitch_i = pidpitch_i+(ki*errorpitch);  
}

if(-3 <errorroll <3)
{
  pidroll_i = pidroll_i+(ki*errorroll);  
}

/*The last part is the derivate. The derivate acts upon the speed of the error.
As we know the speed is the amount of error that produced in a certain amount of
time divided by that time. For taht we will use a variable called previous_error.
We substract that value from the actual error and divide all by the elapsed time. 
Finnaly we multiply the result by the derivate constant*/

pidpitch_d = kd*((errorpitch - previous_errorpitch)/elapsedTime);

pidroll_d = kd*((errorroll - previous_errorroll)/elapsedTime);

/*The final PID values is the sum of each of this 3 parts*/
PIDpitch = pidpitch_p + pidpitch_i + pidpitch_d;

PIDroll = pidroll_p + pidroll_i + pidroll_d;


/*We know taht the min value of PWM signal is 1000us and the max is 2000. So that
tells us that the PID value can/s oscilate more than -1000 and 1000 because when we
have a value of 2000us the maximum value taht we could sybstract is 1000 and when
we have a value of 1000us for the PWM sihnal, the maximum value that we could add is 1000
to reach the maximum 2000us*/
if(PIDpitch < -140)
{
  PIDpitch=-140;
}
if(PIDpitch > 140)
{
  PIDpitch=140;
}

if(PIDroll < -140)
{
  PIDroll=-140;
}
if(PIDroll > 140)
{
  PIDroll=140;
}

/*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/
pwmpitch = pitch - PIDpitch-pitchTrim;
pwmroll = roll - PIDroll+rollTrim;


/*Once again we map the PWM values to be sure that we won't pass the min
and max values. Yes, we've already maped the PID values. But for example, for 
throttle value of 1300, if we sum the max PID value we would have 2300us and
that will mess up the ESC.*/
//Right
if(pwmpitch < 30)
{
  pwmpitch= 30;
}
if(pwmpitch > 140)
{
  pwmpitch=140;
}
//Left
if(pwmroll < 30)
{
  pwmroll= 30;
}
if(pwmroll > 140)
{
  pwmroll=140;
}

/*Finnaly using the servo function we create the PWM pulses with the calculated
width for each pulse*/
esc.writeMicroseconds(throttle);
pitch_servo.write(pwmpitch);
yaw_servo.write(yaw);
roll_servoL.write(140-pwmroll+30);
roll_servoR.write(140-pwmroll+30);
previous_errorpitch = errorpitch; //Remember to store the previous error.
previous_errorroll = errorroll;
Serial.print("PitchTrim: ");  Serial.print(pitchTrim);      Serial.print("    ");
Serial.print("RollTrim: ");   Serial.print(rollTrim);       Serial.print("    ");
Serial.print("AnglePitch");   Serial.print(errorpitch); Serial.print("    ");
Serial.print("AngleRoll");    Serial.print(errorroll); Serial.print("    ");
Serial.print("Pitch");    Serial.print(pitch);       Serial.print("    ");
Serial.print("Roll");     Serial.print(roll);        Serial.print("    ");
Serial.print("PWM Pitch");    Serial.print(pwmpitch);       Serial.print("    ");
Serial.print("PWN Roll");     Serial.print(pwmroll);        Serial.print("    ");
Serial.print("\n");
  
}
//end of loop void
