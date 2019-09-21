/* Receiver code for the Arduino Radio control with PWM output
 *  
 *  THIS ONLY WORKS WITH ATMEGA328p registers!!!!
 *  Install the NRF24 library to your IDE
 * Upload this code to the Arduino UNO
 * Connect a NRF24 module to it:
 
    Module // Arduino UNO
    
    GND    ->   GND
    Vcc    ->   3.3V
    CE     ->   D9
    CSN    ->   D10
    CLK    ->   D13
    MOSI   ->   D11
    MISO   ->   D12

This code receive 3 channels and create a PWM output for each one on D2, D3, D4
Please, like share and subscribe : https://www.youtube.com/channel/UCNMdGm2BwTYoPvY7Yus1QpA
*/

#include<Servo.h>
int pin2;
int pin3;
int pin4;
Servo esc;
Servo servo1;
Servo servo2;
unsigned long previousMillis = 0;                            //Set initial timer value used for the PWM signal

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
const uint64_t pipeIn = 0xE8E8F0F0E1LL;     //Remember that this code is the same as in the transmitter

RF24 radio(9, 10); 

                    //We could use up to 32 channels
struct MyData {
byte throttle;      //We define each byte of data input, in this case just 3 channels
byte pitch;
byte roll;
};

MyData data;

void resetData()
{
//We define the inicial value of each data input
//3 potenciometers will be in the middle position so 127 is the middle from 254
data.throttle = 0;
data.pitch = 130;
data.roll = 130;
}

/**************************************************/

void setup()
{Serial.begin(250000);
  esc.attach(2); // connect ESC signal pin to digital pin D2
 servo1.attach(3);//Aileron or roll servo to pin D3
servo2.attach(4);//Pitch servo to pin D4
esc.writeMicroseconds(1000);
resetData();
radio.begin();
radio.setAutoAck(false);

radio.setDataRate(RF24_250KBPS);
radio.openReadingPipe(1,pipeIn);
//we start the radio comunication
radio.startListening();
}

/**************************************************/

unsigned long lastRecvTime = 0;

void recvData()
{
while ( radio.available() ) {
radio.read(&data, sizeof(MyData));
lastRecvTime = millis(); //here we receive the data
}
}

/**************************************************/

void loop()
{
recvData();
unsigned long now = millis();
//Here we check if we've lost signal, if we did we reset the values 
if ( now - lastRecvTime > 1000 ) {
// Signal lost?
resetData();
}

pin2 = map(data.throttle, 0, 255, 1000, 2000);     //PWM value on digital pin D2
pin3 = map(data.roll,    0, 255, 30, 130);     //(between 0 to 180(in my case i have set it to vary between 30 to 130)PWM value on digital pin D3
pin4 = map(data.pitch,     0, 255, 30, 130);     //PWM value on digital pin D4

  esc.writeMicroseconds(pin2);
  delay(15);
  servo1.write(pin3);
  delay(15);
  servo2.write(pin4);
  delay(15);
  
  
  Serial.print("Throttle: "); Serial.print(pin2);  Serial.print("    ");
  Serial.print("Roll: ");     Serial.print(data.roll);      Serial.print("    ");
  Serial.print("Pitch: ");     Serial.print(data.pitch);      Serial.print("    ");
  Serial.print("\n");

/*To test if the wireless system is working coneect the receiver module to your PC,
 open the serial monitor and 
 set baud rate to 250000 
 Now move the joysticks on your transmitter 
 you should see the values changing
 */
 }


