/*A basic 4 channel transmitter using the nRF24L01 module.*/
/* Like, share and subscribe, https://www.youtube.com/channel/UCNMdGm2BwTYoPvY7Yus1QpA  */
/* Channel Name  Manoj Sharma...(yes, I know I need to change it. */

/* First include the libraries.Google and  Download it. 
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
 */
 
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

/*Create a unique pipe out. The receiver has to 
  wear the same unique code*/
  
const uint64_t pipeOut = 0xE8E8F0F0E1LL; //IMPORTANT: The same as in the receiver

RF24 radio(9, 10); // select  CSN  pin

// The sizeof this struct should not exceed 32 bytes
// This gives us up to 32 8 bits channals
struct MyData {
  byte throttle;
  byte pitch;
  byte roll;   // we will be only using 3 channels Throttle, one servo for roll and one for pitch control.
  };

MyData data;

void resetData() 
{
  //This are the start values of each channal
  // Throttle is 0 in order to stop the motors
  //127 is the middle value of the 10ADC.
    
  data.throttle = 0;
  data.pitch = 127;
  data.roll = 127;
   
 }

void setup()
{
  //Start everything up
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(pipeOut);
  radio.setPALevel(RF24_PA_MAX); //if the connection does'nt work change it to LOW or MIN( options LOW,MIN,HIGH,MAX)
  radio.stopListening();
  resetData();
 }

/**************************************************/

// Returns a corrected value for a joystick position that takes into account
// the values of the outer extents and the middle of the joystick range.
int mapJoystickValues(int val, int lower, int middle, int upper, bool reverse)
{
  val = constrain(val, lower, upper);
  if ( val < middle )
    val = map(val, lower, middle, 0, 128);
  else
    val = map(val, middle, upper, 128, 255);
  return ( reverse ? 255 - val : val );
}

void loop()
{
  // The calibration numbers used here should be measured 
  // for your joysticks till they send the correct values.
  data.throttle = mapJoystickValues( analogRead(A0), 13, 524, 1015, true );//connect the middle pin of the throttle potentionmeter to analog pin A0 and the other two pins to ground and +5v or arduino
  data.roll      = mapJoystickValues( analogRead(A1), 12, 544, 1021, true );//do same procedure here also(anolog pin A1)
  data.pitch    = mapJoystickValues( analogRead(A2), 34, 522, 1020, true );//and here too(Analog pin A2)
  radio.write(&data, sizeof(MyData));
}
