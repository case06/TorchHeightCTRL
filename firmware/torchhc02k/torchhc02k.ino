
/*==================================================================

TorchHeightCTRL  V.02

Torchtable-Heightcontroller for Arduino-Nano,
and TB6600 Stepper-Driver, with jogwheel and cap-sense

see
http://opensourceecology.org/wiki/CNC_Torch_Table_Height_Controller
 
by Case06 21.07.2017,       CC BY SA 4.0

=====================================================================*/
 
// Aidan Williamson code from 11.07.2014
// see http://opensourceecology.org/wiki/Aidan_Williamson_Log_2014#7.2F9
//
//----------------------------------------------------------------------------------------
//|     AD7747 Capacitive Sensor                                                         |
//|   Based on code by  MiG found at:http://forum.arduino.cc/index.php/topic,11882.0.html|                                           |
//|                                                                                      |
//|  Author: Aidan Williamson (although I didn't do much original work)                  |
//|  Written: 7/3/2014                                                                   |
//|  ad7747 datasheet:http://www.analog.com/static/imported-files/data_sheets/AD7747.pdf |
//|                                                                                      |
//|                                                                                      |
//|                                                                                      |
//|                                                                                      |
//-----------------------------------------------------------------------------------------


#include <Wire.h>                  //include the library for i2c communication



// Stepper-Driver
int PUL=6; // D6, Pin9, define Pulse pin
int DIR=5; // D5, Pin8, define Direction pin
int ENA=4; // D4, Pin7, define Enable Pin


// Rotary Encoder (eg. KY-040)
// Warning: The Arduino Nano analog ports A6 and A7
// can NOT be used as digital-input-ports, like A0 to A5.
// see https://forum.arduino.cc/index.php?topic=123176.0
// and there https://forum.arduino.cc/index.php?topic=123176.msg1586523#msg1586523
// as a workaround.
int pinA = A6;  // Pin25 Connected to CLK on KY-040
int pinB = A7;  // Pin26 Connected to DT on KY-040
int pinStateA6 = 0;
int pinStateA7 = 0; 
int pinC = A3;  // Pin22 Connected to SW on KY-040  (normally used for Endstop (!!!))
int encoderPosCount = 0; 
int pinALast;  
int aVal;
boolean bCW;


// handling
int diff = 0;
int steps = 0;


// CapSense     
//int capreadybit = (bitRead(TWDR,0));  //initialize variable readybit to be two wire data register's lsb

# define RANGE 30    // Range in MilliMeters
# define TARGET 15    // Target value is 15mm above ground
# define NSAMPLES 4  // how many samples for averaging

// float offset=8.192100; // value from datasheet
float offset=8.192;
float lastcapf = 0.00;   // last measured capacitance
float mincapf = 0.00;   // lowest distance to the surface; either hand_adjusted or given as home-position from EndStop
float maxcapf = 0.00;   // widest distance

float minattof = 0.00;
float maxattof = 0.00;
//float rangeattof = 0.00;
float targetdistanceaf = 0.00;

float range = 0.00;  // the distance-range
float camm = 0.00;  // the capacitance per millimeter in attoFarad
float cammpf = 0.00;  // the capacitance per millimeter in picoFarad
int distanceSteps = 40 * RANGE;  // how many steps along the whole range; 1 step is 25 microns, so 40 steps are 1 mm
float targetdistance = 0.00;  // contains the balancing-height-value as target-capacitance in pF
float toohigh = 0.00;
float toolow = 0.00;
int upsteps = 0;
int downsteps = 0;
int loopcounter = 0;
     
float f = 0.00; 

float nsamples[NSAMPLES];

float slidav = 0.00;   // sliding average
// float fori = 0.00;
int pos_changed = 0;

    
void setup()
{
  
  //nsamples = new float[NSAMPLES];
  
  // TB6600 setup
  pinMode (PUL, OUTPUT);
  pinMode (DIR, OUTPUT);
  pinMode (ENA, OUTPUT);



  // Rotary Encoder setup

   
  pinMode (pinA,INPUT);
  pinMode (pinB,INPUT);
  pinMode (pinC,INPUT);
   
  // Read Pin A; Whatever state it's in will reflect the last position
  // pinALast = digitalRead(pinA);  
  pinStateA6 = analogRead(A6) > 100 ? 0 : 1;
  pinALast = pinStateA6;  


  Serial.begin (115200);
     
  digitalWrite(ENA,LOW);  // steppers always activated
   
  // Default step-width. When in 1/8-Microstepping mode then a 
  // value of 1 results into a movement of 25 microns step-width
  // (, but you can set it also to bigger values)
  steps = 1;
  
  
  
  // CapSense setup 
  Wire.begin();                   //sets up i2c for operation

  Wire.beginTransmission(0x48);
  Wire.write(0xBF);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(4);
  Wire.beginTransmission(0x48);   // begins write cycle
  Wire.write(0x07);              //address pointer for cap setup register
  Wire.write(0xA0);              //b'10100000' found from datasheet page 16
  Wire.endTransmission();        //ends write cycle
  delay(4);                      // Wait for data to clock out? I'm not 100% sure why this delay is here (or why it's 4ms)
  Wire.beginTransmission(0x48);   //begins transmission again
  Wire.write(0x09);              //address pointer for capacitive channel excitation register
  Wire.write(0x0E);              //recommended value from datasheet
  Wire.endTransmission();
  delay(4);  
  Wire.beginTransmission(0x48);
  Wire.write(0x0A);              //address pointer for the configuration register
  Wire.write(0x21);              //b'00100001' for continuous conversion, arbitrary VTF setting, and mid-range capacitive conversion time
  Wire.endTransmission();
  Wire.beginTransmission(0x48);
  Wire.write(0x0B);              //CAP DAC A Register address (Positive pin data)
  Wire.write(0x80);               //b'10111111' for enable Cap DAC A
  Wire.endTransmission();
  // Serial.println("Hello!");        //test to make sure serial connection is working
  
  
  // Calibration
  delay(100);
  mincapf = doSample();
  delay(100);
  mincapf = doSample();  // dont know why this must happen two times.
  minattof = (mincapf-offset)*100000;

  // digitalWrite(DIR,HIGH); // forward, clockwise
  digitalWrite(DIR,LOW); // backward (=upward), counterclockwise
  
  
  doSteps(distanceSteps);

  delay(300);
  maxcapf = doSample();
  maxattof = (maxcapf-offset)*100000;
  
  delay(100);
  range = mincapf - maxcapf;
  targetdistance = range / RANGE * TARGET;
  
  // check this
  targetdistance = maxcapf + targetdistance;  
  
  cammpf = range / RANGE;
  
  range = range * 100000; // attoFarad
  camm = range / RANGE ;
  
  
  targetdistanceaf = range / RANGE * TARGET;
  
/*
  Serial.print("min: ");
  Serial.print(mincapf,DEC);
  Serial.print("  max: ");
  Serial.println(maxcapf,DEC);

  Serial.print("min aF: ");
  Serial.print(minattof,DEC);
  Serial.print("  max aF: ");
  Serial.print(maxattof,DEC);

   Serial.print("  range: ");
  Serial.print(range,DEC);

  Serial.print(" aF ==>   aF/mm: ");
  Serial.print(camm,DEC);

  Serial.println();
  
  Serial.print("TargetCapacitance: ");
  Serial.println(targetdistance,DEC);

  Serial.print("TargetCapacitance_aF: ");
  Serial.println(targetdistanceaf,DEC);

  
  Serial.println("-----------");
*/

  loopcounter = 0;
  
  lastcapf = getNextSample();
  f= lastcapf - offset;
  f=f * 100000;  // ==> atto-Farad
  slidav = f;   // sliding average, initial value
  
  
} // end of setup()
     
     

 
 
 
int getRotaryEncoder() 
 { 
   
   int stepDiff = 0;
   
   
   // aVal = digitalRead(pinA);
   pinStateA6 = analogRead(A6) > 100 ? 0 : 1;
   aVal = pinStateA6;  
  
   
   if (aVal != pinALast)
   { // Means the knob is rotating
     // if the knob is rotating, we need to determine direction.
     // See also http://henrysbench.capnfatz.com/henrys-bench/arduino-sensors-and-input/keyes-ky-040-arduino-rotary-encoder-user-manual/
     // We do that by reading pin B.
     // if (digitalRead(pinB) != aVal) 
     // if (digitalRead(pinB) == aVal) 
     
     // if (digitalRead(pinB) == aVal) 
     pinStateA7 = analogRead(A7) > 100 ? 0 : 1;
     if (pinStateA7 == aVal) 
     {  // Means pin A Changed first - We're Rotating Clockwise
       encoderPosCount ++;
       bCW = true;
       stepDiff = 1;
     } 
     else 
     {// Otherwise B changed first and we're moving CCW
       bCW = false;
       encoderPosCount--;
       stepDiff = -1;
     }
     /*
     Serial.print ("Rotated: ");
     if (bCW) { Serial.println ("clockwise"); }
     else { Serial.println("counterclockwise"); }
     Serial.print("Encoder Position: ");
     */
//     Serial.print(encoderPosCount);
//     Serial.print("  |  ");
     
   } else stepDiff = 0; // means: knob is not rotated
   
   pinALast = aVal;
   // return encoderPosCount;
   return stepDiff;
 }  // end getRotaryEncoder() 
 

 void doSteps(int numsteps)
 {
     
    for (int i=0; i< numsteps; i++)  
    {
       digitalWrite(PUL,HIGH);
       delayMicroseconds(50);
       digitalWrite(PUL,LOW);
       delayMicroseconds(50);
     }
 } // end doSteps()
 

 
float doSample()
{ 
  
  float capf = 0.00;
  
  // CapSense with AD7747
  Wire.beginTransmission(0x48);   //talking to chip
  Wire.write(byte(0x00));                  //status register address
  Wire.endTransmission();
  Wire.requestFrom(0x48,1);       //request status register data
  int readycap;
  
  readycap=Wire.read();
  if((readycap&0x1)==0)
  {                // ready?
    Wire.beginTransmission(0x48); //arduino asks for data from ad7747
    Wire.write(0x01);             //set address point to capacitive DAC register 1
 
    Wire.endTransmission();       //pointer is set so now we can read the data
 
    Wire.requestFrom(0x48,3);     //reads data from cap DAC registers 1-3
 
    while(Wire.available())
    {
      unsigned char hi,mid,lo;      //1 byte numbers
      long capacitance;            //will be a 3byte number
      float pf;                    //scaled value of capacitance
      hi=Wire.read();
      mid=Wire.read();
      lo=Wire.read();
      capacitance=(hi<<16)+(mid<<8)+lo-0x800000;
      pf=(float)capacitance*-1/(float)0x800000*8.192f;
      //pf=pf-offset;
      //pf=pf*100000;
      capf = pf;
      Serial.print(pf, DEC); //prints the capacitance data in decimal through serial port
    }
       // Serial.println("---");
       Serial.println();
  } // end if
  
  return capf;
  
} // end doSample() 
 


float getNextSample()
{
  float total = 0.00;  
  lastcapf = 0.00;

  for(int i=0; i < NSAMPLES; i++)
  {
    nsamples[i] = 0.00;
    // lastcapf = lastcapf + doSample();
    int doitflag = 0;
    while(!doitflag)
    {
       lastcapf = doSample();
       if(lastcapf != 0)       // if sampling goes to fast doSample() returns 0
       { 
         nsamples[i] = lastcapf;
         doitflag = 1;
       } // end if 
    } // end while
    total = total + nsamples[i];
  } // end for 
  lastcapf = total / NSAMPLES; // Average
  
  return lastcapf;
} // end of getNextSample()

 
 void loop()
{

  
  lastcapf = getNextSample(); 
  
  
  
  if(lastcapf != 0)
  {
    f= lastcapf - offset;
    f=f * 100000;  // ==> atto-Farad
//    Serial.println(f);
    //fori = f;
    
    if(pos_changed == 0)
    {
      slidav = ((slidav * 3) + f) / 4;   // sliding average
      f = slidav;
    }
    else slidav = f;  // get new position for sliding average, otherwise sliding average will thinn out the next move
//    Serial.println(f);


   pos_changed = 0;
//   if(loopcounter < 9)
//   {

    if(f > (maxattof + targetdistanceaf)) // go upward
    {
//      Serial.print("Go up "); 
      toolow = f - (maxattof + targetdistanceaf);
//      Serial.println(toolow,DEC);
      if(toolow > 2.4)
      {
        upsteps = toolow / camm * 40;
//        Serial.println(upsteps);
        digitalWrite(DIR,LOW); // backward (=upward), counterclockwise
        doSteps(upsteps);
        pos_changed = 1;
      }  
    }
 
    if(f < (maxattof + targetdistanceaf)) // go downward
    {
//      Serial.print("Go down "); 
      toohigh = (maxattof + targetdistanceaf) - f;
//      Serial.println(toohigh,DEC); 
      if(toohigh > 2.4)
      {
        downsteps = toohigh / camm * 40;      
//        Serial.println(downsteps);
        digitalWrite(DIR,HIGH); // forward (=downward), clockwise
        doSteps(downsteps);
        pos_changed = 1;
                
      }
    }  
   
   loopcounter++;
   // delay(100);
// } // end if loopcounter 
  
    

  } // end if lastcapf != 0


  // Rotary Encoder    
  diff = getRotaryEncoder();
    
  if(diff != 0)
  {
    // Serial.print("Diff: ");
    // Serial.println(diff);
//    Serial.print(encoderPosCount * steps * 0.025);
//    Serial.println(" mm");
//    Serial.println("   ");
    
        
    if(diff == 1) digitalWrite(DIR,HIGH); // forward, clockwise
    else digitalWrite(DIR,LOW); // backward, counterclockwise
    
    doSteps(steps);
 
  } // else: do nothing
    
 } // end loop()
 
 
 
