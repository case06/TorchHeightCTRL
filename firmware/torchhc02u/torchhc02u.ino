
/*==================================================================

TorchHeightCTRL  V.02

Torchtable-Heightcontroller for Arduino-Nano,
and TB6600 Stepper-Driver, with jogwheel and cap-sense

see
http://opensourceecology.org/wiki/CNC_Torch_Table_Z_Height_Control
http://opensourceecology.org/wiki/CNC_Torch_Table_Height_Controller
 
by Case06 10.08.2017,       CC BY SA 4.0

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
// int pinC = A3;  // Pin22 Connected to SW on KY-040  (normally used for Endstop (!!!))
int pinKnob = 12;  // D12 Connected to SW on KY-040  (Workaround, this pin is normally used by UEXT expansion port (!!!))
int encoderPosCount = 0; 
int pinALast;  
int aVal;
boolean bCW;


// EndStop
int pinEndStop = A3;
int endstop = 1;  // 1 = open , 0 = closed contact
int goDir = 0;  // Direction to go:  0 = upward,  1 = downward



// handling
int diff = 0;
int steps = 0;
int switchmode = 0;   // toggles between 0 = manual mode and 1 = autobalancing mode

// verbose-mode toggles debug-messages on the serial monitor
// 0 = silent
// 1 = output only the raw-data, for host-monitor-tool
// 2 = setup-values (calibration etc.)
// 3 = absolute position and target working-height
// 4 = balancing data
// 9 = all messages
int verbose = 2;    



// CapSense     
//int capreadybit = (bitRead(TWDR,0));  //initialize variable readybit to be two wire data register's lsb

# define RANGE 30.0000000000    // Range in MilliMeters
# define TARGET 15.0000000000    // Default working height Target value is 15mm above ground
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
float target = TARGET;

float range = 0.00;  // the distance-range in attoFarad
float camm = 0.00;  // the capacitance per millimeter in attoFarad



// Movements

int distanceSteps = 40 * RANGE;  // how many steps along the whole range; 1 step is 25 microns, so 40 steps are 1 mm
// float targetdistance = 0.00;  // contains the balancing-height-value as target-capacitance in pF
float toohigh = 0.00;
float toolow = 0.00;
int upsteps = 0;
int downsteps = 0;
float f = 0.00; 
float nsamples[NSAMPLES];
float slidav = 0.00;   // sliding average
// float fori = 0.00;
int pos_changed = 0;

float pos = 0.00; // current height position in mm
float distance2go = 0.00;;
int steps2go = 0;  // 1 step is 25 microns, so 1mm is 40 steps



    
void setup()
{
  
  delay(3000);
  
  //nsamples = new float[NSAMPLES];
  
  // TB6600 setup
  pinMode (PUL, OUTPUT);
  pinMode (DIR, OUTPUT);
  pinMode (ENA, OUTPUT);

  // Rotary Encoder setup
  pinMode (pinA,INPUT);
  pinMode (pinB,INPUT);
  pinMode (pinKnob,INPUT_PULLUP); // Knob
//  pinMode (pinKnob,INPUT); // Knob
  // EndStop setup
  pinMode (pinEndStop,INPUT);
  
   
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
  
/*  
  
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
  
  
  digitalWrite(DIR,HIGH); // go downward for homing
  goDir = 1;
  
  // ======
  // Homing
  // ======
  if((verbose == 2) || (verbose == 3)) Serial.print("Start Homing ..."); 
  while(endstop == 1)
  {
    doSteps(1);
    delay(10);  // dont know why this delay is needed ... but it is.
  }
  if((verbose == 2) || (verbose == 3)) Serial.println("... Homing done !"); 
  

  // ===========    
  // Calibration
  // ===========
  if((verbose == 2) || (verbose == 3)) Serial.print("Start Calibration ..."); 
  delay(100);
  pos = 0.00;   // initial position after homing
  target = TARGET; // initial default working height after homing
  mincapf = doSample();
  delay(100);
  mincapf = doSample();  // dont know why this must happen two times.
  minattof = (mincapf-offset)*100000;


  digitalWrite(DIR,LOW); // backward (=upward), counterclockwise
  goDir = 0;
  
  // Serial.println(endstop); 
  
  doSteps(distanceSteps);

  delay(300);
  pos = 30.00;  // max-position
  target = TARGET; // initial default working height after homing
  maxcapf = doSample();
  maxattof = (maxcapf-offset)*100000;
  delay(100);
  if((verbose == 2) || (verbose == 3)) Serial.println("... Calibration done !"); 
  
  
  // =================================
  // Go back to default working height
  // =================================

  target = TARGET;
  if((verbose == 2) || (verbose == 3) || (verbose == 9))
  {
    Serial.println("Go down to default working height: ");
    Serial.print(target,3);

    Serial.print(" from pos: ");
    Serial.println(pos,3);
  }
  
  digitalWrite(DIR,HIGH); // forward (= downward)
  goDir = 1;

  distance2go = pos - target;
  distance2go = distance2go * 40;  // 1 step is 25 microns, so 1mm is 40 steps
  // steps2go = (int) distance2go;  // When casting from a float to an int, the value is truncated not rounded.  (!)
  steps2go = round(distance2go);
  
  if((verbose == 2) || (verbose == 3) || (verbose == 9))
  {
    Serial.print("  ==> calculated steps: ");
    Serial.println(steps2go);
  }

  doSteps(steps2go);
  target = TARGET;
  verbose = 3;
  
  
  
  
  
  // ============================
  // Calculate calibration values
  // ============================
  range = mincapf - maxcapf;
  // // targetdistance = range / RANGE * target;
  // // targetdistance = maxcapf + targetdistance;  
  range = range * 100000; // attoFarad
  targetdistanceaf = range / RANGE * target;
   
  camm = range / RANGE ; 

  if((verbose == 2) || (verbose == 3))
  { 
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
  
    // Serial.print("TargetCapacitance: ");
    // Serial.println(targetdistance,DEC);

    Serial.print("TargetCapacitance_aF: ");
    Serial.println(targetdistanceaf,DEC);

    Serial.print("target/mm: ");
    Serial.println(target,3);
  
    Serial.println("-----------");
  }





  // =====================================
  // Set initial value for sliding average
  // =====================================
    
  lastcapf = getNextSample();
  f= lastcapf - offset;
  f=f * 100000;  // ==> atto-Farad
  slidav = f;   // sliding average, initial value
  
  */
  
  switchmode = 0;
  
} // end of setup()
     
     
/*
float targetdistance_in_aF()
{
  // range = mincapf - maxcapf;
  // range = range * 100000; // attoFarad
  targetdistanceaf = range / RANGE * target;
  return targetdistanceaf;
}  
*/

 
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
       // check EndStop
       endstop = digitalRead(pinEndStop);
        
       if((endstop == 1) || ((endstop == 0) && (goDir == 0)))
       {
         digitalWrite(PUL,HIGH);
         delayMicroseconds(50);
         digitalWrite(PUL,LOW);
         delayMicroseconds(50);
         
         // update positioncounter
         if(goDir == 0)  // go upward
         {
           pos = pos + 0.0250000000;
           if(switchmode == 0) target = target + 0.0250000000;
         }  
         else   // go downward
         {  
           pos = pos - 0.0250000000;
           if(switchmode == 0) target = target - 0.0250000000;
         }  
         pos_changed = 1;  
           
       }
       else pos = 0.0000000000;  // end if endstop  
       
       
       if((verbose == 3) || (verbose == 9))
       {
         Serial.print("pos: ");
         Serial.print(pos,3);
         Serial.print("  target: ");
         Serial.println(target,3);
       }
       // else Serial.println("condition not fullfilled");
    } // end for
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
      if((verbose == 1) || (verbose == 9)) Serial.print(pf, DEC); //prints the capacitance data in decimal through serial port
    }
       // Serial.println("---");
       if((verbose == 1) || (verbose == 9)) Serial.println();
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

  // check EndStop
  endstop = digitalRead(pinEndStop);
   // Serial.println(endstop);
  
  
  // check switchmode
  if(digitalRead(pinKnob) == LOW)  // Knob pressed
  {
    delay(300);  // for debouncing. maybe better: https://github.com/mathertel/OneButton
    // toggle switchmode
    if(switchmode == 1) switchmode = 0; 
    else switchmode = 1; 
    
    if((verbose == 3) || (verbose == 9))
    {
      Serial.print("Button pressed, new switchmode : ");
      Serial.println(switchmode);
    }
  } 
  
  
// auto-balancing and capsensing-mode  
if(switchmode == 1)
{
  
  delay(10); 
  // interesting: a bigger delay here doesnt necessarily lead to better repeatability-correctness; 
  // maybe holding(-torque) of stepper induces more noise than while running ?
  // Or there are maybe motor-positions which induce less noise than others ?
  lastcapf = getNextSample(); 
  
  
  
  if(lastcapf != 0)
  {
    f= lastcapf - offset;
    f=f * 100000;  // ==> atto-Farad
    
    if(pos_changed == 0)
    {
      slidav = ((slidav * 3) + f) / 4;   // sliding average
      f = slidav;
    }
    else slidav = f;  // get new position for sliding average, otherwise sliding average will thinn out the next move

    pos_changed = 0;


    targetdistanceaf = range / RANGE * target;  // range is capacitance-range measured during calibration
    
    if((verbose == 3) || (verbose == 4) || (verbose == 9))
    {
      Serial.print("pos: ");
      Serial.println(pos,3);
      Serial.print("target: ");
      Serial.println(target,3);
      
      // Serial.print("targetdistanceaf: ");
      // Serial.println(targetdistanceaf,DEC);

      // Serial.print("maxattof: ");
      // Serial.println(maxattof,DEC);

      // Serial.print("minattof: ");
      // Serial.println(minattof,DEC);

      // Serial.print("   |   calc: ");
      Serial.print("calc: ");
      // Serial.print("   |   target-capacity (calculated): ");
      // Serial.println(maxattof + targetdistanceaf,DEC);
      Serial.println(minattof - targetdistanceaf,3);
      
      // Serial.print( "  measured f: ");
      Serial.print( "sample: ");
      Serial.println(f,3);
      Serial.println( "---");
    }


    // if(f > (maxattof + targetdistanceaf)) // go upward
    if(f > (minattof - targetdistanceaf)) // go upward
    {
      // toolow = f - (maxattof + targetdistanceaf);
      toolow = f - (minattof - targetdistanceaf);

      
      if((verbose == 3) || (verbose == 4) || (verbose == 9))
      {
        Serial.print("Go up "); 
        Serial.println(toolow,DEC);
      }  
      
      if(toolow > 2.4)
      {
        upsteps = toolow / camm * 40;
        if((verbose == 3) || (verbose == 4) || (verbose == 9)) Serial.println(upsteps);
        digitalWrite(DIR,LOW); // backward (=upward), counterclockwise
        goDir = 0;
        doSteps(upsteps);
        // pos_changed = 1;
      }  
    }
 
    // if(f < (maxattof + targetdistanceaf)) // go downward
    if(f < (minattof - targetdistanceaf)) // go downward
    {
      // toohigh = (maxattof + targetdistanceaf) - f;
      toohigh = (minattof - targetdistanceaf) - f;
      
      if((verbose == 3) || (verbose == 4) || (verbose == 9))
      {
        Serial.print("Go down "); 
        Serial.println(toohigh,DEC);
      }  
      
      if(toohigh > 2.4)
      {
        downsteps = toohigh / camm * 40;      
        if((verbose == 3) || (verbose == 4) || (verbose == 9))  Serial.println(downsteps);
        digitalWrite(DIR,HIGH); // forward (=downward), clockwise
        goDir = 1;
        doSteps(downsteps);
        // pos_changed = 1;
                
      }
    }  
   
 
  } // end if lastcapf != 0
  
} // end if switchmode == 1



// manual mode
if(switchmode == 0)
{

  // Rotary Encoder    
  diff = getRotaryEncoder();
    
  if(diff != 0)
  {
    // Serial.print("Diff: ");
    // Serial.println(diff);
//    Serial.print(encoderPosCount * steps * 0.025);
//    Serial.println(" mm");
//    Serial.println("   ");
    
        
    if(diff == 1) 
    {
      digitalWrite(DIR,HIGH); // forward/downward, clockwise
      goDir = 1;
      // target = target - 0.025;
    }  
    else
    {
      digitalWrite(DIR,LOW); // backward/upward, counterclockwise
      goDir = 0;
      // target = target + 0.025;
    }  
    
    doSteps(steps);
  //delay(100);
  
  } // else: do nothing
 
} // end if switchmode == 0 
 
 
 
    
 } // end loop()
 
 
 
