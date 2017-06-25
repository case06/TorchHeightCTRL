
/*==================================================================

TorchTable-manual-HeightCTRL  V.1b

Manual Torchtable-Heightcontroller for Arduino-Mega,
Ramps 1.4 and TB6600 Stepper-Driver

see
http://opensourceecology.org/wiki/CNC_Torch_Table_Height_Controller
 
by Case06 06/2017,       CC BY SA 4.0

=====================================================================*/


// Stepper-Driver
int PUL=26; //define Pulse pin
int DIR=28; //define Direction pin
int ENA=24; //define Enable Pin


// Rotary Encoder
int pinA = 40;  // Connected to CLK on KY-040
int pinB = 42;  // Connected to DT on KY-040
int encoderPosCount = 0; 
int pinALast;  
int aVal;
boolean bCW;

// handling
int diff = 0;
int steps = 0;
 


 void setup() 
 { 

  pinMode (PUL, OUTPUT);
  pinMode (DIR, OUTPUT);
  pinMode (ENA, OUTPUT);

   pinMode (pinA,INPUT);
   pinMode (pinB,INPUT);
   
   // Read Pin A; Whatever state it's in will reflect the last position
   pinALast = digitalRead(pinA);  
   
   Serial.begin (115200);
     
   digitalWrite(ENA,LOW);  // steppers always activated
   
   // Default step-width. When in 1/8-Microstepping mode then a 
   // value of 1 results into a movement of 25 microns step-width
   // (, but you can set it also to bigger values)
   steps = 1;  
 } 


 int getRotaryEncoder() 
 { 
   
   int stepDiff = 0;
   
   aVal = digitalRead(pinA);
   if (aVal != pinALast)
   { // Means the knob is rotating
     // if the knob is rotating, we need to determine direction.
     // See also http://henrysbench.capnfatz.com/henrys-bench/arduino-sensors-and-input/keyes-ky-040-arduino-rotary-encoder-user-manual/
     // We do that by reading pin B.
     // if (digitalRead(pinB) != aVal) 
     if (digitalRead(pinB) == aVal) 
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
     Serial.print(encoderPosCount);
     Serial.print("  |  ");
     
   } else stepDiff = 0; // means: knob is not rotated
   
   pinALast = aVal;
   // return encoderPosCount;
   return stepDiff;
 } 


void loop() 
{ 

  diff = getRotaryEncoder();
    
  if(diff != 0)
  {
    // Serial.print("Diff: ");
    // Serial.println(diff);
    Serial.print(encoderPosCount * steps * 0.025);
    Serial.println(" mm");
    Serial.println("   ");
    
        
    if(diff == 1) digitalWrite(DIR,HIGH); // forward, clockwise
    else digitalWrite(DIR,LOW); // backward, counterclockwise
    
    
    for (int i=0; i<steps; i++)  
    {
       digitalWrite(PUL,HIGH);
       delayMicroseconds(50);
       digitalWrite(PUL,LOW);
       delayMicroseconds(50);
     }
     

  } // else: do nothing
    
} // end loop()


