
import processing.opengl.*;
import processing.serial.*;
import java.io.*;
import java.util.*;

// The serial port:
Serial arduinoPort;  
Thread serialThread;
int testMode = 0;
//line delimiter
int lf = 10;


int[] y;
String val;     // Data received from the serial port
float offset=8.192100;
float slidav = 0.00;  // sliding average

int viewfactor = 1;

void setup() {
  // size(viewWidth, viewHeight, OPENGL);
  size(1200, 600, OPENGL);
    smooth();
  //noStroke();

  // font = loadFont("LucidaGrande-48.vlw");
  y = new int[width];

  println(Serial.list());
  arduinoPort = new Serial(this, Serial.list()[4], 115200);
  
  
  
  
    
} // end of setup():    







void draw() 
{

  
  
  background(0); // Read the array from the end to the
  
    
  if ( arduinoPort.available() > 0) 
  {  // If data is available,
    val = arduinoPort.readStringUntil('\n');         // read it and store it in val
  } 
  println(val); //print it out in the console

  float f = float(val);
  
  // f=f - offset;
  // f=f * 100000;  // ==> milli-femto-Farad
  
  slidav = ((slidav * 3) + f) / 4;   // sliding average
  f = slidav;
  
  println(f);
  
  // beginning to avoid overwriting the data
  for (int i = y.length-1; i > 0; i--) 
  {
    y[i] = y[i-1];
  }
  // Add new values to the beginning
  //y[0] = round(f) * 4;
  y[0] = round(f) * viewfactor;
  
  println(y[0]);
  
  
 println(pixelWidth, pixelHeight);
 
  
   // draw grid
  // stroke(20, 200, 100); hellgruentuerkis
  stroke(5, 50, 25);
  // stroke(10, 100, 50);
  
  for (int i = 1; i < 58; i++) 
  {
    
    // dottedLine(0, i*10*4, width-1, i*10*4,20);
    line(0, i*10*viewfactor, width-1, i*10*viewfactor);
    text(i*10, 10, i*10*viewfactor);
  }
 text("[mfF]", 10, 590);
 
  
  
  // draw curve 
  stroke(204, 102, 0); //karminocker
  stroke(255, 194, 0); //amber
    
  // Display each pair of values as a line
  for (int i = 1; i < y.length; i++) 
  {
    
    line(i, y[i], i-1, y[i-1]);
  }
  
 
  
  // display values
  text(f, 100, 590);
  text("mfF", 160, 590);
  
  text("measured:", 830, 590);
  text(val, 900, 590);
  text("pF", 1000, 590);
  
 
  //delay(100);
  
  
  
} // end of draw()  