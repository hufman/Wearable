import processing.serial.*;
import processing.opengl.*;

Serial myPort;  // Create object from Serial class
String val;     // Data received from the serial port
JSONObject data;
long lastReport = 0;
long lastReportEnabled = 0;
float roll  = 0.0F;
float pitch = 0.0F;
float yaw   = 0.0F;
float vx = 0.0F;
float vy = 0.0F;
float vz = 0.0F;
float x = 0.0F;
float y = 0.0F;
float z = 0.0F;

PShape model;

void setup()
{
  // I know that the first port in the serial list on my mac
  // is Serial.list()[0].
  // On Windows machines, this generally opens COM1.
  // Open whatever port is the one you're using.
  String portName = Serial.list()[0]; //change the 0 to a 1 or 2 etc. to match your port
  myPort = new Serial(this, portName, 115200);
  
  size(800, 800, OPENGL);
  frameRate(30);
  model = loadShape("bunny.obj");
  model.scale(15);

}

void draw()
{
  /*
  if ( myPort.available() > 0) 
  {  // If data is available,
    myPort.readStringUntil('{');
    val = myPort.readStringUntil('}');         // read it and store it in val
    if (val != null) {
      data = parseJSONObject("{" + val);
      println("{" + val); //print it out in the console
    }
  } 
  */
  if (lastReportEnabled + 3000 < millis()) {
    myPort.write("{\"command\": \"enable_orientation_reporting\"}\n");
    lastReportEnabled = millis();
  }
  drawBunny();
}

void serialEvent(Serial p)
{
    myPort.readStringUntil('{');
    val = myPort.readStringUntil('}');         // read it and store it in val
    if (val != null) {
      data = parseJSONObject("{" + val);
      if (lastReport + 300 < millis()) {
        println("{" + val); //print it out in the console
        lastReport = millis();
      }
      if (data != null) {
        pitch = data.getFloat("p");
        roll = data.getFloat("r");
        x = data.getFloat("x");
        y = data.getFloat("y");
        z = data.getFloat("z");
        vx = data.getFloat("vx");
        vy = data.getFloat("vy");
        vz = data.getFloat("vz");
      }
    }
}

void drawBunny()
{
  background(0,0, 0);

    // Set a new co-ordinate space
  pushMatrix();

  // Simple 3 point lighting for dramatic effect.
  // Slightly red light in upper right, slightly blue light in upper left, and white light from behind.
  pointLight(255, 200, 200,  400, 400,  500);
  pointLight(200, 200, 255, -400, 400,  500);
  pointLight(255, 255, 255,    0,   0, -500);
  
  // Displace objects from 0,0
  translate(350, 600, 0);
  
  translate(-x*25, -z*25, -y*25);
  // Rotate shapes around the X/Y/Z axis (values in radians, 0..Pi*2)
  rotateX(radians(180));
  rotateX(radians(roll));
  rotateZ(radians(pitch));
  rotateY(radians(yaw));

  pushMatrix();
  noStroke();
  shape(model);
  popMatrix();
  popMatrix();
}
