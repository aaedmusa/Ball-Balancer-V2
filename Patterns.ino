//***3RPS Parallel Manipulator Ball Balancer Code BY Aaed Musa**
//--------------------------------------------------------------

//libraries
#include <AccelStepper.h>
#include <InverseKinematics.h>
#include <MultiStepper.h>
#include <stdint.h>
#include <TouchScreen.h>
#include <math.h>

Machine machine(2, 3.125, 1.75, 3.669291339);     //(d, e, f, g) object to define the lengths of the machine
TouchScreen ts = TouchScreen(A1, A0, A3, A2, 0);  //touch screen pins (XGND, YGND, X5V, Y5V)

//stepper motors
AccelStepper stepperA(1, 1, 2);  //(driver type, STEP, DIR) Driver A
AccelStepper stepperB(1, 3, 4);  //(driver type, STEP, DIR) Driver B
AccelStepper stepperC(1, 5, 6);  //(driver type, STEP, DIR) Driver C
MultiStepper steppers;           // Create instance of MultiStepper

//stepper motor variables
int pos[3];                                            // An array to store the target positions for each stepper motor
int ENA = 0;                                           //enable pin for the drivers
double angOrig = 206.662752199;                        //original angle that each leg starts at
double speed[3] = { 0, 0, 0 }, speedPrev[3], ks = 20;  //the speed of the stepper motor and the speed amplifying constant

//touch screen variables
double Xoffset = 500;  //X offset for the center position of the touchpad
double Yoffset = 500;  //Y offset for the center position of the touchpad

//PID variables
double kp = 4E-4, ki = 2E-6, kd = 7E-3;                                                       //PID constants
double error[2] = { 0, 0 }, errorPrev[2], integr[2] = { 0, 0 }, deriv[2] = { 0, 0 }, out[2];  //PID terms for X and Y directions
long timeI, timeII;                                                                           //variables to capture initial times

//Other Variables
double angToStep = 3200 / 360;  //angle to step conversion factor (steps per degree) for 16 microsteps or 3200 steps/rev
bool detected = 0;              //this value is 1 when the ball is detected and the value is 0 when the ball in not detected

void setup() {
  Serial.begin(115200);
  // Adding the steppers to the steppersControl instance for multi stepper control
  steppers.addStepper(stepperA);
  steppers.addStepper(stepperB);
  steppers.addStepper(stepperC);
  //Enable pin
  pinMode(ENA, OUTPUT);           //define enable pin as output
  digitalWrite(ENA, HIGH);        //sets the drivers off initially
  delay(1000);                    //small delay to allow the user to reset the platform
  digitalWrite(ENA, LOW);         //sets the drivers on
  moveTo(4.25, 0, 0);             //moves the platform to the home position
  steppers.runSpeedToPosition();  //blocks until the platform is at the home position
}
void loop() {
  //PID(0, 0);  //(X setpoint, Y setpoint) -- must be looped
  //moveToPID(0, 0, 5000); //moves the ball to a position and keeps it there (X, Y, wait)
  //linePattern(100, 0, 800, 2);  //moves the ball in a line (rx, ry, wait, num)
  //trianglePattern(3); //moves the ball in a triangle (num)
  //squarePattern(3);  //moves the ball in a square (num)
  //pinBallPattern(200, 600);  //moves the ball in a pinball pattern (Y, wait)
  //ellipsePattern(100, 100, 0, 20, 5);  //moves the ball in an elipse (rx, ry, start, wait, num)
  //sinusoidalPattern(50, 30, 20);  //moves ball in a sinusoidal pattern (A, B, wait)
  //figure8Pattern(200, 0, 10, 5);  //moves the ball in an elipse (r, start, wait, num)
  //DEMO() //does all of the patterns sequentially;
  while (1) {}
}
//moves/positions the platform with the given parameters
void moveTo(double hz, double nx, double ny) {
  //if the ball has been detected
  if (detected) {
    //calculates stepper motor positon
    for (int i = 0; i < 3; i++) {
      pos[i] = round((angOrig - machine.theta(i, hz, nx, ny)) * angToStep);
    }
    //sets calculated speed
    stepperA.setMaxSpeed(speed[A]);
    stepperB.setMaxSpeed(speed[B]);
    stepperC.setMaxSpeed(speed[C]);
    //sets acceleration to be proportional to speed
    stepperA.setAcceleration(speed[A] * 30);
    stepperB.setAcceleration(speed[B] * 30);
    stepperC.setAcceleration(speed[C] * 30);
    //sets target positions
    stepperA.moveTo(pos[A]);
    stepperB.moveTo(pos[B]);
    stepperC.moveTo(pos[C]);
    //runs stepper to target position (increments at most 1 step per call)
    stepperA.run();
    stepperB.run();
    stepperC.run();
  }
  //if the hasn't been detected
  else {
    for (int i = 0; i < 3; i++) {
      pos[i] = round((angOrig - machine.theta(i, hz, 0, 0)) * angToStep);
    }
    //sets max speed
    stepperA.setMaxSpeed(800);
    stepperB.setMaxSpeed(800);
    stepperC.setMaxSpeed(800);
    //moves the stepper motors
    steppers.moveTo(pos);
    steppers.run();  //runs stepper to target position (increments at most 1 step per call)
  }
}
//takes in an X and Y setpoint/position and moves the ball to that position
void PID(double setpointX, double setpointY) {
  TSPoint p = ts.getPoint();  //measure X and Y positions
  //if the ball is detected (the x position will not be 0)
  if (p.x != 0) {
    detected = 1;
    //calculates PID values
    for (int i = 0; i < 2; i++) {
      errorPrev[i] = error[i];                                                                     //sets previous error
      error[i] = (i == 0) * (Xoffset - p.x - setpointX) + (i == 1) * (Yoffset - p.y - setpointY);  //sets error aka X or Y ball position
      integr[i] += error[i] + errorPrev[i];                                                        //calculates the integral of the error (proportional but not equal to the true integral of the error)
      deriv[i] = error[i] - errorPrev[i];                                                          //calcuates the derivative of the error (proportional but not equal to the true derivative of the error)
      deriv[i] = isnan(deriv[i]) || isinf(deriv[i]) ? 0 : deriv[i];                                //checks if the derivative is a real number or infinite
      out[i] = kp * error[i] + ki * integr[i] + kd * deriv[i];                                     //sets output
      out[i] = constrain(out[i], -0.25, 0.25);                                                     //contrains output to have a magnitude of 0.25
    }
    //calculates stepper motor speeds
    for (int i = 0; i < 3; i++) {
      speedPrev[i] = speed[i];                                                                                                           //sets previous speed
      speed[i] = (i == A) * stepperA.currentPosition() + (i == B) * stepperB.currentPosition() + (i == C) * stepperC.currentPosition();  //sets current position
      speed[i] = abs(speed[i] - pos[i]) * ks;                                                                                            //calculates the error in the current position and target position
      speed[i] = constrain(speed[i], speedPrev[i] - 200, speedPrev[i] + 200);                                                            //filters speed by preventing it from beign over 100 away from last speed
      speed[i] = constrain(speed[i], 0, 1000);                                                                                           //constrains sped from 0 to 1000
    }
    Serial.println((String) "X OUT = " + out[0] + "   Y OUT = " + out[1] + "   Speed A: " + speed[A]);  //print X and Y outputs
  }
  //if the ball is not detected (the x value will be 0)
  else {
    //double check that there is no ball
    delay(10);                  //10 millis delay before another reading
    TSPoint p = ts.getPoint();  //measure X and Y positions again to confirm no ball
    if (p.x == 0) {             //if the ball is still not detected
      //Serial.println("BALL NOT DETECTED");
      detected = 0;
    }
  }
  //continues moving platforma and waits until 20 millis has elapsed
  timeI = millis();
  while (millis() - timeI < 20) {
    moveTo(4.25, -out[0], -out[1]);  //moves the platfrom
  }
}
//moves the ball to a location with 
void moveToPID(int X, int Y, int wait) {
  //X = x setpoint
  //Y = y setpoint
  //wait = elapsed time
  timeII = millis();
  while (millis() - timeII < wait) {
    PID(X, Y);
  }
}
//moves the ball in a line
void linePattern(double rx, int ry, int wait, int num) {
  //rx = half the traversal length in the X direction (0 - 200)
  //ry = half the traversal length in the Y direction (0 - 200)
  //wait = time delay between movements in ms (0 - 1000)
  //num = number of interations
  for (int i = 0; i < num; i++) {
    timeII = millis();
    while (millis() - timeII < wait) {
      PID(rx, ry);  //(X setpoint, Y setpoint)
    }
    timeII = millis();
    while (millis() - timeII < wait) {
      PID(-rx, -ry);  //(X setpoint, Y setpoint)
    }
  }
}
//moves the ball in a triangle
void trianglePattern(int num) {
  //num = number of interations
  double s = 400;  //side length
  for (int i = 0; i < num; i++) {
    for (int j = 0; j < 3; j++) {
      timeII = millis();
      while (millis() - timeII < 800) {
        PID((1 - j) * (s / 2), j == 1 ? s * (sqrt(3) / 4) : -s * (sqrt(3) / 4));  //(X setpoint, Y setpoint)
      }
    }
  }
}
//moves the ball in a square
void squarePattern(int num) {
  //num = number of interations
  int s = 400;  //side length
  for (int i = 0; i < num; i++) {
    for (int j = 0; j < 4; j++) {
      timeII = millis();
      while (millis() - timeII < 700) {
        PID(j < 2 ? s / 2 : -s / 2, j == 1 || j == 2 ? s / 2 : -s / 2);  //(X setpoint, Y setpoint)
      }
    }
  }
}
//moves the ball in a pinBall pattern
void pinBallPattern(int Y, int wait) {
  //Y = amplitude - (100-200)
  //wait = time delay between movements in ms (0 - 1000)
  Y *= -1;
  for (int X = 200; X >= -300; X -= 100) {
    timeII = millis();
    while (millis() - timeII < wait) {
      PID(X, Y);  //(X setpoint, Y setpoint)
    }
    Y *= -1;
  }
  for (int X = -200; X <= 300; X += 100) {
    timeII = millis();
    while (millis() - timeII < wait) {
      PID(X, Y);  //(X setpoint, Y setpoint)
    }
    Y *= -1;
  }
}
//moves the ball in an elipse
void ellipsePattern(double rx, int ry, double start, int wait, int num) {
  //rx = x axis radius (0 - 150)
  //ry = y axis radius (0 - 150)
  //start = 0 or 2*PI
  //wait = time delay between movements in ms (0 - 20)
  //num = number of times to traverse the elipse
  double theta;
  for (int i = 0; i < num; i++) {
    theta = start;
    for (double j = 0; j <= 2 * PI; j += 0.1) {
      timeII = millis();
      while (millis() - timeII < wait) {        //moves the ball
        PID(rx * cos(theta), ry * sin(theta));  //(X setpoint, Y setpoint)
      }
      theta += start == 0 ? 0.1 : (start == 2 * PI ? -0.1 : 0);
    }
  }
}
//moves ball in a sinusoidal pattern
void sinusoidalPattern(double ampli, double freq, int wait) {
  //ampli = amplitude
  //freq = frequency
  //wait = time delay between movements in ms (0 - 20)
  for (double X = 300; X >= -300; X -= 5) {
    timeII = millis();
    while (millis() - timeII < wait) {  //moves the ball
      PID(X, ampli * sin(X / freq));    //(X setpoint, Y setpoint)
    }
  }
  for (double X = -300; X <= 300; X += 5) {
    timeII = millis();
    while (millis() - timeII < wait) {  //moves the ball
      PID(X, ampli * sin(X / freq));    //(X setpoint, Y setpoint)
    }
  }
}
//moves the ball in figure 8
void figure8Pattern(double r, double start, int wait, int num) {
  //r = x and Y axis radius (0 - 150)
  //start = 0 or -2*PI
  //wait = time delay between movements in ms (0 - 20)
  //num = number of times to traverse the elipse
  double theta;
  double scale;
  for (int i = 0; i < num; i++) {
    theta = start;
    for (double j = 0; j < 2 * PI; j += 0.05) {
      timeII = millis();
      scale = r * (2 / (3 - cos(2 * theta)));  //moves the ball
      while (millis() - timeII < wait) {
        PID(scale * cos(theta), scale * sin(2 * theta) / 1.5);  //(X setpoint, Y setpoint)
      }
      theta += start == 0 ? -0.05 : (start == -2 * PI ? 0.05 : 0);
    }
  }
}
//demo
void DEMO() {
  moveToPID(0, 0, 8000);
  linePattern(200, 0, 1000, 1);  //moves the ball in a line (rx, ry, wait, num)
  linePattern(200, 0, 600, 2);  //moves the ball in a line (rx, ry, wait, num)
  //triangle demo
  trianglePattern(2);  //moves the ball in a triangle (num)
  //square demo
  squarePattern(2);  //moves the ball in a square (num)
  //pinBall demo
  pinBallPattern(175, 500);  //moves the ball in a pinball pattern (wait)
  pinBallPattern(100, 300);  //moves the ball in a pinball pattern (wait)
  //circle demo
  ellipsePattern(50, 50, 0, 1, 2);     //moves the ball in an elipse (rx, ry, start, wait, num)
  ellipsePattern(100, 100, 0, 10, 2);  //moves the ball in an elipse (rx, ry, start, wait, num)
  ellipsePattern(150, 150, 0, 20, 2);  //moves the ball in an elipse (rx, ry, start, wait, num)
  ellipsePattern(50, 150, 0, 20, 2);  //moves the ball in an elipse (rx, ry, start, wait, num)
  ellipsePattern(150, 50, 0, 20, 2);  //moves the ball in an elipse (rx, ry, start, wait, num)
  //sinusoidal demo
  sinusoidalPattern(50, 30, 10);   //moves ball in a sinusoidal pattern (ampli, freq, wait)
  sinusoidalPattern(100, 50, 10);  //moves ball in a sinusoidal pattern (ampli, freq, wait)
  sinusoidalPattern(150, 80, 10);  //moves ball in a sinusoidal pattern (ampli, freq, wait)
  //figure 8 demo
  figure8Pattern(200, 0, 20, 3);  //moves the ball in an elipse (r, start, wait, num)
  //end
  moveToPID(0, 0, 2000);
  for (int i = 0; i < 3; i++) {
    pos[i] = round((angOrig - machine.theta(i, 4.25, 0, 0.25)) * angToStep);
  }
  stepperA.setMaxSpeed(2000);
  stepperB.setMaxSpeed(2000);
  stepperC.setMaxSpeed(2000);
  steppers.moveTo(pos);
  steppers.runSpeedToPosition();  //blocks until the platform is at the home position
  delay(1000);
  detected = 0;
  moveTo(4.25, 0, 0);             //moves the platform to the home position
  steppers.runSpeedToPosition();  //blocks until the platform is at the home position
  for (int i = 0; i < 20; i++) {
    for (int j = 0; j < 3; j++) {
      pos[j] = round((angOrig - machine.theta(j, 5 - 1.25 * (i % 2 != 0), 0, 0)) * angToStep);
    }
    //sets max speed
    stepperA.setMaxSpeed(2000);
    stepperB.setMaxSpeed(2000);
    stepperC.setMaxSpeed(2000);
    //moves the stepper motors
    steppers.moveTo(pos);
    steppers.runSpeedToPosition();  //runs stepper to target position (increments at most 1 step per call)
  }
}