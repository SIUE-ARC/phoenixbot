#include <SPI.h>
#include <Ethernet.h>
#include <EEPROM.h>
#include <RobotOpenGE.h>



/* I/O Setup */
ROJoystick usb1(1);              // Joystick #1
RODigitalIO dig0In(0, INPUT);    // DIO channel 0, input mode
RODigitalIO dig1Out(1, OUTPUT);  // DIO channel 1, output mode


void setup()
{
  /* Initiate comms */
  RobotOpen.begin(&enabled, &disabled, &timedtasks);
}


/* This is your primary robot loop - all of your code
 * should live here that allows the robot to operate
 */
void enabled() {
  if (usb1.btnA())
    dig1Out.on();
  else
    dig1Out.off();
}


/* This is called while the robot is disabled
 * All outputs are automatically disabled (PWM, Solenoid, Digital Outs)
 */
void disabled() {
  // safety code
}


/* This loop ALWAYS runs - only place code here that can run during a disabled state
 * This is also a good spot to put driver station publish code
 */
void timedtasks() {
  if (dig0In.read())
    RODashboard.debug("Digital 0 High!");
  RODashboard.publish("Uptime Seconds", ROStatus.uptimeSeconds());
}


// !!! DO NOT MODIFY !!!
void loop() {
  RobotOpen.syncDS();
}