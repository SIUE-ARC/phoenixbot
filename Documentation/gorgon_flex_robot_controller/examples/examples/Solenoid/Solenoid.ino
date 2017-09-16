#include <SPI.h>
#include <Ethernet.h>
#include <EEPROM.h>
#include <RobotOpenGE.h>



/* I/O Setup */
ROJoystick usb1(1); // Joystick #1
ROSolenoid sol0(0);
ROSolenoid sol1(1);
ROSolenoid sol2(2);
ROSolenoid sol3(3);
ROSolenoid sol4(4);
ROSolenoid sol5(5);
ROSolenoid sol6(6);
ROSolenoid sol7(7);


void setup()
{
  /* Initiate comms */
  RobotOpen.begin(&enabled, &disabled, &timedtasks);
}


/* This is your primary robot loop - all of your code
 * should live here that allows the robot to operate
 */
void enabled() {
  // Joystick control of each Solenoid
  if (usb1.btnA())
    sol0.on();
  else
    sol0.off();
  if (usb1.btnB())
    sol1.on();
  else
    sol1.off();
  if (usb1.btnX())
    sol2.on();
  else
    sol2.off();
  if (usb1.btnY())
    sol3.on();
  else
    sol3.off();
  if (usb1.btnLShoulder())
    sol4.on();
  else
    sol4.off();
  if (usb1.btnRShoulder())
    sol5.on();
  else
    sol5.off();
  if (usb1.btnSelect())
    sol6.on();
  else
    sol6.off();
  if (usb1.btnStart())
    sol7.on();
  else
    sol7.off();
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
  RODashboard.publish("Uptime Seconds", ROStatus.uptimeSeconds());
}


// !!! DO NOT MODIFY !!!
void loop() {
  RobotOpen.syncDS();
}