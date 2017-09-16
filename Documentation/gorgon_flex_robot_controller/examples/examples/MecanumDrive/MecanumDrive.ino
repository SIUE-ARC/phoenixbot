#include <SPI.h>
#include <Ethernet.h>
#include <EEPROM.h>
#include <RobotOpenGE.h>



/* I/O Setup */
ROJoystick usb1(1);         // Joystick #1
ROPWM pwm0(0);
ROPWM pwm1(1);
ROPWM pwm2(2);
ROPWM pwm3(3);


void setup()
{
  /* Initiate comms */
  RobotOpen.begin(&enabled, &disabled, &timedtasks);
}


/* This is your primary robot loop - all of your code
 * should live here that allows the robot to operate
 */
void enabled() {
  // get desired translation and rotation, scaled to [-127..128] (0 neutral)
  int x = usb1.leftX() - 127;
  int y = (255 - usb1.leftY()) - 127;
  int rotate = usb1.rightX() - 127;

  // calculate wheel throttles
  int lf = x + y + rotate;
  int rf = x - y + rotate;
  int lr = -x + y + rotate;
  int rr = -x - y + rotate;

  // normalize wheel throttles
  int maximum = max(max(abs(lf), abs(rf)), max(abs(lr), abs(rr)));
  if (maximum > 127) {
    lf = (lf / maximum) * 127;
    rf = (rf / maximum) * 127;
    lr = (lr / maximum) * 127;
    rr = (rr / maximum) * 127;
  }

  // Set PWMs, shifted back to [0..255]
  pwm0.write(lf + 127);
  pwm1.write(rf + 127);
  pwm2.write(lr + 127);
  pwm3.write(rr + 127);
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