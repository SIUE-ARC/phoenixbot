#include <SPI.h>
#include <Ethernet.h>
#include <EEPROM.h>
#include <RobotOpenGE.h>



/* Timer Setup */
ROTimer stepOne;   	// First timer step
ROTimer stepTwo;	// Second timer step
ROTimer loopMe;		// A looping timer


void setup()
{
  /* Initiate comms */
  RobotOpen.begin(&enabled, &disabled, &timedtasks);
  stepOne.queue(0);
  loopMe.queue(0);
}


/* This is your primary robot loop - all of your code
 * should live here that allows the robot to operate
 */
void enabled() {
  // don't do anything
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
  if (stepOne.ready()) {
  	RODashboard.debug("Step One!");
  	stepTwo.queue(1000);
  }
  if (stepTwo.ready()) {
  	RODashboard.debug("Step Two!");
  	stepOne.queue(1000);
  }
  if (loopMe.ready()) {
  	RODashboard.debug("I looped!");
  	loopMe.queue(5000);
  }
  RODashboard.publish("Uptime Seconds", ROStatus.uptimeSeconds());
}


// !!! DO NOT MODIFY !!!
void loop() {
  RobotOpen.syncDS();
}