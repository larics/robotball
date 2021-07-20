/**********************************************************************************************
 * Arduino PID Library - Version 1.1.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <PID_v2.h>

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, int Ts, byte POn, byte ControllerDirection)
{
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    inAuto = false;

    PID::SetOutputLimits(0, 255); //default output limit corresponds to
												          //the arduino pwm limits
    PID::SetDeadzone(0);          //deadzone is disabled by default
    SampleTime = Ts;							//default Controller Sample Time is 0.1 seconds

    controllerDirection = ControllerDirection;
    PID::SetTunings(Kp, Ki, Kd, POn);

    lastTime = millis()-SampleTime;
}

/*Constructor (...)*********************************************************
 *    To allow backwards compatability for v1.1, or for people that just want
 *    to use Proportional on Error without explicitly saying so
 ***************************************************************************/

PID::PID(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, int Ts, byte ControllerDirection)
    :PID::PID(Input, Output, Setpoint, Kp, Ki, Kd, Ts, P_ON_E, ControllerDirection)
{

}


/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool PID::Compute()
{
   if(!inAuto) return false;
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
      double input = *myInput;
      double error = *mySetpoint - input;
      if (error > -deadzone && error < deadzone){
         error = 0;
      }
      double dInput = (input - lastInput);
      errSum += error;

      /*Add Proportional on Measurement, if P_ON_M is specified*/
      // if(!pOnE) outputSum-= kp * dInput;

      // if(outputSum > outMax) outputSum= outMax;
      // else if(outputSum < outMin) outputSum= outMin;

      /*Add Proportional on Error, if P_ON_E is specified*/
      double output;
      if(pOnE) output = kp * error;
      else output = 0;

      /*Compute Rest of PID Output*/
      output += ki * errSum - kd * dInput;

	    if(output > outMax)
      {
        output = outMax;
        errSum -= error;
      }
      else if(output < outMin) 
      {
        output = outMin;
        errSum -= error;
      }
	    *myOutput = output;

      /*Remember some variables for next time*/
      lastInput = input;
      lastTime = now;
	    return true;
   }
   else return false;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PID::SetTunings(double Kp, double Ki, double Kd, byte POn)
{
  if (Kp<0 || Ki<0 || Kd<0) return;

  pOn = POn;
  pOnE = POn == P_ON_E;

  dispKp = Kp; dispKi = Ki; dispKd = Kd;

  double SampleTimeInSec = ((double)SampleTime)/1000;
  kp = Kp;
  ki = Ki * SampleTimeInSec;
  kd = Kd / SampleTimeInSec;

  if(controllerDirection == REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }

  // if (ki == 0)
  // {
  //   outputSum = 0;  // Even if Ki is set to zero, outputSum would still have an
  //                   // old value with possibly undesired integral effects.
  // }
}

/* SetTunings(...)*************************************************************
 * Set Tunings using the last-rembered POn setting
 ******************************************************************************/
void PID::SetTunings(double Kp, double Ki, double Kd){
    SetTunings(Kp, Ki, Kd, pOn); 
}


/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID::SetOutputLimits(double Min, double Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;

   if(inAuto)
   {
	   if(*myOutput > outMax) *myOutput = outMax;
	   else if(*myOutput < outMin) *myOutput = outMin;

	   // if(outputSum > outMax) outputSum= outMax;
	   // else if(outputSum < outMin) outputSum= outMin;
   }
}

/* SetDeadzone(...)****************************************************
 *  This function sets the deadzone value. If the current error is within
 *  -value and value, the error will be ignored, i.e. set to zero.
 **************************************************************************/
void PID::SetDeadzone(double value)
{
   deadzone = value;
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PID::SetMode(byte Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !inAuto)
    {  /*we just went from manual to auto*/
        PID::Initialize();
    }
    inAuto = newAuto;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PID::Initialize()
{
  // outputSum = *myOutput;
  errSum = 0;
  lastInput = *myInput;
  // if(outputSum > outMax) outputSum = outMax;
  // else if(outputSum < outMin) outputSum = outMin;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID::GetKp(){ return  dispKp; }
double PID::GetKi(){ return  dispKi;}
double PID::GetKd(){ return  dispKd;}
int PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}

