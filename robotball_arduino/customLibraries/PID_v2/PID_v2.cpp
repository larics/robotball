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
        double Kp, double Ki, double Kd, int Ts, byte ControllerDirection)
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
    PID::SetTunings(Kp, Ki, Kd);

    lastTime = millis()-SampleTime;
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

  if (isnan(*myInput) || isnan(*mySetpoint)) return false;

  unsigned long now = millis();
  if(now - lastTime >= SampleTime)
  {
    lastTime = now;

    /*Compute all the working error variables*/
    double input = *myInput;
    double error = *mySetpoint - input;
    if (error > -deadzone && error < deadzone){
       error = 0;
    }

    /*Compute the control values*/
    double up = kp * error;
    double ui = ki * error + ui_old;
    double ud = kd * (error - error_old);

    double output = up + ui + ud;

    /* Limit the output and integral wind-up*/
    if (output > outMax)
    {
      output = outMax;
      ui = ui_old;
    }
    else if (output < outMin)
    {
      output = outMin;
      ui = ui_old;
    }

    /*Update the values for next time*/
    ui_old = ui;
    error_old = error;

    *myOutput = output;
    return true;
  }
  else return false;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
bool PID::SetTunings(double Kp, double Ki, double Kd)
{
  if (Kp<0 || Ki<0 || Kd<0) return false;
  if (isnan(Kp) || isnan(Ki) || isnan(Kd)) return false;

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

  if (ki == 0)
    ui_old = 0;
    
  return true;
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
      ui_old = 0;
      error_old = 0;
    }
    inAuto = newAuto;
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

