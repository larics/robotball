#ifndef PID_v2_h
#define PID_v2_h
#define LIBRARY_VERSION 1.1.1

class PID
{

  public:

    //Constants used in some of the functions below
    #define AUTOMATIC 1
    #define MANUAL  0
    #define DIRECT  0
    #define REVERSE  1
    #define P_ON_M 0
    #define P_ON_E 1

    //commonly used functions **************************************************************************
    PID(double*, double*, double*,        // * constructor.  links the PID to the Input, Output, and 
        double, double, double,           //   Setpoint.  Initial tuning parameters are also set here.
        int, byte, byte);                 //   (overload for specifying proportional mode)

    PID(double*, double*, double*,        // * constructor.  links the PID to the Input, Output, and 
        double, double, double,           //   Setpoint.  Initial tuning parameters are also set here
        int, byte);
    
    void SetMode(byte Mode);              // * sets PID to either Manual (0) or Auto (non-0)

    bool Compute();                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

    void SetOutputLimits(double, double); // * clamps the output to a specific range. 0-255 by default, but
                                          //   it's likely the user will want to change this depending on
                                          //   the application
    void SetDeadzone(double);             // * ignores the error if it's in the +/- deadzone range. 0 by
                                          //   default which means the deadzone is not active
    


    //available but not commonly used functions ********************************************************
    void SetTunings(double, double,       // * While most users will set the tunings once in the 
                    double);              //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
    void SetTunings(double, double,       // * overload for specifying proportional mode
                    double, byte);             

                        
    //Display functions ****************************************************************
    double GetKp();      // These functions query the pid for interal values.
    double GetKi();      // they were created mainly for the pid front-end,
    double GetKd();      // where it's important to know what is actually 
    int GetMode();       // inside the PID.

  private:
    void Initialize();
    
    double dispKp;        // * we'll hold on to the tuning parameters in user-entered 
    double dispKi;        //   format for display purposes
    double dispKd;        //
      
    double kp;            // * (P)roportional Tuning Parameter
    double ki;            // * (I)ntegral Tuning Parameter
    double kd;            // * (D)erivative Tuning Parameter

    byte controllerDirection;
    byte pOn;

    double *myInput;      // * Pointers to the Input, Output, and Setpoint variables
    double *myOutput;     //   This creates a hard link between the variables and the 
    double *mySetpoint;   //   PID, freeing the user from having to constantly tell us
                          //   what these values are.  with pointers we'll just know.
          
    unsigned long lastTime;
    double errSum, lastInput;

    unsigned long SampleTime;
    double outMin, outMax;
    double deadzone;
    bool inAuto, pOnE;
};
#endif

