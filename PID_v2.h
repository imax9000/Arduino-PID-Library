#ifndef PID_v2_h
#define PID_v2_h

class PID {
 public:
  // Constants used in some of the functions below
  enum Mode { Manual = 0, Automatic = 1 };
  enum Direction { Direct = 0, Reverse = 1 };
  enum P_On { Measurement = 0, Error = 1 };

  // commonly used functions
  // **************************************************************************
  PID(double *, double *,
      double *,  // * constructor.  links the PID to the Input, Output, and
      double, double, double, P_On,
      Direction);  //   Setpoint.  Initial tuning parameters are also set here.
                   //   (overload for specifying proportional mode)

  PID(double *, double *,
      double *,  // * constructor.  links the PID to the Input, Output, and
      double, double, double,
      Direction);  //   Setpoint.  Initial tuning parameters are also set here

  void SetMode(Mode Mode);  // * sets PID to either Manual (0) or Auto (non-0)

  bool Compute();  // * performs the PID calculation.  it should be
                   //   called every time loop() cycles. ON/OFF and
                   //   calculation frequency can be set using SetMode
                   //   SetSampleTime respectively

  void SetOutputLimits(double,
                       double);  // * clamps the output to a specific range.
                                 // 0-255 by default, but
                                 //   it's likely the user will want to change
                                 //   this depending on the application

  // available but not commonly used functions
  // ********************************************************
  void SetTunings(
      double, double,  // * While most users will set the tunings once in the
      double);         //   constructor, this function gives the user the option
                //   of changing tunings during runtime for Adaptive control
  void SetTunings(double,
                  double,  // * overload for specifying proportional mode
                  double, P_On);

  void SetControllerDirection(
      Direction);  // * Sets the Direction, or "Action" of the controller.
                   // DIRECT
                   //   means the output will increase when error is positive.
                   //   REVERSE means the opposite.  it's very unlikely that
                   //   this will be needed once it is set in the constructor.
  void SetSampleTime(
      int);  // * sets the frequency, in Milliseconds, with which
             //   the PID calculation is performed.  default is 100

  // Display functions
  // ****************************************************************
  double GetKp() const;  // These functions query the pid for interal values.
  double GetKi() const;  //  they were created mainly for the pid front-end,
  double GetKd() const;  // where it's important to know what is actually
  Mode GetMode() const;  //  inside the PID.
  Direction GetDirection() const;  //

 private:
  void Initialize();

  double dispKp;  // * we'll hold on to the tuning parameters in user-entered
  double dispKi;  //   format for display purposes
  double dispKd;  //

  double kp;  // * (P)roportional Tuning Parameter
  double ki;  // * (I)ntegral Tuning Parameter
  double kd;  // * (D)erivative Tuning Parameter

  Direction controllerDirection;
  P_On pOn;

  double *myInput;   // * Pointers to the Input, Output, and Setpoint variables
  double *myOutput;  //   This creates a hard link between the variables and the
  double
      *mySetpoint;  //   PID, freeing the user from having to constantly tell us
                    //   what these values are.  with pointers we'll just know.

  unsigned long lastTime;
  double outputSum, lastInput;

  unsigned long SampleTime;
  double outMin, outMax;
  bool inAuto, pOnE;
};

#ifndef PID_v2_SKIP_COMPAT_WITH_v1
const PID::Mode AUTOMATIC = PID::Mode::Automatic;
const PID::Mode MANUAL = PID::Mode::Manual;
const PID::Direction DIRECT = PID::Direction::Direct;
const PID::Direction REVERSE = PID::Direction::Reverse;
const PID::P_On P_ON_M = PID::P_On::Measurement;
const PID::P_On P_ON_E = PID::P_On::Error;
#endif

#endif  // PID_v2_h
