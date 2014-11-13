//-------------------------------------------------------------------
//- project : classes for Aerotech control
//- file : AbstractAerotech.h : generic Aerotech Axis interface
//- connection to HW through SocketPool singleton
//-------------------------------------------------------------------
#ifndef __ABSTRACT_AEROTECH_H__
#define __ABSTRACT_AEROTECH_H__
#include <string>
#include <yat/time/Timer.h>
#include <yat/threading/Mutex.h>

namespace Aerotech_ns
{
#define SIZE_BUFFER 100  // Buffer size for commands
#define NB_DIGITS 4      // Number of digits for parameters sent to controller

//--------------------------------------------
  typedef enum
  {
    NONE_TYPE=0,
    ENSEMBLE_TYPE,
    A3200_TYPE
  }ControllerType;


  class AbstractAerotech
  {
    //- accessible through derivated class (use of polymorphism)
  protected:
    ControllerType m_controller_type;
    yat::Mutex m_lock;
    yat::Timeout moving_timeout;
    double offset;
    std::string axis_name;
    bool is_connected;
    bool send_string (char *string_to_send);     // send a string to "Ensemble" server
    bool send_and_receive (char *command, char *received); // send a string to
    bool send_and_receive (std::string cmd, std::string & resp);// send a string to
    bool send_receive_and_test (char *to_send, char *received);//The string returned is simplified 
    bool is_ready_to_accept_cmd (void);
    void fixe_separateur (char *stringreceived);

    //- only for cA3200 class (not used by cEnsemble)
    //-----------------------------------
    double target_speed;



  public:

    ControllerType get_controller_type (void) 
    {
      return m_controller_type;
    }
    AbstractAerotech (const char * axis_name);
    virtual ~AbstractAerotech (void);
    /**
    *  The function "axis_enable" tries to enable the axis
    *  it returns "true" if the axis answer begins by "%".
    *  to verify that the axis is really enabled, read the 
    *  function "axis_is_enabled"
    **/
    bool axis_enable ();

    /**
    *  The function "axis_disable" tries to disable the axis
    *  it returns "true if the axis answer begins by "%".
    *  to verify that the axis is really disabled, read the
    *  function "axis_is_enabled"
    **/
    bool axis_disable ();
    /**
    *  The "lowlevelcmd" function sends an Aerotech cmd to the controller.
    *  returns the response
    **/
    bool lowlevelcmd (char * argin, char * argout);

    /**
    *  The function "axis_home" tries to send the axis to home position
    *  it returns "true" if the axis answer begins by "%".
    *  to verify that the axis is really homed, use the
    *  function "axis_is_homed"
    *  Warnings with "Ensemble:
    *    This function requires the "HOMEASYNC" script installed in the controller.
    *    This "patch" allows "Ensemble" to answer immediately, before the end of
    *    homing sequence
    *  to verify that the axis is really homed, poll the status with "axis_is_homed" or "axis_is_moving"
    **/
    bool axis_home ();

    /**
    *  The functions "get_axis_software_limit_low"  and
    *  "get_axis_software_limit_high" return the software limits
    *  as  doubles.
    *  Any outside of limits displacement request will be launched but limited
    *  by those limits.
    *  WARNING !
    *  In this case the displacement function returns true (the controller answer
    *  begins by "%" ) and "in position" flag is set to 1
    *  To change these parameters, use "set_axis_software_limit_low" and
    *  "set_axis_software_limit_high" functions
    **/
    bool get_axis_software_limit_low (double &limit);
    bool get_axis_software_limit_high (double &limit);

    /**
    *  The function "get_axis_error_limit" returns the following error limit
    *  as  a double.
    *  If the current followin error, read through "get_axis_error_threshold", is
    *  greater than the limit, axis stops and is disabled.
    *  To change this parameter use "set_axis_error_limit"
    **/
    bool get_axis_error_limit (double &limit);
    bool set_axis_error_limit (double limit);

    /**
    *  The function "get_axis_default_speed" returns the velocity target as a double.
    *  Note: this value is reached only if the displacement is long enough to allow 
    *  full acceleration and deceleration phases completion
    *  To change this parameter use "set_axis_default_speed"
    **/
    bool get_axis_default_speed (double &speed);


    /**
    *  The function "get_axis_home_speed". This function returns the velocity target
    *  used for a homing sequence as a double.
    *  To change this parameter use "set_axis_home_speed" function
    **/
    bool get_axis_home_speed (double &speed);
    bool set_axis_home_speed (double speed);


    /**
    *  The function "get_axis_home_
    " returns the distance between the stage
    *  "Home" sensor and  the actual "Home" position
    *  
    *  WARNING 1 : During homing sequence ("axis_home" command), the axis finds
    *  the stage "Home" *  sensor then moves away of the "home_offset" value and
    *  then position is set to "0.000".
    *  Consequence: If the "home_offset" parameter is changed, the physical "Home"
    *  position is changed, but the displayed position after homing sequence
    *  completion remains "0.000"
    *  
    *  To change this parameter use "set_axis_home_offset" function
    *
    *  WARNING 2 : Controller needs to be turned off and back on to take the
    *  new value in account.
    *
    **/
    bool get_axis_home_offset (double &offset);
    bool set_axis_home_offset (double offset);

    /**
    *  The function "axis_abort" stops any on-going displacement
    *  it returns "true" if the axis answer begins by "%".
    **/
    bool axis_abort();

    /**
    * Use PROGRAM:
    * To run a program named "PROG.PGM"
    * in the task number "n"
    * call the function programrun("PROG.PGM",n)
    * to stop it call the function programstop(task);
    **/
    bool programrun (char *pgmname,int task);
    bool programstop (int task);

    /**
    * Use countsperunit:
    * This function convert "n" ( in physical unit) to encoder counts
    **/
    bool countsperunit (double units, double &encoder);

    /**
    * FREERUN : Command used to move to end of run (not soft limit) with
    * a defined velocity
    * the velocity can be negative
    * to stop call "freerun" with a velocity set to 0... but axis_is_moving do not change
    * to quit this mode use "axis_abort" function
    **/
    bool freerun (double velocity);

    /**
    *  The function "axis_fault_ack" clears all faults on the axis
    **/
    bool axis_fault_ack ();


    /**
    * Functions to set and get offset value;
    **/
    void set_offset (double value);
    double get_offset (void);


    /**
    * Functions to move to an absolute position in user coordinates
    *  (with offset);
    **/    
    bool axis_move_abs_user (double target);

    /**
    * Functions to to read the absolute position in user coordinates
    *  ( with offset);
    **/ 
    bool get_axis_position_user (double &position);

    /**
    * The function compute_new_offset define the current position as current "zero"
    **/    
    bool compute_new_offset (double new_user_pos);

    //-  Moving Timeout is_moving () is true for the specified timeout

    /**
    * All the following functions are "vitual" and are defined in
    * the "cEnsemble" and "cA3200" classes
    **/

    /**
    *  The function "axis_brake_on" activates the brake
    *  it returns "true" if the axis answer begins by "%".
    *  to verify that the brake is really off, use the 
    *  function "axis_is_brake_on"
    **/
    virtual bool axis_brake_on (void)=0;

    /**
    *  The function "axis_brake_off" desactivates the brake
    *  it returns "true" if the axis answer begins by "%".
    *  to verify that the brake is really off, use the 
    *  function "axis_is_brake_off"
    **/
    virtual bool axis_brake_off (void)=0;
    /**
    *  The function "get_axis_status" returns the current status as an 
    *  integer. 
    **/
    virtual bool get_axis_status (int &status)=0;

    /**
    *  All the following function use the axis or driver status
    *  theses function return true if the tested flag is true
    **/
    virtual bool axis_is_enabled (void)=0;
    virtual bool axis_is_moving (void)=0;
    virtual bool axis_is_homing (void)=0;
    virtual bool axis_is_homed (void)=0;
    virtual bool axis_is_in_position (void)=0;
    virtual bool axis_is_accelerating (void)=0;
    virtual bool axis_is_decelerating (void)=0;
    virtual bool axis_is_accel_or_decel (void)=0;
    virtual bool axis_is_cw_EOT (void)=0;
    virtual bool axis_is_ccw_EOT (void)=0;
    virtual bool axis_is_EOT (void)=0;
    virtual bool axis_is_emergency_stop (void)=0;
    virtual bool axis_is_brake_on (void)=0;
    virtual bool axis_is_brake_off (void)=0;

    /**
    *  The function "get_axis_position" returns the current position as a double
    *  Warning: For axes with high resolution (encoder feedback interpolation), 
    *  this value can slightly flutter around the theoretical position
    **/
    virtual bool get_axis_position (double &position)=0;

    /**
    *  The function "axis_move_abs" tries to move the axis to the position defined in
    *  the second parameter. It returns "true" if the axis answer begins by "%".
    *  to verify that the displacement is complete, poll the status and then use
    *  the function "is_moving" or "is_in_position"
    **/
    virtual bool axis_move_abs (double target)=0;

    /**
    *  The function "axis_move_rel" tries to move the axis for a displacement set in
    *  the second parameter. it returns "true" if the axis answer begins by "%".
    *  to verify that the displacement is complete, poll the status and then use
    *  the function "is_moving" or "is_in_position"
    **/
    virtual bool axis_move_rel (double target)=0;

    /**
    *  The function "get_axis_position_error" returns the current error position
    *  as a double. It's the difference between the position returned by encoder and
    *  the theoretical position .
    *  the absolute value of this difference must be lower than the parameter set
    *  with "get_axis_error_limit" function
    **/
    virtual bool get_axis_position_error (double &position)=0;

    /**
    *  The function "get_axis_fault_status" returns the current error status as
    *  an integer. The first 24 bits are signifiants.
    *  if there is no problem the function return 0
    *  to read the error, use the "error_to_string" function
    *   ..example:
    *        int error_status;
    *        char * explanation[200];
    *        if (get_axis_fault_status("X",&error_status)!=0);
    *            {
    *            error_to_string(error_status, explanation);
    *            cout << explanation
    *            };
    **/
    virtual bool get_axis_fault_status (int &fault_status)=0;
    virtual void error_to_string (int fault_status, char * explanation)=0;

    /**
    *  The function "get_axis_velocity" returns the current velocity as a double. 
    *  Do not mix-up with "get_axis_default_speed" .This other function returns
    *  the velocity target 
    *  The value returned by "get_axis_velocity" flutters around 0 when
    *  axis is in position and around the default speed during constant speed motion.
    *  To change this parameter use "set_axis_velocity"
    **/
    virtual bool get_axis_velocity (double &velocity)=0;
    virtual bool set_axis_velocity (double velocity)=0;

    /**
    *  The abstract functions "get_axis_software_limit_low"  and
    *  "get_axis_software_limit_high" return the software limits
    *  as  doubles.
    *  Any outside of limits displacement request will be launched but limited
    *  by those limits.
    *  WARNING !
    *  In this case the displacement function returns true (the controller answer
    *  begins by "%" ) and "in position" flag is set to 1
    *  To change these parameters, use "set_axis_software_limit_low" and
    *  "set_axis_software_limit_high" functions
    **/
    virtual bool set_axis_software_limit_low (double limit)=0;
    virtual bool set_axis_software_limit_high (double limit)=0;

    /**
    * Many commands cannot be executed with a direct string command.
    *  In that case "SETPARM" commands is used to communicate with
    *  Aerotech "Ensemble" and "A3200".
    *  "GETPARM" is used for Aerotech Ensemble and then name of the parameter, 
    *  followed by dat and the axis name with A3200
    **/
    virtual bool set_param (char *param,char *string_to_send)=0;
    virtual bool get_param (char *param,char *string_received)=0;

    /**
    *  The function "get_axis_default_speed" returns the velocity target as a double.
    *  Note: this value is reached only if the displacement is long enough to allow 
    *  full acceleration and deceleration phases completion
    *  To change this parameter use "set_axis_default_speed"
    **/
    virtual bool set_axis_default_speed (double speed)=0;

    /**
    *  The abstract function "get_axis_ramp_rate" .This function returns the ramp
    *  rate as a double.  ( the unit is displacement_unit/s² )
    *  This parameter is used for both acceleration and decceleration ramps
    *  To change this parameter use "set_axis_ramp_rate"
    **/
    virtual bool get_axis_ramp_rate (double &ramp_rate)=0;
    virtual bool set_axis_ramp_rate (double ramp_rate)=0;

    /**
    *  The abstract function "get_digital_input" .This function returns the 
    *  state of the digital inputs as an integer.  
    *  To change the output use "set_digital_output"
    **/
    virtual bool get_digital_input (int &state)=0;
    virtual bool set_digital_output (int state)=0;

    /**
    *  The abstract function "get_analog_input" .This function returns the 
    *  state of the analog input as a double.  
    *  To change the output use "set_analog_output"
    **/
    virtual bool get_analog_input (double &state)=0;
    virtual bool set_analog_output (double state)=0;


    /**
    *  The function "connected_ok" returns the number of successful communication processed.
    **/
    bool connected_ok (void);


    //- unimplemented/unsupported members (for polymorphism use)
    //-----------------------------------
  private:

  public:
    //- will be overloaded in cA3200
    virtual bool setdoubleregister(int number, double value);
    virtual bool getdoubleregister(int number, double &value);


    //- only for cEnsemble class (not used by cA3200)
    //- will be overloaded in cEnsemble
    //-----------------------------------
  public: 
    virtual bool reset(void);
    virtual bool commit_parameters(void);
    virtual bool axis_is_sense_plus (void);

  };


}//- namespace
#endif __ABSTRACT_AEROTECH_H__
