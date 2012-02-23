//---------------------------------------------------------------------------

#ifndef __cEnsemble_H__
#define __cEnsemble_H__

#include <string.h>
#include <yat/network/ClientSocket.h>
#include <yat/Exception.h>
#include <yat/time/Timer.h>
#include <stdio.h>



#define SIZE_BUFFER 100  // Buffer size for commands
#define NB_DIGITS 4      // Number of digits for parameters sent to controller

/*************************************************************************************
*                                                                                    *
*  To use the "ClassEnsemble" class, Aerotech "Ensemble" must be configured as:      * 
*    - TCPIP server (use "Ensemble Configuration Manager")                           *  
*    - Immediate answer after absolute and relative displacements                    *
*    - Using "ASYNCHOME" Command (copy "HomeAsync.bcx" and "HomeVerification.bcx"    *
*      files in the "File System" directory of the "Ensemble" Driver)                *
*    - Parameter names decoded ( copy the file "parametersName.cfg" in               * 
*      the "File System" directory of the "Ensemble" Driver)                         *
*                                                                                    *
*  WARNING: The "ClassEnsemble" class must be implemented once because               *
*   Aerotech  "Ensemble" server can dialog correctly with only one client socket     *
*  Consequences:                                                                     *
*  It is not possible to use as many instances as axes                               *
*  For each axis command the first parameter is the axis_name                        *
*     (which is a 1, 2 or 3 character string defined in the setup of "Ensemble")     * 
*                                                                                    *
*  Version 1.00                                                                      *
*         JP Gaillet 26/12/2011 (with the invaluable help of Jean Coquet [Soleil] )  *
*                                                                                    *
**************************************************************************************/

// ============================================================================
// SHORTCUT TO THE <Ensemble> SINGLETON
// ============================================================================
  #define ENSEMBLE_PROXY ClassEnsemble::instance()


// ============================================================================
//- ClassEnsemble 
//- Interface to the Aerotech Ensemble Driver 
// ============================================================================
class ClassEnsemble
{

 
private:
    // Static flag used to avoid several instances of "ClassEnsemble" class
    //- static int nb_instances;

    //- communication counters
    long com_ok_counter;
    long com_error_counter;

    //-  Moving Timeout is_moving () is true for the specified timeout
    yat::Timeout moving_timeout;
    
   //- Class classEnsemble is a singleton
   //- you must call configure (IP, port) first
   //- then use the class from anywhere using ENSEMBLE_PROXY->enable () for example
    //- the singleton ------------------------
    static ClassEnsemble * singleton;

    /**
    *  Constructor
    *  Creates the object and initialize communication
    *  Requires 2 parameters
    *   Address: the TCP-IP address. example "192.168.1.16"
    *   Port: the selected port. I.e. "8000"
    **/
        ClassEnsemble(char *address,int port);
    /**
    * Destructor
    *  Closes the TCP-IP communication and destroies the object
    **/
        ~ClassEnsemble();

    /**
    *  Private boolean functions are used to communicate through the socket :
    *  These functions return true if communication is OK and if the first
    *  character in the answer is "%" (If An answer is requested and if Command is OK)
    *  
    **/

    yat::ClientSocket * sock;                   // "sock" is the instance of Yat Socket used
    bool create_socket(char *address,int port); // To connect Client to server
    bool close_socket();                        // to disconnet client from server
    bool send_string(char *string_to_send);     // send a string to "Ensemble" server
    bool send_and_receive(char *command, char *received); // send a string to
                                                // Ensemble server and receive answer (as char * )
    bool send_and_receive(std::string cmd, std::string & resp);// send a string to
                                                // Ensemble server and receive answer (as string)
    bool send_receive_and_test(char *to_send, char *received);//The string returned is simplified 
                                                // by removing "%" at beginning and cariage return
                                                // at end 
    void WithoutCRLF(char *chaine);             // remove "Cariage Return" and/or "Line Feed"
                                             
/**
* Many commands cannot be executed with a direct string command.
*  In that case "GETPARM" and "SETPARM" commands are used to communicate with
*  Aerotech "Ensemble" .
**/

    bool get_param(char *axis_name,char *param,char *string_received);
    bool set_param(char *axis_name,char *param,char *string_to_send);
    
 public:

  //----------------------------------------
  // SINGLETON ACCESSOR 
  //----------------------------------------
  static inline ClassEnsemble * instance ()
    throw (yat::Exception)
  {
    if (! ClassEnsemble::singleton)
        THROW_YAT_ERROR(_CPTC("SOFTWARE_ERROR"),
                        _CPTC("unexpected NULL pointer [ClassEnsemble singleton not properly initialized]"),
                        _CPTC("ClassEnsemble::instance"));
    return ClassEnsemble::singleton;
  }

  //----------------------------------------
  //- instanciation of the classEnsemble + configuration
  //----------------------------------------
  static void  initialize (char *address,int port)
    throw (yat::Exception)
  {
    if (ClassEnsemble::singleton)
      return;
    try
    {
      singleton = new ClassEnsemble (address, port);
    }
    catch (...)
    {
      //- throw yat Eception
      THROW_YAT_ERROR(_CPTC("SOFTWARE_ERROR"),
                      _CPTC("unable to allocate internal class [ClassEnsemble singleton not properly initialized]"),
                      _CPTC("ClassEnsemble::configure"));
    }
  }
  //----------------------------------------
  //- release of the classEnsemble
  //----------------------------------------
  static void  close ()
    throw (yat::Exception)
  {
    if (! ClassEnsemble::singleton)
      return;
    delete ClassEnsemble::singleton;
    ClassEnsemble::singleton = 0;
  }

/**
*  The flag "is_connected" is set to true after "create_socket" function call
*  if a problem occurs it is set to false
**/
    bool is_connected;

/**
*  The function "axis_enable" tries to enable the axis
*  it returns "true" if the axis answer begins by "%".
*  to verify that the axis is really enabled, read the status and then use the
*  function "is_enabled"
**/
    bool axis_enable(char *axis_name);

/**
*  The function "axis_disable" tries to disable the axis
*  it returns "true if the axis answer begins by "%".
*  to verify that the axis is really disabled, read the status and then
*  use the function "is_enabled"
**/
    bool axis_disable(char *axis_name);

/**
*  The function "axis_abort" stops any on-going displacement
*  it returns "true" if the axis answer begins by "%".
**/
    bool axis_abort(char *axis_name);

/**
*  The function "axis_fault_ack" clears all faults on the axis
**/
    bool axis_fault_ack(char *axis_name);

/**
*  The function "axis_brake_on" activates the brake
*  it returns "true" if the axis answer begins by "%".
*  to verify that the brake is really activated, read the status and then
*   use the function "is_brake_on"
**/
    bool axis_brake_on(char *axis_name);

    /**
*  The function "axis_brake_off" desactivates the brake
*  it returns "true" if the axis answer begins by "%".
*  to verify that the brake is really off, read the status and then use the
*  function "is_brake_off"
**/
    bool axis_brake_off(char *axis_name);

/**
*  The function "axis_home" tries to send the axis to home position
*  it returns "true" if the axis answer begins by "%".
*  to verify that the axis is really homed, read the status and then use the
*  function "is_homed"
*  Warnings:
*    This function requires the "HOMEASYNC" script installed in the controller.
*    This "patch" allows "Ensemble" to answer immediately, before the end of
*    homing sequence
*    to verify that the axis is really homed, poll the status and then use "is_homed" or "is_moving"
**/
    bool axis_home(char *axis_name);

/**
*  The function "axis_move_abs" tries to move the axis to the position defined in
*  the second parameter. It returns "true" if the axis answer begins by "%".
*  to verify that the displacement is complete, poll the status and then use
*  the function "is_moving" or "is_in_position"
**/
    bool axis_move_abs(char *axis_name,double target);

/**
*  The function "axis_move_rel" tries to move the axis for a displacement set in
*  the second parameter. it returns "true" if the axis answer begins by "%".
*  to verify that the displacement is complete, poll the status and then use
*  the function "is_moving" or "is_in_position"
**/
    bool axis_move_rel(char *axis_name,double target);

/**
*  The function "get_axis_position" returns the current position as a double
*  Warning: For axes with high resolution (encoder feedback interpolation), 
*  this value can slightly flutter around the theoretical position
**/
    bool get_axis_position(char *axis_name,double &position);

/**
*  The function "get_axis_position_error" returns the current error position
*  as a double. It's the difference between the position returned by encoder and
*  the theoretical position .
*  the absolute value of this difference must be lower than the parameter set
*  with "get_axis_error_limit" function
**/
    bool get_axis_position_error(char *axis_name,double &position);

/**
*  The function "get_axis_status" returns the current status as an 
*  integer. Except for bits 20 and 21, all other bits are signifiants.
*  The most important bits are directly usable with the functions described
*  below
*  for example, to know if the axis "X" is moving you can write
*   ....
*       int status
        get_axis_status("X",&status);
        return is_moving(status) ;
**/
    bool get_axis_status(char *axis_name, int &status);


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
    bool get_axis_fault_status(char *axis_name, int &fault_status);
    void error_to_string(int fault_status,char * explanation);

/**
*  The function "get_axis_velocity" returns the current velocity as a double. 
*  Do not mix-up with "get_axis_default_speed" .This other function returns
*  the velocity target 
*  The value returned by "get_axis_velocity" flutters around 0 when
*  axis is in position and around the default speed during constant speed motion.
*  To change this parameter use "set_axis_velocity"
**/
    bool get_axis_velocity(char *axis_name,double &velocity);
    bool set_axis_velocity(char *axis_name,double velocity);

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
    bool get_axis_software_limit_low(char *axis_name,double &limit);
    bool get_axis_software_limit_high(char *axis_name,double &limit);
    bool set_axis_software_limit_low(char *axis_name,double limit);
    bool set_axis_software_limit_high(char *axis_name,double limit);

/**
*  The function "get_axis_error_limit" returns the following error limit
*  as  a double.
*  If the current followin error, read through "get_axis_error_threshold", is
*  greater than the limit, axis stops and is disabled.
*  To change this parameter use "set_axis_error_limit"
**/
    bool get_axis_error_limit(char *axis_name,double &limit);
    bool set_axis_error_limit(char *axis_name,double limit);

/**
*  The function "get_axis_default_speed" returns the velocity target as a double.
*  Note: this value is reached only if the displacement is long enough to allow 
*  full acceleration and deceleration phases completion
*  To change this parameter use "set_axis_default_speed"
**/
    bool get_axis_default_speed(char *axis_name,double &speed);
    bool set_axis_default_speed(char *axis_name,double speed);

/**
*  The function "get_axis_ramp_rate" .This function returns the ramp
*  rate as a double.  ( the unit is displacement_unit/s² )
*  This parameter is used for both acceleration and decceleration ramps
*  To change this parameter use "set_axis_ramp_rate"
**/
    bool get_axis_ramp_rate(char *axis_name,double &ramp_rate);
    bool set_axis_ramp_rate(char *axis_name,double ramp_rate);

/**
*  The function "get_axis_home_speed". This function returns the velocity target
*  used for a homing sequence as a double.
*  To change this parameter use "set_axis_home_speed" function
**/
    bool get_axis_home_speed(char *axis_name,double &speed);
    bool set_axis_home_speed(char *axis_name,double speed);

/**
*  The function "get_axis_home_offset" returns the distance between the stage
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
    bool get_axis_home_offset(char *axis_name,double &offset);
    bool set_axis_home_offset(char *axis_name,double offset);

/**
*  The following functions are usable to decode the status.
*  They can be used after the "get_axis_status" function
**/
    bool is_enabled(int status);        // true if axis is enabled
    bool is_homed(int status);          // true if axis has been homed
                                        // do not change after a movement
    bool is_moving(int status);         // true if moving
    bool is_accelerating( int status);  // true if accelerating
    bool is_decelerating( int status);  // true if decelarating
    bool is_accel_or_decel(int status); // true if accelerating or
                                        // decelarating
    bool is_sense_plus( int status);    // indicates the last or active
                                        // displacement direction
    bool is_homing( int status);        // true if in home cycle
    bool is_cw_EOT( int status);        // state of the CW end of travel
    bool is_ccw_EOT( int status);       // state of the CCW end of run
    bool is_EOT( int status);           // true if any end of run
    bool is_in_position( int status);   // not moving
    bool is_encoder_error( int status); // true if an encoder error
                                        // has occured
    bool is_emergency_stop( int status);// true if emergency stop
                                        // signal is present
    bool is_brake_on( int status) ;     // state of the brake output
    bool is_brake_off( int status) ;    // reverse state of the brake output

/**
*  The "reset" function resets the controller.
*  The communication is lost
*  After this command, to communicate again: call the destructor and build a new object
*   or call "close_socket" then "create_socket"
**/
    bool reset(void);

/**
*  The function "commit_parameters" commits the value of parameters.
*  It is important to call "commit_parameters" before using "reset" function
**/
    bool commit_parameters(void);

/**
*  The function "get_com_ok_counter" returns the number of successful communication processed.
**/
    long get_com_ok_counter (void)
    {
      return com_ok_counter;
    }
/**
*  The function "get_com_error_counter" returns the number of successful communication processed.
**/
    long get_com_error_counter(void)
    {
      return this->com_error_counter;
    }

/**
*  The function "connected_ok" returns the number of successful communication processed.
**/
     bool connected_ok (void)
    {
      return this->is_connected;
    }

  
};

#endif //- __cEnsemble_H__
 