//-------------------------------------------------------------------
//- project : classes for Aerotech control
//- file : cEnsemble.cpp : generic Aerotech Axis interface
//-------------------------------------------------------------------

#include "cEnsemble.h"
namespace Aerotech_ns
{

//-----------------------------------------------------------
//- CTOR
//-----------------------------------------------------------
  cEnsemble::cEnsemble (char * axis_name) : 
AbstractAerotech (axis_name)
{
  m_controller_type = ENSEMBLE_TYPE;
  //std::cout << "cEnsemble::cEnsemble <-" << std::endl;
}

//-----------------------------------------------------------
//- DTOR
//-----------------------------------------------------------
cEnsemble::~cEnsemble ()
{
  //std::cout << "cEnsemble::~cEnsemble <-" << std::endl;

}


//-----------------------------------------------------------
//- get_param
//-----------------------------------------------------------
bool cEnsemble::get_param (char *param, char *string_received)
{
  //- char s[SIZE_BUFFER];
  //- ::memset (s, 0, SIZE_BUFFER);
  std::stringstream s;
  s << "GETPARM(" << this->axis_name << "," << param << ")\n";
  //- std::cout << "cEnsemble::get_param axis_name <" << this->axis_name << "> param <" << param << ">" << std::endl;
  // sprintf (s, "GETPARM(%s,%s)\n", this->axis_name, param);
  bool ok = send_receive_and_test (const_cast <char*> (s.str ().c_str ()), string_received);
  return ok;
}

//-----------------------------------------------------------
//- get_param
//-----------------------------------------------------------
bool cEnsemble::set_param (char *param, char *string_to_send)
{
  if (! is_ready_to_accept_cmd  ())
    return false;
  char s[SIZE_BUFFER];
  sprintf (s, "SETPARM(%s,%s,%s)\n", this->axis_name.c_str (), param, string_to_send);
  bool ok = send_string (s);
  return ok;
} 

//-----------------------------------------------------------
//- axis_brake_on
//- power off the brake solenoid : -> OFF!!!
//-----------------------------------------------------------
bool cEnsemble::axis_brake_on (void)
{
  if (! is_ready_to_accept_cmd  ())
    return false;
  char s[SIZE_BUFFER];
  //- positive security so to brake power off brake solenoid
  sprintf (s, "BRAKE %s OFF\n", this->axis_name.c_str ());
  return send_string (s);
}

//-----------------------------------------------------------
//- axis_brake_off
//- power ON the brake solenoid : -> ON!!!
//-----------------------------------------------------------
bool cEnsemble::axis_brake_off (void)
{
  if (! is_ready_to_accept_cmd  ())
    return false;
  char s[SIZE_BUFFER];
  //- positive security so to unbrake power on brake solenoid
  sprintf (s, "BRAKE %s ON\n", this->axis_name.c_str ());
  return send_string (s);
}

//-----------------------------------------------------------
//- get_axis_status
//-----------------------------------------------------------
bool cEnsemble::get_axis_status (int &status)
{
  char s[SIZE_BUFFER];
  char string_received[SIZE_BUFFER];
  sprintf (s,"AXISSTATUS %s \n", this->axis_name.c_str ());
  bool ok = send_receive_and_test (s, string_received);
  sscanf (string_received, "%i", &status);
  return ok;
}

//-----------------------------------------------------------
//- axis_is_enabled
//-----------------------------------------------------------
bool cEnsemble::axis_is_enabled (void)
{
  int status = 0;
  if (!get_axis_status (status))
    return false;
  return ((status & 1)!=0) ;
}

//-----------------------------------------------------------
//- axis_is_moving
//-----------------------------------------------------------
bool cEnsemble::axis_is_moving (void)
{
  int status = 0;
  if (!get_axis_status (status))
    return false;
  return (status & 8)!=0 ;
}

//-----------------------------------------------------------
//- axis_is_homing
//-----------------------------------------------------------
bool cEnsemble::axis_is_homing (void)
{
  int status = 0;
  if (!get_axis_status (status))
    return false;
  return (status & 0x4000)!=0 ;
}

//-----------------------------------------------------------
//- axis_is_homed
//-----------------------------------------------------------
bool cEnsemble::axis_is_homed (void)
{
  int status = 0;
  if (!get_axis_status (status))
    return false;
  return (status & 0x02)!=0 ;
}

//-----------------------------------------------------------
//- axis_is_in_position
//-----------------------------------------------------------
bool cEnsemble::axis_is_in_position (void)
{
  int status = 0;
  if (!get_axis_status (status))
    return false;
  return (status & 0x04)!=0 ;
}

//-----------------------------------------------------------
//- axis_is_accelerating
//-----------------------------------------------------------
bool cEnsemble::axis_is_accelerating (void)
{
  int status = 0;
  if (!get_axis_status (status))
    return false;
  return (status & 0x10)!=0 ;
}

//-----------------------------------------------------------
//- axis_is_decelerating
//-----------------------------------------------------------
bool cEnsemble::axis_is_decelerating (void)
{
  int status = 0;
  if (!get_axis_status (status))
    return false;
  return (status & 0x20)!=0 ;
}

//-----------------------------------------------------------
//- axis_is_accel_or_decel
//-----------------------------------------------------------
bool cEnsemble::axis_is_accel_or_decel (void)
{
  int status = 0;
  if (!get_axis_status (status))
    return false;
  return (status & 0x30)!=0 ;
}

//-----------------------------------------------------------
//- axis_is_cw_EOT
//-----------------------------------------------------------
bool cEnsemble::axis_is_cw_EOT (void)
{
  int status = 0;
  if (!get_axis_status (status))
    return false;
  return (status & 0x400000)!=0 ;
}

//-----------------------------------------------------------
//- axis_is_ccw_EOT
//-----------------------------------------------------------
bool cEnsemble::axis_is_ccw_EOT (void)
{
  int status = 0;
  if (!get_axis_status (status))
    return false;
  return (status & 0x800000)!=0 ;
}

//-----------------------------------------------------------
//- axis_is_EOT
//-----------------------------------------------------------
bool cEnsemble::axis_is_EOT (void)
{
  int status = 0;
  if (!get_axis_status (status))
    return false;
  return (status & 0xC00000)!=0 ;
}

//-----------------------------------------------------------
//- axis_is_sense_plus
//-----------------------------------------------------------
bool cEnsemble::axis_is_sense_plus (void)
{
  int status = 0;
  if (!get_axis_status (status))
    return false;
  return (status & 0x200)!=0 ;
}

//-----------------------------------------------------------
//- axis_is_emergency_stop
//-----------------------------------------------------------
bool cEnsemble::axis_is_emergency_stop (void)
{
  int status = 0;
  if (!get_axis_status (status))
    return false;
  return (status & 0x20000)!=0 ;
}

//-----------------------------------------------------------
//- axis_is_brake_on
//-----------------------------------------------------------
bool cEnsemble::axis_is_brake_on (void)
{
  int status = 0;
  if (!get_axis_status (status))
    return false;
  return (status & 0x100)!=0 ;
}

//-----------------------------------------------------------
//- axis_is_brake_off
//-----------------------------------------------------------
bool cEnsemble::axis_is_brake_off (void)
{
  int status = 0;
  if (!get_axis_status (status))
    return false;
  return (status & 0x100)==0 ;
}

//-----------------------------------------------------------
//- axis_move_abs
//-----------------------------------------------------------
bool cEnsemble::axis_move_abs (double target)
{
  if (! is_ready_to_accept_cmd  ())
    return false;
  char sformat[SIZE_BUFFER];
  char s[SIZE_BUFFER];
  this->moving_timeout.restart ();
  sprintf (sformat, "MOVEABS %%s  %%.%df\n", NB_DIGITS);
  sprintf (s, sformat, this->axis_name.c_str (), target);
  return send_string (s);
}

//-----------------------------------------------------------
//- axis_move_rel
//-----------------------------------------------------------
bool cEnsemble::axis_move_rel (double target)
{
  if (! is_ready_to_accept_cmd  ())
    return false;
  char sformat[SIZE_BUFFER];
  char s[SIZE_BUFFER];
  this->moving_timeout.restart ();
  // first, create a format corresponding to "NB_DIGITS" 
  sprintf (sformat, "MOVEINC %%s  %%.%df\n", NB_DIGITS);
  // then create the string to be sent to controller using this format 
  sprintf( s, sformat, this->axis_name.c_str (), target);
  return send_string (s);
}

//-----------------------------------------------------------
//- get_axis_position
//-----------------------------------------------------------
bool cEnsemble::get_axis_position (double &position)
{
  char spos[SIZE_BUFFER];
  char string_received[SIZE_BUFFER];
  sprintf (spos, "PFBK %s \n", this->axis_name.c_str ());
  bool ok = send_receive_and_test (spos, string_received);
  sscanf (string_received,"%lf", &position);
  return ok;
}

//-----------------------------------------------------------
//- get_axis_position_error
//-----------------------------------------------------------
bool cEnsemble::get_axis_position_error (double &position)
{
  char s[SIZE_BUFFER];
  char string_received[SIZE_BUFFER];
  sprintf (s, "PERR %s \n", this->axis_name.c_str ());
  bool ok = send_receive_and_test(s,string_received);
  sscanf (string_received, "%lf", &position);
  return ok;
}

//-----------------------------------------------------------
//- get_axis_position_error
//-----------------------------------------------------------
bool cEnsemble::get_axis_fault_status (int &fault_status)
{
  char s[SIZE_BUFFER];
  char string_received[SIZE_BUFFER];
  sprintf (s, "AXISFAULT %s \n", this->axis_name.c_str ());
  bool ok = send_receive_and_test (s, string_received);
  //- std::cout << "cEnsemble::get_axis_fault_status axis <" << axis_name << "> fault # <" << string_received << ">" << std::endl;
  sscanf (string_received, "%d", &fault_status);
  return ok;
}

//-----------------------------------------------------------
//- error_to_string
//-----------------------------------------------------------
void cEnsemble::error_to_string (int fault_status,char *explanation)
{
  // This procedure creates as many strings as many faults are found
  // the strings are separated by "carriage return" <CR>
  // in the buffer
  explanation[0]=0;
  if ((fault_status & 0x000001)!=0) {strcat(explanation,"Position error [use FaultAck]\n");}
  if ((fault_status & 0x000002)!=0) {strcat(explanation,"Over current [use FaultAck]\n");}
  if ((fault_status & 0x000004)!=0) {strcat(explanation,"Positive end of run [use FaultAck]\n");}
  if ((fault_status & 0x000008)!=0) {strcat(explanation,"Negative end of run [use FaultAck]\n");}
  if ((fault_status & 0x000010)!=0) {strcat(explanation,"Positive soft limit [use FaultAck]\n");}
  if ((fault_status & 0x000020)!=0) {strcat(explanation,"Negative soft limit [use FaultAck]\n");}
  if ((fault_status & 0x000040)!=0) {strcat(explanation,"Amplifier fault [use FaultAck]\n");}
  if ((fault_status & 0x000080)!=0) {strcat(explanation,"Feedback fault [use FaultAck]\n");}
  if ((fault_status & 0x000100)!=0) {strcat(explanation,"Velocity fault [use FaultAck]\n");}
  if ((fault_status & 0x000200)!=0) {strcat(explanation,"Hall senser fault [use FaultAck]\n");}
  if ((fault_status & 0x000400)!=0) {strcat(explanation,"Velocity command fault [use FaultAck]\n");}
  if ((fault_status & 0x000800)!=0) {strcat(explanation,"Emergency stop [fix and use FaultAck]\n");}
  if ((fault_status & 0x001000)!=0) {strcat(explanation,"Velocity error fault [use FaultAck]\n");}
  if ((fault_status & 0x008000)!=0) {strcat(explanation,"External fault [check wired securities?]\n");}
  if ((fault_status & 0x020000)!=0) {strcat(explanation,"Motor temperature [use FaultAck]\n");}
  if ((fault_status & 0x040000)!=0) {strcat(explanation,"Amplifier temperature [use FaultAck]\n");}
  if ((fault_status & 0x080000)!=0) {strcat(explanation,"Encoder fault [use FaultAck, call maintenance]\n");}
  if ((fault_status & 0x100000)!=0) {strcat(explanation,"Communication lost [resart device, reset driver..]\n");}
  if ((fault_status & 0x400000)!=0) {strcat(explanation,"Following error [use FaultAck]\n");}
}

//-----------------------------------------------------------
//- get_axis_velocity
//-----------------------------------------------------------
bool cEnsemble::get_axis_velocity (double &velocity)
{
  char s[SIZE_BUFFER];
  char string_received[SIZE_BUFFER];
  sprintf (s, "VFBK %s \n", this->axis_name.c_str ());
  bool ok = send_receive_and_test(s,string_received);
  sscanf(string_received,"%lf",&velocity);
  return ok;
}

//-----------------------------------------------------------
//- set_axis_velocity
//-----------------------------------------------------------
bool cEnsemble::set_axis_velocity (double velocity)
{
  if (! is_ready_to_accept_cmd  ())
    return false;
  char sformat[SIZE_BUFFER];
  char s[SIZE_BUFFER];
  sprintf (sformat, "VCMD %%s  %%.%df\n", NB_DIGITS);
  sprintf( s, sformat, this->axis_name.c_str (), velocity);
  return send_string (s);
}

//-----------------------------------------------------------
//- set_axis_software_limit_low
//-----------------------------------------------------------
bool cEnsemble::set_axis_software_limit_low (double limit)
{
  char s[SIZE_BUFFER];
  sprintf(s, "%lf", limit);
  bool ok = set_param (const_cast <char*> ("softwareLimitLow"), s);
  return ok;
}

//-----------------------------------------------------------
//- set_axis_software_limit_high
//-----------------------------------------------------------
bool cEnsemble::set_axis_software_limit_high (double limit)
{
  char s[SIZE_BUFFER];
  sprintf (s, "%lf", limit);
  bool ok = set_param (const_cast <char*> ("softwareLimitHigh"), s);
  return ok;
}

//-----------------------------------------------------------
//- set_axis_default_speed
//-----------------------------------------------------------
bool cEnsemble::set_axis_default_speed (double speed)
{
  char s[SIZE_BUFFER];
  char sformat[SIZE_BUFFER];
  sprintf (s, "%lf", speed);
  bool ok = set_param(const_cast <char*> ("DefaultSpeed"), s);
  if (ok)
  {
    // send a "MOVEINC Axis F speed" - a relative nul displacement to set the velocity
    sprintf (sformat, "MOVEINC %%s F %%.%df\n", NB_DIGITS);
    sprintf (s, sformat, this->axis_name.c_str (), speed);
    ok = send_string (s);
  }
  return ok;
}

//-----------------------------------------------------------
//- set_axis_ramp_rate
//-----------------------------------------------------------
bool cEnsemble::set_axis_ramp_rate (double ramp_rate)
{
  if (! is_ready_to_accept_cmd  ())
    return false;
  char s[SIZE_BUFFER];
  char sformat[SIZE_BUFFER];
  sprintf(s,"%lf",ramp_rate);
  bool ok = set_param (const_cast <char*> ("Defaultramprate"), s);
  if (ok)
  {
    sprintf(sformat, "RAMP RATE %%s  %%.%df\n", NB_DIGITS);
    sprintf(s, sformat, this->axis_name.c_str (), ramp_rate);
    ok = send_string (s);
  }
  return ok;
}

//-----------------------------------------------------------
//- reset
//-----------------------------------------------------------
bool cEnsemble::reset ()
{
  bool ok = send_string (const_cast <char*> ("RESET\n"));
  if (ok) 
  {
    is_connected = false;
  }
  return ok;    
}

//-----------------------------------------------------------
//- commit_parameters
//-----------------------------------------------------------
bool cEnsemble::commit_parameters ()
{

  char s[SIZE_BUFFER];
  char string_received[SIZE_BUFFER];
  sprintf (s, "AXISFAULT ALL \n");
  bool ok = send_receive_and_test (s, string_received);
  int fault_status = -1;
  sscanf (string_received, "%d", &fault_status);
  if (fault_status != 0)
    return false;
  sprintf(s, "COMITPARAMETERS \n");
  return send_string (s);
}

//-----------------------------------------------------------
//- get_digital_input
//-----------------------------------------------------------
bool cEnsemble::get_digital_input (int &state)
{
  char spos[SIZE_BUFFER];
  char string_received[SIZE_BUFFER];
  sprintf (spos, "DIN %s \n", this->axis_name.c_str ());
  bool ok = send_receive_and_test (spos, string_received);
  sscanf (string_received, "%i", &state);
  return ok;
}

//-----------------------------------------------------------
//- set_digital_output
//-----------------------------------------------------------
bool cEnsemble::set_digital_output (int state)
{
  char spos[SIZE_BUFFER];
  sprintf (spos, "DOUT %s %i \n", this->axis_name.c_str (), state);
  return send_string (spos);
}  

//-----------------------------------------------------------
//- get_analog_input
//-----------------------------------------------------------
bool cEnsemble::get_analog_input (double &state)
{
  char spos[SIZE_BUFFER];
  char string_received[SIZE_BUFFER];
  sprintf (spos, "AIN %s \n", this->axis_name.c_str ());
  bool ok = send_receive_and_test (spos, string_received);
  sscanf (string_received, "%df", &state);
  return ok;
}

//-----------------------------------------------------------
//- set_analog_output
//-----------------------------------------------------------
bool cEnsemble::set_analog_output (double state)
{
  char spos[SIZE_BUFFER];
  sprintf (spos, "AOUT %s %df \n", this->axis_name.c_str (), state);
  return send_string (spos);
}  

//-----------------------------------------------------------
//- get_axis_ramp_rate
//-----------------------------------------------------------
bool cEnsemble::get_axis_ramp_rate (double &ramp_rate)
{
  char string_received[SIZE_BUFFER];
  char s [32];
  ::memset (s, 0, 32);
  strcpy (s, "DefaultRampRate");
  bool ok = get_param (s, string_received);
  sscanf (string_received, "%lf", &ramp_rate);
  return ok;
}

} // -namespace


