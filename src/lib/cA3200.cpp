//--------------------------------------------------------------------------------
//- Project : Aerotech
//- Abstract : control of Aerotech Axes drivers Ensemble and A3200
//- file : cA3200.cpp
//- implementation of AbstractAxis for A3200 drivers
//--------------------------------------------------------------------------------

#include "cA3200.h"

namespace Aerotech_ns
{

//-----------------------------------------------------------
//- CTOR
//-----------------------------------------------------------
cA3200::cA3200 (char * _axis_name) :
  AbstractAerotech (_axis_name)
{
  m_controller_type = A3200_TYPE;
  //- std::cout << "cA3200::cA3200 <-" << std::endl;
  //- initialize target_speed with current speed value
  this->get_axis_default_speed (this->target_speed);
}

//-----------------------------------------------------------
//- DTOR
//-----------------------------------------------------------
cA3200::~cA3200 ()
{
  //- NOOP DTOR
  //- std::cout << "cA3200::~cA3200 <-" << std::endl;
}

//-----------------------------------------------------------
//- get_axis_status
//-----------------------------------------------------------
bool cA3200::get_axis_status (int &status)
{
  char s[SIZE_BUFFER];
  char string_received[SIZE_BUFFER];
  sprintf (s,"AXISSTATUS (%s ,DATAITEM_AxisStatus)\n", this->axis_name.c_str ());
  bool ok = send_receive_and_test (s, string_received);
  sscanf (string_received, "%i", &status);
  return ok;
}

//-----------------------------------------------------------
//- get_drive_status
//-----------------------------------------------------------
bool cA3200::get_drive_status (int &status)
{
  char s[SIZE_BUFFER];
  char string_received[SIZE_BUFFER];
  sprintf (s, "AXISSTATUS (%s ,DATAITEM_DriveStatus)\n", this->axis_name.c_str ());
  bool ok = send_receive_and_test (s, string_received);
  sscanf(string_received, "%i", &status);
  return ok;
}

//-----------------------------------------------------------
//- axis_brake_on
//-----------------------------------------------------------
bool cA3200::axis_brake_on (void)
{
  if (! is_ready_to_accept_cmd ())
    return false;
  char s[SIZE_BUFFER];
  //- positive security so to brake power off brake solenoid
  sprintf (s, "BRAKE %s 0\n", this->axis_name.c_str ());
  return send_string (s);
}

//-----------------------------------------------------------
//- axis_brake_off
//-----------------------------------------------------------
bool cA3200::axis_brake_off (void)
{
  if (! is_ready_to_accept_cmd ())
    return false;
  char s[SIZE_BUFFER];
  //- positive security so to unbrake power on brake solenoid
  sprintf (s, "BRAKE %s 1\n", this->axis_name.c_str ());
  return send_string (s);
}

//-----------------------------------------------------------
//- axis_is_enabled
//-----------------------------------------------------------
bool cA3200::axis_is_enabled (void)
{
  int status = 0;
  if (!get_drive_status (status))
    return false;
  return ((status & 1) != 0);
}

//-----------------------------------------------------------
//- axis_is_moving
//-----------------------------------------------------------
bool cA3200::axis_is_moving (void)
{
  int status = 0;
  if (! get_drive_status (status))
    return false;
  return ((status & 0x2000000) == 0);
}

//-----------------------------------------------------------
//- axis_is_homing
//-----------------------------------------------------------
bool cA3200::axis_is_homing (void)
{
  int status = 0;
  if (! get_axis_status (status))
    return false;
  return ((status & 0x10) != 0);
}

//-----------------------------------------------------------
//- axis_is_homed
//-----------------------------------------------------------
bool cA3200::axis_is_homed (void)
{
  int status = 0;
  if (! get_axis_status (status))
    return false;
  return ((status & 0x01) != 0) ;
}

//-----------------------------------------------------------
//- axis_is_in_position
//-----------------------------------------------------------
bool cA3200::axis_is_in_position (void)
{
  int status = 0;
  if (! get_drive_status (status))
    return false;
  return ((status & 0x2000000) != 0);
}

//-----------------------------------------------------------
//- axis_is_accelerating
//-----------------------------------------------------------
bool cA3200::axis_is_accelerating (void)
{
  int status = 0;
  if (! get_drive_status (status))
    return false;
  return ((status & 0x8000000) != 0);
}

//-----------------------------------------------------------
//- axis_is_decelerating
//-----------------------------------------------------------
bool cA3200::axis_is_decelerating (void)
{
  int status = 0;
  if (! get_drive_status (status))
    return false;
  return ((status & 0x10000000) != 0);
}

//-----------------------------------------------------------
//- axis_is_accel_or_decel
//-----------------------------------------------------------
bool cA3200::axis_is_accel_or_decel (void)
{
  int status = 0;
  if (! get_drive_status (status))
    return false;
  return ((status & 0x18000000) != 0);
}

//-----------------------------------------------------------
//- axis_is_cw_EOT
//-----------------------------------------------------------
bool cA3200::axis_is_cw_EOT(void)
{
  int status = 0;
  if (! get_drive_status (status))
    return false;
  return ((status & 0x02) != 0);
}

//-----------------------------------------------------------
//- axis_is_ccw_EOT
//-----------------------------------------------------------
bool cA3200::axis_is_ccw_EOT(void)
{
  int status = 0;
  if (! get_drive_status (status))
    return false;
  return ((status & 0x04) != 0);
}

//-----------------------------------------------------------
//- axis_is_EOT
//-----------------------------------------------------------
bool cA3200::axis_is_EOT (void)
{
  int status = 0;
  if (! get_drive_status (status))
    return false;
  return ((status & 0x06) != 0);
}

//-----------------------------------------------------------
//- axis_is_emergency_stop
//-----------------------------------------------------------
bool cA3200::axis_is_emergency_stop (void)
{
  int status = 0;
  if (! get_drive_status (status))
    return false;
  return ((status & 0x400) != 0);
}

//-----------------------------------------------------------
//- axis_is_brake_on
//-----------------------------------------------------------
bool cA3200::axis_is_brake_on(void)
{
  int status = 0;
  if (! get_drive_status (status))
    return false;
  return ((status & 0x800) != 0);
}

//-----------------------------------------------------------
//- axis_is_brake_off
//-----------------------------------------------------------
bool cA3200::axis_is_brake_off(void)
{
  int status = 0;
  if (! get_drive_status (status))
    return false;
  return ((status & 0x800) == 0);
}

//-----------------------------------------------------------
//- axis_move_abs
//-----------------------------------------------------------
bool cA3200::axis_move_abs (double target)
{
  if (! is_ready_to_accept_cmd ())
    return false;
  if (target_speed <= 0.0)
    this->get_axis_default_speed (this->target_speed);
  char sformat[SIZE_BUFFER];
  char s[SIZE_BUFFER];
  this->moving_timeout.restart ();
  sprintf (sformat, "MOVEABS %%s  %%.%df  %%.%df\n", NB_DIGITS, NB_DIGITS);
  sprintf (s, sformat, this->axis_name.c_str (), target, target_speed);
  return send_string (s);
}

//-----------------------------------------------------------
//- axis_move_rel
//-----------------------------------------------------------
bool cA3200::axis_move_rel (double target)
{
  if (! is_ready_to_accept_cmd ())
    return false;
  if (target_speed <= 0.0)
    this->get_axis_default_speed (this->target_speed);
  char sformat[SIZE_BUFFER];
  char s[SIZE_BUFFER];
  this->moving_timeout.restart ();
  // first, create a format corresponding to "NB_DIGITS" 
  sprintf (sformat, "MOVEINC %%s  %%.%df  %%.%df\n", NB_DIGITS, NB_DIGITS);
  // then create the string to be sent to controller using this format 
  sprintf(s, sformat, this->axis_name.c_str (), target, target_speed);
  return send_string (s);
}


//-----------------------------------------------------------
//- get_axis_position
//-----------------------------------------------------------
bool cA3200::get_axis_position (double &position)
{
  char s[SIZE_BUFFER];
  char string_received[SIZE_BUFFER];
  sprintf(s, "AXISSTATUS (%s ,DATAITEM_PositionFeedback)\n", this->axis_name.c_str ());
  bool ok = send_receive_and_test (s, string_received);
  fixe_separateur (string_received);
  sscanf(string_received, "%lf,", &position);
  return ok;
}

//-----------------------------------------------------------
//- get_axis_position_error
//-----------------------------------------------------------
bool cA3200::get_axis_position_error (double &position)
{
  char s[SIZE_BUFFER];
  char string_received[SIZE_BUFFER];
  sprintf(s, "AXISSTATUS (%s ,DATAITEM_PositionError)\n", this->axis_name.c_str ());
  bool ok = send_receive_and_test(s, string_received);
  fixe_separateur (string_received);
  sscanf (string_received, "%lf", &position);
  return ok;
}

//-----------------------------------------------------------
//- get_axis_fault_status
//-----------------------------------------------------------
bool cA3200::get_axis_fault_status (int &fault_status)
{
  char s[SIZE_BUFFER];
  char string_received[SIZE_BUFFER];
  sprintf(s, "AXISSTATUS (%s ,DATAITEM_AxisFault)\n", this->axis_name.c_str ());
  bool ok = send_receive_and_test(s, string_received);
  sscanf (string_received, "%i", &fault_status);
  return ok;
}

//-----------------------------------------------------------
//- error_to_string
//-----------------------------------------------------------
void cA3200::error_to_string (int fault_status,char *explanation)
// This procedure creates as many strings as many faults are found
// the strings are separated by "cariage return"
// in the buffer
{
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
  if ((fault_status & 0x004000)!=0) {strcat(explanation,"Probe Inpout fault error fault [use FaultAck]\n");}
  if ((fault_status & 0x008000)!=0) {strcat(explanation,"External fault [check wired securities?]\n");}
  if ((fault_status & 0x020000)!=0) {strcat(explanation,"Motor temperature [use FaultAck]\n");}
  if ((fault_status & 0x040000)!=0) {strcat(explanation,"Amplifier temperature [use FaultAck]\n");}
  if ((fault_status & 0x080000)!=0) {strcat(explanation,"Encoder fault [use FaultAck, call maintenance]\n");}
  if ((fault_status & 0x100000)!=0) {strcat(explanation,"Communication lost [resart device, reset driver..]\n");}
  if ((fault_status & 0x400000)!=0) {strcat(explanation,"Gantry Scaling error [use FaultAck]\n");}
  if ((fault_status & 0x800000)!=0) {strcat(explanation,"Marker search fault  [use FaultAck]\n");}
}

//-----------------------------------------------------------
//- get_axis_velocity
//-----------------------------------------------------------
bool cA3200::get_axis_velocity(double &velocity)
{
  char s[SIZE_BUFFER];
  char string_received[SIZE_BUFFER];
  sprintf (s, "AXISSTATUS (%s ,DATAITEM_VelocityFeedback)\n", this->axis_name.c_str ());
  bool ok = send_receive_and_test (s, string_received);
  fixe_separateur (string_received);
  sscanf (string_received, "%lf", &velocity);
  return ok;
}

//-----------------------------------------------------------
//- set_axis_velocity
//-----------------------------------------------------------
bool cA3200::set_axis_velocity (double velocity)
{
  if (! is_ready_to_accept_cmd ())
    return false;
  target_speed = velocity;
  return true;
}

//-----------------------------------------------------------
//- set_param
//-----------------------------------------------------------
bool cA3200::set_param (char *param, char *string_to_send)
{
  if (! is_ready_to_accept_cmd ())
    return false;
  char s[SIZE_BUFFER];
  sprintf (s,"SETPARM %s %s %s \n", this->axis_name.c_str (), param, string_to_send);
  bool ok = send_string (s);
  return ok;
} 

//-----------------------------------------------------------
//- get_param
//-----------------------------------------------------------
bool cA3200::get_param (char *param, char *string_received)
{
  char s[SIZE_BUFFER];
  ::memset (s, 0, SIZE_BUFFER);
  
  sprintf (s, "%s.%s\n", param, this->axis_name.c_str ());
  bool ok = send_receive_and_test (s, string_received);
  return ok;
}

//-----------------------------------------------------------
//- set_axis_software_limit_low
//-----------------------------------------------------------
bool cA3200::set_axis_software_limit_low(double limit)
{
  char s[SIZE_BUFFER];
  
  sprintf (s, "%lf", limit);
  bool ok = set_param (const_cast <char*> ("SoftwareLimitLow"), s);
  if (ok)
  {
    ok = set_param (const_cast <char*> ("SoftwareLimitSetup"), "6");
  }
  return ok;
}

//-----------------------------------------------------------
//- set_axis_software_limit_high
//-----------------------------------------------------------
bool cA3200::set_axis_software_limit_high (double limit)
{
  char s[SIZE_BUFFER];
  
  sprintf (s, "%lf", limit);
  bool ok = set_param (const_cast <char*> ("SoftwareLimitHigh"), s);
  if (ok)
  {
    ok = set_param (const_cast <char*> ("SoftwareLimitSetup"), "6");
  }
  return ok;
}

//-----------------------------------------------------------
//- set_axis_default_speed
//-----------------------------------------------------------
bool cA3200::set_axis_default_speed (double speed)
{
  char s[SIZE_BUFFER];
  sprintf(s, "%lf", speed);
  bool ok = set_param (const_cast <char*> ("DefaultSpeed"), s);
  if (ok)
  {
    target_speed = speed; 
  }
  return ok;
}

//-----------------------------------------------------------
//- set_axis_ramp_rate
//-----------------------------------------------------------
bool cA3200::set_axis_ramp_rate (double ramp_rate)
{
  if (! is_ready_to_accept_cmd ())
    return false;
  char s[SIZE_BUFFER];
  
  sprintf(s, "%lf", ramp_rate);
  bool ok = set_param (const_cast <char*> ("DefaultRampRate"), s);
  if (ok)
  {
    sprintf(s, "RAMP RATE %s %lf\n", this->axis_name.c_str (), ramp_rate);
    return send_string (s);
  }
  return false;
}

//-----------------------------------------------------------
//- get_digital_input
//-----------------------------------------------------------
bool cA3200::get_digital_input (int &state)
{
  char s[SIZE_BUFFER];
  char string_received[SIZE_BUFFER];
  
  sprintf(s, "AXISSTATUS (%s ,DATAITEM_DigitalInput)\n", this->axis_name.c_str ());
  bool ok = send_receive_and_test (s, string_received);
  sscanf (string_received, "%i", &state);
  return ok;
}

//-----------------------------------------------------------
//- set_digital_output
//-----------------------------------------------------------
bool cA3200::set_digital_output (int state)
{
  char s[SIZE_BUFFER];
  sprintf (s, "%i", state);
  bool ok = set_param (const_cast <char*> ("DigitalOutput"), s);
  return ok;
} 

//-----------------------------------------------------------
//- get_analog_input
//-----------------------------------------------------------
bool cA3200::get_analog_input (double &state)
{
  char s[SIZE_BUFFER];
  char string_received[SIZE_BUFFER];
  sprintf (s, "AXISSTATUS (%s ,DATAITEM_AnalogInput0)\n", this->axis_name.c_str ());
  bool ok = send_receive_and_test (s, string_received);
  fixe_separateur (string_received);
  sscanf (string_received, "%lf", &state);
  return ok;
}

//-----------------------------------------------------------
//- set_analog_output
//-----------------------------------------------------------
bool cA3200::set_analog_output (double state)
{
  char s[SIZE_BUFFER];
  sprintf (s, "%i", state);
  bool ok = set_param (const_cast <char*> ("AnalogOutput0"), s);
  return ok;
} 

//-----------------------------------------------------------
//- setdoubleregister
//-----------------------------------------------------------
bool cA3200::setdoubleregister (int number, double value)
{
  char s[SIZE_BUFFER];
  sprintf (s, "$global[%i] = %lf \n", number,value);
  return send_string (s);
}

//-----------------------------------------------------------
//- getdoubleregister
//-----------------------------------------------------------
bool cA3200::getdoubleregister(int number, double &value)
{
  char s[SIZE_BUFFER];
  char string_received[SIZE_BUFFER];
  sprintf (s, "$global[%i]\n", number);
  bool ok = send_receive_and_test (s, string_received);
  fixe_separateur (string_received);
  sscanf (string_received, "%lf", &value);
  return ok;
}

//-----------------------------------------------------------
//- get_axis_ramp_rate
//-----------------------------------------------------------
bool cA3200::get_axis_ramp_rate (double &ramp_rate)
{
  char s[SIZE_BUFFER];
  char string_received[SIZE_BUFFER];
  sprintf (s, "AXISSTATUS (%s ,DATAITEM_AccelerationRate)\n", this->axis_name.c_str ());
  bool ok = send_receive_and_test (s, string_received);
  fixe_separateur (string_received);
  sscanf (string_received, "%lf", &ramp_rate);
  return ok;
}

} //- namespace