//-------------------------------------------------------------------
//- project : classes for Aerotech control
//- file : AbstractAerotech.cpp : specific Ensemble Axis interface
//- connection to HW through SocketPool singleton
//-------------------------------------------------------------------

#include "AbstractAerotech.h"
#include "SocketPool.h"
#include <iostream>
#include <stdio.h>
#include <string.h>

namespace Aerotech_ns
{

//-----------------------------------------------------------
//- CTOR
//-----------------------------------------------------------
AbstractAerotech::AbstractAerotech (const char * _axis_name) :
    axis_name (_axis_name),
    is_connected (false),
    offset (0.0),
    m_controller_type (NONE_TYPE)
{
  //- configure the moving timer
  this->moving_timeout.set_unit (yat::Timeout::TMO_UNIT_MSEC);
  this->moving_timeout.set_value (25.0);
  this->moving_timeout.restart ();

  //- check if SocketPool is alive
  try
  {
    SocketPool::ClassState st = SOCK_POOL->state ();
    if (st != SocketPool::SW_CONFIGURED)
      is_connected = false;
    else
      is_connected = true;
    return;
  }
  catch (...)
  {
//  std::cout << "ClassEnsemble::ClassEnsemble caught (...) trying to create socket on " << address << std::endl;
    is_connected = false;
    return;
  }
}

//-----------------------------------------------------------
//- DTOR
//-----------------------------------------------------------
AbstractAerotech::~AbstractAerotech(void)
{
  //- NOOP DTOR
}


/************************************************************
* Comments about these functions are in "cEnsemble.h" file
************************************************************/

//-----------------------------------------------------------
//- send_string (checks cmd accepted by controller)
//-----------------------------------------------------------
bool AbstractAerotech::send_string (char *string_to_send)  //send string to controller
  {  //send string to controller
  char string_received [SIZE_BUFFER];
  send_and_receive (string_to_send, string_received);
  return (string_received[0]=='%');
  }

//-----------------------------------------------------------
//- send_string (checks cmd accepted by controller)
//- send string to controller and return answer in "received"
//-----------------------------------------------------------
bool AbstractAerotech::send_receive_and_test (char *to_send, char *received)
{
  bool ok = send_and_receive (to_send, received);
std::cout << "AbstractAerotech::send_receive_and_test cmd <" << to_send << "> response <" << received << ">" << std::endl;
  if (!ok)
  {
    //- std::cout << "AbstractAerotech::send_receive_and_test  NOT OK!!!" << std::endl;
    return false;
  }
  if (received[0] == '%')
  {
    //- std::cout << "AbstractAerotech::send_receive_and_test before remove %" << std::endl;
    received[0] = ' ';
    //- std::cout << "AbstractAerotech::send_receive_and_test after remove % response = <" << received << ">" << std::endl;
    return true;
  }
  else
  {
    //- std::cout << "AbstractAerotech::send_receive_and_test PAS TROUVE le %" << std::endl;
    return false;
  }
}


//-----------------------------------------------------------
//- send_and_receive (char *, char *)
//-----------------------------------------------------------
bool AbstractAerotech::send_and_receive (char *command, char *received)
{
  std::string resp;

  //- just call the std::string 
  bool ok = this->send_and_receive (std::string (command), resp);
  //- std::cout << "AbstractAerotech::send_and_receive cmd <" << command << "> reponse <" << resp << ">" << std::endl;
  //- copy in the C string
  long length = resp.copy (received, resp.size (), 0);
  received [length] = char (0);

  return ok;
}

//-----------------------------------------------------------
//- send_and_receive 
//- sends to the controller throuh SocketPool
//-----------------------------------------------------------
bool AbstractAerotech::send_and_receive (std::string cmd, std::string & resp)
{
  try
  {//- CRITICAL SECTION
    yat::AutoMutex <yat::Mutex> guard(this->m_lock);
    SOCK_POOL->write_read(cmd, resp);
    return true;
  } //- END CRITICAL SECTION
  catch (yat::SocketException &ye)
  {
    std::cerr << "AbstractAerotech::send_receive Axis <" 
              << this->axis_name << "> "
              << "yat SocketException caught desc <" << ye.errors[0].desc 
              << "> trying to send cmd <" << cmd << ">" << std::endl;
    return false;
  }
  catch (yat::Exception &ye)
  {
    std::cerr << "AbstractAerotech::send_receive Axis <" 
              << this->axis_name << "> "
              << "yat Exception caught desc <" << ye.errors[0].desc 
              << "> trying to send cmd <" << cmd << ">" << std::endl;
    return false;
  }
  catch (...)
  {
    std::cerr << "AbstractAerotech::send_receive Axis <" 
              << this->axis_name << "> "
              << "(...) Exception caught trying to send cmd <" << cmd << ">" << std::endl;
    return false;
  }
}

//-----------------------------------------------------------
//- AbstractAerotech::axis_enable
//-----------------------------------------------------------
bool AbstractAerotech::axis_enable (void)
{
  if (! is_ready_to_accept_cmd  ())
    return false;
  char s[SIZE_BUFFER];
  sprintf (s,"ENABLE %s \n",this->axis_name.c_str ());
  return send_string (s);
}

//-----------------------------------------------------------
//- AbstractAerotech::axis_disable
//-----------------------------------------------------------
bool AbstractAerotech::axis_disable (void)
{
  if (! is_ready_to_accept_cmd  ())
    return false;
  char s[SIZE_BUFFER];
  sprintf (s,"DISABLE %s \n",this->axis_name.c_str ());
  return send_string (s);
}

//-----------------------------------------------------------
//- AbstractAerotech::lowlevelcmd
//-----------------------------------------------------------
bool AbstractAerotech::lowlevelcmd (char * argin, char * argout)
{
  char tmp [256];
  ::memset (tmp, 0, 256);
  strncpy (tmp, argin, 254);
  strcat (tmp, "\n");
  return send_and_receive (tmp, argout);
}

//-----------------------------------------------------------
//- AbstractAerotech::is_ready_to_accept_cmd
//-----------------------------------------------------------
bool AbstractAerotech::is_ready_to_accept_cmd  (void)
  {
  int f_status = 0;
  if (!get_axis_fault_status (f_status))
    return false;
  return f_status == 0;
  }

//-----------------------------------------------------------
//- AbstractAerotech::axis_abort
//-----------------------------------------------------------
bool AbstractAerotech::axis_abort (void)
{
  if (! is_ready_to_accept_cmd  ())
    return false;
  char s[SIZE_BUFFER];
  sprintf (s,"ABORT %s \n",this->axis_name.c_str ());
  return send_string (s);
}

//-----------------------------------------------------------
//- AbstractAerotech::axis_fault_ack
//-----------------------------------------------------------
bool AbstractAerotech::axis_fault_ack (void)
{
  char s[SIZE_BUFFER];
  sprintf (s,"FAULTACK %s \n", this->axis_name.c_str ());
  return send_string (s);
}

//-----------------------------------------------------------
//- AbstractAerotech::set_wait_mode
//-----------------------------------------------------------
bool AbstractAerotech::set_wait_mode (short wait_mode)
{
  if (! is_ready_to_accept_cmd  ())
    return false;
  char s[SIZE_BUFFER];
  //- 0: no wait | 1: Wait move | 2: wait move + in pos
  sprintf (s, "WAIT MODE %i\n", wait_mode);
  return send_string (s);
}

//-----------------------------------------------------------
//- AbstractAerotech::axis_home
//-----------------------------------------------------------
bool AbstractAerotech::axis_home (void)
{
  if (! is_ready_to_accept_cmd  ())
    return false;
  int status = 0;
  if (!axis_is_enabled  ())
    return false;
  if ( axis_is_homing  ())
    return false;
  if ( axis_is_moving  ())
    return false;
  char s[SIZE_BUFFER];
  sprintf (s,"HOMEASYNC %s \n", this->axis_name.c_str ());
  return send_string (s);
}

//-----------------------------------------------------------
//- AbstractAerotech::get_axis_software_limit_low
//-----------------------------------------------------------
bool AbstractAerotech::get_axis_software_limit_low (double &limit)
{
  char string_received[SIZE_BUFFER];
  char s [32];
  ::memset (s, 0, 32);
  strcpy (s, "SoftwareLimitLow");
  bool ok = get_param (s, string_received);
  sscanf (string_received, "%lf", &limit);
  return ok;
}

//-----------------------------------------------------------
//- AbstractAerotech::get_axis_software_limit_high
//-----------------------------------------------------------
bool AbstractAerotech::get_axis_software_limit_high (double &limit)
{
  char string_received[SIZE_BUFFER];
  char s [32];
  ::memset (s, 0, 32);
  strcpy (s, "SoftwareLimitHigh");
  bool ok = get_param (s, string_received);
  sscanf (string_received, "%lf", &limit);
  return ok;
}

//-----------------------------------------------------------
//- AbstractAerotech::get_axis_error_limit
//-----------------------------------------------------------
bool AbstractAerotech::get_axis_error_limit (double &limit)
{
  char string_received[SIZE_BUFFER];
  char s [32];
  ::memset (s, 0, 32);
  strcpy (s, "PositionErrorThreshold");
  bool ok = get_param (s, string_received);
  fixe_separateur (string_received);
  sscanf (string_received, "%lf", &limit);
  return ok;
}

//-----------------------------------------------------------
//- AbstractAerotech::set_axis_error_limit
//-----------------------------------------------------------
bool AbstractAerotech::set_axis_error_limit(double limit)
{
  char s[SIZE_BUFFER];
  sprintf (s,"%lf",limit);
  return (set_param (const_cast <char*> ("PositionErrorThreshold"), s));
}

//-----------------------------------------------------------
//- AbstractAerotech::get_axis_default_speed
//-----------------------------------------------------------
bool AbstractAerotech::get_axis_default_speed (double &speed)
{
  char string_received[SIZE_BUFFER];
  char s [32];
  ::memset (s, 0, 32);
  strcpy (s, "DefaultSpeed");
  bool ok = get_param (s, string_received);
  sscanf (string_received, "%lf", &speed);
  return ok;
}


//-----------------------------------------------------------
//- AbstractAerotech::get_axis_home_speed
//-----------------------------------------------------------
bool AbstractAerotech::get_axis_home_speed (double &home_speed)
{
  char string_received[SIZE_BUFFER];
  char s [32];
  ::memset (s, 0, 32);
  strcpy (s, "HomeSpeed");
  bool ok = get_param (s, string_received);
  sscanf (string_received, "%lf", &home_speed);
  return ok;
}

//-----------------------------------------------------------
//- AbstractAerotech::set_axis_home_speed
//-----------------------------------------------------------
bool AbstractAerotech::set_axis_home_speed (double home_speed)
{
  char s[SIZE_BUFFER];
  sprintf (s, "%lf", home_speed);
  return (set_param ( const_cast <char*> ("HomeSpeed"), s));
}

//-----------------------------------------------------------
//- AbstractAerotech::get_axis_home_offset
//-----------------------------------------------------------
bool AbstractAerotech::get_axis_home_offset (double &offset)
{
  char string_received[SIZE_BUFFER];
  char s [32];
  ::memset (s, 0, 32);
  strcpy (s, "HomeOffset");
  bool ok = get_param (s, string_received);
  sscanf (string_received, "%lf", &offset);
  return ok;
}

//-----------------------------------------------------------
//- AbstractAerotech::set_axis_home_offset
//-----------------------------------------------------------
bool AbstractAerotech::set_axis_home_offset (double offset)
{
  char s[SIZE_BUFFER];
  sprintf(s, "%lf", offset);
  return (set_param (const_cast <char*> ("HomeOffset"), s));
}

//-----------------------------------------------------------
//- AbstractAerotech::connected_ok
//-----------------------------------------------------------
bool AbstractAerotech::connected_ok (void)
{
  //- check if SocketPool is alive
  try
  {
    SocketPool::ClassState st = SOCK_POOL->state ();
    if (st != SocketPool::SW_CONFIGURED)
      is_connected = false;
    else
      is_connected = true;
    return is_connected;
    //- std::cout << "AbstractAerotech::connected_ok is_connected = " << is_connected << std::endl;
    return is_connected;
  }
  catch (...)
  {
//  std::cout << "ClassEnsemble::ClassEnsemble caught (...) trying to create socket on " << address << std::endl;
    is_connected = false;
    return is_connected;
  }

}

//-----------------------------------------------------------
//- AbstractAerotech::programrun
//-----------------------------------------------------------
bool AbstractAerotech::programrun (char *pgmname, int task)
{
  char s[SIZE_BUFFER];
  sprintf (s, "PROGRAM %i RUN \"%s\"\n", task, pgmname);
  return send_string (s);
}

//-----------------------------------------------------------
//- AbstractAerotech::programstop
//-----------------------------------------------------------
bool AbstractAerotech::programstop (int task)
{
  char s[SIZE_BUFFER];
  sprintf (s, "PROGRAM %i STOP \n", task);
  return send_string (s);
}

//-----------------------------------------------------------
//- AbstractAerotech::countsperunit
//-----------------------------------------------------------
bool AbstractAerotech::countsperunit (double units, double &encoder)
{
  char string_received[SIZE_BUFFER];
  char s [32];
  ::memset (s, 0, 32);
  strcpy (s, "CountsPerUnit");

  bool ok = get_param (s, string_received);
  sscanf (string_received, "%lf", &encoder);
  return ok;
}
      
//-----------------------------------------------------------
//- AbstractAerotech::freerun
//-----------------------------------------------------------
bool AbstractAerotech::freerun (double velocity)
{
  char s[SIZE_BUFFER];
  sprintf (s, "FREERUN %s %lf \n", this->axis_name.c_str (), velocity);
  return send_string (s);
}


//-----------------------------------------------------------
//- AbstractAerotech::set_offset
//-----------------------------------------------------------
void AbstractAerotech::set_offset (double value)
{
  offset = value; 
}
    
//-----------------------------------------------------------
//- AbstractAerotech::get_offset
//-----------------------------------------------------------
double AbstractAerotech::get_offset(void)
{
   return offset;
}

//-----------------------------------------------------------
//- AbstractAerotech::axis_move_abs_user
//-----------------------------------------------------------
bool AbstractAerotech::axis_move_abs_user (double target)
{
  return axis_move_abs (target - offset);
}

//-----------------------------------------------------------
//- AbstractAerotech::get_axis_position_user
//-----------------------------------------------------------
bool AbstractAerotech::get_axis_position_user (double &position)
{
  bool ok;
  ok = get_axis_position (position);
  position = position + offset;
  return ok;
}

//-----------------------------------------------------------
//- AbstractAerotech::compute_new_offset
//-----------------------------------------------------------
bool AbstractAerotech::compute_new_offset (double new_user_pos)
{
  bool ok;
  double position;
  ok = get_axis_position (position);
  offset = (new_user_pos -position);
  return ok;
}


//-----------------------------------------------------------
//- AbstractAerotech::fixe_separateur
//-----------------------------------------------------------
void AbstractAerotech::fixe_separateur (char *stringreceived)
{
  for (int i = 0; stringreceived[i]!= 0; i++)
  {
    if (stringreceived[i] == ',')
      stringreceived[i] = '.';
  }
}

//-----------------------------------------------------------
//- AbstractAerotech::setdoubleregister
//- cA3200 specific function
//- default implementation for polymorphism
//-----------------------------------------------------------
bool AbstractAerotech::setdoubleregister (int number, double value)
{
  //- default implementation : does nothing
  return false;
}

//-----------------------------------------------------------
//- AbstractAerotech::getdoubleregister
//- cA3200 specific function
//- default implementation for polymorphism
//-----------------------------------------------------------
bool AbstractAerotech::getdoubleregister (int number, double &value)
{
  //- default implementation : does nothing
  return false;
}

//-----------------------------------------------------------
//- AbstractAerotech::reset
//- default implementation for polymorphism
//-----------------------------------------------------------
bool AbstractAerotech::reset (void)
{
  //- default implementation : does nothing
  return false;
}

//-----------------------------------------------------------
//- AbstractAerotech::commit_parameters
//- cEnsemble specific function
//- default implementation for polymorphism
//-----------------------------------------------------------
bool AbstractAerotech::commit_parameters (void)
{
  //- default implementation : does nothing
  return false;
}

//-----------------------------------------------------------
//- AbstractAerotech::axis_is_sense_plus
//- cEnsemble specific function
//- default implementation for polymorphism
//-----------------------------------------------------------
bool AbstractAerotech::axis_is_sense_plus (void)
{
  //- default implementation : does nothing
  return false;
}

} //- namespace
