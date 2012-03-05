//---------------------------------------------------------------------------
#include "cEnsemble.h"
#include <iostream>

// Version 1.00  JP Gaillet 13/12/2011
//---------------------------------------------------------------------------

// ============================================================================
// SINGLETON Initialisation 
// ============================================================================
ClassEnsemble * ClassEnsemble::singleton = 0;


/************************************************************
* Comments about these functions are in "cEnsemble.h" file
************************************************************/

bool ClassEnsemble::send_string(char *string_to_send)  //send string to controller
{  //send string to controller

  char string_received[SIZE_BUFFER];
  send_and_receive(string_to_send,string_received);
  return (string_received[0]=='%');
}

bool ClassEnsemble::send_receive_and_test(char *to_send, char *received)
{      //send string to controller and return answer in "received"
  bool ok;
  ok=send_and_receive(to_send,received);
  if (received[0]!=0)
  {
    // strcpy(received,received+1);
    received[0] = ' ';
  }
  return ok;
}

bool ClassEnsemble::axis_enable(char *axis_name)
{
  if (! is_ready_to_accept_cmd (axis_name))
    return false;
  char s[SIZE_BUFFER];
  sprintf(s,"ENABLE %s \n",axis_name);
  return send_string(s);
}

bool ClassEnsemble::axis_disable(char *axis_name)
{
  if (! is_ready_to_accept_cmd (axis_name))
    return false;
  char s[SIZE_BUFFER];
  sprintf(s,"DISABLE %s \n",axis_name);
  return send_string(s);
}

bool ClassEnsemble::axis_home(char *axis_name)
{
  if (! is_ready_to_accept_cmd (axis_name))
    return false;
  int status = 0;
  if (!get_axis_status (axis_name, status))
    return false;
  if (! is_enabled (status))
    return false;
  if ( is_homing (status) || is_moving (status))
    return false;
  char s[SIZE_BUFFER];
  sprintf(s,"HOMEASYNC %s \n",axis_name);
  return send_string(s);
}

bool ClassEnsemble::commit_parameters()
{
  if (! is_ready_to_accept_cmd (const_cast <char*> ("ALL")))
    return false;
  char s[SIZE_BUFFER];
  sprintf(s,"COMITPARAMETERS \n");
  return send_string(s);
}

bool ClassEnsemble::axis_abort(char *axis_name)
{
  if (! is_ready_to_accept_cmd (axis_name))
    return false;
  char s[SIZE_BUFFER];
  sprintf(s,"ABORT %s \n",axis_name);
  return send_string(s);
}

bool ClassEnsemble::axis_fault_ack(char *axis_name)
{
  char s[SIZE_BUFFER];
  sprintf(s,"FAULTACK %s \n",axis_name);
  return send_string(s);
}

bool ClassEnsemble::axis_brake_on(char *axis_name)
{
  if (! is_ready_to_accept_cmd (axis_name))
    return false;
  char s[SIZE_BUFFER];
  sprintf(s,"BRAKE %s ON\n",axis_name);
  return send_string(s);
}

bool ClassEnsemble::axis_brake_off(char *axis_name)
{
  if (! is_ready_to_accept_cmd (axis_name))
    return false;
  char s[SIZE_BUFFER];
  sprintf(s,"BRAKE %s OFF\n",axis_name);
  return send_string(s);
}

bool ClassEnsemble::axis_move_abs(char *axis_name,double target)
{
  if (! is_ready_to_accept_cmd (axis_name))
    return false;
  char sformat[SIZE_BUFFER];
  char s[SIZE_BUFFER];
  this->moving_timeout.restart ();
  sprintf(sformat,"MOVEABS %%s  %%.%df\n",NB_DIGITS);
  sprintf(s,sformat,axis_name,target);
  return send_string(s);
}

bool ClassEnsemble::axis_move_rel(char *axis_name,double target)
{
  if (! is_ready_to_accept_cmd (axis_name))
    return false;
  char sformat[SIZE_BUFFER];
  char s[SIZE_BUFFER];
  this->moving_timeout.restart ();
  // first, create a format corresponding to "NB_DIGITS" 
  sprintf(sformat,"MOVEINC %%s  %%.%df\n",NB_DIGITS);
  // then create the string to be sent to controller using this format 
  sprintf(s,sformat,axis_name,target);
  return send_string(s);
}

bool ClassEnsemble::get_axis_position(char *axis_name,double &position)
{
  char spos[SIZE_BUFFER];
  char string_received[SIZE_BUFFER];
  bool ok;
  sprintf(spos,"PFBK %s \n",axis_name);
  ok=send_receive_and_test(spos,string_received);
  double tmp = 0.;
  sscanf(string_received,"%lf", &tmp);
  position = tmp;
  return ok;
}

bool ClassEnsemble::get_axis_position_error(char *axis_name,double &position)
{
  char s[SIZE_BUFFER];
  char string_received[SIZE_BUFFER];
  bool ok;
  sprintf(s,"PERR %s \n",axis_name);
  ok=send_receive_and_test(s,string_received);
  double tmp = 0.;
  sscanf(string_received,"%lf",&tmp);
  position = tmp;
  return ok;
}

bool ClassEnsemble::get_axis_velocity(char *axis_name,double &velocity)
{
  char s[SIZE_BUFFER];
  char string_received[SIZE_BUFFER];
  bool ok;
  sprintf(s,"VFBK %s \n",axis_name);
  ok=send_receive_and_test(s,string_received);
  sscanf(string_received,"%lf",&velocity);
  return ok;
}

bool ClassEnsemble::get_axis_status(char *axis_name, int &status)
{
  char s[SIZE_BUFFER];
  char string_received[SIZE_BUFFER];
  bool ok;
  sprintf(s,"AXISSTATUS %s \n",axis_name);
  ok=send_receive_and_test(s,string_received);
  sscanf(string_received,"%i",&status);
  return ok;
}

bool ClassEnsemble::get_axis_fault_status(char *axis_name, int &fault_status)
{
  char s[SIZE_BUFFER];
  char string_received[SIZE_BUFFER];
  bool ok;
  sprintf(s,"AXISFAULT %s \n",axis_name);
  ok=send_receive_and_test(s,string_received);
  sscanf(string_received,"%d",&fault_status);
  return ok;
}

bool ClassEnsemble::is_ready_to_accept_cmd (char *axis_name)
{
  int f_status = 0;
  if (!get_axis_fault_status (axis_name, f_status))
    return false;

  return f_status == 0;
}


void ClassEnsemble::error_to_string( int fault_status,char *explanation)
{               // This procedure creates as many strings as many faults are found
  // the strings are separated by "cariage return"
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

bool ClassEnsemble::get_param(char *axis_name,char *param,char *string_received)
{
  char s[SIZE_BUFFER];
  bool ok;
  sprintf(s,"GETPARM(%s,%s)\n",axis_name,param);
  ok=send_receive_and_test(s,string_received);
  return ok;
}

bool ClassEnsemble::set_param(char *axis_name,char *param,char *string_to_send)
{
  if (! is_ready_to_accept_cmd (axis_name))
    return false;
  char s[SIZE_BUFFER];
  bool ok;
  sprintf(s,"SETPARM(%s,%s,%s)\n",axis_name,param,string_to_send);
  ok=send_string(s);
  return ok;
}

bool ClassEnsemble::get_axis_software_limit_low(char *axis_name,double &limit)
{
  char string_received[SIZE_BUFFER];
  bool ok;
  ok=get_param(axis_name,const_cast <char*> ("softwareLimitLow"),string_received);
  sscanf(string_received,"%lf",&limit);
  return ok;
}

bool ClassEnsemble::get_axis_software_limit_high(char *axis_name,double &limit)
{
  char string_received[SIZE_BUFFER];
  bool ok;
  ok=get_param(axis_name,const_cast <char*> ("softwareLimitHigh"),string_received);
  sscanf(string_received,"%lf",&limit);
  return ok;
}

bool ClassEnsemble::set_axis_software_limit_low(char *axis_name,double limit)
{
  char s[SIZE_BUFFER];
  bool ok;
  sprintf(s,"%lf",limit);
  ok=set_param(axis_name,const_cast <char*> ("softwareLimitLow"),s);
  return ok;;
}

bool ClassEnsemble::set_axis_software_limit_high(char *axis_name,double limit)
{
  char s[SIZE_BUFFER];
  bool ok;
  sprintf(s,"%lf",limit);
  ok=set_param(axis_name,const_cast <char*> ("softwareLimitHigh"),s);
  return ok;
}


bool ClassEnsemble::set_axis_velocity(char *axis_name,double velocity)
{
  if (! is_ready_to_accept_cmd (axis_name))
    return false;
  char sformat[SIZE_BUFFER];
  char s[SIZE_BUFFER];
  sprintf(sformat,"VCMD %%s  %%.%df\n",NB_DIGITS);
  sprintf(s,sformat,axis_name,velocity);
  return send_string(s);
}

bool ClassEnsemble::get_axis_error_limit(char *axis_name,double &limit)
{
  char string_received[SIZE_BUFFER];
  bool ok;
  ok=get_param(axis_name,const_cast <char*> ("PositionErrorThreshold"),string_received);
  sscanf(string_received,"%lf",&limit);
  return ok;
}

bool ClassEnsemble::set_axis_error_limit(char *axis_name,double limit)
{
  char s[SIZE_BUFFER];
  sprintf(s,"%lf",limit);
  return(set_param(axis_name,const_cast <char*> ("PositionErrorThreshold"),s));
}

bool ClassEnsemble::get_axis_default_speed(char *axis_name,double &speed)
{
  char string_received[SIZE_BUFFER];
  bool ok;
  ok=get_param(axis_name,const_cast <char*> ("DefaultSpeed"),string_received);
  sscanf(string_received,"%lf",&speed);
  return ok;
}

bool ClassEnsemble::set_axis_default_speed(char *axis_name,double speed)
{
  char s[SIZE_BUFFER];
  char sformat[SIZE_BUFFER];
  bool ok;
  sprintf(s,"%lf",speed);
  ok=set_param(axis_name,const_cast <char*> ("DefaultSpeed"),s);
  if (ok)
  {
    // send a "MOVEINC Axis F speed" - a relative nul displacement to set the velocity
    sprintf(sformat,"MOVEINC %%s F %%.%df\n",NB_DIGITS);
    sprintf(s,sformat,axis_name,speed);
    ok= send_string(s);
  }
  return ok;
}

bool ClassEnsemble::get_axis_ramp_rate(char *axis_name,double &ramp_rate)
{
  char string_received[SIZE_BUFFER];
  bool ok;
  ok=get_param(axis_name,const_cast <char*> ("Defaultramprate"),string_received);
  sscanf(string_received,"%lf",&ramp_rate);
  return ok;
}

bool ClassEnsemble::set_axis_ramp_rate(char *axis_name,double ramp_rate)
{
  if (! is_ready_to_accept_cmd (axis_name))
    return false;
  char s[SIZE_BUFFER];
  char sformat[SIZE_BUFFER];
  bool ok;
  sprintf(s,"%lf",ramp_rate);
  ok=set_param(axis_name,const_cast <char*> ("Defaultramprate"),s);
  if (ok)
  {
    sprintf(sformat,"RAMP RATE %%s  %%.%df\n",NB_DIGITS);
    sprintf(s,sformat,axis_name,ramp_rate);
    ok= send_string(s);
  }
  return ok;
}

bool ClassEnsemble::get_axis_home_speed(char *axis_name,double &home_speed)
{
  char string_received[SIZE_BUFFER];
  bool ok;
  ok=get_param(axis_name,const_cast <char*> ("homespeed"),string_received);
  sscanf(string_received,"%lf",&home_speed);
  return ok;
}

bool ClassEnsemble::set_axis_home_speed(char *axis_name,double home_speed)
{
  char s[SIZE_BUFFER];
  sprintf(s,"%lf",home_speed);
  return (set_param(axis_name,const_cast <char*> ("homespeed"),s));
}

bool ClassEnsemble::get_axis_home_offset(char *axis_name,double &offset)
{
  char string_received[SIZE_BUFFER];
  bool ok;
  ok=get_param(axis_name,const_cast <char*> ("homeoffset"),string_received);
  sscanf(string_received,"%lf",&offset);
  return ok;
}

bool ClassEnsemble::set_axis_home_offset(char *axis_name,double offset)
{
  char s[SIZE_BUFFER];
  sprintf(s,"%lf",offset);
  return (set_param(axis_name,const_cast <char*> ("homeoffset"),s));
}

bool ClassEnsemble::lowlevelcmd(char * argin, char * argout)
{
  char tmp [256];
  ::memset (tmp, 0, 256);
  strncpy (tmp, argin, 254);
  strcat (tmp, "\n");
  return send_and_receive (tmp, argout);
}


bool ClassEnsemble::is_enabled( int status)
{
  return (status & 1)!=0;
}

bool ClassEnsemble::is_homed( int status)
{
  return (status & 2)!=0;
}

bool ClassEnsemble::is_in_position( int status)
{
  return (status & 4)!=0;
}

bool ClassEnsemble::is_moving( int status)
{
  //- min time  
  if (this->moving_timeout.time_to_expiration () > 0)
    return true;

  return (status & 8)!=0;
}

bool ClassEnsemble::is_accelerating( int status)
{
  return (status & 0x10)!=0;
}

bool ClassEnsemble::is_decelerating( int status)
{
  return (status & 0x20)!=0;
}

bool ClassEnsemble::is_accel_or_decel( int status)
{
  return (status & 0x30)!=0;
}

bool ClassEnsemble::is_sense_plus( int status)
{
  return (status & 0x200)!=0;
}

bool ClassEnsemble::is_homing( int status)
{
  return (status & 0x4000)!=0;
}

bool ClassEnsemble::is_cw_EOT( int status)
{
  return ((status & 0x400000)!=0);
}

bool ClassEnsemble::is_ccw_EOT( int status)
{
  return ((status & 0x800000)!=0);
}

bool ClassEnsemble::is_EOT( int status)
{
  return ((status & 0xC00000)!=0);
}

bool ClassEnsemble::is_encoder_error( int status)
{
  return (status & 0x60000000)!=0;
}

bool ClassEnsemble::is_emergency_stop( int status)
{
  return (status & 0x20000)!=0;
}

bool ClassEnsemble::is_brake_on( int status)
{
  return (status & 0x100)!=0;
}

bool ClassEnsemble::is_brake_off( int status)
{
  return (status & 0x100)==0;
}

bool ClassEnsemble::reset()
{
  bool ok;
  ok=send_string(const_cast <char*> ("RESET\n"));
  if (ok) 
  {
    is_connected=false;
  }
  return ok;    
}

bool ClassEnsemble::create_socket(char *address,int port)
{
  yat::Socket::init();

  try
  {
    sock = new yat::ClientSocket ();
    sock->set_option(yat::Socket::SOCK_OPT_KEEP_ALIVE, 1);
    sock->set_option(yat::Socket::SOCK_OPT_NO_DELAY, 1);
    sock->set_option(yat::Socket::SOCK_OPT_OTIMEOUT, 1000);
    sock->set_option(yat::Socket::SOCK_OPT_ITIMEOUT, 2500);

    yat::Address adr (address, port);
    sock->connect (adr);
    return true;
  }
  catch (yat::SocketException &ye)
  {
    return false;
  }
  catch (...)
  {
    return false;
  }

};

bool ClassEnsemble::close_socket()
{
  try
  {
    sock->disconnect ();
    delete sock;
    yat::Socket::terminate();
  }
  catch (...)
  {}
  return true;

};

void ClassEnsemble::WithoutCRLF(char *chaine)
{
  int i;
  for ( i=0 ; chaine[i]!=0  ; i++ )
  {
    if (chaine[i]==13) chaine[i]=0;
    if (chaine[i]==10) chaine[i]=0;
  }
}

//-------------------------------------------------------------------------
bool ClassEnsemble::send_and_receive(char *command, char *received)
{
  try
  {
    { //- CRITICAL SECTION
      yat::AutoMutex <yat::Mutex> guard (this->m_lock);
      sock->send (command);
      sock->receive (received, size_t (SIZE_BUFFER));
    } //- END CRITICAL SECTION
    this->com_ok_counter ++;
    return true;
  }
  catch (yat::SocketException &ye)
  {
    this->com_error_counter ++;
    return false;
  }
  catch (...)
  {
    this->com_error_counter ++;
    return false;
  }
}

//-------------------------------------------------------------------------
bool ClassEnsemble::send_and_receive(std::string cmd, std::string & resp)
{
  try
  {
    sock->send (cmd);
    sock->receive (resp);
    this->com_ok_counter ++;
    return true;
  }
  catch (yat::SocketException &ye)
  {
    this->com_error_counter ++;
    return false;
  }
  catch (...)
  {
    this->com_error_counter ++;
    return false;
  }
}
//-------------------------------------------------------------------------
// constructor
ClassEnsemble::ClassEnsemble(char *address,int port) : sock (0)
{

  std::cout << "ClassEnsemble::ClassEnsemble entering <-" << std::endl;

  this->com_ok_counter = 0;
  this->com_error_counter = 0;
  //- configure the moving timer
  is_connected = false;
  this->moving_timeout.set_unit (yat::Timeout::TMO_UNIT_MSEC);
  this->moving_timeout.set_value (25.0);
  this->moving_timeout.restart ();


  try
  {
    std::cout << "ClassEnsemble::ClassEnsemble try to connect to " << address << " port " << port << std::endl;
    is_connected = create_socket (address, port);
    std::cout << "ClassEnsemble::ClassEnsemble connected = " << is_connected << std::endl;
  }
  catch (yat::Exception &ye)
  {
    std::cout << "ClassEnsemble::ClassEnsemble caught yat Exception trying to create socket on " << address 
      << " desc : [" << ye.errors[0].desc << "]" << std::endl;
    is_connected = false;
  }
  catch (...)
  {
    std::cout << "ClassEnsemble::ClassEnsemble caught (...) trying to create socket on " << address << std::endl;
    is_connected = false;
  }
  //- something went wrong creating socket
  if (sock == 0)
    is_connected = false;
}

//-------------------------------------------------------------------------
// destructor
ClassEnsemble::~ClassEnsemble()
{
  close_socket ();
  sock = 0;
}



