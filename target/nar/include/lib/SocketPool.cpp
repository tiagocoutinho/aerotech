//-------------------------------------------------------------------
//- project : classes for Aerotech control
//- file : SocketPool.cpp : Socket management
//-  Aerotech allows an unique user socket to share between multiple Axes 
//- so create a singleton to manage it
//-------------------------------------------------------------------
#include <iostream>

#include "SocketPool.h"
#include <yat4tango/ExceptionHelper.h>

namespace Aerotech_ns
{

//------------------------------------------------
// SocketPool : Singleton Management
//-----------------------------------------------
SocketPool * SocketPool::singleton = 0;

//----------------------------------------
// SocketPool CTOR
//----------------------------------------

SocketPool::SocketPool () : 
  internal_state (SW_NOT_INITIALIZED),
  com_ok_counter (0),
  com_error_counter (0)
{
	status_str.resize (512);
}

// ============================================================================
// Config::Config
// ============================================================================
SocketPool::Config::Config ()
{
  ip_address           = "Not Initialised";
  port                 = 0;
  in_tmout             = 500;
  o_tmout              = 500;
  start_tmout          = 2000;
}

SocketPool::Config::Config (const Config & _src)
{
  *this = _src;
}
// ============================================================================
// Config::operator =
// ============================================================================
void SocketPool::Config::operator = (const Config & _src)
{
  ip_address           = _src.ip_address;
  port                 = _src.port;
  in_tmout             = _src.in_tmout;
  o_tmout              = _src.o_tmout;
  start_tmout          = _src.start_tmout;
}

//----------------------------------------
// SocketPool DTOR
//----------------------------------------
SocketPool::~SocketPool ()
{
  this->internal_state = SW_NOT_CONFIGURED;
	// Cleanup all created sockets

  sock->disconnect ();


  if (sock)
    delete sock;
  sock = 0;

  yat::Socket::terminate ();
}


//----------------------------------------
// SocketPool Singleton Initialization  
//----------------------------------------

void SocketPool::initialize (void)
{
  this->internal_state = SW_NOT_INITIALIZED;

  if (!singleton)
    singleton = new SocketPool ();
  if (!singleton)
  {
    this->internal_state = SW_ALLOC_ERROR;
    THROW_DEVFAILED ("OUT_OF_MEMORY",
                     "SocketPool Singleton creation failed",
                     "SocketPool::initialize ()");
  }
  this->internal_state = SW_INITIALIZED;
}

//----------------------------------------
// Release (delete) SocketPool
//----------------------------------------
void SocketPool::release (void)
{
  if (singleton)
    delete singleton;
  singleton = 0;
}
  
//----------------------------------------
// SocketPool socket creation 
//----------------------------------------
void SocketPool::configure (const Config & c)
{

  std::cout << "*********************** SocketPool::configure entering!********************" << std::endl;

  if (this->internal_state != SW_INITIALIZED)
    THROW_DEVFAILED ("SOFTWARE ERROR",
                     "SocketPool Singleton creation failed",
                     "SocketPool::configure ()");

  sock = 0;
  this->conf = c;

  //- YAT socket magic stuff
  try
  {
    yat::Socket::init();
    //- for blocking sockets (moving, homing...)
    sock     = new yat::ClientSocket ();
    sock->set_option(yat::Socket::SOCK_OPT_KEEP_ALIVE, 1);
    sock->set_option(yat::Socket::SOCK_OPT_NO_DELAY, 1);
    sock->set_option(yat::Socket::SOCK_OPT_OTIMEOUT, conf.o_tmout);
    sock->set_option(yat::Socket::SOCK_OPT_ITIMEOUT, conf.in_tmout);

    //- connect to the Socket (default protocol is yat::Socket::TCP_PROTOCOL)
    yat::Address addr(conf.ip_address, conf.port);
    sock->connect(addr);
    std::cout << "********** SocketPool::configure connected OK" << std::endl;
    this->internal_state = SW_CONFIGURED;
  }
  catch (yat::SocketException &e)
  {
    this->internal_state = SW_INITIALIZATION_ERROR;
    {
      yat::AutoMutex <yat::Mutex> guard (this->status_lock);
      status_str = "Sockets Initialization Error caught SocketException <" + e.errors[0].desc + "> [check properties, Kill/Restart Dserver]\n";
      THROW_DEVFAILED ("OPERATION_NOT_ALLOWED",
                       "Socket creation failed",
                       "SocketPool::initialize()");
    }
  }
  catch (...)
  {
    this->internal_state = SW_INITIALIZATION_ERROR;
    {
      yat::AutoMutex <yat::Mutex> guard (this->status_lock);
      status_str = "Sockets Initialization Error caught (...) [check properties, Kill/Restart Dserver]\n";
      THROW_DEVFAILED ("OPERATION_NOT_ALLOWED",
                       "Socket creation failed",
                       "SocketPool::initialize()");
    }
  }
  this->internal_state = SW_CONFIGURED;
  {
    yat::AutoMutex <yat::Mutex> guard (this->status_lock);
    status_str = "Socket Initialialized\n";
  }
}

//------------------------------------------------
// SocketPool : Interface to YAT Socket signature
//-----------------------------------------------
void SocketPool::write_read(std::string cmd, std::string & resp)
{
  try
  {
    yat::AutoMutex <yat::Mutex> guard (this->sock_lock);
    sock->send (cmd);
    sock->receive (resp);
    com_ok_counter ++;

  }
  catch (yat::SocketException &e)
  {
    com_error_counter++;
    THROW_YAT_TO_TANGO_EXCEPTION (e);

  }
  catch (...)
  {
    com_error_counter++;
    THROW_DEVFAILED ("UNKNOWN_ERROR",
                     "(...) exception trying to communicate with HW [check Aerotech, call programmer, love him better]",
                     "SocketPool::write_read()");

  }
}

} //- namespace


