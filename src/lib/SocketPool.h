//-------------------------------------------------------------------
//- project : classes for Aerotech control
//- file : SocketPool.h : Socket management
//-  Aerotech allows an unique user socket to share between multiple Axes 
//- so create a singleton to manage it
//-------------------------------------------------------------------
#ifndef __SOCKETPOOL_H__
#define __SOCKETPOOL_H__

#include <yat/network/ClientSocket.h>
#include <yat/threading/Mutex.h>
#include <vector>
#include <iostream>

namespace Aerotech_ns
{
// ============================================================================
// SHORTCUT TO THE <SocketPool> SINGLETON
// ============================================================================
#define SOCK_POOL SocketPool::instance ()

  static const std::string SocketStatesStr[8] =
  {
    "SW_UNKNOWN_STATE",
    "SW_NOT_INITIALIZED",
    "SW_INITIALIZED",
    "SW_INITIALIZATION_ERROR",
    "SW_NOT_CONFIGURED",
    "SW_ALLOC_ERROR",
    "SW_CONFIGURED",
    "SW_CONFIGURATION_ERROR"
  };

class SocketPool
{

//-------------------------------------------
public: 
  //-------------------------------------------
  //- configs, enums, const
  //-------------------------------------------
  typedef struct Config
  {
    std::string ip_address;
    int port;
    int in_tmout;
    int o_tmout;
    int start_tmout;

    //- Ctor -------------
    Config ();
    //- Ctor -------------
    Config ( const Config & _src);
    //- operator = -------
    void operator = (const Config & src);
  } Config;
  //----------------------------------------
  typedef enum
  {
    SW_UNKNOWN_STATE,
    SW_NOT_INITIALIZED,
    SW_INITIALIZED,
    SW_INITIALIZATION_ERROR,
    SW_NOT_CONFIGURED,
    SW_ALLOC_ERROR,
    SW_CONFIGURED,
    SW_CONFIGURATION_ERROR
  }ClassState;

  //----------------------------------------

	//- creation of singleton if not already done
  void initialize (void);

	//- Manage lists of blocking and non-blocking sockets
  void configure (const Config & conf);

  //- release SocketPool
  void release (void);

  //-------------------------------------------
  //- get the class state /status
  ClassState state (void) 
  {
    //- std::cout << "SocketPool::State = " << SocketStatesStr [internal_state] << std::endl;
    return internal_state;
  }
  std::string status (void) 
  {
    yat::AutoMutex <yat::Mutex> guard (this->status_lock);
    return status_str;
  }


  //-------------------------------------------
	// Provide an interface to YAT Sockets
  void write_read (std::string cmd, std::string & resp);

  //-------------------------------------------
	static SocketPool * instance ()
	{
		if (! SocketPool::singleton)
			SocketPool::singleton = new SocketPool();
		return SocketPool::singleton;
	}

  long get_comm_success_counter (void)
  { return com_ok_counter;};
  long get_comm_errors_counter (void)
  { return com_error_counter;};


//-------------------------------------------
private :

	//----------------------------------------
  //- CTOR / DTOR
	//----------------------------------------
  SocketPool (); 
	~SocketPool ();

	//----------------------------------------
	//- the singleton 
	//----------------------------------------
	static SocketPool * singleton;

	//----------------------------------------
  //- configuration structure
  Config conf;

	//----------------------------------------
	// The list of all created sockets
  yat::ClientSocket * sock;

	//----------------------------------------
  //- the class state/status and usefull stuff
  ClassState internal_state;
  std::string status_str;
  yat::Mutex status_lock;
  yat::Mutex sock_lock;

  //- communication counters
  long com_ok_counter;
  long com_error_counter;


};
} //- namespace
#endif  //__SOCKETPOOL_H__
