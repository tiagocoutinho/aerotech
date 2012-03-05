static const char *RcsId = "$Id: EnsembleBox.cpp,v 1.3 2012-03-05 08:43:07 jean_coquet Exp $";
//+=============================================================================
//
// file :         EnsembleBox.cpp
//
// description :  C++ source for the EnsembleBox and its commands. 
//                The class is derived from Device. It represents the
//                CORBA servant object which will be accessed from the
//                network. All commands which can be executed on the
//                EnsembleBox are implemented in this file.
//
// project :      TANGO Device Server
//
// $Author: jean_coquet $
//
// $Revision: 1.3 $
//
// $Revision: 1.3 $
// $Date: 2012-03-05 08:43:07 $
//
// SVN only:
// $HeadURL: $
//
// CVS only:
// $Source: /users/chaize/newsvn/cvsroot/Motion/Aerotech/src/EnsembleBox.cpp,v $
// $Log: not supported by cvs2svn $
// Revision 1.2  2012/03/02 15:45:12  jean_coquet
// mise au point avec le materiel
//
// Revision 1.1  2012/02/23 17:46:18  olivierroux
// - initial import #21894
//
//
// copyleft :    Synchrotron SOLEIL 
//               L'Orme des merisiers - Saint Aubin
//               BP48 - 91192 Gif sur Yvette
//               FRANCE
//
//-=============================================================================
//
//  		This file is generated by POGO
//	(Program Obviously used to Generate tango Object)
//
//         (c) - Software Engineering Group - ESRF
//=============================================================================



//===================================================================
//
//	The following table gives the correspondence
//	between commands and method name.
//
//  Command name     |  Method name
//	----------------------------------------
//  State            |  dev_state()
//  Status           |  dev_status()
//  Reset            |  reset()
//  ExecLowLevelCmd  |  exec_low_level_cmd()
//  SaveInFlash      |  save_in_flash()
//
//===================================================================


#include <tango.h>
#include <EnsembleBox.h>
#include <EnsembleBoxClass.h>
#include <helpers/PogoHelper.h>
#include <yat4tango/ExceptionHelper.h>
#include "cEnsemble.h"


namespace EnsembleBox_ns
{

//+----------------------------------------------------------------------------
//
// method : 		EnsembleBox::EnsembleBox(string &s)
// 
// description : 	constructor for simulated EnsembleBox
//
// in : - cl : Pointer to the DeviceClass object
//      - s : Device name 
//
//-----------------------------------------------------------------------------
EnsembleBox::EnsembleBox(Tango::DeviceClass *cl,string &s)
:Tango::Device_4Impl(cl,s.c_str())
{
	init_device();
}

EnsembleBox::EnsembleBox(Tango::DeviceClass *cl,const char *s)
:Tango::Device_4Impl(cl,s)
{
	init_device();
}

EnsembleBox::EnsembleBox(Tango::DeviceClass *cl,const char *s,const char *d)
:Tango::Device_4Impl(cl,s,d)
{
	init_device();
}
//+----------------------------------------------------------------------------
//
// method : 		EnsembleBox::delete_device()
// 
// description : 	will be called at device destruction or at init command.
//
//-----------------------------------------------------------------------------
void EnsembleBox::delete_device()
{
	//	Delete device allocated objects

  DELETE_SCALAR_ATTRIBUTE(attr_oKCommandCounter_read);
	DELETE_SCALAR_ATTRIBUTE(attr_badCommandCounter_read);

  ClassEnsemble::close ();
}

//+----------------------------------------------------------------------------
//
// method : 		EnsembleBox::init_device()
// 
// description : 	will be called at device initialization.
//
//-----------------------------------------------------------------------------
void EnsembleBox::init_device()
{
	INFO_STREAM << "EnsembleBox::EnsembleBox() create device " << device_name << endl;

	// Initialise variables to default values
	//--------------------------------------------
	this->m_status_str = "initializing device...";
	this->m_properties_missing = false;
	this->m_init_device_done = false;

	CREATE_SCALAR_ATTRIBUTE(attr_oKCommandCounter_read, long (0));
	CREATE_SCALAR_ATTRIBUTE(attr_badCommandCounter_read, long (0));

	get_device_property();
  if (m_properties_missing)
    return;

  try
  {
    ClassEnsemble::initialize (const_cast <char *> (iPAddress.c_str ()), port);
  }
  catch (yat::Exception & ye)
  {
    ERROR_STREAM << "initialization failed - YAT exception caught" << std::endl;
    this->m_status_str = "device initialization failed ["
                       + std::string(ye.errors[0].desc)
                       + std::string("]");
    return;
  }
  catch (...)
  {
    ERROR_STREAM << "initialization failed [unknown exception caught]" << std::endl;
    this->m_status_str = "device initialization failed [unknown exception caught]";
    return;
  }
  this->m_init_device_done = true;
}


//+----------------------------------------------------------------------------
//
// method : 		EnsembleBox::get_device_property()
// 
// description : 	Read the device properties from database.
//
//-----------------------------------------------------------------------------
void EnsembleBox::get_device_property()
{
	//	Initialize your default values here (if not done with  POGO).
	//------------------------------------------------------------------
	iPAddress = "must be defined";
  port = 8000;
  tCPTimeOut = 2000;

	//	Read device properties from database.(Automatic code generation)
	//------------------------------------------------------------------
	Tango::DbData	dev_prop;
	dev_prop.push_back(Tango::DbDatum("IPAddress"));
	dev_prop.push_back(Tango::DbDatum("Port"));
	dev_prop.push_back(Tango::DbDatum("TCPTimeOut"));

	//	Call database and extract values
	//--------------------------------------------
	if (Tango::Util::instance()->_UseDb==true)
		get_db_device()->get_property(dev_prop);
	Tango::DbDatum	def_prop, cl_prop;
	EnsembleBoxClass	*ds_class =
		(static_cast<EnsembleBoxClass *>(get_device_class()));
	int	i = -1;

	//	Try to initialize IPAddress from class property
	cl_prop = ds_class->get_class_property(dev_prop[++i].name);
	if (cl_prop.is_empty()==false)	cl_prop  >>  iPAddress;
	else {
		//	Try to initialize IPAddress from default device value
		def_prop = ds_class->get_default_device_property(dev_prop[i].name);
		if (def_prop.is_empty()==false)	def_prop  >>  iPAddress;
	}
	//	And try to extract IPAddress value from database
	if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  iPAddress;

	//	Try to initialize Port from class property
	cl_prop = ds_class->get_class_property(dev_prop[++i].name);
	if (cl_prop.is_empty()==false)	cl_prop  >>  port;
	else {
		//	Try to initialize Port from default device value
		def_prop = ds_class->get_default_device_property(dev_prop[i].name);
		if (def_prop.is_empty()==false)	def_prop  >>  port;
	}
	//	And try to extract Port value from database
	if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  port;

	//	Try to initialize TCPTimeOut from class property
	cl_prop = ds_class->get_class_property(dev_prop[++i].name);
	if (cl_prop.is_empty()==false)	cl_prop  >>  tCPTimeOut;
	else {
		//	Try to initialize TCPTimeOut from default device value
		def_prop = ds_class->get_default_device_property(dev_prop[i].name);
		if (def_prop.is_empty()==false)	def_prop  >>  tCPTimeOut;
	}
	//	And try to extract TCPTimeOut value from database
	if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  tCPTimeOut;



	//	End of Automatic code generation
	//------------------------------------------------------------------
  Tango::DbData data_put;
  if (dev_prop[0].is_empty()==true || iPAddress.find ("must be defined") != std::string::npos)
	{
    ERROR_STREAM << "EnsembleBox::get_device_property IPAddress not defined" << std::endl;
    this->m_status_str = "EnsembleBox::get_device_property IPAddress not defined \n[set IP Address and restart device]";
		this->m_properties_missing = true;
		Tango::DbDatum	property("IPAddress");
		property	<<	iPAddress;
		data_put.push_back(property);
	}
	if(!data_put.empty())
		get_db_device()->put_property(data_put);
}
//+----------------------------------------------------------------------------
//
// method : 		EnsembleBox::always_executed_hook()
// 
// description : 	method always executed before any command is executed
//
//-----------------------------------------------------------------------------
void EnsembleBox::always_executed_hook()
{
	
}
//+----------------------------------------------------------------------------
//
// method : 		EnsembleBox::read_attr_hardware
// 
// description : 	Hardware acquisition for attributes.
//
//-----------------------------------------------------------------------------
void EnsembleBox::read_attr_hardware(vector<long> &attr_list)
{
	DEBUG_STREAM << "EnsembleBox::read_attr_hardware(vector<long> &attr_list) entering... "<< endl;
	//	Add your own code here
}

//+----------------------------------------------------------------------------
//
// method : 		EnsembleBox::read_oKCommandCounter
// 
// description : 	Extract real attribute values for oKCommandCounter acquisition result.
//
//-----------------------------------------------------------------------------
void EnsembleBox::read_oKCommandCounter(Tango::Attribute &attr)
{
	DEBUG_STREAM << "EnsembleBox::read_oKCommandCounter(Tango::Attribute &attr) entering... "<< endl;
  *attr_oKCommandCounter_read = ENSEMBLE_PROXY->get_com_ok_counter ();
  attr.set_value (attr_oKCommandCounter_read);
}

//+----------------------------------------------------------------------------
//
// method : 		EnsembleBox::read_badCommandCounter
// 
// description : 	Extract real attribute values for badCommandCounter acquisition result.
//
//-----------------------------------------------------------------------------
void EnsembleBox::read_badCommandCounter(Tango::Attribute &attr)
{
	DEBUG_STREAM << "EnsembleBox::read_badCommandCounter(Tango::Attribute &attr) entering... "<< endl;
  *attr_badCommandCounter_read = ENSEMBLE_PROXY->get_com_error_counter ();
  attr.set_value (attr_badCommandCounter_read);
}

//+------------------------------------------------------------------
/**
 *	method:	EnsembleBox::reset
 *
 *	description:	method to execute "Reset"
 *	Reset the controller
 *
 *
 */
//+------------------------------------------------------------------
void EnsembleBox::reset()
{
	DEBUG_STREAM << "EnsembleBox::reset(): entering... !" << endl;
	//	Add your own code to control device here

  ENSEMBLE_PROXY->reset ();
  //- TODO : try this?
//-   this->delete_device ();
//-   this->init_device ();
}


//+------------------------------------------------------------------
/**
 *	method:	EnsembleBox::dev_state
 *
 *	description:	method to execute "State"
 *	This command gets the device state (stored in its <i>device_state</i> data member) and returns it to the caller.
 *
 * @return	State Code
 *
 */
//+------------------------------------------------------------------
Tango::DevState EnsembleBox::dev_state()
{
  Tango::DevState	argout = Tango::UNKNOWN;
	DEBUG_STREAM << "EnsembleBox::dev_state(): entering... !" << endl;

	//	Add your own code to control device here
  if (!m_init_device_done || m_properties_missing)
  {
    argout = Tango::FAULT;
    set_state (argout);
    return argout;
  }
  bool connected = ENSEMBLE_PROXY->connected_ok ();
  argout = connected? Tango::OPEN : Tango::CLOSE;
	set_state(argout);
	return argout;
}

//+------------------------------------------------------------------
/**
 *	method:	EnsembleBox::dev_status
 *
 *	description:	method to execute "Status"
 *	This command gets the device status (stored in its <i>device_status</i> data member) and returns it to the caller.
 *
 * @return	Status description
 *
 */
//+------------------------------------------------------------------
Tango::ConstDevString EnsembleBox::dev_status()
{
	DEBUG_STREAM << "EnsembleBox::dev_status(): entering... !" << endl;

	//	Add your own code to control device here
  if (!m_init_device_done || m_properties_missing)
  {
    set_status (m_status_str.c_str ());
    return m_status_str.c_str ();
  }
  m_status_str.clear ();
  m_status_str = "device is up and running\n";
  bool connected = ENSEMBLE_PROXY->connected_ok ();
  if (connected)
    m_status_str += "communication opened\n";
  else
    m_status_str += "communication closed\n";
  set_status (m_status_str.c_str ());
  return m_status_str.c_str ();
}

//+------------------------------------------------------------------
/**
 *	method:	EnsembleBox::exec_low_level_cmd
 *
 *	description:	method to execute "ExecLowLevelCmd"
 *	executes a Aerotech cmd and returns the response
 *	** WARNING : EXPERT USERS ONLY! YOU CAN CRASH THE CONTROLLER! **
 *
 * @param	argin	
 * @return	
 *
 */
//+------------------------------------------------------------------
Tango::DevString EnsembleBox::exec_low_level_cmd(Tango::DevString argin)
{
	//	POGO has generated a method core with argout allocation.
	//	If you would like to use a static reference without copying,
	//	See "TANGO Device Server Programmer's Manual"
	//		(chapter : Writing a TANGO DS / Exchanging data)
	//------------------------------------------------------------
	Tango::DevString	argout  = new char[256];
  ::memset (argout, 0, 256);


	DEBUG_STREAM << "EnsembleBox::exec_low_level_cmd(): entering... !" << endl;

	//	Add your own code to control device here
  if (!ENSEMBLE_PROXY->lowlevelcmd (argin, argout))
    THROW_DEVFAILED ("COMMAND_FAILED",
                    "command failed [controller refused command]",
                    "EnsembleBox::exec_low_level_cmd");
	return argout;
}


//+------------------------------------------------------------------
/**
 *	method:	EnsembleBox::save_in_flash
 *
 *	description:	method to execute "SaveInFlash"
 *	writes in non volatile memory the parameters for all controllers
 *
 *
 */
//+------------------------------------------------------------------
void EnsembleBox::save_in_flash()
{
	DEBUG_STREAM << "EnsembleBox::save_in_flash(): entering... !" << endl;

	//	Add your own code to control device here
  if (! m_init_device_done || m_properties_missing)
    THROW_DEVFAILED ("OPERATION_NOT_ALLOWED",
                     "device not properly initialized [check properties, communication lost]",
                     "EnsembleBox::save_in_flash");
  bool ok = ENSEMBLE_PROXY->commit_parameters ();
  if (!ok)
   THROW_DEVFAILED ("COMMAND_FAILED",
                    "command failed [controller refused command]",
                    "EnsembleAxis::save_in_flash");}

}	//	namespace
