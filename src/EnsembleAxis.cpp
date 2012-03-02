static const char *RcsId = "$Id: EnsembleAxis.cpp,v 1.2 2012-03-02 15:45:12 jean_coquet Exp $";
//+=============================================================================
//
// file :         EnsembleAxis.cpp
//
// description :  C++ source for the EnsembleAxis and its commands. 
//                The class is derived from Device. It represents the
//                CORBA servant object which will be accessed from the
//                network. All commands which can be executed on the
//                EnsembleAxis are implemented in this file.
//
// project :      TANGO Device Server
//
// $Author: jean_coquet $
//
// $Revision: 1.2 $
//
// $Revision: 1.2 $
// $Date: 2012-03-02 15:45:12 $
//
// SVN only:
// $HeadURL: $
//
// CVS only:
// $Source: /users/chaize/newsvn/cvsroot/Motion/Aerotech/src/EnsembleAxis.cpp,v $
// $Log: not supported by cvs2svn $
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
//  Command name                 |  Method name
//	----------------------------------------
//  State                        |  dev_state()
//  Status                       |  dev_status()
//  Stop                         |  stop()
//  InitializeReferencePosition  |  initialize_reference_position()
//  BrakeON                      |  brake_on()
//  BrakeOFF                     |  brake_off()
//  FaultAck                     |  fault_ack()
//  Enable                       |  enable()
//  Disable                      |  disable()
//
//===================================================================


#include <tango.h>
#include <EnsembleAxis.h>
#include <EnsembleAxisClass.h>
#include <helpers/PogoHelper.h>
#include <yat4tango/ExceptionHelper.h>
#include <yat/Portability.h>
#include "cEnsemble.h"

namespace EnsembleAxis_ns
{

//+----------------------------------------------------------------------------
//
// method : 		EnsembleAxis::EnsembleAxis(string &s)
// 
// description : 	constructor for simulated EnsembleAxis
//
// in : - cl : Pointer to the DeviceClass object
//      - s : Device name 
//
//-----------------------------------------------------------------------------
EnsembleAxis::EnsembleAxis(Tango::DeviceClass *cl,string &s)
:Tango::Device_4Impl(cl,s.c_str())
{
	init_device();
}

EnsembleAxis::EnsembleAxis(Tango::DeviceClass *cl,const char *s)
:Tango::Device_4Impl(cl,s)
{
	init_device();
}

EnsembleAxis::EnsembleAxis(Tango::DeviceClass *cl,const char *s,const char *d)
:Tango::Device_4Impl(cl,s,d)
{
	init_device();
}
//+----------------------------------------------------------------------------
//
// method : 		EnsembleAxis::delete_device()
// 
// description : 	will be called at device destruction or at init command.
//
//-----------------------------------------------------------------------------
void EnsembleAxis::delete_device()
{
	//	Delete device allocated objects
  DELETE_SCALAR_ATTRIBUTE(attr_position_read);
  DELETE_SCALAR_ATTRIBUTE(attr_offset_read);
  DELETE_SCALAR_ATTRIBUTE(attr_velocity_read);
  DELETE_SCALAR_ATTRIBUTE(attr_isBrakeOn_read);

}

//+----------------------------------------------------------------------------
//
// method : 		EnsembleAxis::init_device()
// 
// description : 	will be called at device initialization.
//
//-----------------------------------------------------------------------------
void EnsembleAxis::init_device()
{
	INFO_STREAM << "EnsembleAxis::EnsembleAxis() create device " << device_name << endl;

	// Initialise variables to default values
	//--------------------------------------------
	this->m_status_str = "initializing device...";
	this->m_properties_missing = false;
	this->m_init_device_done = false;
  this->axis_name = 0;

  CREATE_SCALAR_ATTRIBUTE(attr_position_read);
  CREATE_SCALAR_ATTRIBUTE(attr_offset_read);
  CREATE_SCALAR_ATTRIBUTE(attr_velocity_read);
  CREATE_SCALAR_ATTRIBUTE(attr_isBrakeOn_read);
  get_device_property();
  if (m_properties_missing)
    return;

  this->axis_name = const_cast <char*> (axisId.c_str ());

  bool connected = ENSEMBLE_PROXY->connected_ok ();
  if (!connected)
  {
    ERROR_STREAM << "EnsembleAxis::init_device could not connect to Ensemble controller [check connections , try to restart DS]" << std::endl;
    m_status_str = "could not connect to Ensemble controller [check connections , try to restart DS]\n";
    return;
  }

	this->m_init_device_done = true;
  
}


//+----------------------------------------------------------------------------
//
// method : 		EnsembleAxis::get_device_property()
// 
// description : 	Read the device properties from database.
//
//-----------------------------------------------------------------------------
void EnsembleAxis::get_device_property()
{
	//	Initialize your default values here (if not done with  POGO).
	//------------------------------------------------------------------
  axisId = "X";
  axisPositionRatio = 1.0;

	//	Read device properties from database.(Automatic code generation)
	//------------------------------------------------------------------
	Tango::DbData	dev_prop;
	dev_prop.push_back(Tango::DbDatum("AxisId"));
	dev_prop.push_back(Tango::DbDatum("AxisPositionRatio"));

	//	Call database and extract values
	//--------------------------------------------
	if (Tango::Util::instance()->_UseDb==true)
		get_db_device()->get_property(dev_prop);
	Tango::DbDatum	def_prop, cl_prop;
	EnsembleAxisClass	*ds_class =
		(static_cast<EnsembleAxisClass *>(get_device_class()));
	int	i = -1;

	//	Try to initialize AxisId from class property
	cl_prop = ds_class->get_class_property(dev_prop[++i].name);
	if (cl_prop.is_empty()==false)	cl_prop  >>  axisId;
	else {
		//	Try to initialize AxisId from default device value
		def_prop = ds_class->get_default_device_property(dev_prop[i].name);
		if (def_prop.is_empty()==false)	def_prop  >>  axisId;
	}
	//	And try to extract AxisId value from database
	if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  axisId;

	//	Try to initialize AxisPositionRatio from class property
	cl_prop = ds_class->get_class_property(dev_prop[++i].name);
	if (cl_prop.is_empty()==false)	cl_prop  >>  axisPositionRatio;
	else {
		//	Try to initialize AxisPositionRatio from default device value
		def_prop = ds_class->get_default_device_property(dev_prop[i].name);
		if (def_prop.is_empty()==false)	def_prop  >>  axisPositionRatio;
	}
	//	And try to extract AxisPositionRatio value from database
	if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  axisPositionRatio;



	//	End of Automatic code generation
	//------------------------------------------------------------------
  Tango::DbData data_put;
  if (dev_prop[0].is_empty()==true)
	{
    INFO_STREAM << "EnsembleBox::get_device_property AxisId not defined [set to default value <X>]" << std::endl;
    this->m_status_str = "EnsembleBox::get_device_property AxisId not defined [set to default value <X>]";
		Tango::DbDatum	property("AxisId");
		property	<<	axisId;
		data_put.push_back(property);
	}
  if (dev_prop[1].is_empty()==true)
	{
    INFO_STREAM << "EnsembleBox::get_device_property AxisPositionRatio not defined [set to default value <1.0>]" << std::endl;
    this->m_status_str = "EnsembleBox::get_device_property AxisPositionRatio not defined [set to default value <1.0>]";
		Tango::DbDatum	property("AxisPositionRatio");
		property	<<	axisPositionRatio;
		data_put.push_back(property);
	}
	if(!data_put.empty())
		get_db_device()->put_property(data_put);
}

//+----------------------------------------------------------------------------
//
// method : 		EnsembleAxis::always_executed_hook()
// 
// description : 	method always executed before any command is executed
//
//-----------------------------------------------------------------------------
void EnsembleAxis::always_executed_hook()
{
	
}
//+----------------------------------------------------------------------------
//
// method : 		EnsembleAxis::read_attr_hardware
// 
// description : 	Hardware acquisition for attributes.
//
//-----------------------------------------------------------------------------
void EnsembleAxis::read_attr_hardware(vector<long> &attr_list)
{
	DEBUG_STREAM << "EnsembleAxis::read_attr_hardware(vector<long> &attr_list) entering... "<< endl;
	//	Add your own code here
}
//+----------------------------------------------------------------------------
//
// method : 		EnsembleAxis::read_relativeMove
// 
// description : 	Extract real attribute values for relativeMove acquisition result.
//
//-----------------------------------------------------------------------------
void EnsembleAxis::read_relativeMove(Tango::Attribute &attr)
{
	DEBUG_STREAM << "EnsembleAxis::read_relativeMove(Tango::Attribute &attr) entering... "<< endl;
}

//+----------------------------------------------------------------------------
//
// method : 		EnsembleAxis::write_relativeMove
// 
// description : 	Write relativeMove attribute values to hardware.
//
//-----------------------------------------------------------------------------
void EnsembleAxis::write_relativeMove(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "EnsembleAxis::write_relativeMove(Tango::WAttribute &attr) entering... "<< endl;
  if (! is_init ())
    THROW_DEVFAILED ("OPERATION_NOT_ALLOWED",
                     "device not properly initialized [check properties, communication lost]",
                     "EnsembleAxis::write_relativeMove");

  //- already moving?
  int st = 0;
  ENSEMBLE_PROXY->get_axis_status (this->axis_name, st);
  if (ENSEMBLE_PROXY->is_moving (st)         ||
      ENSEMBLE_PROXY->is_accel_or_decel (st) ||
      ENSEMBLE_PROXY->is_homing (st))
  {
    THROW_DEVFAILED ("OPERATION_NOT_ALLOWED",
                     "axis is already moving [wait for movement end]",
                     "EnsembleAxis::write_relativeMove");
  }
  ENSEMBLE_PROXY->get_axis_fault_status (this->axis_name, st);
  if (st != 0)
  {
    char err_text [256];
    ENSEMBLE_PROXY->error_to_string (st, err_text);
    THROW_DEVFAILED ("OPERATION_NOT_ALLOWED",
                     err_text,
                     "EnsembleAxis::write_relative_move");
  }
  attr.get_write_value (attr_relativeMove_write);
  ENSEMBLE_PROXY->axis_move_rel (this->axis_name, attr_position_write);

}

//+----------------------------------------------------------------------------
//
// method : 		EnsembleAxis::read_isBrakeOn
// 
// description : 	Extract real attribute values for isBrakeOn acquisition result.
//
//-----------------------------------------------------------------------------
void EnsembleAxis::read_isBrakeOn(Tango::Attribute &attr)
{
	DEBUG_STREAM << "EnsembleAxis::read_isBrakeOn(Tango::Attribute &attr) entering... "<< endl;
  if (! is_init ())
    return;
  int st = 0;
  ENSEMBLE_PROXY->get_axis_status (this->axis_name, st);
  *attr_isBrakeOn_read = ENSEMBLE_PROXY->is_brake_on (st);
  attr.set_value (attr_isBrakeOn_read);
}


//+----------------------------------------------------------------------------
//
// method : 		EnsembleAxis::read_position
// 
// description : 	Extract real attribute values for position acquisition result.
//
//-----------------------------------------------------------------------------
void EnsembleAxis::read_position(Tango::Attribute &attr)
{
	DEBUG_STREAM << "EnsembleAxis::read_position(Tango::Attribute &attr) entering... "<< endl;
  if (! is_init ())
    return;
//-  double tmp = yat::IEEE_NAN;
  double tmp = IEEE_NAN;
  ENSEMBLE_PROXY->get_axis_position (this->axis_name, tmp);
  *attr_position_read = tmp + *attr_offset_read;
  attr.set_value (attr_position_read);
}

//+----------------------------------------------------------------------------
//
// method : 		EnsembleAxis::write_position
// 
// description : 	Write position attribute values to hardware.
//
//-----------------------------------------------------------------------------
void EnsembleAxis::write_position(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "EnsembleAxis::write_position(Tango::WAttribute &attr) entering... "<< endl;
  if (! is_init ())
    THROW_DEVFAILED ("OPERATION_NOT_ALLOWED",
                     "device not properly initialized [check properties, communication lost]",
                     "EnsembleAxis::write_position");

  //- already moving?
  int st = 0;
  ENSEMBLE_PROXY->get_axis_status (this->axis_name, st);
  if (ENSEMBLE_PROXY->is_moving (st)         ||
      ENSEMBLE_PROXY->is_accel_or_decel (st) ||
      ENSEMBLE_PROXY->is_homing (st))
  {
    THROW_DEVFAILED ("OPERATION_NOT_ALLOWED",
                     "axis is already moving [wait for movement end]",
                     "EnsembleAxis::write_position");
  }
  ENSEMBLE_PROXY->get_axis_fault_status (this->axis_name, st);
  if (st != 0)
  {
    char err_text [256];
    ENSEMBLE_PROXY->error_to_string (st, err_text);
    THROW_DEVFAILED ("OPERATION_NOT_ALLOWED",
                     err_text,
                     "EnsembleAxis::write_position");
  }

  attr.get_write_value (attr_position_write);
  attr_position_write -= *attr_offset_read;
  ENSEMBLE_PROXY->axis_move_abs (this->axis_name, attr_position_write);

}

//+----------------------------------------------------------------------------
//
// method : 		EnsembleAxis::read_offset
// 
// description : 	Extract real attribute values for offset acquisition result.
//
//-----------------------------------------------------------------------------
void EnsembleAxis::read_offset(Tango::Attribute &attr)
{
	DEBUG_STREAM << "EnsembleAxis::read_offset(Tango::Attribute &attr) entering... "<< endl;
  if (! is_init ())
    return;
  attr.set_value (attr_offset_read);
}

//+----------------------------------------------------------------------------
//
// method : 		EnsembleAxis::write_offset
// 
// description : 	Write offset attribute values to hardware.
//
//-----------------------------------------------------------------------------
void EnsembleAxis::write_offset(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "EnsembleAxis::write_offset(Tango::WAttribute &attr) entering... "<< endl;
  if (! is_init ())
    THROW_DEVFAILED ("OPERATION_NOT_ALLOWED",
                     "device not properly initialized [check properties, communication lost]",
                     "EnsembleAxis::write_offset");
  attr.get_write_value (attr_offset_write);
  *attr_offset_read = attr_offset_write;
}

//+----------------------------------------------------------------------------
//
// method : 		EnsembleAxis::read_velocity
// 
// description : 	Extract real attribute values for velocity acquisition result.
//
//-----------------------------------------------------------------------------
void EnsembleAxis::read_velocity(Tango::Attribute &attr)
{
	DEBUG_STREAM << "EnsembleAxis::read_velocity(Tango::Attribute &attr) entering... "<< endl;
  if (! is_init ())
    return;
  ENSEMBLE_PROXY->get_axis_default_speed (axis_name, *attr_velocity_read);
  attr.set_value (attr_velocity_read);

}

//+----------------------------------------------------------------------------
//
// method : 		EnsembleAxis::write_velocity
// 
// description : 	Write velocity attribute values to hardware.
//
//-----------------------------------------------------------------------------
void EnsembleAxis::write_velocity(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "EnsembleAxis::write_velocity(Tango::WAttribute &attr) entering... "<< endl;
  if (! is_init ())
    THROW_DEVFAILED ("OPERATION_NOT_ALLOWED",
                     "device not properly initialized [check properties, communication lost]",
                     "EnsembleAxis::write_velocity");

  int st = 0;
  ENSEMBLE_PROXY->get_axis_fault_status (this->axis_name, st);
  if (st != 0)
  {
    char err_text [256];
    ENSEMBLE_PROXY->error_to_string (st, err_text);
    THROW_DEVFAILED ("OPERATION_NOT_ALLOWED",
                     err_text,
                     "EnsembleAxis::write_position");
  }

  attr.get_write_value (attr_velocity_write);
  ENSEMBLE_PROXY->set_axis_default_speed (axis_name, attr_velocity_write);
}



//+------------------------------------------------------------------
/**
 *	method:	EnsembleAxis::stop
 *
 *	description:	method to execute "Stop"
 *	Stops motor
 *
 *
 */
//+------------------------------------------------------------------
void EnsembleAxis::stop()
{
	DEBUG_STREAM << "EnsembleAxis::stop(): entering... !" << endl;

	//	Add your own code to control device here
  bool ok = false;
  ok = ENSEMBLE_PROXY->axis_abort (axis_name);
  if (!ok)
   THROW_DEVFAILED ("COMMAND_FAILED",
                    "command failed [controller refused command]",
                    "EnsembleAxis::stop");
}

//+------------------------------------------------------------------
/**
 *	method:	EnsembleAxis::initialize_reference_position
 *
 *	description:	method to execute "InitializeReferencePosition"
 *	makes a Home 0
 *
 *
 */
//+------------------------------------------------------------------
void EnsembleAxis::initialize_reference_position()
{
	DEBUG_STREAM << "EnsembleAxis::initialize_reference_position(): entering... !" << endl;

	//	Add your own code to control device here
  if (! is_init ())
    THROW_DEVFAILED ("OPERATION_NOT_ALLOWED",
                     "device not properly initialized [check properties, communication lost]",
                     "EnsembleAxis::initialize_reference_position");
  bool ok = ENSEMBLE_PROXY->axis_home (axis_name);
  if (!ok)
   THROW_DEVFAILED ("COMMAND_FAILED",
                    "command failed [controller refused command]",
                    "EnsembleAxis::initialize_reference_position");
}

//+------------------------------------------------------------------
/**
 *	method:	EnsembleAxis::brake_on
 *
 *	description:	method to execute "BrakeON"
 *	applies the brake
 *
 *
 */
//+------------------------------------------------------------------
void EnsembleAxis::brake_on()
{
	DEBUG_STREAM << "EnsembleAxis::brake_on(): entering... !" << endl;

	//	Add your own code to control device here
  if (! is_init ())
    THROW_DEVFAILED ("OPERATION_NOT_ALLOWED",
                     "device not properly initialized [check properties, communication lost]",
                     "EnsembleAxis::brake_on");
  bool ok = ENSEMBLE_PROXY->axis_brake_on (this->axis_name);
  if (!ok)
   THROW_DEVFAILED ("COMMAND_FAILED",
                    "command failed [controller refused command]",
                    "EnsembleAxis::brake_on");

}

//+------------------------------------------------------------------
/**
 *	method:	EnsembleAxis::brake_off
 *
 *	description:	method to execute "BrakeOFF"
 *	release the brake
 *
 *
 */
//+------------------------------------------------------------------
void EnsembleAxis::brake_off()
{
	DEBUG_STREAM << "EnsembleAxis::brake_off(): entering... !" << endl;

	//	Add your own code to control device here
  if (! is_init ())
    THROW_DEVFAILED ("OPERATION_NOT_ALLOWED",
                     "device not properly initialized [check properties, communication lost]",
                     "EnsembleAxis::brake_off");
  bool ok = ENSEMBLE_PROXY->axis_brake_off (this->axis_name);
  if (!ok)
   THROW_DEVFAILED ("COMMAND_FAILED",
                    "command failed [controller refused command]",
                    "EnsembleAxis::brake_off");
}


//+------------------------------------------------------------------
/**
 *	method:	EnsembleAxis::fault_ack
 *
 *	description:	method to execute "FaultAck"
 *	Acknowledges and clears faults on the Ensemble Driver
 *
 *
 */
//+------------------------------------------------------------------
void EnsembleAxis::fault_ack()
{
	DEBUG_STREAM << "EnsembleAxis::fault_ack(): entering... !" << endl;

	//	Add your own code to control device here
  if (! is_init ())
    THROW_DEVFAILED ("OPERATION_NOT_ALLOWED",
                     "device not properly initialized [check properties, communication lost]",
                     "EnsembleAxis::fault_ack");
  bool ok = ENSEMBLE_PROXY->axis_fault_ack (this->axis_name);
  if (!ok)
   THROW_DEVFAILED ("COMMAND_FAILED",
                    "command failed [controller refused command]",
                    "EnsembleAxis::fault_ack");
}

//+------------------------------------------------------------------
/**
 *	method:	EnsembleAxis::dev_state
 *
 *	description:	method to execute "State"
 *	This command gets the device state (stored in its <i>device_state</i> data member) and returns it to the caller.
 *
 * @return	State Code
 *
 */
//+------------------------------------------------------------------
Tango::DevState EnsembleAxis::dev_state()
{
  Tango::DevState	argout = Tango::UNKNOWN;
	DEBUG_STREAM << "EnsembleAxis::dev_state(): entering... !" << endl;

	//	Add your own code to control device here
  if (!m_init_device_done || m_properties_missing)
  {
    argout = Tango::FAULT;
    set_state (argout);
    return argout;
  }
  if (!ENSEMBLE_PROXY->connected_ok ())
  {
    argout = Tango::FAULT;
    set_state (argout);
    return argout;
  }

  int st = 0;
  int err = 0;
  ENSEMBLE_PROXY->get_axis_status (this->axis_name, st);
  ENSEMBLE_PROXY->get_axis_fault_status (this->axis_name, err);

  //- could not move (brake ON or driver disabled)
  if (ENSEMBLE_PROXY->is_brake_on (st) ||
      !ENSEMBLE_PROXY->is_enabled (st))
  {
    argout = Tango::DISABLE;
    set_state (argout);
    return argout;
  }

  //- moving
  if (ENSEMBLE_PROXY->is_moving (st)         ||
      ENSEMBLE_PROXY->is_accel_or_decel (st) ||
      ENSEMBLE_PROXY->is_homing (st))
  {
    argout = Tango::MOVING;
    set_state (argout);
    return argout;
  }

  //- Alarm
  if (!ENSEMBLE_PROXY->is_homed (st) ||
      ((err & 0x3C) != 0))            //- Hard or soft limits
  {
    argout = Tango::ALARM;
    set_state (argout);
    return argout;
  }

  //- fault
  if (ENSEMBLE_PROXY->is_emergency_stop (st) ||
      ENSEMBLE_PROXY->is_encoder_error  (st) ||
      err != 0)
  {
    argout = Tango::FAULT;
    set_state (argout);
    return argout;
  } 
  
  //- Standby
  if (ENSEMBLE_PROXY->is_in_position (st))
  {
    argout = Tango::STANDBY;
    set_state (argout);
    return argout;
  }
 
  //- out of position, not moving,... dont know.
  argout = Tango::ALARM;
	set_state(argout);
	return argout;
}

//+------------------------------------------------------------------
/**
 *	method:	EnsembleAxis::dev_status
 *
 *	description:	method to execute "Status"
 *	This command gets the device status (stored in its <i>device_status</i> data member) and returns it to the caller.
 *
 * @return	Status description
 *
 */
//+------------------------------------------------------------------
Tango::ConstDevString EnsembleAxis::dev_status()
{
	DEBUG_STREAM << "EnsembleAxis::dev_status(): entering... !" << endl;

	//	Add your own code to control device here
  if (!m_init_device_done || m_properties_missing)
  {
    set_status (m_status_str.c_str ());
    return m_status_str.c_str ();
  }
  m_status_str.clear ();

  if (!ENSEMBLE_PROXY->connected_ok ())
  {
    m_status_str += "communication not working\n";
    return m_status_str.c_str ();
  }

  int st = 0;
  ENSEMBLE_PROXY->get_axis_status (this->axis_name, st);
  if (ENSEMBLE_PROXY->is_enabled (st))
    m_status_str += "Axis Enabled\n";
  else
    m_status_str += "Axis Disabled\n";
  if (ENSEMBLE_PROXY->is_encoder_error (st))
    m_status_str += "Encoder Error\n";
  if (ENSEMBLE_PROXY->is_encoder_error (st))
    m_status_str += "Encoder Error\n";
  if (ENSEMBLE_PROXY->is_emergency_stop (st))
    m_status_str += "Emergency Stop\n";
  if (ENSEMBLE_PROXY->is_brake_on (st))
    m_status_str += "Brake ON\n";
  if (ENSEMBLE_PROXY->is_brake_off (st))
    m_status_str += "Brake OFF\n";
  if (ENSEMBLE_PROXY->is_homed (st))
    m_status_str += "Axis Homing Done\n";
  else
    m_status_str += "Axis NOT HOMED\n";
  if (ENSEMBLE_PROXY->is_moving (st))
    m_status_str += "Axis Moving\n";
  if (ENSEMBLE_PROXY->is_accelerating (st))
    m_status_str += "Axis Accelerating\n";
  if (ENSEMBLE_PROXY->is_decelerating (st))
    m_status_str += "Axis Decelerating\n";
  if (ENSEMBLE_PROXY->is_accel_or_decel (st))
    m_status_str += "Axis Accelerating OR decelerating\n";
  if (ENSEMBLE_PROXY->is_in_position (st))
    m_status_str += "Axis in position\n";
  else
    m_status_str += "Axis NOT in position\n";

  //- errors if any
  int err;
  if (ENSEMBLE_PROXY->get_axis_fault_status (axis_name, err))
  {
    char errors [512];
    ENSEMBLE_PROXY->error_to_string(err, errors);
    m_status_str += errors;
  }

  return m_status_str.c_str ();


}

//+------------------------------------------------------------------
/**
 *	internal method:	EnsembleAxis::is_init
 *
 * @return	boo true is init of device is ok 
 *
 */
//+------------------------------------------------------------------
bool EnsembleAxis::is_init()
{
	DEBUG_STREAM << "EnsembleAxis::is_init(): entering... !" << endl;

	//	Add your own code to control device here
  if (!m_init_device_done || m_properties_missing)
    return false;
  if (! ENSEMBLE_PROXY->connected_ok ())
    return false;

  return true;
}



//+------------------------------------------------------------------
/**
 *	method:	EnsembleAxis::enable
 *
 *	description:	method to execute "Enable"
 *	enables the related driver
 *
 *
 */
//+------------------------------------------------------------------
void EnsembleAxis::enable()
{
	DEBUG_STREAM << "EnsembleAxis::enable(): entering... !" << endl;

	//	Add your own code to control device here
  if (! is_init ())
    THROW_DEVFAILED ("OPERATION_NOT_ALLOWED",
                     "device not properly initialized [check properties, communication lost]",
                     "EnsembleAxis::enable");
  bool ok = ENSEMBLE_PROXY->axis_enable (this->axis_name);
  if (!ok)
   THROW_DEVFAILED ("COMMAND_FAILED",
                    "command failed [controller refused command]",
                    "EnsembleAxis::enable");

}

//+------------------------------------------------------------------
/**
 *	method:	EnsembleAxis::disable
 *
 *	description:	method to execute "Disable"
 *	disables the related driver
 *
 *
 */
//+------------------------------------------------------------------
void EnsembleAxis::disable()
{
	DEBUG_STREAM << "EnsembleAxis::disable(): entering... !" << endl;

	//	Add your own code to control device here
  if (! is_init ())
    THROW_DEVFAILED ("OPERATION_NOT_ALLOWED",
                     "device not properly initialized [check properties, communication lost]",
                     "EnsembleAxis::diable");
  bool ok = ENSEMBLE_PROXY->axis_disable (this->axis_name);
  if (!ok)
   THROW_DEVFAILED ("COMMAND_FAILED",
                    "command failed [controller refused command]",
                    "EnsembleAxis::disable");
}

}	//	namespace
