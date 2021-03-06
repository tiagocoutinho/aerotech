static const char *RcsId = "$Id:  $";
//+=============================================================================
//
// file :         A3200Expert.cpp
//
// description :  C++ source for the A3200Expert and its commands. 
//                The class is derived from Device. It represents the
//                CORBA servant object which will be accessed from the
//                network. All commands which can be executed on the
//                A3200Expert are implemented in this file.
//
// project :      TANGO Device Server
//
// $Author:  $
//
// $Revision:  $
//
// $Revision:  $
// $Date:  $
//
// SVN only:
// $HeadURL: $
//
// CVS only:
// $Source:  $
// $Log:  $
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
//  Command name       |  Method name
//	----------------------------------------
//  State              |  dev_state()
//  Status             |  dev_status()
//  SetDoubleRegister  |  set_double_register()
//  GetDoubleRegister  |  get_double_register()
//
//===================================================================


#include <tango.h>
#include <A3200Expert.h>
#include <A3200ExpertClass.h>
#include <PogoHelper.h>
#include <yat4tango/ExceptionHelper.h>
#include "lib/cA3200.h"

namespace A3200Expert_ns
{

//+----------------------------------------------------------------------------
//
// method : 		A3200Expert::A3200Expert(string &s)
// 
// description : 	constructor for simulated A3200Expert
//
// in : - cl : Pointer to the DeviceClass object
//      - s : Device name 
//
//-----------------------------------------------------------------------------
A3200Expert::A3200Expert(Tango::DeviceClass *cl,string &s)
	:Tango::Device_4Impl(cl,s.c_str())
{
	init_device();
}

A3200Expert::A3200Expert(Tango::DeviceClass *cl,const char *s)
	:Tango::Device_4Impl(cl,s)
{
	init_device();
}

A3200Expert::A3200Expert(Tango::DeviceClass *cl,const char *s,const char *d)
	:Tango::Device_4Impl(cl,s,d)
{
	init_device();
}
//+----------------------------------------------------------------------------
//
// method : 		A3200Expert::delete_device()
// 
// description : 	will be called at device destruction or at init command.
//
//-----------------------------------------------------------------------------
void A3200Expert::delete_device()
{
	//	Delete device allocated objects
	DELETE_SCALAR_ATTRIBUTE(attr_positionError_read);
	DELETE_SCALAR_ATTRIBUTE(attr_currentVelocity_read);
	DELETE_SCALAR_ATTRIBUTE(attr_lowLimit_read);
	DELETE_SCALAR_ATTRIBUTE(attr_highLimit_read);
	DELETE_SCALAR_ATTRIBUTE(attr_rampRate_read);
	DELETE_SCALAR_ATTRIBUTE(attr_homeVelocity_read);
	DELETE_SCALAR_ATTRIBUTE(attr_homeOffset_read);
	DELETE_SCALAR_ATTRIBUTE(attr_statusRaw_read);
	DELETE_SCALAR_ATTRIBUTE(attr_errorRaw_read);
	if (axis)
		delete axis;
	axis = 0;
}

//+----------------------------------------------------------------------------
//
// method : 		A3200Expert::init_device()
// 
// description : 	will be called at device initialization.
//
//-----------------------------------------------------------------------------
void A3200Expert::init_device()
{
	INFO_STREAM << "A3200Expert::A3200Expert() create device " << device_name << endl;

	// Initialise variables to default values
	//--------------------------------------------
	m_properties_missing = false;
	m_init_device_done = false;
	this->axis = 0;

	CREATE_SCALAR_ATTRIBUTE(attr_positionError_read);
	CREATE_SCALAR_ATTRIBUTE(attr_currentVelocity_read);
	CREATE_SCALAR_ATTRIBUTE(attr_lowLimit_read);
	CREATE_SCALAR_ATTRIBUTE(attr_highLimit_read);
	CREATE_SCALAR_ATTRIBUTE(attr_rampRate_read);
	CREATE_SCALAR_ATTRIBUTE(attr_homeVelocity_read);
	CREATE_SCALAR_ATTRIBUTE(attr_homeOffset_read);
	CREATE_SCALAR_ATTRIBUTE(attr_statusRaw_read);
	CREATE_SCALAR_ATTRIBUTE(attr_errorRaw_read);

	get_device_property();
	if (m_properties_missing)
		return;


	try
	{
		axis = new Aerotech_ns::cA3200 (const_cast <char *> (axisId.c_str ()));
	}
	catch (Tango::DevFailed &e)
	{
		ERROR_STREAM << "initialization failed - DevFailed exception caught" << e << std::endl;
		this->m_status_str = "device initialization failed caught Tango::DevFailed : \n"
			+ std::string(e.errors[0].desc);
		return;
	}
	catch (...)
	{
		ERROR_STREAM << "initialization failed [unknown exception caught]" << std::endl;
		this->m_status_str = "device initialization failed [(...) exception caught]";
		return;
	}

	this->m_init_device_done = true;
}


//+----------------------------------------------------------------------------
//
// method : 		A3200Expert::get_device_property()
// 
// description : 	Read the device properties from database.
//
//-----------------------------------------------------------------------------
void A3200Expert::get_device_property()
{
	//	Initialize your default values here (if not done with  POGO).
	//------------------------------------------------------------------

	//	Read device properties from database.(Automatic code generation)
	//------------------------------------------------------------------
	Tango::DbData	dev_prop;
	dev_prop.push_back(Tango::DbDatum("AxisId"));
	dev_prop.push_back(Tango::DbDatum("ControllerType"));

	//	Call database and extract values
	//--------------------------------------------
	if (Tango::Util::instance()->_UseDb==true)
		get_db_device()->get_property(dev_prop);
	Tango::DbDatum	def_prop, cl_prop;
	A3200ExpertClass	*ds_class =
		(static_cast<A3200ExpertClass *>(get_device_class()));
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

	//	Try to initialize ControllerType from class property
	cl_prop = ds_class->get_class_property(dev_prop[++i].name);
	if (cl_prop.is_empty()==false)	cl_prop  >>  controllerType;
	else {
		//	Try to initialize ControllerType from default device value
		def_prop = ds_class->get_default_device_property(dev_prop[i].name);
		if (def_prop.is_empty()==false)	def_prop  >>  controllerType;
	}
	//	And try to extract ControllerType value from database
	if (dev_prop[i].is_empty()==false)	dev_prop[i]  >>  controllerType;

	//	End of Automatic code generation
	//------------------------------------------------------------------
	Tango::DbData data_put;
	if (dev_prop[0].is_empty()==true || axisId.find ("must be defined") != std::string::npos)
	{
		m_properties_missing = true;
		INFO_STREAM << "A3200Expert::get_device_property AxisId not defined [fix and restart device]" << std::endl;
		this->m_status_str = "AxisId not defined [fix and restart device]";
		Tango::DbDatum	property("AxisId");
		property	<<	axisId;
		data_put.push_back(property);
	}
	if ((dev_prop[1].is_empty()==true) || (controllerType.find ("A3200") == std::string::npos))
	{
		ERROR_STREAM << "A3200Expert::get_device_property ControllerType not defined" << std::endl;
		this->m_status_str = "ControllerType not defined \n[set ControllerType to [A3200|ENSEMBLE] and restart device]";
		this->m_properties_missing = true;
		Tango::DbDatum	property("ControllerType");
		property	<<	controllerType;
		data_put.push_back(property);
	}
	if(!data_put.empty())
		get_db_device()->put_property(data_put);
}
//+----------------------------------------------------------------------------
//
// method : 		A3200Expert::always_executed_hook()
// 
// description : 	method always executed before any command is executed
//
//-----------------------------------------------------------------------------
void A3200Expert::always_executed_hook()
{

}
//+----------------------------------------------------------------------------
//
// method : 		A3200Expert::read_attr_hardware
// 
// description : 	Hardware acquisition for attributes.
//
//-----------------------------------------------------------------------------
void A3200Expert::read_attr_hardware(vector<long> &attr_list)
{
	DEBUG_STREAM << "A3200Expert::read_attr_hardware(vector<long> &attr_list) entering... "<< endl;
	//	Add your own code here
}

//+----------------------------------------------------------------------------
//
// method : 		A3200Expert::read_positionError
// 
// description : 	Extract real attribute values for positionError acquisition result.
//
//-----------------------------------------------------------------------------
void A3200Expert::read_positionError(Tango::Attribute &attr)
{
	DEBUG_STREAM << "A3200Expert::read_positionError(Tango::Attribute &attr) entering... "<< endl;
	if (! is_init ())
		return;
	if (axis->get_axis_position_error (*attr_positionError_read))
		attr.set_value (attr_positionError_read);
	else
		ERROR_STREAM << "A3200Expert::read_positionError could not read value on controller" << std::endl;
}

//+----------------------------------------------------------------------------
//
// method : 		A3200Expert::read_currentVelocity
// 
// description : 	Extract real attribute values for currentVelocity acquisition result.
//
//-----------------------------------------------------------------------------
void A3200Expert::read_currentVelocity(Tango::Attribute &attr)
{
	DEBUG_STREAM << "A3200Expert::read_currentVelocity(Tango::Attribute &attr) entering... "<< endl;
	if (! is_init ())
		return;
	if (axis->get_axis_velocity (*attr_currentVelocity_read))
		attr.set_value (attr_currentVelocity_read);
	else
		ERROR_STREAM << "A3200Expert::read_currentVelocity could not read value on controller" << std::endl;
}

//+----------------------------------------------------------------------------
//
// method : 		A3200Expert::write_currentVelocity
// 
// description : 	Write currentVelocity attribute values to hardware.
//
//-----------------------------------------------------------------------------
void A3200Expert::write_currentVelocity(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "A3200Expert::write_currentVelocity(Tango::WAttribute &attr) entering... "<< endl;
	if (! is_init ())
		THROW_DEVFAILED ("OPERATION_NOT_ALLOWED",
						"device not properly initialized [check properties, communication lost]",
						"EnsembleAxis::write_currentVelocity");


	attr.get_write_value (attr_currentVelocity_write);
	if (!axis->set_axis_velocity (attr_currentVelocity_write))
		THROW_DEVFAILED ("OPERATION_NOT_ALLOWED",
						"controller refused the command",
						"EnsembleAxis::write_currentVelocity");
}

//+----------------------------------------------------------------------------
//
// method : 		A3200Expert::read_lowLimit
// 
// description : 	Extract real attribute values for lowLimit acquisition result.
//
//-----------------------------------------------------------------------------
void A3200Expert::read_lowLimit(Tango::Attribute &attr)
{
	DEBUG_STREAM << "A3200Expert::read_lowLimit(Tango::Attribute &attr) entering... "<< endl;
	if (! is_init ())
		return;
	if (axis->get_axis_software_limit_low (*attr_lowLimit_read))
		attr.set_value (attr_lowLimit_read);
	else
		ERROR_STREAM << "A3200Expert::read_lowLimit could not read value on controller" << std::endl;
}

//+----------------------------------------------------------------------------
//
// method : 		A3200Expert::write_lowLimit
// 
// description : 	Write lowLimit attribute values to hardware.
//
//-----------------------------------------------------------------------------
void A3200Expert::write_lowLimit(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "A3200Expert::write_lowLimit(Tango::WAttribute &attr) entering... "<< endl;
	if (! is_init ())
		THROW_DEVFAILED ("OPERATION_NOT_ALLOWED",
						"device not properly initialized [check properties, communication lost]",
						"EnsembleAxis::write_lowLimit");


	attr.get_write_value (attr_lowLimit_write);
	if (!axis->set_axis_software_limit_low (attr_lowLimit_write))
		THROW_DEVFAILED ("OPERATION_NOT_ALLOWED",
						"controller refused the command",
						"EnsembleAxis::write_lowLimit");
}

//+----------------------------------------------------------------------------
//
// method : 		A3200Expert::read_highLimit
// 
// description : 	Extract real attribute values for highLimit acquisition result.
//
//-----------------------------------------------------------------------------
void A3200Expert::read_highLimit(Tango::Attribute &attr)
{
	DEBUG_STREAM << "A3200Expert::read_highLimit(Tango::Attribute &attr) entering... "<< endl;
	if (! is_init ())
		return;
	if (axis->get_axis_software_limit_high (*attr_highLimit_read))
		attr.set_value (attr_highLimit_read);
	else
		ERROR_STREAM << "A3200Expert::read_highLimit could not read value on controller" << std::endl;
}

//+----------------------------------------------------------------------------
//
// method : 		A3200Expert::write_highLimit
// 
// description : 	Write highLimit attribute values to hardware.
//
//-----------------------------------------------------------------------------
void A3200Expert::write_highLimit(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "A3200Expert::write_highLimit(Tango::WAttribute &attr) entering... "<< endl;
	if (! is_init ())
		THROW_DEVFAILED ("OPERATION_NOT_ALLOWED",
						"device not properly initialized [check properties, communication lost]",
						"EnsembleAxis::write_lowLimit");


	attr.get_write_value (attr_highLimit_write);
	if (!axis->set_axis_software_limit_high (attr_highLimit_write))
		THROW_DEVFAILED ("OPERATION_NOT_ALLOWED",
						"controller refused the command",
						"EnsembleAxis::write_highLimit");
}

//+----------------------------------------------------------------------------
//
// method : 		A3200Expert::read_rampRate
// 
// description : 	Extract real attribute values for rampRate acquisition result.
//
//-----------------------------------------------------------------------------
void A3200Expert::read_rampRate(Tango::Attribute &attr)
{
	DEBUG_STREAM << "A3200Expert::read_rampRate(Tango::Attribute &attr) entering... "<< endl;
	if (! is_init ())
		return;
	if (axis->get_axis_ramp_rate (*attr_rampRate_read))
		attr.set_value (attr_rampRate_read);
	else
		ERROR_STREAM << "A3200Expert::read_rampRate could not read value on controller" << std::endl;
}

//+----------------------------------------------------------------------------
//
// method : 		A3200Expert::write_rampRate
// 
// description : 	Write rampRate attribute values to hardware.
//
//-----------------------------------------------------------------------------
void A3200Expert::write_rampRate(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "A3200Expert::write_rampRate(Tango::WAttribute &attr) entering... "<< endl;
	if (! is_init ())
		THROW_DEVFAILED ("OPERATION_NOT_ALLOWED",
						"device not properly initialized [check properties, communication lost]",
						"EnsembleAxis::write_rampRate");


	attr.get_write_value (attr_rampRate_write);
	if (!axis->axis_move_abs (attr_rampRate_write))
		THROW_DEVFAILED ("OPERATION_NOT_ALLOWED",
						"controller refused the command",
						"EnsembleAxis::write_rampRate");
}

//+----------------------------------------------------------------------------
//
// method : 		A3200Expert::read_homeVelocity
// 
// description : 	Extract real attribute values for homeVelocity acquisition result.
//
//-----------------------------------------------------------------------------
void A3200Expert::read_homeVelocity(Tango::Attribute &attr)
{
	DEBUG_STREAM << "A3200Expert::read_homeVelocity(Tango::Attribute &attr) entering... "<< endl;
	if (! is_init ())
		return;
	if (axis->get_axis_home_speed (*attr_homeVelocity_read))
		attr.set_value (attr_homeVelocity_read);
	else
		ERROR_STREAM << "A3200Expert::read_homeVelocity could not read value on controller" << std::endl;
}

//+----------------------------------------------------------------------------
//
// method : 		A3200Expert::write_homeVelocity
// 
// description : 	Write homeVelocity attribute values to hardware.
//
//-----------------------------------------------------------------------------
void A3200Expert::write_homeVelocity(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "A3200Expert::write_homeVelocity(Tango::WAttribute &attr) entering... "<< endl;
	if (! is_init ())
		THROW_DEVFAILED ("OPERATION_NOT_ALLOWED",
						"device not properly initialized [check properties, communication lost]",
						"EnsembleAxis::write_homeVelocity");


	attr.get_write_value (attr_homeVelocity_write);
	if (!axis->set_axis_home_speed (attr_homeVelocity_write))
		THROW_DEVFAILED ("OPERATION_NOT_ALLOWED",
						"controller refused the command ",
						"EnsembleAxis::write_homeVelocity");
}

//+----------------------------------------------------------------------------
//
// method : 		A3200Expert::read_homeOffset
// 
// description : 	Extract real attribute values for homeOffset acquisition result.
//
//-----------------------------------------------------------------------------
void A3200Expert::read_homeOffset(Tango::Attribute &attr)
{
	DEBUG_STREAM << "A3200Expert::read_homeOffset(Tango::Attribute &attr) entering... "<< endl;
	if (! is_init ())
		return;
	if (axis->get_axis_home_offset (*attr_homeOffset_read))
		attr.set_value (attr_homeOffset_read);
	else
		ERROR_STREAM << "A3200Expert::read_homeOffset could not read value on controller" << std::endl;
}

//+----------------------------------------------------------------------------
//
// method : 		A3200Expert::write_homeOffset
// 
// description : 	Write homeOffset attribute values to hardware.
//
//-----------------------------------------------------------------------------
void A3200Expert::write_homeOffset(Tango::WAttribute &attr)
{
	DEBUG_STREAM << "A3200Expert::write_homeOffset(Tango::WAttribute &attr) entering... "<< endl;
	if (! is_init ())
		THROW_DEVFAILED ("OPERATION_NOT_ALLOWED",
						"device not properly initialized [check properties, communication lost]",
						"EnsembleAxis::write_homeOffset");


	attr.get_write_value (attr_homeOffset_write);
	if (!axis->set_axis_home_offset (attr_homeOffset_write))
		THROW_DEVFAILED ("OPERATION_NOT_ALLOWED",
						"controller refused the command ",
						"EnsembleAxis::write_homeOffset");
}

//+----------------------------------------------------------------------------
//
// method : 		A3200Expert::read_statusRaw
// 
// description : 	Extract real attribute values for statusRaw acquisition result.
//
//-----------------------------------------------------------------------------
void A3200Expert::read_statusRaw(Tango::Attribute &attr)
{
	DEBUG_STREAM << "A3200Expert::read_statusRaw(Tango::Attribute &attr) entering... "<< endl;
	if (! is_init ())
		return;
	int i = 0;
	if (axis->get_axis_status (i))
	{
		*attr_statusRaw_read = static_cast <long> (i);
		attr.set_value (attr_statusRaw_read);
	}
	else
		ERROR_STREAM << "A3200Expert::read_statusRaw could not read value on controller" << std::endl;
}


//+----------------------------------------------------------------------------
//
// method : 		A3200Expert::read_errorRaw
// 
// description : 	Extract real attribute values for errorRaw acquisition result.
//
//-----------------------------------------------------------------------------
void A3200Expert::read_errorRaw(Tango::Attribute &attr)
{
	DEBUG_STREAM << "A3200Expert::read_errorRaw(Tango::Attribute &attr) entering... "<< endl;
	if (! is_init ())
		return;
	int i = 0;
	if (axis->get_axis_fault_status (i))
	{
		*attr_errorRaw_read =  static_cast <long> (i);
		attr.set_value (attr_errorRaw_read);
	}
	else
		ERROR_STREAM << "A3200Expert::read_errorRaw could not read value on controller" << std::endl;
}

//+------------------------------------------------------------------
/**
*	method:	A3200Expert::set_double_register
*
*	description:	method to execute "SetDoubleRegister"
*	sets a double value in a A3200 Register
*
* @param	argin	Register Index, Value
*
*/
//+------------------------------------------------------------------
void A3200Expert::set_double_register(const Tango::DevVarDoubleArray *argin)
{
	DEBUG_STREAM << "A3200Expert::set_double_register(): entering... !" << endl;

	//	Add your own code to control device here
	size_t len = argin->length();
	if (len != 2)
		THROW_DEVFAILED ("DATA_OUT_OF_RANGE",
						"you must provide 2 values : 1 for register index, 1 for the value you want to seet",
						"EnsembleAxis::set_double_register");

	int index = static_cast <int> ((*argin)[0]);
	double value = (*argin)[1];
	if (!axis->setdoubleregister (index, value))
		THROW_DEVFAILED ("OPERATION_NOT_ALLOWED",
						"controller refused the command ",
						"EnsembleAxis::set_register_value");

}

//+------------------------------------------------------------------
/**
*	method:	A3200Expert::get_double_register
*
*	description:	method to execute "GetDoubleRegister"
*	read a double value in the specified register
*
* @param	argin	the register index
* @return	the value
*
*/
//+------------------------------------------------------------------
Tango::DevDouble A3200Expert::get_double_register(Tango::DevLong argin)
{
	Tango::DevDouble	argout ;
	DEBUG_STREAM << "A3200Expert::get_double_register(): entering... !" << endl;

	//	Add your own code to control device here
	if (!axis->getdoubleregister (argin, argout))
		THROW_DEVFAILED ("OPERATION_NOT_ALLOWED",
						"controller refused the command ",
						"EnsembleAxis::get_double_register");

	return argout;
}
//+------------------------------------------------------------------
/**
*	method:	A3200Expert::dev_state
*
*	description:	method to execute "State"
*	This command gets the device state (stored in its <i>device_state</i> data member) and returns it to the caller.
*
* @return	State Code
*
*/
//+------------------------------------------------------------------
Tango::DevState A3200Expert::dev_state()
{
	Tango::DevState	argout = DeviceImpl::dev_state();
	DEBUG_STREAM << "A3200Expert::dev_state(): entering... !" << endl;

	//	Add your own code to control device here
	if (!m_init_device_done || m_properties_missing)
	{
		argout = Tango::FAULT;
		set_state (argout);
		return argout;
	}
	if (!axis->connected_ok ())
	{
		argout = Tango::FAULT;
		set_state (argout);
		return argout;
	}

	int err = 0;
	axis->get_axis_fault_status (err);

	//- could not move (brake ON or driver disabled)
	if (axis->axis_is_brake_on () ||
		!axis->axis_is_enabled ())
	{
		argout = Tango::DISABLE;
		set_state (argout);
		return argout;
	}

	//- moving
	if (axis->axis_is_moving ()         ||
		axis->axis_is_accel_or_decel () ||
		axis->axis_is_homing ())
	{
		argout = Tango::MOVING;
		set_state (argout);
		return argout;
	}

	//- Alarm
	if (!axis->axis_is_homed () ||
		((err & 0x3C) != 0))            //- Hard or soft limits)
	{
		argout = Tango::ALARM;
		set_state (argout);
		return argout;
	}

	//- fault
	if (axis->axis_is_emergency_stop () || err != 0)
	{
		argout = Tango::FAULT;
		set_state (argout);
		return argout;
	}

	//- Standby
	if (axis->axis_is_in_position ())
	{
		argout = Tango::STANDBY;
		set_state (argout);
		return argout;
	}



	//- out of position, not moving,... dont know.
	argout = Tango::ALARM;
	set_state(argout);
	return argout;}

//+------------------------------------------------------------------
/**
*	method:	A3200Expert::dev_status
*
*	description:	method to execute "Status"
*	This command gets the device status (stored in its <i>device_status</i> data member) and returns it to the caller.
*
* @return	Status description
*
*/
//+------------------------------------------------------------------
Tango::ConstDevString A3200Expert::dev_status()
{
	DEBUG_STREAM << "A3200Expert::dev_status(): entering... !" << endl;

	//	Add your own code to control device here
	if (!m_init_device_done || m_properties_missing)
	{
		set_status (m_status_str.c_str ());
		return m_status_str.c_str ();
	}
	m_status_str.clear ();

	if (!axis->connected_ok ())
	{
		m_status_str += "communication not working\n";
		return m_status_str.c_str ();
	}

	if (axis->axis_is_enabled ())
		m_status_str += "Axis Enabled\n";
	else
		m_status_str += "Axis Disabled\n";
	if (axis->axis_is_emergency_stop ())
		m_status_str += "Emergency Stop\n";
	if (axis->axis_is_brake_on ())
		m_status_str += "Brake ON\n";
	if (axis->axis_is_brake_off ())
		m_status_str += "Brake OFF\n";
	if (axis->axis_is_homed ())
		m_status_str += "Axis Homing Done\n";
	else
		m_status_str += "Axis NOT HOMED\n";
	if (axis->axis_is_moving ())
		m_status_str += "Axis Moving\n";
	if (axis->axis_is_accelerating ())
		m_status_str += "Axis Accelerating\n";
	if (axis->axis_is_decelerating ())
		m_status_str += "Axis Decelerating\n";
	if (axis->axis_is_accel_or_decel ())
		m_status_str += "Axis Accelerating OR decelerating\n";
	if (axis->axis_is_in_position ())
		m_status_str += "Axis in position\n";
	else
		m_status_str += "Axis NOT in position\n";

	//- errors if any
	int err;
	if (axis->get_axis_fault_status (err))
	{
		char errors [512];
		axis->error_to_string(err, errors);
		m_status_str += errors;
	}

	return m_status_str.c_str ();

}


//+------------------------------------------------------------------
/**
*	internal method:	A3200Expert::is_init
*
* @return	boo true is init of device is ok 
*
*/
//+------------------------------------------------------------------
bool A3200Expert::is_init()
{
	DEBUG_STREAM << "A3200Expert::is_init(): entering... !" << endl;

	//	Add your own code to control device here
	if (!m_init_device_done || m_properties_missing)
		return false;
	if (!axis->connected_ok ())
		return false;

	return true;
}


}	//	namespace
