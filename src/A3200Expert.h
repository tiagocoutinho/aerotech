//=============================================================================
//
// file :        A3200Expert.h
//
// description : Include for the A3200Expert class.
//
// project :	Aerotech
//
// $Author:  $
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
//		 BP48 - 91192 Gif sur Yvette
//               FRANCE
//
//=============================================================================
//
//  		This file is generated by POGO
//	(Program Obviously used to Generate tango Object)
//
//         (c) - Software Engineering Group - ESRF
//=============================================================================
#ifndef _A3200EXPERT_H
#define _A3200EXPERT_H

#include <tango.h>
#include "lib/cA3200.h"
//using namespace Tango;

/**
 * @author	$Author:  $
 * @version	$Revision:  $
 */

 //	Add your own constant definitions here.
 //-----------------------------------------------


namespace A3200Expert_ns
{

/**
 * Class Description:
 * handles Aerotech A3200 drivers expert functions
 */

/*
 *	Device States Description:
 */


class A3200Expert: public Tango::Device_4Impl
{
public :
	//	Add your own data members here
	//-----------------------------------------


	//	Here is the Start of the automatic code generation part
	//-------------------------------------------------------------	
/**
 *	@name attributes
 *	Attribute member data.
 */
//@{
		Tango::DevDouble	*attr_positionError_read;
		Tango::DevDouble	*attr_currentVelocity_read;
		Tango::DevDouble	attr_currentVelocity_write;
		Tango::DevDouble	*attr_lowLimit_read;
		Tango::DevDouble	attr_lowLimit_write;
		Tango::DevDouble	*attr_highLimit_read;
		Tango::DevDouble	attr_highLimit_write;
		Tango::DevDouble	*attr_rampRate_read;
		Tango::DevDouble	attr_rampRate_write;
		Tango::DevDouble	*attr_homeVelocity_read;
		Tango::DevDouble	attr_homeVelocity_write;
		Tango::DevDouble	*attr_homeOffset_read;
		Tango::DevDouble	attr_homeOffset_write;
		Tango::DevLong	*attr_statusRaw_read;
		Tango::DevLong	*attr_errorRaw_read;
//@}

/**
 * @name Device properties
 * Device properties member data.
 */
//@{
/**
 *	Axis name as configured in the controller
 */
	string	axisId;
/**
 *	controller type
 *	valid values : ENSEMBLE, A3200
 *	must be defined
 *	No default value
 */
	string	controllerType;
//@}

/**
 *	@name Device properties
 *	Device property member data.
 */
//@{
//@}

/**@name Constructors
 * Miscellaneous constructors */
//@{
/**
 * Constructs a newly allocated Command object.
 *
 *	@param cl	Class.
 *	@param s 	Device Name
 */
	A3200Expert(Tango::DeviceClass *cl,string &s);
/**
 * Constructs a newly allocated Command object.
 *
 *	@param cl	Class.
 *	@param s 	Device Name
 */
	A3200Expert(Tango::DeviceClass *cl,const char *s);
/**
 * Constructs a newly allocated Command object.
 *
 *	@param cl	Class.
 *	@param s 	Device name
 *	@param d	Device description.
 */
	A3200Expert(Tango::DeviceClass *cl,const char *s,const char *d);
//@}

/**@name Destructor
 * Only one destructor is defined for this class */
//@{
/**
 * The object destructor.
 */	
	~A3200Expert() {delete_device();};
/**
 *	will be called at device destruction or at init command.
 */
	void delete_device();
//@}

	
/**@name Miscellaneous methods */
//@{
/**
 *	Initialize the device
 */
	virtual void init_device();
/**
 *	Always executed method before execution command method.
 */
	virtual void always_executed_hook();

//@}

/**
 * @name A3200Expert methods prototypes
 */

//@{
/**
 *	Hardware acquisition for attributes.
 */
	virtual void read_attr_hardware(vector<long> &attr_list);
/**
 *	Extract real attribute values for positionError acquisition result.
 */
	virtual void read_positionError(Tango::Attribute &attr);
/**
 *	Extract real attribute values for currentVelocity acquisition result.
 */
	virtual void read_currentVelocity(Tango::Attribute &attr);
/**
 *	Write currentVelocity attribute values to hardware.
 */
	virtual void write_currentVelocity(Tango::WAttribute &attr);
/**
 *	Extract real attribute values for lowLimit acquisition result.
 */
	virtual void read_lowLimit(Tango::Attribute &attr);
/**
 *	Write lowLimit attribute values to hardware.
 */
	virtual void write_lowLimit(Tango::WAttribute &attr);
/**
 *	Extract real attribute values for highLimit acquisition result.
 */
	virtual void read_highLimit(Tango::Attribute &attr);
/**
 *	Write highLimit attribute values to hardware.
 */
	virtual void write_highLimit(Tango::WAttribute &attr);
/**
 *	Extract real attribute values for rampRate acquisition result.
 */
	virtual void read_rampRate(Tango::Attribute &attr);
/**
 *	Write rampRate attribute values to hardware.
 */
	virtual void write_rampRate(Tango::WAttribute &attr);
/**
 *	Extract real attribute values for homeVelocity acquisition result.
 */
	virtual void read_homeVelocity(Tango::Attribute &attr);
/**
 *	Write homeVelocity attribute values to hardware.
 */
	virtual void write_homeVelocity(Tango::WAttribute &attr);
/**
 *	Extract real attribute values for homeOffset acquisition result.
 */
	virtual void read_homeOffset(Tango::Attribute &attr);
/**
 *	Write homeOffset attribute values to hardware.
 */
	virtual void write_homeOffset(Tango::WAttribute &attr);
/**
 *	Extract real attribute values for statusRaw acquisition result.
 */
	virtual void read_statusRaw(Tango::Attribute &attr);
/**
 *	Extract real attribute values for errorRaw acquisition result.
 */
	virtual void read_errorRaw(Tango::Attribute &attr);
/**
 *	Read/Write allowed for positionError attribute.
 */
	virtual bool is_positionError_allowed(Tango::AttReqType type);
/**
 *	Read/Write allowed for currentVelocity attribute.
 */
	virtual bool is_currentVelocity_allowed(Tango::AttReqType type);
/**
 *	Read/Write allowed for lowLimit attribute.
 */
	virtual bool is_lowLimit_allowed(Tango::AttReqType type);
/**
 *	Read/Write allowed for highLimit attribute.
 */
	virtual bool is_highLimit_allowed(Tango::AttReqType type);
/**
 *	Read/Write allowed for rampRate attribute.
 */
	virtual bool is_rampRate_allowed(Tango::AttReqType type);
/**
 *	Read/Write allowed for homeVelocity attribute.
 */
	virtual bool is_homeVelocity_allowed(Tango::AttReqType type);
/**
 *	Read/Write allowed for homeOffset attribute.
 */
	virtual bool is_homeOffset_allowed(Tango::AttReqType type);
/**
 *	Read/Write allowed for statusRaw attribute.
 */
	virtual bool is_statusRaw_allowed(Tango::AttReqType type);
/**
 *	Read/Write allowed for errorRaw attribute.
 */
	virtual bool is_errorRaw_allowed(Tango::AttReqType type);
/**
 *	Execution allowed for SetDoubleRegister command.
 */
	virtual bool is_SetDoubleRegister_allowed(const CORBA::Any &any);
/**
 *	Execution allowed for GetDoubleRegister command.
 */
	virtual bool is_GetDoubleRegister_allowed(const CORBA::Any &any);
/**
 * This command gets the device state (stored in its <i>device_state</i> data member) and returns it to the caller.
 *	@return	State Code
 *	@exception DevFailed
 */
	virtual Tango::DevState	dev_state();
/**
 * This command gets the device status (stored in its <i>device_status</i> data member) and returns it to the caller.
 *	@return	Status description
 *	@exception DevFailed
 */
	virtual Tango::ConstDevString	dev_status();
/**
 * sets a double value in a A3200 Register
 *	@param	argin	Register Index, Value
 *	@exception DevFailed
 */
	void	set_double_register(const Tango::DevVarDoubleArray *);
/**
 * read a double value in the specified register
 *	@param	argin	the register index
 *	@return	the value
 *	@exception DevFailed
 */
	Tango::DevDouble	get_double_register(Tango::DevLong);

/**
 *	Read the device properties from database
 */
	 void get_device_property();
//@}

	//	Here is the end of the automatic code generation part
	//-------------------------------------------------------------	



protected :	
	//	Add your own data members here
	//-----------------------------------------
	//- init utilities
	bool m_init_device_done;
	bool m_properties_missing;

	//- the device status string
	std::string m_status_str;

	//- check wether or not the device is properly init
	bool is_init ();
	Aerotech_ns::cA3200 * axis;
};

}	// namespace_ns

#endif	// _A3200EXPERT_H
