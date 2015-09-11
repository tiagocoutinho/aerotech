//=============================================================================
//
// file :        EnsembleExpert.h
//
// description : Include for the EnsembleExpert class.
//
// project :	Aerotech
//
// $Author: olivierroux $
//
// $Revision: 1.1 $
// $Date: 2012/02/23 17:46:18 $
//
// SVN only:
// $HeadURL: $
//
// CVS only:
// $Source: /cvsroot/tango-ds/Motion/Aerotech/src/EnsembleExpert.h,v $
// $Log: EnsembleExpert.h,v $
// Revision 1.1  2012/02/23 17:46:18  olivierroux
// - initial import #21894
//
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
#ifndef _ENSEMBLEEXPERT_H
#define _ENSEMBLEEXPERT_H

#include <tango.h>
#include "lib/cEnsemble.h"
//using namespace Tango;

/**
 * @author	$Author: olivierroux $
 * @version	$Revision: 1.1 $
 */

 //	Add your own constant definitions here.
 //-----------------------------------------------


namespace EnsembleExpert_ns
{

/**
 * Class Description:
 * handles Aerotech Ensemble drivers expert functions
 */

/*
 *	Device States Description:
 */


class EnsembleExpert: public Tango::Device_4Impl
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
		Tango::DevDouble	*attr_velocityFeedback_read;
		Tango::DevDouble	*attr_velocityCommand_read;
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
		Tango::DevBoolean	*attr_calibrationActive_read;
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
	EnsembleExpert(Tango::DeviceClass *cl,string &s);
/**
 * Constructs a newly allocated Command object.
 *
 *	@param cl	Class.
 *	@param s 	Device Name
 */
	EnsembleExpert(Tango::DeviceClass *cl,const char *s);
/**
 * Constructs a newly allocated Command object.
 *
 *	@param cl	Class.
 *	@param s 	Device name
 *	@param d	Device description.
 */
	EnsembleExpert(Tango::DeviceClass *cl,const char *s,const char *d);
//@}

/**@name Destructor
 * Only one destructor is defined for this class */
//@{
/**
 * The object destructor.
 */	
	~EnsembleExpert() {delete_device();};
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
 * @name EnsembleExpert methods prototypes
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
 *	Extract real attribute values for velocityFeedback acquisition result.
 */
	virtual void read_velocityFeedback(Tango::Attribute &attr);
/**
 *	Extract real attribute values for velocityCommand acquisition result.
 */
	virtual void read_velocityCommand(Tango::Attribute &attr);
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
 *	Extract real attribute values for calibrationActive acquisition result.
 */
	virtual void read_calibrationActive(Tango::Attribute &attr);
/**
 *	Read/Write allowed for positionError attribute.
 */
	virtual bool is_positionError_allowed(Tango::AttReqType type);
/**
 *	Read/Write allowed for velocityFeedback attribute.
 */
	virtual bool is_velocityFeedback_allowed(Tango::AttReqType type);
/**
 *	Read/Write allowed for velocityCommand attribute.
 */
	virtual bool is_velocityCommand_allowed(Tango::AttReqType type);
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
 *	Read/Write allowed for calibrationActive attribute.
 */
	virtual bool is_calibrationActive_allowed(Tango::AttReqType type);
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
	Aerotech_ns::cEnsemble * axis;
};

}	// namespace_ns

#endif	// _ENSEMBLEEXPERT_H
