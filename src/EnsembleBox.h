//=============================================================================
//
// file :        EnsembleBox.h
//
// description : Include for the EnsembleBox class.
//
// project :	Ensemble
//
// $Author: olivierroux $
//
// $Revision: 1.1 $
// $Date: 2012-02-23 17:46:18 $
//
// SVN only:
// $HeadURL: $
//
// CVS only:
// $Source: /users/chaize/newsvn/cvsroot/Motion/Aerotech/src/EnsembleBox.h,v $
// $Log: not supported by cvs2svn $
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
#ifndef _ENSEMBLEBOX_H
#define _ENSEMBLEBOX_H

#include <tango.h>
//using namespace Tango;

/**
 * @author	$Author: olivierroux $
 * @version	$Revision: 1.1 $
 */

 //	Add your own constant definitions here.
 //-----------------------------------------------


namespace EnsembleBox_ns
{

/**
 * Class Description:
 * controls communication with Aerotech Ensemble through Ethernet
 */

/*
 *	Device States Description:
 */


class EnsembleBox: public Tango::Device_4Impl
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
		Tango::DevLong	*attr_oKCommandCounter_read;
		Tango::DevLong	*attr_badCommandCounter_read;
//@}

/**
 * @name Device properties
 * Device properties member data.
 */
//@{
/**
 *	IP Address 
 */
	string	iPAddress;
/**
 *	TCP/IP port for communication
 *	
 */
	Tango::DevLong	port;
/**
 *	valeur de timeout en secondes
 */
	Tango::DevLong	tCPTimeOut;
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
	EnsembleBox(Tango::DeviceClass *cl,string &s);
/**
 * Constructs a newly allocated Command object.
 *
 *	@param cl	Class.
 *	@param s 	Device Name
 */
	EnsembleBox(Tango::DeviceClass *cl,const char *s);
/**
 * Constructs a newly allocated Command object.
 *
 *	@param cl	Class.
 *	@param s 	Device name
 *	@param d	Device description.
 */
	EnsembleBox(Tango::DeviceClass *cl,const char *s,const char *d);
//@}

/**@name Destructor
 * Only one destructor is defined for this class */
//@{
/**
 * The object destructor.
 */	
	~EnsembleBox() {delete_device();};
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
 * @name EnsembleBox methods prototypes
 */

//@{
/**
 *	Hardware acquisition for attributes.
 */
	virtual void read_attr_hardware(vector<long> &attr_list);
/**
 *	Extract real attribute values for oKCommandCounter acquisition result.
 */
	virtual void read_oKCommandCounter(Tango::Attribute &attr);
/**
 *	Extract real attribute values for badCommandCounter acquisition result.
 */
	virtual void read_badCommandCounter(Tango::Attribute &attr);
/**
 *	Read/Write allowed for oKCommandCounter attribute.
 */
	virtual bool is_oKCommandCounter_allowed(Tango::AttReqType type);
/**
 *	Read/Write allowed for badCommandCounter attribute.
 */
	virtual bool is_badCommandCounter_allowed(Tango::AttReqType type);
/**
 *	Execution allowed for Reset command.
 */
	virtual bool is_Reset_allowed(const CORBA::Any &any);
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
 * Reset the controller
 *	@exception DevFailed
 */
	void	reset();

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
  void check_init () 
    throw (Tango::DevFailed);
};

}	// namespace_ns

#endif	// _ENSEMBLEBOX_H
