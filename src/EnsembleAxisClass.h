//=============================================================================
//
// file :         EnsembleAxisClass.h
//
// description :  Include for the EnsembleAxisClass root class.
//                This class is the singleton class for
//                the EnsembleAxis device class.
//                It contains all properties and methods which the 
//                EnsembleAxis requires only once e.g. the commands.
//			
// project :      TANGO Device Server
//
// $Author: jean_coquet $
//
// $Revision: 1.2 $
// $Date: 2012-03-02 15:45:13 $
//
// SVN only:
// $HeadURL: $
//
// CVS only:
// $Source: /users/chaize/newsvn/cvsroot/Motion/Aerotech/src/EnsembleAxisClass.h,v $
// $Log: not supported by cvs2svn $
// Revision 1.1  2012/02/23 17:46:18  olivierroux
// - initial import #21894
//
//
// copyleft :     European Synchrotron Radiation Facility
//                BP 220, Grenoble 38043
//                FRANCE
//
//=============================================================================
//
//  		This file is generated by POGO
//	(Program Obviously used to Generate tango Object)
//
//         (c) - Software Engineering Group - ESRF
//=============================================================================

#ifndef _ENSEMBLEAXISCLASS_H
#define _ENSEMBLEAXISCLASS_H

#include <tango.h>
#include <EnsembleAxis.h>


namespace EnsembleAxis_ns
{//=====================================
//	Define classes for attributes
//=====================================
class relativeMoveAttrib: public Tango::Attr
{
public:
	relativeMoveAttrib():Attr("relativeMove", Tango::DEV_DOUBLE, Tango::WRITE) {};
	~relativeMoveAttrib() {};
	
	virtual void read(Tango::DeviceImpl *dev,Tango::Attribute &att)
	{(static_cast<EnsembleAxis *>(dev))->read_relativeMove(att);}
	virtual void write(Tango::DeviceImpl *dev,Tango::WAttribute &att)
	{(static_cast<EnsembleAxis *>(dev))->write_relativeMove(att);}
	virtual bool is_allowed(Tango::DeviceImpl *dev,Tango::AttReqType ty)
	{return (static_cast<EnsembleAxis *>(dev))->is_relativeMove_allowed(ty);}
};

class isBrakeOnAttrib: public Tango::Attr
{
public:
	isBrakeOnAttrib():Attr("isBrakeOn", Tango::DEV_BOOLEAN, Tango::READ) {};
	~isBrakeOnAttrib() {};
	
	virtual void read(Tango::DeviceImpl *dev,Tango::Attribute &att)
	{(static_cast<EnsembleAxis *>(dev))->read_isBrakeOn(att);}
	virtual bool is_allowed(Tango::DeviceImpl *dev,Tango::AttReqType ty)
	{return (static_cast<EnsembleAxis *>(dev))->is_isBrakeOn_allowed(ty);}
};

class velocityAttrib: public Tango::Attr
{
public:
	velocityAttrib():Attr("velocity", Tango::DEV_DOUBLE, Tango::READ_WRITE) {};
	~velocityAttrib() {};
	
	virtual void read(Tango::DeviceImpl *dev,Tango::Attribute &att)
	{(static_cast<EnsembleAxis *>(dev))->read_velocity(att);}
	virtual void write(Tango::DeviceImpl *dev,Tango::WAttribute &att)
	{(static_cast<EnsembleAxis *>(dev))->write_velocity(att);}
	virtual bool is_allowed(Tango::DeviceImpl *dev,Tango::AttReqType ty)
	{return (static_cast<EnsembleAxis *>(dev))->is_velocity_allowed(ty);}
};

class offsetAttrib: public Tango::Attr
{
public:
	offsetAttrib():Attr("offset", Tango::DEV_DOUBLE, Tango::READ_WRITE) {};
	~offsetAttrib() {};
	
	virtual void read(Tango::DeviceImpl *dev,Tango::Attribute &att)
	{(static_cast<EnsembleAxis *>(dev))->read_offset(att);}
	virtual void write(Tango::DeviceImpl *dev,Tango::WAttribute &att)
	{(static_cast<EnsembleAxis *>(dev))->write_offset(att);}
	virtual bool is_allowed(Tango::DeviceImpl *dev,Tango::AttReqType ty)
	{return (static_cast<EnsembleAxis *>(dev))->is_offset_allowed(ty);}
};

class positionAttrib: public Tango::Attr
{
public:
	positionAttrib():Attr("position", Tango::DEV_DOUBLE, Tango::READ_WRITE) {};
	~positionAttrib() {};
	
	virtual void read(Tango::DeviceImpl *dev,Tango::Attribute &att)
	{(static_cast<EnsembleAxis *>(dev))->read_position(att);}
	virtual void write(Tango::DeviceImpl *dev,Tango::WAttribute &att)
	{(static_cast<EnsembleAxis *>(dev))->write_position(att);}
	virtual bool is_allowed(Tango::DeviceImpl *dev,Tango::AttReqType ty)
	{return (static_cast<EnsembleAxis *>(dev))->is_position_allowed(ty);}
};

//=========================================
//	Define classes for commands
//=========================================
class DisableClass : public Tango::Command
{
public:
	DisableClass(const char   *name,
	               Tango::CmdArgType in,
				   Tango::CmdArgType out,
				   const char        *in_desc,
				   const char        *out_desc,
				   Tango::DispLevel  level)
	:Command(name,in,out,in_desc,out_desc, level)	{};

	DisableClass(const char   *name,
	               Tango::CmdArgType in,
				   Tango::CmdArgType out)
	:Command(name,in,out)	{};
	~DisableClass() {};
	
	virtual CORBA::Any *execute (Tango::DeviceImpl *dev, const CORBA::Any &any);
	virtual bool is_allowed (Tango::DeviceImpl *dev, const CORBA::Any &any)
	{return (static_cast<EnsembleAxis *>(dev))->is_Disable_allowed(any);}
};



class EnableClass : public Tango::Command
{
public:
	EnableClass(const char   *name,
	               Tango::CmdArgType in,
				   Tango::CmdArgType out,
				   const char        *in_desc,
				   const char        *out_desc,
				   Tango::DispLevel  level)
	:Command(name,in,out,in_desc,out_desc, level)	{};

	EnableClass(const char   *name,
	               Tango::CmdArgType in,
				   Tango::CmdArgType out)
	:Command(name,in,out)	{};
	~EnableClass() {};
	
	virtual CORBA::Any *execute (Tango::DeviceImpl *dev, const CORBA::Any &any);
	virtual bool is_allowed (Tango::DeviceImpl *dev, const CORBA::Any &any)
	{return (static_cast<EnsembleAxis *>(dev))->is_Enable_allowed(any);}
};



class FaultAckClass : public Tango::Command
{
public:
	FaultAckClass(const char   *name,
	               Tango::CmdArgType in,
				   Tango::CmdArgType out,
				   const char        *in_desc,
				   const char        *out_desc,
				   Tango::DispLevel  level)
	:Command(name,in,out,in_desc,out_desc, level)	{};

	FaultAckClass(const char   *name,
	               Tango::CmdArgType in,
				   Tango::CmdArgType out)
	:Command(name,in,out)	{};
	~FaultAckClass() {};
	
	virtual CORBA::Any *execute (Tango::DeviceImpl *dev, const CORBA::Any &any);
	virtual bool is_allowed (Tango::DeviceImpl *dev, const CORBA::Any &any)
	{return (static_cast<EnsembleAxis *>(dev))->is_FaultAck_allowed(any);}
};



class BrakeOFFCmd : public Tango::Command
{
public:
	BrakeOFFCmd(const char   *name,
	               Tango::CmdArgType in,
				   Tango::CmdArgType out,
				   const char        *in_desc,
				   const char        *out_desc,
				   Tango::DispLevel  level)
	:Command(name,in,out,in_desc,out_desc, level)	{};

	BrakeOFFCmd(const char   *name,
	               Tango::CmdArgType in,
				   Tango::CmdArgType out)
	:Command(name,in,out)	{};
	~BrakeOFFCmd() {};
	
	virtual CORBA::Any *execute (Tango::DeviceImpl *dev, const CORBA::Any &any);
	virtual bool is_allowed (Tango::DeviceImpl *dev, const CORBA::Any &any)
	{return (static_cast<EnsembleAxis *>(dev))->is_BrakeOFF_allowed(any);}
};



class BrakeONClass : public Tango::Command
{
public:
	BrakeONClass(const char   *name,
	               Tango::CmdArgType in,
				   Tango::CmdArgType out,
				   const char        *in_desc,
				   const char        *out_desc,
				   Tango::DispLevel  level)
	:Command(name,in,out,in_desc,out_desc, level)	{};

	BrakeONClass(const char   *name,
	               Tango::CmdArgType in,
				   Tango::CmdArgType out)
	:Command(name,in,out)	{};
	~BrakeONClass() {};
	
	virtual CORBA::Any *execute (Tango::DeviceImpl *dev, const CORBA::Any &any);
	virtual bool is_allowed (Tango::DeviceImpl *dev, const CORBA::Any &any)
	{return (static_cast<EnsembleAxis *>(dev))->is_BrakeON_allowed(any);}
};



class InitializeReferencePositionClass : public Tango::Command
{
public:
	InitializeReferencePositionClass(const char   *name,
	               Tango::CmdArgType in,
				   Tango::CmdArgType out,
				   const char        *in_desc,
				   const char        *out_desc,
				   Tango::DispLevel  level)
	:Command(name,in,out,in_desc,out_desc, level)	{};

	InitializeReferencePositionClass(const char   *name,
	               Tango::CmdArgType in,
				   Tango::CmdArgType out)
	:Command(name,in,out)	{};
	~InitializeReferencePositionClass() {};
	
	virtual CORBA::Any *execute (Tango::DeviceImpl *dev, const CORBA::Any &any);
	virtual bool is_allowed (Tango::DeviceImpl *dev, const CORBA::Any &any)
	{return (static_cast<EnsembleAxis *>(dev))->is_InitializeReferencePosition_allowed(any);}
};



class StopClass : public Tango::Command
{
public:
	StopClass(const char   *name,
	               Tango::CmdArgType in,
				   Tango::CmdArgType out,
				   const char        *in_desc,
				   const char        *out_desc,
				   Tango::DispLevel  level)
	:Command(name,in,out,in_desc,out_desc, level)	{};

	StopClass(const char   *name,
	               Tango::CmdArgType in,
				   Tango::CmdArgType out)
	:Command(name,in,out)	{};
	~StopClass() {};
	
	virtual CORBA::Any *execute (Tango::DeviceImpl *dev, const CORBA::Any &any);
	virtual bool is_allowed (Tango::DeviceImpl *dev, const CORBA::Any &any)
	{return (static_cast<EnsembleAxis *>(dev))->is_Stop_allowed(any);}
};



//
// The EnsembleAxisClass singleton definition
//

class
#ifdef WIN32
	__declspec(dllexport)
#endif
	EnsembleAxisClass : public Tango::DeviceClass
{
public:
//	properties member data

//	add your own data members here
//------------------------------------

public:
	Tango::DbData	cl_prop;
	Tango::DbData	cl_def_prop;
	Tango::DbData	dev_def_prop;

//	Method prototypes
	static EnsembleAxisClass *init(const char *);
	static EnsembleAxisClass *instance();
	~EnsembleAxisClass();
	Tango::DbDatum	get_class_property(string &);
	Tango::DbDatum	get_default_device_property(string &);
	Tango::DbDatum	get_default_class_property(string &);
	
protected:
	EnsembleAxisClass(string &);
	static EnsembleAxisClass *_instance;
	void command_factory();
	void get_class_property();
	void attribute_factory(vector<Tango::Attr *> &);
	void write_class_property();
	void set_default_property();
	string get_cvstag();
	string get_cvsroot();

private:
	void device_factory(const Tango::DevVarStringArray *);
};


}	//	namespace EnsembleAxis_ns

#endif // _ENSEMBLEAXISCLASS_H
