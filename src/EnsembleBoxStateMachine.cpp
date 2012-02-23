static const char *RcsId = "$Id $";
//+=============================================================================
//
// file :         EnsembleBoxStateMachine.cpp
//
// description :  C++ source for the EnsembleBox and its alowed 
//                methods for commands and attributes
//
// project :      TANGO Device Server
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
// $Source: /users/chaize/newsvn/cvsroot/Motion/Aerotech/src/EnsembleBoxStateMachine.cpp,v $
// $Log: not supported by cvs2svn $
//
// copyleft :     European Synchrotron Radiation Facility
//                BP 220, Grenoble 38043
//                FRANCE
//
//-=============================================================================
//
//  		This file is generated by POGO
//	(Program Obviously used to Generate tango Object)
//
//         (c) - Software Engineering Group - ESRF
//=============================================================================

#include <tango.h>
#include <EnsembleBox.h>
#include <EnsembleBoxClass.h>

/*====================================================================
 *	This file contains the methods to allow commands and attributes
 * read or write execution.
 *
 * If you wand to add your own code, add it between 
 * the "End/Re-Start of Generated Code" comments.
 *
 * If you want, you can also add your own methods.
 *====================================================================
 */

namespace EnsembleBox_ns
{

//=================================================
//		Attributes Allowed Methods
//=================================================

//+----------------------------------------------------------------------------
//
// method : 		EnsembleBox::is_oKCommandCounter_allowed
// 
// description : 	Read/Write allowed for oKCommandCounter attribute.
//
//-----------------------------------------------------------------------------
bool EnsembleBox::is_oKCommandCounter_allowed(Tango::AttReqType type)
{
		//	End of Generated Code

		//	Re-Start of Generated Code
	return true;
}
//+----------------------------------------------------------------------------
//
// method : 		EnsembleBox::is_badCommandCounter_allowed
// 
// description : 	Read/Write allowed for badCommandCounter attribute.
//
//-----------------------------------------------------------------------------
bool EnsembleBox::is_badCommandCounter_allowed(Tango::AttReqType type)
{
		//	End of Generated Code

		//	Re-Start of Generated Code
	return true;
}

//=================================================
//		Commands Allowed Methods
//=================================================

//+----------------------------------------------------------------------------
//
// method : 		EnsembleBox::is_Reset_allowed
// 
// description : 	Execution allowed for Reset command.
//
//-----------------------------------------------------------------------------
bool EnsembleBox::is_Reset_allowed(const CORBA::Any &any)
{
		//	End of Generated Code

		//	Re-Start of Generated Code
	return true;
}

}	// namespace EnsembleBox_ns
