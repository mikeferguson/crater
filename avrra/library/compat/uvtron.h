//***************************************************************************//
//                   Copyright 2004-2008 Michael E. Ferguson                 //
//***************************************************************************//
// uvtron.h - device driver for using Hammatsu UVTron sensors				 //
// This is a component of the AVRRA ROBOTIC DEVELOPMENT SYSTEM       		 //
//***************************************************************************//

#ifndef AVRRA_SENSOR_UVTRON
#define AVRRA_SENSOR UVTRON

#include "digital.h"

//******************************uvtron Functions*****************************//

/** Initializes UVTron driver system. */
void uvtronInit(char channel){
	digitalSetDirection(channel, AVRRA_INPUT);
	digitalSetData(channel,AVRRA_LOW);
}

/** = (T=1/F=0) whether or not there is uv light present. */
char uvtronGetData(char channel){
	if(digitalGetData(channel) == 0xFF){
		return 1;
	}
	return 0;
}

#endif

//******************************End of uvtron.h******************************//
// REVISIONS:
// 1/24/08 - ported to AVRRA, updated uvtronInit to new sensor model
