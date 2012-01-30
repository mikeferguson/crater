//***************************************************************************//
//                   Copyright 2004-2008 Michael E. Ferguson                 //
//***************************************************************************//
// pyro.h - device driver for Eltec 442-3 PyroElectric sensors				 //
// This is a component of the AVRRA ROBOTIC DEVELOPMENT SYSTEM       		 //
//***************************************************************************//
// This driver works with the external window comparator. To adjust comparator:
//	turn potentiometer towards front of robot to widen window
//  turn potentiometer towards rear of robot to decrease window width 
//		wider window = higher change in heat needed to trip

#ifndef AVRRA_SENSOR_PYRO
#define AVRRA_SENSOR_PYRO

#include "digital.h"

//*******************************pyro Functions******************************//

/** Initializes pyro-electric driver system. */
void pyroInit(char channel){
	digitalSetDirection(channel,AVRRA_INPUT);
	digitalSetData(channel,AVRRA_LOW);
}

/** = high when pyro-electric sensor detects a heat body. (1|0) */
signed char pyroGetData(char channel){
	if(digitalGetData(channel) > 0){
		return 0;
	}
	return 1;				// flame in line with sight
}

#endif

//*******************************End of pyro.h*******************************//
// REVISIONS
// 3/8/06 - Added inversion of data, clarification of functions
// 1/24/08 - Ported to AVRRA, changed pyroInit to new sensor model
