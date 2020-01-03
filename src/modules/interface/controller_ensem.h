
#ifndef __CONTROLLER_ENSEM_H__
#define __CONTROLLER_ENSEM_H__

#include "stabilizer_types.h"


void controllerENSEMInit(void);
bool controllerENSEMTest(void);
void controllerENSEM(commande_t *commande, X_t *reff, X_t *X,const uint32_t tick);

#endif //__CONTROLLER_ENSEM_H__
