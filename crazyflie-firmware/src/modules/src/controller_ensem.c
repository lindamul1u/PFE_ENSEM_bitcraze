

#include <math.h>

#include "param.h"
#include "log.h"
#include "math3d.h"

#include "controller_ensem.h"



static float kp=53.2010;
static float kdp=74.8919;
static float kd2p=56.3921;
static float kd3p=20.0179;


static float kpsi=17.0082;
static float kdpsi=17.9804;


void controllerENSEMReset(void)
{

	static float u1=0;
	static float u2=0;
	static float u3=0;
	static float u4=0;
}

void controllerENSEMInit(void)
{
	controllerENSEMReset();
}

bool controllerENSEMTest(void)
{
  return true;
}

void controllerENSEM(commande_t *commande, reff_t *reff,X_t *X,const uint32_t tick)


{// V=[p^(4)_reff psi(2)_reff]-K[X_tilde-reff]

commande->c1=(reff->d4p.x)-kp*(X->x1 -reff->position.x)-kdp*(X->x4-reff->velocity.x)-kd2p*(X->x7-reff->acceleration.x)-kd3p*(X->x10-reff->jerk.x);
commande->c2=(reff->d4p.y)-kp*(X->x2-reff->position.y)-kdp*(X->x5-reff->velocity.y)-kd2p*(X->x8-reff->acceleration.y)-kd3p*(X->x11-reff->jerk.y);
commande->c3=(reff->d4p.z)-kp*(X->x3 -reff->position.z)-kdp*(X->x6-reff->velocity.z)-kd2p*(X->x9-reff->acceleration.z)-kd3p*(X->x12-reff->jerk.z);
commande->c4=(reff->d2psi)-kpsi*(X->x13 -reff->psi)-kdpsi*(X->x14-reff->dpsi);



}


LOG_GROUP_START(ctrlEnsem)
LOG_ADD(LOG_FLOAT, u1, &u1)
LOG_ADD(LOG_FLOAT, u2, &u2)
LOG_ADD(LOG_FLOAT, u3, &u3)
LOG_ADD(LOG_FLOAT, u4, &u4)
LOG_ADD(LOG_FLOAT, x, &x)
LOG_ADD(LOG_FLOAT, y, &y)
LOG_ADD(LOG_FLOAT, z, &z)


LOG_GROUP_STOP(ctrlEnsem)
