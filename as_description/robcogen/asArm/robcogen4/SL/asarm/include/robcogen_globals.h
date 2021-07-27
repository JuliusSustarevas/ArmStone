#ifndef _IIT_ASARM_SL__ROBOGEN_GLOBALS_H_
#define _IIT_ASARM_SL__ROBOGEN_GLOBALS_H_

#include <iit/robots/asarm/declarations.h>
#include <iit/robots/asarm/kinematics_parameters.h>
#include <iit/robots/asarm/transforms.h>
#include <iit/robots/asarm/inertia_properties.h>
#include <iit/robots/asarm/forward_dynamics.h>
#include <iit/robots/asarm/inverse_dynamics.h>

#include <SL.h>
#include <SL_user.h>

namespace robot = iit::asArm;

namespace iit {
namespace asArm {
namespace SL {

extern iit::asArm::HomogeneousTransforms* homogeneousTransforms;
extern iit::asArm::MotionTransforms* motionTransforms;
extern iit::asArm::ForceTransforms* forceTransforms;
extern iit::asArm::dyn::InertiaProperties* linksInertia;
extern iit::asArm::dyn::ForwardDynamics*   fwdDynEngine;
extern iit::asArm::dyn::InverseDynamics*   invDynEngine;

extern iit::asArm::HomogeneousTransforms::MatrixType world_X_base;

inline void updateEndeffectorsParams(SL_endeff* eff) {
    //TODO
}

void createDefaultTransformsAndDynamics();

void update__world_X_base(const SL_Cstate& base_pos,const SL_quat& base_orient);

}
}
}



#endif
