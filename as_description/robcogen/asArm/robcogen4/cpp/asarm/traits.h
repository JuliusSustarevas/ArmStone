#ifndef IIT_ROBOGEN__ASARM_TRAITS_H_
#define IIT_ROBOGEN__ASARM_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"
#include "jacobians.h"
#include <iit/rbd/traits/TraitSelector.h>


namespace iit {
namespace asArm {

namespace tpl {

template <typename SCALAR>
struct Traits {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef SCALAR S;

    typedef typename asArm::JointIdentifiers JointID;
    typedef typename asArm::LinkIdentifiers  LinkID;
    typedef typename iit::rbd::tpl::TraitSelector<SCALAR>::Trait Trait;

    typedef typename asArm::tpl::JointState<SCALAR> JointState;



    typedef typename asArm::tpl::HomogeneousTransforms<Trait> HomogeneousTransforms;
    typedef typename asArm::tpl::MotionTransforms<Trait> MotionTransforms;
    typedef typename asArm::tpl::ForceTransforms<Trait> ForceTransforms;
    typedef typename asArm::tpl::Jacobians<Trait> Jacobians;

    typedef typename iit::asArm::dyn::tpl::InertiaProperties<Trait> InertiaProperties;
    typedef typename iit::asArm::dyn::tpl::ForwardDynamics<Trait> FwdDynEngine;
    typedef typename iit::asArm::dyn::tpl::InverseDynamics<Trait> InvDynEngine;
    typedef typename iit::asArm::dyn::tpl::JSIM<Trait> JSIM;

    static const int joints_count = asArm::jointsCount;
    static const int links_count  = asArm::linksCount;
    static const bool floating_base = true;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};

template <typename SCALAR>
inline const typename Traits<SCALAR>::JointID*  Traits<SCALAR>::orderedJointIDs() {
    return asArm::orderedJointIDs;
}
template <typename SCALAR>
inline const typename Traits<SCALAR>::LinkID*  Traits<SCALAR>::orderedLinkIDs() {
    return asArm::orderedLinkIDs;
}

}

typedef tpl::Traits<double> Traits; // default instantiation - backward compatibility...

}
}

#endif
