#ifndef RCG__ARMSTONE_TRAITS_H_
#define RCG__ARMSTONE_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"

namespace armstone {
namespace rcg {
struct Traits {
    typedef typename armstone::rcg::ScalarTraits ScalarTraits;

    typedef typename armstone::rcg::JointState JointState;

    typedef typename armstone::rcg::JointIdentifiers JointID;
    typedef typename armstone::rcg::LinkIdentifiers  LinkID;

    typedef typename armstone::rcg::HomogeneousTransforms HomogeneousTransforms;
    typedef typename armstone::rcg::MotionTransforms MotionTransforms;
    typedef typename armstone::rcg::ForceTransforms ForceTransforms;

    typedef typename armstone::rcg::InertiaProperties InertiaProperties;
    typedef typename armstone::rcg::ForwardDynamics FwdDynEngine;
    typedef typename armstone::rcg::InverseDynamics InvDynEngine;
    typedef typename armstone::rcg::JSIM JSIM;

    static const int joints_count = armstone::rcg::jointsCount;
    static const int links_count  = armstone::rcg::linksCount;
    static const bool floating_base = true;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};


inline const Traits::JointID*  Traits::orderedJointIDs() {
    return armstone::rcg::orderedJointIDs;
}
inline const Traits::LinkID*  Traits::orderedLinkIDs() {
    return armstone::rcg::orderedLinkIDs;
}

}
}

#endif
