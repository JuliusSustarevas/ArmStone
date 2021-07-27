#ifndef RCG_ARMSTONE_FORWARD_DYNAMICS_H_
#define RCG_ARMSTONE_FORWARD_DYNAMICS_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "transforms.h"
#include "inertia_properties.h"
#include "link_data_map.h"

namespace armstone {
namespace rcg {

/**
 * The Forward Dynamics routine for the robot armstone.
 *
 * The parameters common to most of the methods are the joint status \c q, the
 * joint velocities \c qd and the joint forces \c tau. The accelerations \c qdd
 * will be filled with the computed values. Overloaded methods without the \c q
 * parameter use the current configuration of the robot; they are provided for
 * the sake of efficiency, in case the kinematics transforms of the robot have
 * already been updated elsewhere with the most recent configuration (eg by a
 * call to setJointStatus()), so that it would be useless to compute them again.
 */
class ForwardDynamics {
public:
    typedef LinkDataMap<Force> ExtForces;

    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot armstone, which will be used by this instance
     *     to compute the dynamics.
     */
    ForwardDynamics(InertiaProperties& in, MotionTransforms& tr);
    /** \name Forward dynamics
     * The Articulated-Body-Algorithm to compute the joint accelerations
     */ ///@{
    /**
     * \param qdd the joint accelerations vector (output parameter).
     * \param base_link_footprint_a
     * \param base_link_footprint_v
     * \param g the gravity acceleration vector, expressed in the
     *          base coordinates
     * \param q the joint status vector
     * \param qd the joint velocities vector
     * \param tau the joint forces (torque or force)
     * \param fext the external forces, optional. Each force must be
     *              expressed in the reference frame of the link it is
     *              exerted on.
     */
    void fd(
       JointState& qdd, Acceleration& base_link_footprint_a, // output parameters,
       const Velocity& base_link_footprint_v, const Acceleration& g,
       const JointState& q, const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    void fd(
        JointState& qdd, Acceleration& base_link_footprint_a, // output parameters,
        const Velocity& base_link_footprint_v, const Acceleration& g,
        const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    ///@}

    /** Updates all the kinematics transforms used by this instance. */
    void setJointStatus(const JointState& q) const;

private:
    InertiaProperties* inertiaProps;
    MotionTransforms* motionTransforms;

    Matrix66 vcross; // support variable
    Matrix66 Ia_r;   // support variable, articulated inertia in the case of a revolute joint
    // Link 'base_link_footprint'
    Matrix66 base_link_footprint_AI;
    Force base_link_footprint_p;

    // Link 'omniwheel_fl' :
    Matrix66 omniwheel_fl_AI;
    Velocity omniwheel_fl_a;
    Velocity omniwheel_fl_v;
    Velocity omniwheel_fl_c;
    Force    omniwheel_fl_p;

    Column6 omniwheel_fl_U;
    Scalar omniwheel_fl_D;
    Scalar omniwheel_fl_u;
    // Link 'omniwheel_fr' :
    Matrix66 omniwheel_fr_AI;
    Velocity omniwheel_fr_a;
    Velocity omniwheel_fr_v;
    Velocity omniwheel_fr_c;
    Force    omniwheel_fr_p;

    Column6 omniwheel_fr_U;
    Scalar omniwheel_fr_D;
    Scalar omniwheel_fr_u;
    // Link 'omniwheel_bl' :
    Matrix66 omniwheel_bl_AI;
    Velocity omniwheel_bl_a;
    Velocity omniwheel_bl_v;
    Velocity omniwheel_bl_c;
    Force    omniwheel_bl_p;

    Column6 omniwheel_bl_U;
    Scalar omniwheel_bl_D;
    Scalar omniwheel_bl_u;
    // Link 'omniwheel_br' :
    Matrix66 omniwheel_br_AI;
    Velocity omniwheel_br_a;
    Velocity omniwheel_br_v;
    Velocity omniwheel_br_c;
    Force    omniwheel_br_p;

    Column6 omniwheel_br_U;
    Scalar omniwheel_br_D;
    Scalar omniwheel_br_u;
    // Link 'xarmlink1' :
    Matrix66 xarmlink1_AI;
    Velocity xarmlink1_a;
    Velocity xarmlink1_v;
    Velocity xarmlink1_c;
    Force    xarmlink1_p;

    Column6 xarmlink1_U;
    Scalar xarmlink1_D;
    Scalar xarmlink1_u;
    // Link 'xarmlink2' :
    Matrix66 xarmlink2_AI;
    Velocity xarmlink2_a;
    Velocity xarmlink2_v;
    Velocity xarmlink2_c;
    Force    xarmlink2_p;

    Column6 xarmlink2_U;
    Scalar xarmlink2_D;
    Scalar xarmlink2_u;
    // Link 'xarmlink3' :
    Matrix66 xarmlink3_AI;
    Velocity xarmlink3_a;
    Velocity xarmlink3_v;
    Velocity xarmlink3_c;
    Force    xarmlink3_p;

    Column6 xarmlink3_U;
    Scalar xarmlink3_D;
    Scalar xarmlink3_u;
    // Link 'xarmlink4' :
    Matrix66 xarmlink4_AI;
    Velocity xarmlink4_a;
    Velocity xarmlink4_v;
    Velocity xarmlink4_c;
    Force    xarmlink4_p;

    Column6 xarmlink4_U;
    Scalar xarmlink4_D;
    Scalar xarmlink4_u;
    // Link 'xarmlink5' :
    Matrix66 xarmlink5_AI;
    Velocity xarmlink5_a;
    Velocity xarmlink5_v;
    Velocity xarmlink5_c;
    Force    xarmlink5_p;

    Column6 xarmlink5_U;
    Scalar xarmlink5_D;
    Scalar xarmlink5_u;
    // Link 'xarmlink6' :
    Matrix66 xarmlink6_AI;
    Velocity xarmlink6_a;
    Velocity xarmlink6_v;
    Velocity xarmlink6_c;
    Force    xarmlink6_p;

    Column6 xarmlink6_U;
    Scalar xarmlink6_D;
    Scalar xarmlink6_u;
private:
    static const ExtForces zeroExtForces;
};

inline void ForwardDynamics::setJointStatus(const JointState& q) const {
    (motionTransforms-> fr_omniwheel_fl_X_fr_base_link_footprint)(q);
    (motionTransforms-> fr_omniwheel_fr_X_fr_base_link_footprint)(q);
    (motionTransforms-> fr_omniwheel_bl_X_fr_base_link_footprint)(q);
    (motionTransforms-> fr_omniwheel_br_X_fr_base_link_footprint)(q);
    (motionTransforms-> fr_xarmlink1_X_fr_base_link_footprint)(q);
    (motionTransforms-> fr_xarmlink2_X_fr_xarmlink1)(q);
    (motionTransforms-> fr_xarmlink3_X_fr_xarmlink2)(q);
    (motionTransforms-> fr_xarmlink4_X_fr_xarmlink3)(q);
    (motionTransforms-> fr_xarmlink5_X_fr_xarmlink4)(q);
    (motionTransforms-> fr_xarmlink6_X_fr_xarmlink5)(q);
}

inline void ForwardDynamics::fd(
    JointState& qdd, Acceleration& base_link_footprint_a, // output parameters,
    const Velocity& base_link_footprint_v, const Acceleration& g,
    const JointState& q,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    setJointStatus(q);
    fd(qdd, base_link_footprint_a, base_link_footprint_v, g, qd, tau, fext);
}

}
}

#endif
