#ifndef RCG_ARMSTONE_INVERSE_DYNAMICS_H_
#define RCG_ARMSTONE_INVERSE_DYNAMICS_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "inertia_properties.h"
#include "transforms.h"
#include "link_data_map.h"

namespace armstone {
namespace rcg {

/**
 * The Inverse Dynamics routine for the robot armstone.
 *
 * In addition to the full Newton-Euler algorithm, specialized versions to compute
 * only certain terms are provided.
 * The parameters common to most of the methods are the joint status vector \c q, the
 * joint velocity vector \c qd and the acceleration vector \c qdd.
 *
 * Additional overloaded methods are provided without the \c q parameter. These
 * methods use the current configuration of the robot; they are provided for the
 * sake of efficiency, in case the motion transforms of the robot have already
 * been updated elsewhere with the most recent configuration (eg by a call to
 * setJointStatus()), so that it is useless to compute them again.
 *
 * Whenever present, the external forces parameter is a set of external
 * wrenches acting on the robot links. Each wrench must be expressed in
 * the reference frame of the link it is excerted on.
 */
class InverseDynamics {
public:
    typedef LinkDataMap<Force> ExtForces;

    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot armstone, which will be used by this instance
     *     to compute inverse-dynamics.
     */
    InverseDynamics(InertiaProperties& in, MotionTransforms& tr);

    /** \name Inverse dynamics
     * The full algorithm for the inverse dynamics of this robot.
     *
     * All the spatial vectors in the parameters are expressed in base coordinates,
     * besides the external forces: each force must be expressed in the reference
     * frame of the link it is acting on.
     * \param[out] jForces the joint force vector required to achieve the desired accelerations
     * \param[out] baseAccel the spatial acceleration of the robot base
     * \param[in] g the gravity acceleration, as a spatial vector;
     *              gravity implicitly specifies the orientation of the base in space
     * \param[in] base_link_footprint_v the spatial velocity of the base
     * \param[in] q the joint position vector
     * \param[in] qd the joint velocity vector
     * \param[in] qdd the desired joint acceleration vector
     * \param[in] fext the external forces acting on the links; this parameters
     *            defaults to zero
     */ ///@{
    void id(
        JointState& jForces, Acceleration& base_link_footprint_a,
        const Acceleration& g, const Velocity& base_link_footprint_v,
        const JointState& q, const JointState& qd, const JointState& qdd,
        const ExtForces& fext = zeroExtForces);
    void id(
        JointState& jForces, Acceleration& base_link_footprint_a,
        const Acceleration& g, const Velocity& base_link_footprint_v,
        const JointState& qd, const JointState& qdd,
        const ExtForces& fext = zeroExtForces);
    ///@}
    /** \name Inverse dynamics, fully actuated base
     * The inverse dynamics algorithm for the floating base robot,
     * in the assumption of a fully actuated base.
     *
     * All the spatial vectors in the parameters are expressed in base coordinates,
     * besides the external forces: each force must be expressed in the reference
     * frame of the link it is acting on.
     * \param[out] baseWrench the spatial force to be applied to the robot base to achieve
     *             the desired accelerations
     * \param[out] jForces the joint force vector required to achieve the desired accelerations
     * \param[in] g the gravity acceleration, as a spatial vector;
     *              gravity implicitly specifies the orientation of the base in space
     * \param[in] base_link_footprint_v the spatial velocity of the base
     * \param[in] baseAccel the desired spatial acceleration of the robot base
     * \param[in] q the joint position vector
     * \param[in] qd the joint velocity vector
     * \param[in] qdd the desired joint acceleration vector
     * \param[in] fext the external forces acting on the links; this parameters
     *            defaults to zero
     */ ///@{
    void id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& base_link_footprint_v, const Acceleration& baseAccel,
        const JointState& q, const JointState& qd, const JointState& qdd, const ExtForces& fext = zeroExtForces);
    void id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& base_link_footprint_v, const Acceleration& baseAccel,
        const JointState& qd, const JointState& qdd, const ExtForces& fext = zeroExtForces);
    ///@}
    /** \name Gravity terms, fully actuated base
     */
    ///@{
    void G_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const JointState& q);
    void G_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g);
    ///@}
    /** \name Centrifugal and Coriolis terms, fully actuated base
     *
     * These functions take only velocity inputs, that is, they assume
     * a zero spatial acceleration of the base (in addition to zero acceleration
     * at the actuated joints).
     * Note that this is NOT the same as imposing zero acceleration
     * at the virtual 6-dof-floting-base joint, which would result, in general,
     * in a non-zero spatial acceleration of the base, due to velocity
     * product terms.
     */
    ///@{
    void C_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Velocity& base_link_footprint_v, const JointState& q, const JointState& qd);
    void C_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Velocity& base_link_footprint_v, const JointState& qd);
    ///@}
    /** Updates all the kinematics transforms used by the inverse dynamics routine. */
    void setJointStatus(const JointState& q) const;

public:
    /** \name Getters
     * These functions return various spatial quantities used internally
     * by the inverse dynamics routines, like the spatial acceleration
     * of the links.
     *
     * The getters can be useful to retrieve the additional data that is not
     * returned explicitly by the inverse dynamics routines even though it
     * is computed. For example, after a call to the inverse dynamics,
     * the spatial velocity of all the links has been determined and
     * can be accessed.
     *
     * However, beware that certain routines might not use some of the
     * spatial quantities, which therefore would retain their last value
     * without being updated nor reset (for example, the spatial velocity
     * of the links is unaffected by the computation of the gravity terms).
     */
    ///@{
    const Force& getForce_base_link_footprint() const { return base_link_footprint_f; }
    const Velocity& getVelocity_omniwheel_fl() const { return omniwheel_fl_v; }
    const Acceleration& getAcceleration_omniwheel_fl() const { return omniwheel_fl_a; }
    const Force& getForce_omniwheel_fl() const { return omniwheel_fl_f; }
    const Velocity& getVelocity_omniwheel_fr() const { return omniwheel_fr_v; }
    const Acceleration& getAcceleration_omniwheel_fr() const { return omniwheel_fr_a; }
    const Force& getForce_omniwheel_fr() const { return omniwheel_fr_f; }
    const Velocity& getVelocity_omniwheel_bl() const { return omniwheel_bl_v; }
    const Acceleration& getAcceleration_omniwheel_bl() const { return omniwheel_bl_a; }
    const Force& getForce_omniwheel_bl() const { return omniwheel_bl_f; }
    const Velocity& getVelocity_omniwheel_br() const { return omniwheel_br_v; }
    const Acceleration& getAcceleration_omniwheel_br() const { return omniwheel_br_a; }
    const Force& getForce_omniwheel_br() const { return omniwheel_br_f; }
    const Velocity& getVelocity_xarmlink1() const { return xarmlink1_v; }
    const Acceleration& getAcceleration_xarmlink1() const { return xarmlink1_a; }
    const Force& getForce_xarmlink1() const { return xarmlink1_f; }
    const Velocity& getVelocity_xarmlink2() const { return xarmlink2_v; }
    const Acceleration& getAcceleration_xarmlink2() const { return xarmlink2_a; }
    const Force& getForce_xarmlink2() const { return xarmlink2_f; }
    const Velocity& getVelocity_xarmlink3() const { return xarmlink3_v; }
    const Acceleration& getAcceleration_xarmlink3() const { return xarmlink3_a; }
    const Force& getForce_xarmlink3() const { return xarmlink3_f; }
    const Velocity& getVelocity_xarmlink4() const { return xarmlink4_v; }
    const Acceleration& getAcceleration_xarmlink4() const { return xarmlink4_a; }
    const Force& getForce_xarmlink4() const { return xarmlink4_f; }
    const Velocity& getVelocity_xarmlink5() const { return xarmlink5_v; }
    const Acceleration& getAcceleration_xarmlink5() const { return xarmlink5_a; }
    const Force& getForce_xarmlink5() const { return xarmlink5_f; }
    const Velocity& getVelocity_xarmlink6() const { return xarmlink6_v; }
    const Acceleration& getAcceleration_xarmlink6() const { return xarmlink6_a; }
    const Force& getForce_xarmlink6() const { return xarmlink6_f; }
    ///@}
protected:
    void secondPass_fullyActuated(JointState& jForces);

private:
    InertiaProperties* inertiaProps;
    MotionTransforms* xm;
private:
    Matrix66 vcross; // support variable
    // Link 'omniwheel_fl' :
    const InertiaMatrix& omniwheel_fl_I;
    Velocity      omniwheel_fl_v;
    Acceleration  omniwheel_fl_a;
    Force         omniwheel_fl_f;
    // Link 'omniwheel_fr' :
    const InertiaMatrix& omniwheel_fr_I;
    Velocity      omniwheel_fr_v;
    Acceleration  omniwheel_fr_a;
    Force         omniwheel_fr_f;
    // Link 'omniwheel_bl' :
    const InertiaMatrix& omniwheel_bl_I;
    Velocity      omniwheel_bl_v;
    Acceleration  omniwheel_bl_a;
    Force         omniwheel_bl_f;
    // Link 'omniwheel_br' :
    const InertiaMatrix& omniwheel_br_I;
    Velocity      omniwheel_br_v;
    Acceleration  omniwheel_br_a;
    Force         omniwheel_br_f;
    // Link 'xarmlink1' :
    const InertiaMatrix& xarmlink1_I;
    Velocity      xarmlink1_v;
    Acceleration  xarmlink1_a;
    Force         xarmlink1_f;
    // Link 'xarmlink2' :
    const InertiaMatrix& xarmlink2_I;
    Velocity      xarmlink2_v;
    Acceleration  xarmlink2_a;
    Force         xarmlink2_f;
    // Link 'xarmlink3' :
    const InertiaMatrix& xarmlink3_I;
    Velocity      xarmlink3_v;
    Acceleration  xarmlink3_a;
    Force         xarmlink3_f;
    // Link 'xarmlink4' :
    const InertiaMatrix& xarmlink4_I;
    Velocity      xarmlink4_v;
    Acceleration  xarmlink4_a;
    Force         xarmlink4_f;
    // Link 'xarmlink5' :
    const InertiaMatrix& xarmlink5_I;
    Velocity      xarmlink5_v;
    Acceleration  xarmlink5_a;
    Force         xarmlink5_f;
    // Link 'xarmlink6' :
    const InertiaMatrix& xarmlink6_I;
    Velocity      xarmlink6_v;
    Acceleration  xarmlink6_a;
    Force         xarmlink6_f;

    // The robot base
    const InertiaMatrix& base_link_footprint_I;
    InertiaMatrix base_link_footprint_Ic;
    Force         base_link_footprint_f;
    // The composite inertia tensors
    const InertiaMatrix& omniwheel_fl_Ic;
    const InertiaMatrix& omniwheel_fr_Ic;
    const InertiaMatrix& omniwheel_bl_Ic;
    const InertiaMatrix& omniwheel_br_Ic;
    InertiaMatrix xarmlink1_Ic;
    InertiaMatrix xarmlink2_Ic;
    InertiaMatrix xarmlink3_Ic;
    InertiaMatrix xarmlink4_Ic;
    InertiaMatrix xarmlink5_Ic;
    const InertiaMatrix& xarmlink6_Ic;

private:
    static const ExtForces zeroExtForces;
};

inline void InverseDynamics::setJointStatus(const JointState& q) const
{
    (xm->fr_omniwheel_fl_X_fr_base_link_footprint)(q);
    (xm->fr_omniwheel_fr_X_fr_base_link_footprint)(q);
    (xm->fr_omniwheel_bl_X_fr_base_link_footprint)(q);
    (xm->fr_omniwheel_br_X_fr_base_link_footprint)(q);
    (xm->fr_xarmlink1_X_fr_base_link_footprint)(q);
    (xm->fr_xarmlink2_X_fr_xarmlink1)(q);
    (xm->fr_xarmlink3_X_fr_xarmlink2)(q);
    (xm->fr_xarmlink4_X_fr_xarmlink3)(q);
    (xm->fr_xarmlink5_X_fr_xarmlink4)(q);
    (xm->fr_xarmlink6_X_fr_xarmlink5)(q);
}

inline void InverseDynamics::id(
    JointState& jForces, Acceleration& base_link_footprint_a,
    const Acceleration& g, const Velocity& base_link_footprint_v,
    const JointState& q, const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    setJointStatus(q);
    id(jForces, base_link_footprint_a, g, base_link_footprint_v,
       qd, qdd, fext);
}

inline void InverseDynamics::G_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Acceleration& g, const JointState& q)
{
    setJointStatus(q);
    G_terms_fully_actuated(baseWrench, jForces, g);
}

inline void InverseDynamics::C_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Velocity& base_link_footprint_v, const JointState& q, const JointState& qd)
{
    setJointStatus(q);
    C_terms_fully_actuated(baseWrench, jForces, base_link_footprint_v, qd);
}

inline void InverseDynamics::id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& base_link_footprint_v, const Acceleration& baseAccel,
        const JointState& q, const JointState& qd, const JointState& qdd, const ExtForces& fext)
{
    setJointStatus(q);
    id_fully_actuated(baseWrench, jForces, g, base_link_footprint_v,
        baseAccel, qd, qdd, fext);
}

}
}

#endif
