#include "forward_dynamics.h"

#include <Eigen/Cholesky>
#include <iit/rbd/robcogen_commons.h>

using namespace iit::rbd;

// Initialization of static-const data
const armstone::rcg::ForwardDynamics::ExtForces
    armstone::rcg::ForwardDynamics::zeroExtForces(Force::Zero());

armstone::rcg::ForwardDynamics::ForwardDynamics(InertiaProperties& inertia, MotionTransforms& transforms) :
    inertiaProps( & inertia ),
    motionTransforms( & transforms )
{
    omniwheel_fl_v.setZero();
    omniwheel_fl_c.setZero();
    omniwheel_fr_v.setZero();
    omniwheel_fr_c.setZero();
    omniwheel_bl_v.setZero();
    omniwheel_bl_c.setZero();
    omniwheel_br_v.setZero();
    omniwheel_br_c.setZero();
    xarmlink1_v.setZero();
    xarmlink1_c.setZero();
    xarmlink2_v.setZero();
    xarmlink2_c.setZero();
    xarmlink3_v.setZero();
    xarmlink3_c.setZero();
    xarmlink4_v.setZero();
    xarmlink4_c.setZero();
    xarmlink5_v.setZero();
    xarmlink5_c.setZero();
    xarmlink6_v.setZero();
    xarmlink6_c.setZero();

    vcross.setZero();
    Ia_r.setZero();

}

void armstone::rcg::ForwardDynamics::fd(
    JointState& qdd,
    Acceleration& base_link_footprint_a,
    const Velocity& base_link_footprint_v,
    const Acceleration& g,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    
    base_link_footprint_AI = inertiaProps->getTensor_base_link_footprint();
    base_link_footprint_p = - fext[BASE_LINK_FOOTPRINT];
    omniwheel_fl_AI = inertiaProps->getTensor_omniwheel_fl();
    omniwheel_fl_p = - fext[OMNIWHEEL_FL];
    omniwheel_fr_AI = inertiaProps->getTensor_omniwheel_fr();
    omniwheel_fr_p = - fext[OMNIWHEEL_FR];
    omniwheel_bl_AI = inertiaProps->getTensor_omniwheel_bl();
    omniwheel_bl_p = - fext[OMNIWHEEL_BL];
    omniwheel_br_AI = inertiaProps->getTensor_omniwheel_br();
    omniwheel_br_p = - fext[OMNIWHEEL_BR];
    xarmlink1_AI = inertiaProps->getTensor_xarmlink1();
    xarmlink1_p = - fext[XARMLINK1];
    xarmlink2_AI = inertiaProps->getTensor_xarmlink2();
    xarmlink2_p = - fext[XARMLINK2];
    xarmlink3_AI = inertiaProps->getTensor_xarmlink3();
    xarmlink3_p = - fext[XARMLINK3];
    xarmlink4_AI = inertiaProps->getTensor_xarmlink4();
    xarmlink4_p = - fext[XARMLINK4];
    xarmlink5_AI = inertiaProps->getTensor_xarmlink5();
    xarmlink5_p = - fext[XARMLINK5];
    xarmlink6_AI = inertiaProps->getTensor_xarmlink6();
    xarmlink6_p = - fext[XARMLINK6];
    // ---------------------- FIRST PASS ---------------------- //
    // Note that, during the first pass, the articulated inertias are really
    //  just the spatial inertia of the links (see assignments above).
    //  Afterwards things change, and articulated inertias shall not be used
    //  in functions which work specifically with spatial inertias.
    
    // + Link omniwheel_fl
    //  - The spatial velocity:
    omniwheel_fl_v = (motionTransforms-> fr_omniwheel_fl_X_fr_base_link_footprint) * base_link_footprint_v;
    omniwheel_fl_v(AZ) += qd(MOTOR_WHEEL_JOINT_FL);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(omniwheel_fl_v, vcross);
    omniwheel_fl_c = vcross.col(AZ) * qd(MOTOR_WHEEL_JOINT_FL);
    
    //  - The bias force term:
    omniwheel_fl_p += vxIv(omniwheel_fl_v, omniwheel_fl_AI);
    
    // + Link omniwheel_fr
    //  - The spatial velocity:
    omniwheel_fr_v = (motionTransforms-> fr_omniwheel_fr_X_fr_base_link_footprint) * base_link_footprint_v;
    omniwheel_fr_v(AZ) += qd(MOTOR_WHEEL_JOINT_FR);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(omniwheel_fr_v, vcross);
    omniwheel_fr_c = vcross.col(AZ) * qd(MOTOR_WHEEL_JOINT_FR);
    
    //  - The bias force term:
    omniwheel_fr_p += vxIv(omniwheel_fr_v, omniwheel_fr_AI);
    
    // + Link omniwheel_bl
    //  - The spatial velocity:
    omniwheel_bl_v = (motionTransforms-> fr_omniwheel_bl_X_fr_base_link_footprint) * base_link_footprint_v;
    omniwheel_bl_v(AZ) += qd(MOTOR_WHEEL_JOINT_BL);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(omniwheel_bl_v, vcross);
    omniwheel_bl_c = vcross.col(AZ) * qd(MOTOR_WHEEL_JOINT_BL);
    
    //  - The bias force term:
    omniwheel_bl_p += vxIv(omniwheel_bl_v, omniwheel_bl_AI);
    
    // + Link omniwheel_br
    //  - The spatial velocity:
    omniwheel_br_v = (motionTransforms-> fr_omniwheel_br_X_fr_base_link_footprint) * base_link_footprint_v;
    omniwheel_br_v(AZ) += qd(MOTOR_WHEEL_JOINT_BR);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(omniwheel_br_v, vcross);
    omniwheel_br_c = vcross.col(AZ) * qd(MOTOR_WHEEL_JOINT_BR);
    
    //  - The bias force term:
    omniwheel_br_p += vxIv(omniwheel_br_v, omniwheel_br_AI);
    
    // + Link xarmlink1
    //  - The spatial velocity:
    xarmlink1_v = (motionTransforms-> fr_xarmlink1_X_fr_base_link_footprint) * base_link_footprint_v;
    xarmlink1_v(AZ) += qd(XARMJOINT1);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(xarmlink1_v, vcross);
    xarmlink1_c = vcross.col(AZ) * qd(XARMJOINT1);
    
    //  - The bias force term:
    xarmlink1_p += vxIv(xarmlink1_v, xarmlink1_AI);
    
    // + Link xarmlink2
    //  - The spatial velocity:
    xarmlink2_v = (motionTransforms-> fr_xarmlink2_X_fr_xarmlink1) * xarmlink1_v;
    xarmlink2_v(AZ) += qd(XARMJOINT2);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(xarmlink2_v, vcross);
    xarmlink2_c = vcross.col(AZ) * qd(XARMJOINT2);
    
    //  - The bias force term:
    xarmlink2_p += vxIv(xarmlink2_v, xarmlink2_AI);
    
    // + Link xarmlink3
    //  - The spatial velocity:
    xarmlink3_v = (motionTransforms-> fr_xarmlink3_X_fr_xarmlink2) * xarmlink2_v;
    xarmlink3_v(AZ) += qd(XARMJOINT3);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(xarmlink3_v, vcross);
    xarmlink3_c = vcross.col(AZ) * qd(XARMJOINT3);
    
    //  - The bias force term:
    xarmlink3_p += vxIv(xarmlink3_v, xarmlink3_AI);
    
    // + Link xarmlink4
    //  - The spatial velocity:
    xarmlink4_v = (motionTransforms-> fr_xarmlink4_X_fr_xarmlink3) * xarmlink3_v;
    xarmlink4_v(AZ) += qd(XARMJOINT4);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(xarmlink4_v, vcross);
    xarmlink4_c = vcross.col(AZ) * qd(XARMJOINT4);
    
    //  - The bias force term:
    xarmlink4_p += vxIv(xarmlink4_v, xarmlink4_AI);
    
    // + Link xarmlink5
    //  - The spatial velocity:
    xarmlink5_v = (motionTransforms-> fr_xarmlink5_X_fr_xarmlink4) * xarmlink4_v;
    xarmlink5_v(AZ) += qd(XARMJOINT5);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(xarmlink5_v, vcross);
    xarmlink5_c = vcross.col(AZ) * qd(XARMJOINT5);
    
    //  - The bias force term:
    xarmlink5_p += vxIv(xarmlink5_v, xarmlink5_AI);
    
    // + Link xarmlink6
    //  - The spatial velocity:
    xarmlink6_v = (motionTransforms-> fr_xarmlink6_X_fr_xarmlink5) * xarmlink5_v;
    xarmlink6_v(AZ) += qd(XARMJOINT6);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(xarmlink6_v, vcross);
    xarmlink6_c = vcross.col(AZ) * qd(XARMJOINT6);
    
    //  - The bias force term:
    xarmlink6_p += vxIv(xarmlink6_v, xarmlink6_AI);
    
    // + The floating base body
    base_link_footprint_p += vxIv(base_link_footprint_v, base_link_footprint_AI);
    
    // ---------------------- SECOND PASS ---------------------- //
    Matrix66 IaB;
    Force pa;
    
    // + Link xarmlink6
    xarmlink6_u = tau(XARMJOINT6) - xarmlink6_p(AZ);
    xarmlink6_U = xarmlink6_AI.col(AZ);
    xarmlink6_D = xarmlink6_U(AZ);
    
    compute_Ia_revolute(xarmlink6_AI, xarmlink6_U, xarmlink6_D, Ia_r);  // same as: Ia_r = xarmlink6_AI - xarmlink6_U/xarmlink6_D * xarmlink6_U.transpose();
    pa = xarmlink6_p + Ia_r * xarmlink6_c + xarmlink6_U * xarmlink6_u/xarmlink6_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_xarmlink6_X_fr_xarmlink5, IaB);
    xarmlink5_AI += IaB;
    xarmlink5_p += (motionTransforms-> fr_xarmlink6_X_fr_xarmlink5).transpose() * pa;
    
    // + Link xarmlink5
    xarmlink5_u = tau(XARMJOINT5) - xarmlink5_p(AZ);
    xarmlink5_U = xarmlink5_AI.col(AZ);
    xarmlink5_D = xarmlink5_U(AZ);
    
    compute_Ia_revolute(xarmlink5_AI, xarmlink5_U, xarmlink5_D, Ia_r);  // same as: Ia_r = xarmlink5_AI - xarmlink5_U/xarmlink5_D * xarmlink5_U.transpose();
    pa = xarmlink5_p + Ia_r * xarmlink5_c + xarmlink5_U * xarmlink5_u/xarmlink5_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_xarmlink5_X_fr_xarmlink4, IaB);
    xarmlink4_AI += IaB;
    xarmlink4_p += (motionTransforms-> fr_xarmlink5_X_fr_xarmlink4).transpose() * pa;
    
    // + Link xarmlink4
    xarmlink4_u = tau(XARMJOINT4) - xarmlink4_p(AZ);
    xarmlink4_U = xarmlink4_AI.col(AZ);
    xarmlink4_D = xarmlink4_U(AZ);
    
    compute_Ia_revolute(xarmlink4_AI, xarmlink4_U, xarmlink4_D, Ia_r);  // same as: Ia_r = xarmlink4_AI - xarmlink4_U/xarmlink4_D * xarmlink4_U.transpose();
    pa = xarmlink4_p + Ia_r * xarmlink4_c + xarmlink4_U * xarmlink4_u/xarmlink4_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_xarmlink4_X_fr_xarmlink3, IaB);
    xarmlink3_AI += IaB;
    xarmlink3_p += (motionTransforms-> fr_xarmlink4_X_fr_xarmlink3).transpose() * pa;
    
    // + Link xarmlink3
    xarmlink3_u = tau(XARMJOINT3) - xarmlink3_p(AZ);
    xarmlink3_U = xarmlink3_AI.col(AZ);
    xarmlink3_D = xarmlink3_U(AZ);
    
    compute_Ia_revolute(xarmlink3_AI, xarmlink3_U, xarmlink3_D, Ia_r);  // same as: Ia_r = xarmlink3_AI - xarmlink3_U/xarmlink3_D * xarmlink3_U.transpose();
    pa = xarmlink3_p + Ia_r * xarmlink3_c + xarmlink3_U * xarmlink3_u/xarmlink3_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_xarmlink3_X_fr_xarmlink2, IaB);
    xarmlink2_AI += IaB;
    xarmlink2_p += (motionTransforms-> fr_xarmlink3_X_fr_xarmlink2).transpose() * pa;
    
    // + Link xarmlink2
    xarmlink2_u = tau(XARMJOINT2) - xarmlink2_p(AZ);
    xarmlink2_U = xarmlink2_AI.col(AZ);
    xarmlink2_D = xarmlink2_U(AZ);
    
    compute_Ia_revolute(xarmlink2_AI, xarmlink2_U, xarmlink2_D, Ia_r);  // same as: Ia_r = xarmlink2_AI - xarmlink2_U/xarmlink2_D * xarmlink2_U.transpose();
    pa = xarmlink2_p + Ia_r * xarmlink2_c + xarmlink2_U * xarmlink2_u/xarmlink2_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_xarmlink2_X_fr_xarmlink1, IaB);
    xarmlink1_AI += IaB;
    xarmlink1_p += (motionTransforms-> fr_xarmlink2_X_fr_xarmlink1).transpose() * pa;
    
    // + Link xarmlink1
    xarmlink1_u = tau(XARMJOINT1) - xarmlink1_p(AZ);
    xarmlink1_U = xarmlink1_AI.col(AZ);
    xarmlink1_D = xarmlink1_U(AZ);
    
    compute_Ia_revolute(xarmlink1_AI, xarmlink1_U, xarmlink1_D, Ia_r);  // same as: Ia_r = xarmlink1_AI - xarmlink1_U/xarmlink1_D * xarmlink1_U.transpose();
    pa = xarmlink1_p + Ia_r * xarmlink1_c + xarmlink1_U * xarmlink1_u/xarmlink1_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_xarmlink1_X_fr_base_link_footprint, IaB);
    base_link_footprint_AI += IaB;
    base_link_footprint_p += (motionTransforms-> fr_xarmlink1_X_fr_base_link_footprint).transpose() * pa;
    
    // + Link omniwheel_br
    omniwheel_br_u = tau(MOTOR_WHEEL_JOINT_BR) - omniwheel_br_p(AZ);
    omniwheel_br_U = omniwheel_br_AI.col(AZ);
    omniwheel_br_D = omniwheel_br_U(AZ);
    
    compute_Ia_revolute(omniwheel_br_AI, omniwheel_br_U, omniwheel_br_D, Ia_r);  // same as: Ia_r = omniwheel_br_AI - omniwheel_br_U/omniwheel_br_D * omniwheel_br_U.transpose();
    pa = omniwheel_br_p + Ia_r * omniwheel_br_c + omniwheel_br_U * omniwheel_br_u/omniwheel_br_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_omniwheel_br_X_fr_base_link_footprint, IaB);
    base_link_footprint_AI += IaB;
    base_link_footprint_p += (motionTransforms-> fr_omniwheel_br_X_fr_base_link_footprint).transpose() * pa;
    
    // + Link omniwheel_bl
    omniwheel_bl_u = tau(MOTOR_WHEEL_JOINT_BL) - omniwheel_bl_p(AZ);
    omniwheel_bl_U = omniwheel_bl_AI.col(AZ);
    omniwheel_bl_D = omniwheel_bl_U(AZ);
    
    compute_Ia_revolute(omniwheel_bl_AI, omniwheel_bl_U, omniwheel_bl_D, Ia_r);  // same as: Ia_r = omniwheel_bl_AI - omniwheel_bl_U/omniwheel_bl_D * omniwheel_bl_U.transpose();
    pa = omniwheel_bl_p + Ia_r * omniwheel_bl_c + omniwheel_bl_U * omniwheel_bl_u/omniwheel_bl_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_omniwheel_bl_X_fr_base_link_footprint, IaB);
    base_link_footprint_AI += IaB;
    base_link_footprint_p += (motionTransforms-> fr_omniwheel_bl_X_fr_base_link_footprint).transpose() * pa;
    
    // + Link omniwheel_fr
    omniwheel_fr_u = tau(MOTOR_WHEEL_JOINT_FR) - omniwheel_fr_p(AZ);
    omniwheel_fr_U = omniwheel_fr_AI.col(AZ);
    omniwheel_fr_D = omniwheel_fr_U(AZ);
    
    compute_Ia_revolute(omniwheel_fr_AI, omniwheel_fr_U, omniwheel_fr_D, Ia_r);  // same as: Ia_r = omniwheel_fr_AI - omniwheel_fr_U/omniwheel_fr_D * omniwheel_fr_U.transpose();
    pa = omniwheel_fr_p + Ia_r * omniwheel_fr_c + omniwheel_fr_U * omniwheel_fr_u/omniwheel_fr_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_omniwheel_fr_X_fr_base_link_footprint, IaB);
    base_link_footprint_AI += IaB;
    base_link_footprint_p += (motionTransforms-> fr_omniwheel_fr_X_fr_base_link_footprint).transpose() * pa;
    
    // + Link omniwheel_fl
    omniwheel_fl_u = tau(MOTOR_WHEEL_JOINT_FL) - omniwheel_fl_p(AZ);
    omniwheel_fl_U = omniwheel_fl_AI.col(AZ);
    omniwheel_fl_D = omniwheel_fl_U(AZ);
    
    compute_Ia_revolute(omniwheel_fl_AI, omniwheel_fl_U, omniwheel_fl_D, Ia_r);  // same as: Ia_r = omniwheel_fl_AI - omniwheel_fl_U/omniwheel_fl_D * omniwheel_fl_U.transpose();
    pa = omniwheel_fl_p + Ia_r * omniwheel_fl_c + omniwheel_fl_U * omniwheel_fl_u/omniwheel_fl_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_omniwheel_fl_X_fr_base_link_footprint, IaB);
    base_link_footprint_AI += IaB;
    base_link_footprint_p += (motionTransforms-> fr_omniwheel_fl_X_fr_base_link_footprint).transpose() * pa;
    
    // + The acceleration of the floating base base_link_footprint, without gravity
    base_link_footprint_a = - base_link_footprint_AI.llt().solve(base_link_footprint_p);  // base_link_footprint_a = - IA^-1 * base_link_footprint_p
    
    // ---------------------- THIRD PASS ---------------------- //
    omniwheel_fl_a = (motionTransforms-> fr_omniwheel_fl_X_fr_base_link_footprint) * base_link_footprint_a + omniwheel_fl_c;
    qdd(MOTOR_WHEEL_JOINT_FL) = (omniwheel_fl_u - omniwheel_fl_U.dot(omniwheel_fl_a)) / omniwheel_fl_D;
    omniwheel_fl_a(AZ) += qdd(MOTOR_WHEEL_JOINT_FL);
    
    omniwheel_fr_a = (motionTransforms-> fr_omniwheel_fr_X_fr_base_link_footprint) * base_link_footprint_a + omniwheel_fr_c;
    qdd(MOTOR_WHEEL_JOINT_FR) = (omniwheel_fr_u - omniwheel_fr_U.dot(omniwheel_fr_a)) / omniwheel_fr_D;
    omniwheel_fr_a(AZ) += qdd(MOTOR_WHEEL_JOINT_FR);
    
    omniwheel_bl_a = (motionTransforms-> fr_omniwheel_bl_X_fr_base_link_footprint) * base_link_footprint_a + omniwheel_bl_c;
    qdd(MOTOR_WHEEL_JOINT_BL) = (omniwheel_bl_u - omniwheel_bl_U.dot(omniwheel_bl_a)) / omniwheel_bl_D;
    omniwheel_bl_a(AZ) += qdd(MOTOR_WHEEL_JOINT_BL);
    
    omniwheel_br_a = (motionTransforms-> fr_omniwheel_br_X_fr_base_link_footprint) * base_link_footprint_a + omniwheel_br_c;
    qdd(MOTOR_WHEEL_JOINT_BR) = (omniwheel_br_u - omniwheel_br_U.dot(omniwheel_br_a)) / omniwheel_br_D;
    omniwheel_br_a(AZ) += qdd(MOTOR_WHEEL_JOINT_BR);
    
    xarmlink1_a = (motionTransforms-> fr_xarmlink1_X_fr_base_link_footprint) * base_link_footprint_a + xarmlink1_c;
    qdd(XARMJOINT1) = (xarmlink1_u - xarmlink1_U.dot(xarmlink1_a)) / xarmlink1_D;
    xarmlink1_a(AZ) += qdd(XARMJOINT1);
    
    xarmlink2_a = (motionTransforms-> fr_xarmlink2_X_fr_xarmlink1) * xarmlink1_a + xarmlink2_c;
    qdd(XARMJOINT2) = (xarmlink2_u - xarmlink2_U.dot(xarmlink2_a)) / xarmlink2_D;
    xarmlink2_a(AZ) += qdd(XARMJOINT2);
    
    xarmlink3_a = (motionTransforms-> fr_xarmlink3_X_fr_xarmlink2) * xarmlink2_a + xarmlink3_c;
    qdd(XARMJOINT3) = (xarmlink3_u - xarmlink3_U.dot(xarmlink3_a)) / xarmlink3_D;
    xarmlink3_a(AZ) += qdd(XARMJOINT3);
    
    xarmlink4_a = (motionTransforms-> fr_xarmlink4_X_fr_xarmlink3) * xarmlink3_a + xarmlink4_c;
    qdd(XARMJOINT4) = (xarmlink4_u - xarmlink4_U.dot(xarmlink4_a)) / xarmlink4_D;
    xarmlink4_a(AZ) += qdd(XARMJOINT4);
    
    xarmlink5_a = (motionTransforms-> fr_xarmlink5_X_fr_xarmlink4) * xarmlink4_a + xarmlink5_c;
    qdd(XARMJOINT5) = (xarmlink5_u - xarmlink5_U.dot(xarmlink5_a)) / xarmlink5_D;
    xarmlink5_a(AZ) += qdd(XARMJOINT5);
    
    xarmlink6_a = (motionTransforms-> fr_xarmlink6_X_fr_xarmlink5) * xarmlink5_a + xarmlink6_c;
    qdd(XARMJOINT6) = (xarmlink6_u - xarmlink6_U.dot(xarmlink6_a)) / xarmlink6_D;
    xarmlink6_a(AZ) += qdd(XARMJOINT6);
    
    
    // + Add gravity to the acceleration of the floating base
    base_link_footprint_a += g;
}
