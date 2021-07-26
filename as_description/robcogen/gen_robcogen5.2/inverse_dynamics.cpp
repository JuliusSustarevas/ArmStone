#include <iit/rbd/robcogen_commons.h>

#include "inverse_dynamics.h"
#include "inertia_properties.h"
#ifndef EIGEN_NO_DEBUG
    #include <iostream>
#endif
using namespace std;
using namespace iit::rbd;
using namespace armstone::rcg;

// Initialization of static-const data
const armstone::rcg::InverseDynamics::ExtForces
armstone::rcg::InverseDynamics::zeroExtForces(Force::Zero());

armstone::rcg::InverseDynamics::InverseDynamics(InertiaProperties& inertia, MotionTransforms& transforms) :
    inertiaProps( & inertia ),
    xm( & transforms ),
    omniwheel_fl_I(inertiaProps->getTensor_omniwheel_fl() ),
    omniwheel_fr_I(inertiaProps->getTensor_omniwheel_fr() ),
    omniwheel_bl_I(inertiaProps->getTensor_omniwheel_bl() ),
    omniwheel_br_I(inertiaProps->getTensor_omniwheel_br() ),
    xarmlink1_I(inertiaProps->getTensor_xarmlink1() ),
    xarmlink2_I(inertiaProps->getTensor_xarmlink2() ),
    xarmlink3_I(inertiaProps->getTensor_xarmlink3() ),
    xarmlink4_I(inertiaProps->getTensor_xarmlink4() ),
    xarmlink5_I(inertiaProps->getTensor_xarmlink5() ),
    xarmlink6_I(inertiaProps->getTensor_xarmlink6() )
    ,
        base_link_footprint_I( inertiaProps->getTensor_base_link_footprint() ),
        omniwheel_fl_Ic(omniwheel_fl_I),
        omniwheel_fr_Ic(omniwheel_fr_I),
        omniwheel_bl_Ic(omniwheel_bl_I),
        omniwheel_br_Ic(omniwheel_br_I),
        xarmlink6_Ic(xarmlink6_I)
{
#ifndef EIGEN_NO_DEBUG
    std::cout << "Robot armstone, InverseDynamics::InverseDynamics()" << std::endl;
    std::cout << "Compiled with Eigen debug active" << std::endl;
#endif
    omniwheel_fl_v.setZero();
    omniwheel_fr_v.setZero();
    omniwheel_bl_v.setZero();
    omniwheel_br_v.setZero();
    xarmlink1_v.setZero();
    xarmlink2_v.setZero();
    xarmlink3_v.setZero();
    xarmlink4_v.setZero();
    xarmlink5_v.setZero();
    xarmlink6_v.setZero();

    vcross.setZero();
}

void armstone::rcg::InverseDynamics::id(
    JointState& jForces, Acceleration& base_link_footprint_a,
    const Acceleration& g, const Velocity& base_link_footprint_v,
    const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    base_link_footprint_Ic = base_link_footprint_I;
    xarmlink1_Ic = xarmlink1_I;
    xarmlink2_Ic = xarmlink2_I;
    xarmlink3_Ic = xarmlink3_I;
    xarmlink4_Ic = xarmlink4_I;
    xarmlink5_Ic = xarmlink5_I;

    // First pass, link 'omniwheel_fl'
    omniwheel_fl_v = ((xm->fr_omniwheel_fl_X_fr_base_link_footprint) * base_link_footprint_v);
    omniwheel_fl_v(iit::rbd::AZ) += qd(MOTOR_WHEEL_JOINT_FL);
    
    motionCrossProductMx<Scalar>(omniwheel_fl_v, vcross);
    
    omniwheel_fl_a = (vcross.col(iit::rbd::AZ) * qd(MOTOR_WHEEL_JOINT_FL));
    omniwheel_fl_a(iit::rbd::AZ) += qdd(MOTOR_WHEEL_JOINT_FL);
    
    omniwheel_fl_f = omniwheel_fl_I * omniwheel_fl_a + vxIv(omniwheel_fl_v, omniwheel_fl_I);
    
    // First pass, link 'omniwheel_fr'
    omniwheel_fr_v = ((xm->fr_omniwheel_fr_X_fr_base_link_footprint) * base_link_footprint_v);
    omniwheel_fr_v(iit::rbd::AZ) += qd(MOTOR_WHEEL_JOINT_FR);
    
    motionCrossProductMx<Scalar>(omniwheel_fr_v, vcross);
    
    omniwheel_fr_a = (vcross.col(iit::rbd::AZ) * qd(MOTOR_WHEEL_JOINT_FR));
    omniwheel_fr_a(iit::rbd::AZ) += qdd(MOTOR_WHEEL_JOINT_FR);
    
    omniwheel_fr_f = omniwheel_fr_I * omniwheel_fr_a + vxIv(omniwheel_fr_v, omniwheel_fr_I);
    
    // First pass, link 'omniwheel_bl'
    omniwheel_bl_v = ((xm->fr_omniwheel_bl_X_fr_base_link_footprint) * base_link_footprint_v);
    omniwheel_bl_v(iit::rbd::AZ) += qd(MOTOR_WHEEL_JOINT_BL);
    
    motionCrossProductMx<Scalar>(omniwheel_bl_v, vcross);
    
    omniwheel_bl_a = (vcross.col(iit::rbd::AZ) * qd(MOTOR_WHEEL_JOINT_BL));
    omniwheel_bl_a(iit::rbd::AZ) += qdd(MOTOR_WHEEL_JOINT_BL);
    
    omniwheel_bl_f = omniwheel_bl_I * omniwheel_bl_a + vxIv(omniwheel_bl_v, omniwheel_bl_I);
    
    // First pass, link 'omniwheel_br'
    omniwheel_br_v = ((xm->fr_omniwheel_br_X_fr_base_link_footprint) * base_link_footprint_v);
    omniwheel_br_v(iit::rbd::AZ) += qd(MOTOR_WHEEL_JOINT_BR);
    
    motionCrossProductMx<Scalar>(omniwheel_br_v, vcross);
    
    omniwheel_br_a = (vcross.col(iit::rbd::AZ) * qd(MOTOR_WHEEL_JOINT_BR));
    omniwheel_br_a(iit::rbd::AZ) += qdd(MOTOR_WHEEL_JOINT_BR);
    
    omniwheel_br_f = omniwheel_br_I * omniwheel_br_a + vxIv(omniwheel_br_v, omniwheel_br_I);
    
    // First pass, link 'xarmlink1'
    xarmlink1_v = ((xm->fr_xarmlink1_X_fr_base_link_footprint) * base_link_footprint_v);
    xarmlink1_v(iit::rbd::AZ) += qd(XARMJOINT1);
    
    motionCrossProductMx<Scalar>(xarmlink1_v, vcross);
    
    xarmlink1_a = (vcross.col(iit::rbd::AZ) * qd(XARMJOINT1));
    xarmlink1_a(iit::rbd::AZ) += qdd(XARMJOINT1);
    
    xarmlink1_f = xarmlink1_I * xarmlink1_a + vxIv(xarmlink1_v, xarmlink1_I);
    
    // First pass, link 'xarmlink2'
    xarmlink2_v = ((xm->fr_xarmlink2_X_fr_xarmlink1) * xarmlink1_v);
    xarmlink2_v(iit::rbd::AZ) += qd(XARMJOINT2);
    
    motionCrossProductMx<Scalar>(xarmlink2_v, vcross);
    
    xarmlink2_a = (xm->fr_xarmlink2_X_fr_xarmlink1) * xarmlink1_a + vcross.col(iit::rbd::AZ) * qd(XARMJOINT2);
    xarmlink2_a(iit::rbd::AZ) += qdd(XARMJOINT2);
    
    xarmlink2_f = xarmlink2_I * xarmlink2_a + vxIv(xarmlink2_v, xarmlink2_I);
    
    // First pass, link 'xarmlink3'
    xarmlink3_v = ((xm->fr_xarmlink3_X_fr_xarmlink2) * xarmlink2_v);
    xarmlink3_v(iit::rbd::AZ) += qd(XARMJOINT3);
    
    motionCrossProductMx<Scalar>(xarmlink3_v, vcross);
    
    xarmlink3_a = (xm->fr_xarmlink3_X_fr_xarmlink2) * xarmlink2_a + vcross.col(iit::rbd::AZ) * qd(XARMJOINT3);
    xarmlink3_a(iit::rbd::AZ) += qdd(XARMJOINT3);
    
    xarmlink3_f = xarmlink3_I * xarmlink3_a + vxIv(xarmlink3_v, xarmlink3_I);
    
    // First pass, link 'xarmlink4'
    xarmlink4_v = ((xm->fr_xarmlink4_X_fr_xarmlink3) * xarmlink3_v);
    xarmlink4_v(iit::rbd::AZ) += qd(XARMJOINT4);
    
    motionCrossProductMx<Scalar>(xarmlink4_v, vcross);
    
    xarmlink4_a = (xm->fr_xarmlink4_X_fr_xarmlink3) * xarmlink3_a + vcross.col(iit::rbd::AZ) * qd(XARMJOINT4);
    xarmlink4_a(iit::rbd::AZ) += qdd(XARMJOINT4);
    
    xarmlink4_f = xarmlink4_I * xarmlink4_a + vxIv(xarmlink4_v, xarmlink4_I);
    
    // First pass, link 'xarmlink5'
    xarmlink5_v = ((xm->fr_xarmlink5_X_fr_xarmlink4) * xarmlink4_v);
    xarmlink5_v(iit::rbd::AZ) += qd(XARMJOINT5);
    
    motionCrossProductMx<Scalar>(xarmlink5_v, vcross);
    
    xarmlink5_a = (xm->fr_xarmlink5_X_fr_xarmlink4) * xarmlink4_a + vcross.col(iit::rbd::AZ) * qd(XARMJOINT5);
    xarmlink5_a(iit::rbd::AZ) += qdd(XARMJOINT5);
    
    xarmlink5_f = xarmlink5_I * xarmlink5_a + vxIv(xarmlink5_v, xarmlink5_I);
    
    // First pass, link 'xarmlink6'
    xarmlink6_v = ((xm->fr_xarmlink6_X_fr_xarmlink5) * xarmlink5_v);
    xarmlink6_v(iit::rbd::AZ) += qd(XARMJOINT6);
    
    motionCrossProductMx<Scalar>(xarmlink6_v, vcross);
    
    xarmlink6_a = (xm->fr_xarmlink6_X_fr_xarmlink5) * xarmlink5_a + vcross.col(iit::rbd::AZ) * qd(XARMJOINT6);
    xarmlink6_a(iit::rbd::AZ) += qdd(XARMJOINT6);
    
    xarmlink6_f = xarmlink6_I * xarmlink6_a + vxIv(xarmlink6_v, xarmlink6_I);
    
    // The force exerted on the floating base by the links
    base_link_footprint_f = vxIv(base_link_footprint_v, base_link_footprint_I);
    

    // Add the external forces:
    base_link_footprint_f -= fext[BASE_LINK_FOOTPRINT];
    omniwheel_fl_f -= fext[OMNIWHEEL_FL];
    omniwheel_fr_f -= fext[OMNIWHEEL_FR];
    omniwheel_bl_f -= fext[OMNIWHEEL_BL];
    omniwheel_br_f -= fext[OMNIWHEEL_BR];
    xarmlink1_f -= fext[XARMLINK1];
    xarmlink2_f -= fext[XARMLINK2];
    xarmlink3_f -= fext[XARMLINK3];
    xarmlink4_f -= fext[XARMLINK4];
    xarmlink5_f -= fext[XARMLINK5];
    xarmlink6_f -= fext[XARMLINK6];

    InertiaMatrix Ic_spare;
    iit::rbd::transformInertia<Scalar>(xarmlink6_Ic, (xm->fr_xarmlink6_X_fr_xarmlink5).transpose(), Ic_spare);
    xarmlink5_Ic += Ic_spare;
    xarmlink5_f = xarmlink5_f + (xm->fr_xarmlink6_X_fr_xarmlink5).transpose() * xarmlink6_f;
    
    iit::rbd::transformInertia<Scalar>(xarmlink5_Ic, (xm->fr_xarmlink5_X_fr_xarmlink4).transpose(), Ic_spare);
    xarmlink4_Ic += Ic_spare;
    xarmlink4_f = xarmlink4_f + (xm->fr_xarmlink5_X_fr_xarmlink4).transpose() * xarmlink5_f;
    
    iit::rbd::transformInertia<Scalar>(xarmlink4_Ic, (xm->fr_xarmlink4_X_fr_xarmlink3).transpose(), Ic_spare);
    xarmlink3_Ic += Ic_spare;
    xarmlink3_f = xarmlink3_f + (xm->fr_xarmlink4_X_fr_xarmlink3).transpose() * xarmlink4_f;
    
    iit::rbd::transformInertia<Scalar>(xarmlink3_Ic, (xm->fr_xarmlink3_X_fr_xarmlink2).transpose(), Ic_spare);
    xarmlink2_Ic += Ic_spare;
    xarmlink2_f = xarmlink2_f + (xm->fr_xarmlink3_X_fr_xarmlink2).transpose() * xarmlink3_f;
    
    iit::rbd::transformInertia<Scalar>(xarmlink2_Ic, (xm->fr_xarmlink2_X_fr_xarmlink1).transpose(), Ic_spare);
    xarmlink1_Ic += Ic_spare;
    xarmlink1_f = xarmlink1_f + (xm->fr_xarmlink2_X_fr_xarmlink1).transpose() * xarmlink2_f;
    
    iit::rbd::transformInertia<Scalar>(xarmlink1_Ic, (xm->fr_xarmlink1_X_fr_base_link_footprint).transpose(), Ic_spare);
    base_link_footprint_Ic += Ic_spare;
    base_link_footprint_f = base_link_footprint_f + (xm->fr_xarmlink1_X_fr_base_link_footprint).transpose() * xarmlink1_f;
    
    iit::rbd::transformInertia<Scalar>(omniwheel_br_Ic, (xm->fr_omniwheel_br_X_fr_base_link_footprint).transpose(), Ic_spare);
    base_link_footprint_Ic += Ic_spare;
    base_link_footprint_f = base_link_footprint_f + (xm->fr_omniwheel_br_X_fr_base_link_footprint).transpose() * omniwheel_br_f;
    
    iit::rbd::transformInertia<Scalar>(omniwheel_bl_Ic, (xm->fr_omniwheel_bl_X_fr_base_link_footprint).transpose(), Ic_spare);
    base_link_footprint_Ic += Ic_spare;
    base_link_footprint_f = base_link_footprint_f + (xm->fr_omniwheel_bl_X_fr_base_link_footprint).transpose() * omniwheel_bl_f;
    
    iit::rbd::transformInertia<Scalar>(omniwheel_fr_Ic, (xm->fr_omniwheel_fr_X_fr_base_link_footprint).transpose(), Ic_spare);
    base_link_footprint_Ic += Ic_spare;
    base_link_footprint_f = base_link_footprint_f + (xm->fr_omniwheel_fr_X_fr_base_link_footprint).transpose() * omniwheel_fr_f;
    
    iit::rbd::transformInertia<Scalar>(omniwheel_fl_Ic, (xm->fr_omniwheel_fl_X_fr_base_link_footprint).transpose(), Ic_spare);
    base_link_footprint_Ic += Ic_spare;
    base_link_footprint_f = base_link_footprint_f + (xm->fr_omniwheel_fl_X_fr_base_link_footprint).transpose() * omniwheel_fl_f;
    

    // The base acceleration due to the force due to the movement of the links
    base_link_footprint_a = - base_link_footprint_Ic.inverse() * base_link_footprint_f;
    
    omniwheel_fl_a = xm->fr_omniwheel_fl_X_fr_base_link_footprint * base_link_footprint_a;
    jForces(MOTOR_WHEEL_JOINT_FL) = (omniwheel_fl_Ic.row(iit::rbd::AZ) * omniwheel_fl_a + omniwheel_fl_f(iit::rbd::AZ));
    
    omniwheel_fr_a = xm->fr_omniwheel_fr_X_fr_base_link_footprint * base_link_footprint_a;
    jForces(MOTOR_WHEEL_JOINT_FR) = (omniwheel_fr_Ic.row(iit::rbd::AZ) * omniwheel_fr_a + omniwheel_fr_f(iit::rbd::AZ));
    
    omniwheel_bl_a = xm->fr_omniwheel_bl_X_fr_base_link_footprint * base_link_footprint_a;
    jForces(MOTOR_WHEEL_JOINT_BL) = (omniwheel_bl_Ic.row(iit::rbd::AZ) * omniwheel_bl_a + omniwheel_bl_f(iit::rbd::AZ));
    
    omniwheel_br_a = xm->fr_omniwheel_br_X_fr_base_link_footprint * base_link_footprint_a;
    jForces(MOTOR_WHEEL_JOINT_BR) = (omniwheel_br_Ic.row(iit::rbd::AZ) * omniwheel_br_a + omniwheel_br_f(iit::rbd::AZ));
    
    xarmlink1_a = xm->fr_xarmlink1_X_fr_base_link_footprint * base_link_footprint_a;
    jForces(XARMJOINT1) = (xarmlink1_Ic.row(iit::rbd::AZ) * xarmlink1_a + xarmlink1_f(iit::rbd::AZ));
    
    xarmlink2_a = xm->fr_xarmlink2_X_fr_xarmlink1 * xarmlink1_a;
    jForces(XARMJOINT2) = (xarmlink2_Ic.row(iit::rbd::AZ) * xarmlink2_a + xarmlink2_f(iit::rbd::AZ));
    
    xarmlink3_a = xm->fr_xarmlink3_X_fr_xarmlink2 * xarmlink2_a;
    jForces(XARMJOINT3) = (xarmlink3_Ic.row(iit::rbd::AZ) * xarmlink3_a + xarmlink3_f(iit::rbd::AZ));
    
    xarmlink4_a = xm->fr_xarmlink4_X_fr_xarmlink3 * xarmlink3_a;
    jForces(XARMJOINT4) = (xarmlink4_Ic.row(iit::rbd::AZ) * xarmlink4_a + xarmlink4_f(iit::rbd::AZ));
    
    xarmlink5_a = xm->fr_xarmlink5_X_fr_xarmlink4 * xarmlink4_a;
    jForces(XARMJOINT5) = (xarmlink5_Ic.row(iit::rbd::AZ) * xarmlink5_a + xarmlink5_f(iit::rbd::AZ));
    
    xarmlink6_a = xm->fr_xarmlink6_X_fr_xarmlink5 * xarmlink5_a;
    jForces(XARMJOINT6) = (xarmlink6_Ic.row(iit::rbd::AZ) * xarmlink6_a + xarmlink6_f(iit::rbd::AZ));
    

    base_link_footprint_a += g;
}


void armstone::rcg::InverseDynamics::G_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Acceleration& g)
{
    const Acceleration& base_link_footprint_a = -g;

    // Link 'omniwheel_fl'
    omniwheel_fl_a = (xm->fr_omniwheel_fl_X_fr_base_link_footprint) * base_link_footprint_a;
    omniwheel_fl_f = omniwheel_fl_I * omniwheel_fl_a;
    // Link 'omniwheel_fr'
    omniwheel_fr_a = (xm->fr_omniwheel_fr_X_fr_base_link_footprint) * base_link_footprint_a;
    omniwheel_fr_f = omniwheel_fr_I * omniwheel_fr_a;
    // Link 'omniwheel_bl'
    omniwheel_bl_a = (xm->fr_omniwheel_bl_X_fr_base_link_footprint) * base_link_footprint_a;
    omniwheel_bl_f = omniwheel_bl_I * omniwheel_bl_a;
    // Link 'omniwheel_br'
    omniwheel_br_a = (xm->fr_omniwheel_br_X_fr_base_link_footprint) * base_link_footprint_a;
    omniwheel_br_f = omniwheel_br_I * omniwheel_br_a;
    // Link 'xarmlink1'
    xarmlink1_a = (xm->fr_xarmlink1_X_fr_base_link_footprint) * base_link_footprint_a;
    xarmlink1_f = xarmlink1_I * xarmlink1_a;
    // Link 'xarmlink2'
    xarmlink2_a = (xm->fr_xarmlink2_X_fr_xarmlink1) * xarmlink1_a;
    xarmlink2_f = xarmlink2_I * xarmlink2_a;
    // Link 'xarmlink3'
    xarmlink3_a = (xm->fr_xarmlink3_X_fr_xarmlink2) * xarmlink2_a;
    xarmlink3_f = xarmlink3_I * xarmlink3_a;
    // Link 'xarmlink4'
    xarmlink4_a = (xm->fr_xarmlink4_X_fr_xarmlink3) * xarmlink3_a;
    xarmlink4_f = xarmlink4_I * xarmlink4_a;
    // Link 'xarmlink5'
    xarmlink5_a = (xm->fr_xarmlink5_X_fr_xarmlink4) * xarmlink4_a;
    xarmlink5_f = xarmlink5_I * xarmlink5_a;
    // Link 'xarmlink6'
    xarmlink6_a = (xm->fr_xarmlink6_X_fr_xarmlink5) * xarmlink5_a;
    xarmlink6_f = xarmlink6_I * xarmlink6_a;

    base_link_footprint_f = base_link_footprint_I * base_link_footprint_a;

    secondPass_fullyActuated(jForces);

    baseWrench = base_link_footprint_f;
}

void armstone::rcg::InverseDynamics::C_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Velocity& base_link_footprint_v, const JointState& qd)
{
    // Link 'omniwheel_fl'
    omniwheel_fl_v = ((xm->fr_omniwheel_fl_X_fr_base_link_footprint) * base_link_footprint_v);
    omniwheel_fl_v(iit::rbd::AZ) += qd(MOTOR_WHEEL_JOINT_FL);
    motionCrossProductMx<Scalar>(omniwheel_fl_v, vcross);
    omniwheel_fl_a = (vcross.col(iit::rbd::AZ) * qd(MOTOR_WHEEL_JOINT_FL));
    omniwheel_fl_f = omniwheel_fl_I * omniwheel_fl_a + vxIv(omniwheel_fl_v, omniwheel_fl_I);
    
    // Link 'omniwheel_fr'
    omniwheel_fr_v = ((xm->fr_omniwheel_fr_X_fr_base_link_footprint) * base_link_footprint_v);
    omniwheel_fr_v(iit::rbd::AZ) += qd(MOTOR_WHEEL_JOINT_FR);
    motionCrossProductMx<Scalar>(omniwheel_fr_v, vcross);
    omniwheel_fr_a = (vcross.col(iit::rbd::AZ) * qd(MOTOR_WHEEL_JOINT_FR));
    omniwheel_fr_f = omniwheel_fr_I * omniwheel_fr_a + vxIv(omniwheel_fr_v, omniwheel_fr_I);
    
    // Link 'omniwheel_bl'
    omniwheel_bl_v = ((xm->fr_omniwheel_bl_X_fr_base_link_footprint) * base_link_footprint_v);
    omniwheel_bl_v(iit::rbd::AZ) += qd(MOTOR_WHEEL_JOINT_BL);
    motionCrossProductMx<Scalar>(omniwheel_bl_v, vcross);
    omniwheel_bl_a = (vcross.col(iit::rbd::AZ) * qd(MOTOR_WHEEL_JOINT_BL));
    omniwheel_bl_f = omniwheel_bl_I * omniwheel_bl_a + vxIv(omniwheel_bl_v, omniwheel_bl_I);
    
    // Link 'omniwheel_br'
    omniwheel_br_v = ((xm->fr_omniwheel_br_X_fr_base_link_footprint) * base_link_footprint_v);
    omniwheel_br_v(iit::rbd::AZ) += qd(MOTOR_WHEEL_JOINT_BR);
    motionCrossProductMx<Scalar>(omniwheel_br_v, vcross);
    omniwheel_br_a = (vcross.col(iit::rbd::AZ) * qd(MOTOR_WHEEL_JOINT_BR));
    omniwheel_br_f = omniwheel_br_I * omniwheel_br_a + vxIv(omniwheel_br_v, omniwheel_br_I);
    
    // Link 'xarmlink1'
    xarmlink1_v = ((xm->fr_xarmlink1_X_fr_base_link_footprint) * base_link_footprint_v);
    xarmlink1_v(iit::rbd::AZ) += qd(XARMJOINT1);
    motionCrossProductMx<Scalar>(xarmlink1_v, vcross);
    xarmlink1_a = (vcross.col(iit::rbd::AZ) * qd(XARMJOINT1));
    xarmlink1_f = xarmlink1_I * xarmlink1_a + vxIv(xarmlink1_v, xarmlink1_I);
    
    // Link 'xarmlink2'
    xarmlink2_v = ((xm->fr_xarmlink2_X_fr_xarmlink1) * xarmlink1_v);
    xarmlink2_v(iit::rbd::AZ) += qd(XARMJOINT2);
    motionCrossProductMx<Scalar>(xarmlink2_v, vcross);
    xarmlink2_a = (xm->fr_xarmlink2_X_fr_xarmlink1) * xarmlink1_a + vcross.col(iit::rbd::AZ) * qd(XARMJOINT2);
    xarmlink2_f = xarmlink2_I * xarmlink2_a + vxIv(xarmlink2_v, xarmlink2_I);
    
    // Link 'xarmlink3'
    xarmlink3_v = ((xm->fr_xarmlink3_X_fr_xarmlink2) * xarmlink2_v);
    xarmlink3_v(iit::rbd::AZ) += qd(XARMJOINT3);
    motionCrossProductMx<Scalar>(xarmlink3_v, vcross);
    xarmlink3_a = (xm->fr_xarmlink3_X_fr_xarmlink2) * xarmlink2_a + vcross.col(iit::rbd::AZ) * qd(XARMJOINT3);
    xarmlink3_f = xarmlink3_I * xarmlink3_a + vxIv(xarmlink3_v, xarmlink3_I);
    
    // Link 'xarmlink4'
    xarmlink4_v = ((xm->fr_xarmlink4_X_fr_xarmlink3) * xarmlink3_v);
    xarmlink4_v(iit::rbd::AZ) += qd(XARMJOINT4);
    motionCrossProductMx<Scalar>(xarmlink4_v, vcross);
    xarmlink4_a = (xm->fr_xarmlink4_X_fr_xarmlink3) * xarmlink3_a + vcross.col(iit::rbd::AZ) * qd(XARMJOINT4);
    xarmlink4_f = xarmlink4_I * xarmlink4_a + vxIv(xarmlink4_v, xarmlink4_I);
    
    // Link 'xarmlink5'
    xarmlink5_v = ((xm->fr_xarmlink5_X_fr_xarmlink4) * xarmlink4_v);
    xarmlink5_v(iit::rbd::AZ) += qd(XARMJOINT5);
    motionCrossProductMx<Scalar>(xarmlink5_v, vcross);
    xarmlink5_a = (xm->fr_xarmlink5_X_fr_xarmlink4) * xarmlink4_a + vcross.col(iit::rbd::AZ) * qd(XARMJOINT5);
    xarmlink5_f = xarmlink5_I * xarmlink5_a + vxIv(xarmlink5_v, xarmlink5_I);
    
    // Link 'xarmlink6'
    xarmlink6_v = ((xm->fr_xarmlink6_X_fr_xarmlink5) * xarmlink5_v);
    xarmlink6_v(iit::rbd::AZ) += qd(XARMJOINT6);
    motionCrossProductMx<Scalar>(xarmlink6_v, vcross);
    xarmlink6_a = (xm->fr_xarmlink6_X_fr_xarmlink5) * xarmlink5_a + vcross.col(iit::rbd::AZ) * qd(XARMJOINT6);
    xarmlink6_f = xarmlink6_I * xarmlink6_a + vxIv(xarmlink6_v, xarmlink6_I);
    

    base_link_footprint_f = vxIv(base_link_footprint_v, base_link_footprint_I);

    secondPass_fullyActuated(jForces);

    baseWrench = base_link_footprint_f;
}

void armstone::rcg::InverseDynamics::id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& base_link_footprint_v, const Acceleration& baseAccel,
        const JointState& qd, const JointState& qdd, const ExtForces& fext)
{
    Acceleration base_link_footprint_a = baseAccel -g;

    // First pass, link 'omniwheel_fl'
    omniwheel_fl_v = ((xm->fr_omniwheel_fl_X_fr_base_link_footprint) * base_link_footprint_v);
    omniwheel_fl_v(iit::rbd::AZ) += qd(MOTOR_WHEEL_JOINT_FL);
    
    motionCrossProductMx<Scalar>(omniwheel_fl_v, vcross);
    
    omniwheel_fl_a = (xm->fr_omniwheel_fl_X_fr_base_link_footprint) * base_link_footprint_a + vcross.col(iit::rbd::AZ) * qd(MOTOR_WHEEL_JOINT_FL);
    omniwheel_fl_a(iit::rbd::AZ) += qdd(MOTOR_WHEEL_JOINT_FL);
    
    omniwheel_fl_f = omniwheel_fl_I * omniwheel_fl_a + vxIv(omniwheel_fl_v, omniwheel_fl_I) - fext[OMNIWHEEL_FL];
    
    // First pass, link 'omniwheel_fr'
    omniwheel_fr_v = ((xm->fr_omniwheel_fr_X_fr_base_link_footprint) * base_link_footprint_v);
    omniwheel_fr_v(iit::rbd::AZ) += qd(MOTOR_WHEEL_JOINT_FR);
    
    motionCrossProductMx<Scalar>(omniwheel_fr_v, vcross);
    
    omniwheel_fr_a = (xm->fr_omniwheel_fr_X_fr_base_link_footprint) * base_link_footprint_a + vcross.col(iit::rbd::AZ) * qd(MOTOR_WHEEL_JOINT_FR);
    omniwheel_fr_a(iit::rbd::AZ) += qdd(MOTOR_WHEEL_JOINT_FR);
    
    omniwheel_fr_f = omniwheel_fr_I * omniwheel_fr_a + vxIv(omniwheel_fr_v, omniwheel_fr_I) - fext[OMNIWHEEL_FR];
    
    // First pass, link 'omniwheel_bl'
    omniwheel_bl_v = ((xm->fr_omniwheel_bl_X_fr_base_link_footprint) * base_link_footprint_v);
    omniwheel_bl_v(iit::rbd::AZ) += qd(MOTOR_WHEEL_JOINT_BL);
    
    motionCrossProductMx<Scalar>(omniwheel_bl_v, vcross);
    
    omniwheel_bl_a = (xm->fr_omniwheel_bl_X_fr_base_link_footprint) * base_link_footprint_a + vcross.col(iit::rbd::AZ) * qd(MOTOR_WHEEL_JOINT_BL);
    omniwheel_bl_a(iit::rbd::AZ) += qdd(MOTOR_WHEEL_JOINT_BL);
    
    omniwheel_bl_f = omniwheel_bl_I * omniwheel_bl_a + vxIv(omniwheel_bl_v, omniwheel_bl_I) - fext[OMNIWHEEL_BL];
    
    // First pass, link 'omniwheel_br'
    omniwheel_br_v = ((xm->fr_omniwheel_br_X_fr_base_link_footprint) * base_link_footprint_v);
    omniwheel_br_v(iit::rbd::AZ) += qd(MOTOR_WHEEL_JOINT_BR);
    
    motionCrossProductMx<Scalar>(omniwheel_br_v, vcross);
    
    omniwheel_br_a = (xm->fr_omniwheel_br_X_fr_base_link_footprint) * base_link_footprint_a + vcross.col(iit::rbd::AZ) * qd(MOTOR_WHEEL_JOINT_BR);
    omniwheel_br_a(iit::rbd::AZ) += qdd(MOTOR_WHEEL_JOINT_BR);
    
    omniwheel_br_f = omniwheel_br_I * omniwheel_br_a + vxIv(omniwheel_br_v, omniwheel_br_I) - fext[OMNIWHEEL_BR];
    
    // First pass, link 'xarmlink1'
    xarmlink1_v = ((xm->fr_xarmlink1_X_fr_base_link_footprint) * base_link_footprint_v);
    xarmlink1_v(iit::rbd::AZ) += qd(XARMJOINT1);
    
    motionCrossProductMx<Scalar>(xarmlink1_v, vcross);
    
    xarmlink1_a = (xm->fr_xarmlink1_X_fr_base_link_footprint) * base_link_footprint_a + vcross.col(iit::rbd::AZ) * qd(XARMJOINT1);
    xarmlink1_a(iit::rbd::AZ) += qdd(XARMJOINT1);
    
    xarmlink1_f = xarmlink1_I * xarmlink1_a + vxIv(xarmlink1_v, xarmlink1_I) - fext[XARMLINK1];
    
    // First pass, link 'xarmlink2'
    xarmlink2_v = ((xm->fr_xarmlink2_X_fr_xarmlink1) * xarmlink1_v);
    xarmlink2_v(iit::rbd::AZ) += qd(XARMJOINT2);
    
    motionCrossProductMx<Scalar>(xarmlink2_v, vcross);
    
    xarmlink2_a = (xm->fr_xarmlink2_X_fr_xarmlink1) * xarmlink1_a + vcross.col(iit::rbd::AZ) * qd(XARMJOINT2);
    xarmlink2_a(iit::rbd::AZ) += qdd(XARMJOINT2);
    
    xarmlink2_f = xarmlink2_I * xarmlink2_a + vxIv(xarmlink2_v, xarmlink2_I) - fext[XARMLINK2];
    
    // First pass, link 'xarmlink3'
    xarmlink3_v = ((xm->fr_xarmlink3_X_fr_xarmlink2) * xarmlink2_v);
    xarmlink3_v(iit::rbd::AZ) += qd(XARMJOINT3);
    
    motionCrossProductMx<Scalar>(xarmlink3_v, vcross);
    
    xarmlink3_a = (xm->fr_xarmlink3_X_fr_xarmlink2) * xarmlink2_a + vcross.col(iit::rbd::AZ) * qd(XARMJOINT3);
    xarmlink3_a(iit::rbd::AZ) += qdd(XARMJOINT3);
    
    xarmlink3_f = xarmlink3_I * xarmlink3_a + vxIv(xarmlink3_v, xarmlink3_I) - fext[XARMLINK3];
    
    // First pass, link 'xarmlink4'
    xarmlink4_v = ((xm->fr_xarmlink4_X_fr_xarmlink3) * xarmlink3_v);
    xarmlink4_v(iit::rbd::AZ) += qd(XARMJOINT4);
    
    motionCrossProductMx<Scalar>(xarmlink4_v, vcross);
    
    xarmlink4_a = (xm->fr_xarmlink4_X_fr_xarmlink3) * xarmlink3_a + vcross.col(iit::rbd::AZ) * qd(XARMJOINT4);
    xarmlink4_a(iit::rbd::AZ) += qdd(XARMJOINT4);
    
    xarmlink4_f = xarmlink4_I * xarmlink4_a + vxIv(xarmlink4_v, xarmlink4_I) - fext[XARMLINK4];
    
    // First pass, link 'xarmlink5'
    xarmlink5_v = ((xm->fr_xarmlink5_X_fr_xarmlink4) * xarmlink4_v);
    xarmlink5_v(iit::rbd::AZ) += qd(XARMJOINT5);
    
    motionCrossProductMx<Scalar>(xarmlink5_v, vcross);
    
    xarmlink5_a = (xm->fr_xarmlink5_X_fr_xarmlink4) * xarmlink4_a + vcross.col(iit::rbd::AZ) * qd(XARMJOINT5);
    xarmlink5_a(iit::rbd::AZ) += qdd(XARMJOINT5);
    
    xarmlink5_f = xarmlink5_I * xarmlink5_a + vxIv(xarmlink5_v, xarmlink5_I) - fext[XARMLINK5];
    
    // First pass, link 'xarmlink6'
    xarmlink6_v = ((xm->fr_xarmlink6_X_fr_xarmlink5) * xarmlink5_v);
    xarmlink6_v(iit::rbd::AZ) += qd(XARMJOINT6);
    
    motionCrossProductMx<Scalar>(xarmlink6_v, vcross);
    
    xarmlink6_a = (xm->fr_xarmlink6_X_fr_xarmlink5) * xarmlink5_a + vcross.col(iit::rbd::AZ) * qd(XARMJOINT6);
    xarmlink6_a(iit::rbd::AZ) += qdd(XARMJOINT6);
    
    xarmlink6_f = xarmlink6_I * xarmlink6_a + vxIv(xarmlink6_v, xarmlink6_I) - fext[XARMLINK6];
    

    // The base
    base_link_footprint_f = base_link_footprint_I * base_link_footprint_a + vxIv(base_link_footprint_v, base_link_footprint_I) - fext[BASE_LINK_FOOTPRINT];

    secondPass_fullyActuated(jForces);

    baseWrench = base_link_footprint_f;
}


void armstone::rcg::InverseDynamics::secondPass_fullyActuated(JointState& jForces)
{
    // Link 'xarmlink6'
    jForces(XARMJOINT6) = xarmlink6_f(iit::rbd::AZ);
    xarmlink5_f += xm->fr_xarmlink6_X_fr_xarmlink5.transpose() * xarmlink6_f;
    // Link 'xarmlink5'
    jForces(XARMJOINT5) = xarmlink5_f(iit::rbd::AZ);
    xarmlink4_f += xm->fr_xarmlink5_X_fr_xarmlink4.transpose() * xarmlink5_f;
    // Link 'xarmlink4'
    jForces(XARMJOINT4) = xarmlink4_f(iit::rbd::AZ);
    xarmlink3_f += xm->fr_xarmlink4_X_fr_xarmlink3.transpose() * xarmlink4_f;
    // Link 'xarmlink3'
    jForces(XARMJOINT3) = xarmlink3_f(iit::rbd::AZ);
    xarmlink2_f += xm->fr_xarmlink3_X_fr_xarmlink2.transpose() * xarmlink3_f;
    // Link 'xarmlink2'
    jForces(XARMJOINT2) = xarmlink2_f(iit::rbd::AZ);
    xarmlink1_f += xm->fr_xarmlink2_X_fr_xarmlink1.transpose() * xarmlink2_f;
    // Link 'xarmlink1'
    jForces(XARMJOINT1) = xarmlink1_f(iit::rbd::AZ);
    base_link_footprint_f += xm->fr_xarmlink1_X_fr_base_link_footprint.transpose() * xarmlink1_f;
    // Link 'omniwheel_br'
    jForces(MOTOR_WHEEL_JOINT_BR) = omniwheel_br_f(iit::rbd::AZ);
    base_link_footprint_f += xm->fr_omniwheel_br_X_fr_base_link_footprint.transpose() * omniwheel_br_f;
    // Link 'omniwheel_bl'
    jForces(MOTOR_WHEEL_JOINT_BL) = omniwheel_bl_f(iit::rbd::AZ);
    base_link_footprint_f += xm->fr_omniwheel_bl_X_fr_base_link_footprint.transpose() * omniwheel_bl_f;
    // Link 'omniwheel_fr'
    jForces(MOTOR_WHEEL_JOINT_FR) = omniwheel_fr_f(iit::rbd::AZ);
    base_link_footprint_f += xm->fr_omniwheel_fr_X_fr_base_link_footprint.transpose() * omniwheel_fr_f;
    // Link 'omniwheel_fl'
    jForces(MOTOR_WHEEL_JOINT_FL) = omniwheel_fl_f(iit::rbd::AZ);
    base_link_footprint_f += xm->fr_omniwheel_fl_X_fr_base_link_footprint.transpose() * omniwheel_fl_f;
}
