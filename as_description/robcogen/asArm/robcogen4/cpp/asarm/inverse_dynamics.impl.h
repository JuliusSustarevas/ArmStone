// Initialization of static-const data
template <typename TRAIT>
const typename iit::asArm::dyn::tpl::InverseDynamics<TRAIT>::ExtForces
iit::asArm::dyn::tpl::InverseDynamics<TRAIT>::zeroExtForces(Force::Zero());

template <typename TRAIT>
iit::asArm::dyn::tpl::InverseDynamics<TRAIT>::InverseDynamics(IProperties& inertia, MTransforms& transforms) :
    inertiaProps( & inertia ),
    xm( & transforms ),
    xarmlink1_I(inertiaProps->getTensor_xarmlink1() ),
    xarmlink2_I(inertiaProps->getTensor_xarmlink2() ),
    xarmlink3_I(inertiaProps->getTensor_xarmlink3() ),
    xarmlink4_I(inertiaProps->getTensor_xarmlink4() ),
    xarmlink5_I(inertiaProps->getTensor_xarmlink5() ),
    xarmlink6_I(inertiaProps->getTensor_xarmlink6() )
    ,
        base_link_I( inertiaProps->getTensor_base_link() ),
        xarmlink6_Ic(xarmlink6_I)
{
#ifndef EIGEN_NO_DEBUG
    std::cout << "Robot asArm, InverseDynamics<TRAIT>::InverseDynamics()" << std::endl;
    std::cout << "Compiled with Eigen debug active" << std::endl;
#endif
    xarmlink1_v.setZero();
    xarmlink2_v.setZero();
    xarmlink3_v.setZero();
    xarmlink4_v.setZero();
    xarmlink5_v.setZero();
    xarmlink6_v.setZero();

    vcross.setZero();
}

template <typename TRAIT>
void iit::asArm::dyn::tpl::InverseDynamics<TRAIT>::id(
    JointState& jForces, Acceleration& base_link_a,
    const Acceleration& g, const Velocity& base_link_v,
    const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    base_link_Ic = base_link_I;
    xarmlink1_Ic = xarmlink1_I;
    xarmlink2_Ic = xarmlink2_I;
    xarmlink3_Ic = xarmlink3_I;
    xarmlink4_Ic = xarmlink4_I;
    xarmlink5_Ic = xarmlink5_I;

    // First pass, link 'xarmlink1'
    xarmlink1_v = ((xm->fr_xarmlink1_X_fr_base_link) * base_link_v);
    xarmlink1_v(iit::rbd::AZ) += qd(XARMJOINT1);
    
    iit::rbd::motionCrossProductMx<Scalar>(xarmlink1_v, vcross);
    
    xarmlink1_a = (vcross.col(iit::rbd::AZ) * qd(XARMJOINT1));
    xarmlink1_a(iit::rbd::AZ) += qdd(XARMJOINT1);
    
    xarmlink1_f = xarmlink1_I * xarmlink1_a + iit::rbd::vxIv(xarmlink1_v, xarmlink1_I);
    
    // First pass, link 'xarmlink2'
    xarmlink2_v = ((xm->fr_xarmlink2_X_fr_xarmlink1) * xarmlink1_v);
    xarmlink2_v(iit::rbd::AZ) += qd(XARMJOINT2);
    
    iit::rbd::motionCrossProductMx<Scalar>(xarmlink2_v, vcross);
    
    xarmlink2_a = (xm->fr_xarmlink2_X_fr_xarmlink1) * xarmlink1_a + vcross.col(iit::rbd::AZ) * qd(XARMJOINT2);
    xarmlink2_a(iit::rbd::AZ) += qdd(XARMJOINT2);
    
    xarmlink2_f = xarmlink2_I * xarmlink2_a + iit::rbd::vxIv(xarmlink2_v, xarmlink2_I);
    
    // First pass, link 'xarmlink3'
    xarmlink3_v = ((xm->fr_xarmlink3_X_fr_xarmlink2) * xarmlink2_v);
    xarmlink3_v(iit::rbd::AZ) += qd(XARMJOINT3);
    
    iit::rbd::motionCrossProductMx<Scalar>(xarmlink3_v, vcross);
    
    xarmlink3_a = (xm->fr_xarmlink3_X_fr_xarmlink2) * xarmlink2_a + vcross.col(iit::rbd::AZ) * qd(XARMJOINT3);
    xarmlink3_a(iit::rbd::AZ) += qdd(XARMJOINT3);
    
    xarmlink3_f = xarmlink3_I * xarmlink3_a + iit::rbd::vxIv(xarmlink3_v, xarmlink3_I);
    
    // First pass, link 'xarmlink4'
    xarmlink4_v = ((xm->fr_xarmlink4_X_fr_xarmlink3) * xarmlink3_v);
    xarmlink4_v(iit::rbd::AZ) += qd(XARMJOINT4);
    
    iit::rbd::motionCrossProductMx<Scalar>(xarmlink4_v, vcross);
    
    xarmlink4_a = (xm->fr_xarmlink4_X_fr_xarmlink3) * xarmlink3_a + vcross.col(iit::rbd::AZ) * qd(XARMJOINT4);
    xarmlink4_a(iit::rbd::AZ) += qdd(XARMJOINT4);
    
    xarmlink4_f = xarmlink4_I * xarmlink4_a + iit::rbd::vxIv(xarmlink4_v, xarmlink4_I);
    
    // First pass, link 'xarmlink5'
    xarmlink5_v = ((xm->fr_xarmlink5_X_fr_xarmlink4) * xarmlink4_v);
    xarmlink5_v(iit::rbd::AZ) += qd(XARMJOINT5);
    
    iit::rbd::motionCrossProductMx<Scalar>(xarmlink5_v, vcross);
    
    xarmlink5_a = (xm->fr_xarmlink5_X_fr_xarmlink4) * xarmlink4_a + vcross.col(iit::rbd::AZ) * qd(XARMJOINT5);
    xarmlink5_a(iit::rbd::AZ) += qdd(XARMJOINT5);
    
    xarmlink5_f = xarmlink5_I * xarmlink5_a + iit::rbd::vxIv(xarmlink5_v, xarmlink5_I);
    
    // First pass, link 'xarmlink6'
    xarmlink6_v = ((xm->fr_xarmlink6_X_fr_xarmlink5) * xarmlink5_v);
    xarmlink6_v(iit::rbd::AZ) += qd(XARMJOINT6);
    
    iit::rbd::motionCrossProductMx<Scalar>(xarmlink6_v, vcross);
    
    xarmlink6_a = (xm->fr_xarmlink6_X_fr_xarmlink5) * xarmlink5_a + vcross.col(iit::rbd::AZ) * qd(XARMJOINT6);
    xarmlink6_a(iit::rbd::AZ) += qdd(XARMJOINT6);
    
    xarmlink6_f = xarmlink6_I * xarmlink6_a + iit::rbd::vxIv(xarmlink6_v, xarmlink6_I);
    
    // The force exerted on the floating base by the links
    base_link_f = iit::rbd::vxIv(base_link_v, base_link_I);
    

    // Add the external forces:
    base_link_f -= fext[BASE_LINK];
    xarmlink1_f -= fext[XARMLINK1];
    xarmlink2_f -= fext[XARMLINK2];
    xarmlink3_f -= fext[XARMLINK3];
    xarmlink4_f -= fext[XARMLINK4];
    xarmlink5_f -= fext[XARMLINK5];
    xarmlink6_f -= fext[XARMLINK6];

    xarmlink5_Ic = xarmlink5_Ic + (xm->fr_xarmlink6_X_fr_xarmlink5).transpose() * xarmlink6_Ic * (xm->fr_xarmlink6_X_fr_xarmlink5);
    xarmlink5_f = xarmlink5_f + (xm->fr_xarmlink6_X_fr_xarmlink5).transpose() * xarmlink6_f;
    
    xarmlink4_Ic = xarmlink4_Ic + (xm->fr_xarmlink5_X_fr_xarmlink4).transpose() * xarmlink5_Ic * (xm->fr_xarmlink5_X_fr_xarmlink4);
    xarmlink4_f = xarmlink4_f + (xm->fr_xarmlink5_X_fr_xarmlink4).transpose() * xarmlink5_f;
    
    xarmlink3_Ic = xarmlink3_Ic + (xm->fr_xarmlink4_X_fr_xarmlink3).transpose() * xarmlink4_Ic * (xm->fr_xarmlink4_X_fr_xarmlink3);
    xarmlink3_f = xarmlink3_f + (xm->fr_xarmlink4_X_fr_xarmlink3).transpose() * xarmlink4_f;
    
    xarmlink2_Ic = xarmlink2_Ic + (xm->fr_xarmlink3_X_fr_xarmlink2).transpose() * xarmlink3_Ic * (xm->fr_xarmlink3_X_fr_xarmlink2);
    xarmlink2_f = xarmlink2_f + (xm->fr_xarmlink3_X_fr_xarmlink2).transpose() * xarmlink3_f;
    
    xarmlink1_Ic = xarmlink1_Ic + (xm->fr_xarmlink2_X_fr_xarmlink1).transpose() * xarmlink2_Ic * (xm->fr_xarmlink2_X_fr_xarmlink1);
    xarmlink1_f = xarmlink1_f + (xm->fr_xarmlink2_X_fr_xarmlink1).transpose() * xarmlink2_f;
    
    base_link_Ic = base_link_Ic + (xm->fr_xarmlink1_X_fr_base_link).transpose() * xarmlink1_Ic * (xm->fr_xarmlink1_X_fr_base_link);
    base_link_f = base_link_f + (xm->fr_xarmlink1_X_fr_base_link).transpose() * xarmlink1_f;
    

    // The base acceleration due to the force due to the movement of the links
    base_link_a = - base_link_Ic.inverse() * base_link_f;
    
    xarmlink1_a = xm->fr_xarmlink1_X_fr_base_link * base_link_a;
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
    

    base_link_a += g;
}

template <typename TRAIT>
void iit::asArm::dyn::tpl::InverseDynamics<TRAIT>::G_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Acceleration& g)
{
    const Acceleration& base_link_a = -g;

    // Link 'xarmlink1'
    xarmlink1_a = (xm->fr_xarmlink1_X_fr_base_link) * base_link_a;
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

    base_link_f = base_link_I * base_link_a;

    secondPass_fullyActuated(jForces);

    baseWrench = base_link_f;
}

template <typename TRAIT>
void iit::asArm::dyn::tpl::InverseDynamics<TRAIT>::C_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Velocity& base_link_v, const JointState& qd)
{
    // Link 'xarmlink1'
    xarmlink1_v = ((xm->fr_xarmlink1_X_fr_base_link) * base_link_v);
    xarmlink1_v(iit::rbd::AZ) += qd(XARMJOINT1);
    iit::rbd::motionCrossProductMx<Scalar>(xarmlink1_v, vcross);
    xarmlink1_a = (vcross.col(iit::rbd::AZ) * qd(XARMJOINT1));
    xarmlink1_f = xarmlink1_I * xarmlink1_a + iit::rbd::vxIv(xarmlink1_v, xarmlink1_I);
    
    // Link 'xarmlink2'
    xarmlink2_v = ((xm->fr_xarmlink2_X_fr_xarmlink1) * xarmlink1_v);
    xarmlink2_v(iit::rbd::AZ) += qd(XARMJOINT2);
    iit::rbd::motionCrossProductMx<Scalar>(xarmlink2_v, vcross);
    xarmlink2_a = (xm->fr_xarmlink2_X_fr_xarmlink1) * xarmlink1_a + vcross.col(iit::rbd::AZ) * qd(XARMJOINT2);
    xarmlink2_f = xarmlink2_I * xarmlink2_a + iit::rbd::vxIv(xarmlink2_v, xarmlink2_I);
    
    // Link 'xarmlink3'
    xarmlink3_v = ((xm->fr_xarmlink3_X_fr_xarmlink2) * xarmlink2_v);
    xarmlink3_v(iit::rbd::AZ) += qd(XARMJOINT3);
    iit::rbd::motionCrossProductMx<Scalar>(xarmlink3_v, vcross);
    xarmlink3_a = (xm->fr_xarmlink3_X_fr_xarmlink2) * xarmlink2_a + vcross.col(iit::rbd::AZ) * qd(XARMJOINT3);
    xarmlink3_f = xarmlink3_I * xarmlink3_a + iit::rbd::vxIv(xarmlink3_v, xarmlink3_I);
    
    // Link 'xarmlink4'
    xarmlink4_v = ((xm->fr_xarmlink4_X_fr_xarmlink3) * xarmlink3_v);
    xarmlink4_v(iit::rbd::AZ) += qd(XARMJOINT4);
    iit::rbd::motionCrossProductMx<Scalar>(xarmlink4_v, vcross);
    xarmlink4_a = (xm->fr_xarmlink4_X_fr_xarmlink3) * xarmlink3_a + vcross.col(iit::rbd::AZ) * qd(XARMJOINT4);
    xarmlink4_f = xarmlink4_I * xarmlink4_a + iit::rbd::vxIv(xarmlink4_v, xarmlink4_I);
    
    // Link 'xarmlink5'
    xarmlink5_v = ((xm->fr_xarmlink5_X_fr_xarmlink4) * xarmlink4_v);
    xarmlink5_v(iit::rbd::AZ) += qd(XARMJOINT5);
    iit::rbd::motionCrossProductMx<Scalar>(xarmlink5_v, vcross);
    xarmlink5_a = (xm->fr_xarmlink5_X_fr_xarmlink4) * xarmlink4_a + vcross.col(iit::rbd::AZ) * qd(XARMJOINT5);
    xarmlink5_f = xarmlink5_I * xarmlink5_a + iit::rbd::vxIv(xarmlink5_v, xarmlink5_I);
    
    // Link 'xarmlink6'
    xarmlink6_v = ((xm->fr_xarmlink6_X_fr_xarmlink5) * xarmlink5_v);
    xarmlink6_v(iit::rbd::AZ) += qd(XARMJOINT6);
    iit::rbd::motionCrossProductMx<Scalar>(xarmlink6_v, vcross);
    xarmlink6_a = (xm->fr_xarmlink6_X_fr_xarmlink5) * xarmlink5_a + vcross.col(iit::rbd::AZ) * qd(XARMJOINT6);
    xarmlink6_f = xarmlink6_I * xarmlink6_a + iit::rbd::vxIv(xarmlink6_v, xarmlink6_I);
    

    base_link_f = iit::rbd::vxIv(base_link_v, base_link_I);

    secondPass_fullyActuated(jForces);

    baseWrench = base_link_f;
}

template <typename TRAIT>
void iit::asArm::dyn::tpl::InverseDynamics<TRAIT>::id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& base_link_v, const Acceleration& baseAccel,
        const JointState& qd, const JointState& qdd, const ExtForces& fext)
{
    Acceleration base_link_a = baseAccel -g;

    // First pass, link 'xarmlink1'
    xarmlink1_v = ((xm->fr_xarmlink1_X_fr_base_link) * base_link_v);
    xarmlink1_v(iit::rbd::AZ) += qd(XARMJOINT1);
    
    iit::rbd::motionCrossProductMx<Scalar>(xarmlink1_v, vcross);
    
    xarmlink1_a = (xm->fr_xarmlink1_X_fr_base_link) * base_link_a + vcross.col(iit::rbd::AZ) * qd(XARMJOINT1);
    xarmlink1_a(iit::rbd::AZ) += qdd(XARMJOINT1);
    
    xarmlink1_f = xarmlink1_I * xarmlink1_a + iit::rbd::vxIv(xarmlink1_v, xarmlink1_I) - fext[XARMLINK1];
    
    // First pass, link 'xarmlink2'
    xarmlink2_v = ((xm->fr_xarmlink2_X_fr_xarmlink1) * xarmlink1_v);
    xarmlink2_v(iit::rbd::AZ) += qd(XARMJOINT2);
    
    iit::rbd::motionCrossProductMx<Scalar>(xarmlink2_v, vcross);
    
    xarmlink2_a = (xm->fr_xarmlink2_X_fr_xarmlink1) * xarmlink1_a + vcross.col(iit::rbd::AZ) * qd(XARMJOINT2);
    xarmlink2_a(iit::rbd::AZ) += qdd(XARMJOINT2);
    
    xarmlink2_f = xarmlink2_I * xarmlink2_a + iit::rbd::vxIv(xarmlink2_v, xarmlink2_I) - fext[XARMLINK2];
    
    // First pass, link 'xarmlink3'
    xarmlink3_v = ((xm->fr_xarmlink3_X_fr_xarmlink2) * xarmlink2_v);
    xarmlink3_v(iit::rbd::AZ) += qd(XARMJOINT3);
    
    iit::rbd::motionCrossProductMx<Scalar>(xarmlink3_v, vcross);
    
    xarmlink3_a = (xm->fr_xarmlink3_X_fr_xarmlink2) * xarmlink2_a + vcross.col(iit::rbd::AZ) * qd(XARMJOINT3);
    xarmlink3_a(iit::rbd::AZ) += qdd(XARMJOINT3);
    
    xarmlink3_f = xarmlink3_I * xarmlink3_a + iit::rbd::vxIv(xarmlink3_v, xarmlink3_I) - fext[XARMLINK3];
    
    // First pass, link 'xarmlink4'
    xarmlink4_v = ((xm->fr_xarmlink4_X_fr_xarmlink3) * xarmlink3_v);
    xarmlink4_v(iit::rbd::AZ) += qd(XARMJOINT4);
    
    iit::rbd::motionCrossProductMx<Scalar>(xarmlink4_v, vcross);
    
    xarmlink4_a = (xm->fr_xarmlink4_X_fr_xarmlink3) * xarmlink3_a + vcross.col(iit::rbd::AZ) * qd(XARMJOINT4);
    xarmlink4_a(iit::rbd::AZ) += qdd(XARMJOINT4);
    
    xarmlink4_f = xarmlink4_I * xarmlink4_a + iit::rbd::vxIv(xarmlink4_v, xarmlink4_I) - fext[XARMLINK4];
    
    // First pass, link 'xarmlink5'
    xarmlink5_v = ((xm->fr_xarmlink5_X_fr_xarmlink4) * xarmlink4_v);
    xarmlink5_v(iit::rbd::AZ) += qd(XARMJOINT5);
    
    iit::rbd::motionCrossProductMx<Scalar>(xarmlink5_v, vcross);
    
    xarmlink5_a = (xm->fr_xarmlink5_X_fr_xarmlink4) * xarmlink4_a + vcross.col(iit::rbd::AZ) * qd(XARMJOINT5);
    xarmlink5_a(iit::rbd::AZ) += qdd(XARMJOINT5);
    
    xarmlink5_f = xarmlink5_I * xarmlink5_a + iit::rbd::vxIv(xarmlink5_v, xarmlink5_I) - fext[XARMLINK5];
    
    // First pass, link 'xarmlink6'
    xarmlink6_v = ((xm->fr_xarmlink6_X_fr_xarmlink5) * xarmlink5_v);
    xarmlink6_v(iit::rbd::AZ) += qd(XARMJOINT6);
    
    iit::rbd::motionCrossProductMx<Scalar>(xarmlink6_v, vcross);
    
    xarmlink6_a = (xm->fr_xarmlink6_X_fr_xarmlink5) * xarmlink5_a + vcross.col(iit::rbd::AZ) * qd(XARMJOINT6);
    xarmlink6_a(iit::rbd::AZ) += qdd(XARMJOINT6);
    
    xarmlink6_f = xarmlink6_I * xarmlink6_a + iit::rbd::vxIv(xarmlink6_v, xarmlink6_I) - fext[XARMLINK6];
    

    // The base
    base_link_f = base_link_I * base_link_a + iit::rbd::vxIv(base_link_v, base_link_I) - fext[BASE_LINK];

    secondPass_fullyActuated(jForces);

    baseWrench = base_link_f;
}

template <typename TRAIT>
void iit::asArm::dyn::tpl::InverseDynamics<TRAIT>::secondPass_fullyActuated(JointState& jForces)
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
    base_link_f += xm->fr_xarmlink1_X_fr_base_link.transpose() * xarmlink1_f;
}

