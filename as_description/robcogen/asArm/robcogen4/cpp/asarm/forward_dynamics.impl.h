
// Initialization of static-const data
template <typename TRAIT>
const typename iit::asArm::dyn::tpl::ForwardDynamics<TRAIT>::ExtForces
    iit::asArm::dyn::tpl::ForwardDynamics<TRAIT>::zeroExtForces(Force::Zero());

template <typename TRAIT>
iit::asArm::dyn::tpl::ForwardDynamics<TRAIT>::ForwardDynamics(iit::asArm::dyn::tpl::InertiaProperties<TRAIT>& inertia, MTransforms& transforms) :
    inertiaProps( & inertia ),
    motionTransforms( & transforms )
{
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

template <typename TRAIT>
void iit::asArm::dyn::tpl::ForwardDynamics<TRAIT>::fd(
    JointState& qdd,
    Acceleration& base_link_a,
    const Velocity& base_link_v,
    const Acceleration& g,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    
    base_link_AI = inertiaProps->getTensor_base_link();
    base_link_p = - fext[BASE_LINK];
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
    
    // + Link xarmlink1
    //  - The spatial velocity:
    xarmlink1_v = (motionTransforms-> fr_xarmlink1_X_fr_base_link) * base_link_v;
    xarmlink1_v(iit::rbd::AZ) += qd(XARMJOINT1);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(xarmlink1_v, vcross);
    xarmlink1_c = vcross.col(iit::rbd::AZ) * qd(XARMJOINT1);
    
    //  - The bias force term:
    xarmlink1_p += iit::rbd::vxIv(xarmlink1_v, xarmlink1_AI);
    
    // + Link xarmlink2
    //  - The spatial velocity:
    xarmlink2_v = (motionTransforms-> fr_xarmlink2_X_fr_xarmlink1) * xarmlink1_v;
    xarmlink2_v(iit::rbd::AZ) += qd(XARMJOINT2);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(xarmlink2_v, vcross);
    xarmlink2_c = vcross.col(iit::rbd::AZ) * qd(XARMJOINT2);
    
    //  - The bias force term:
    xarmlink2_p += iit::rbd::vxIv(xarmlink2_v, xarmlink2_AI);
    
    // + Link xarmlink3
    //  - The spatial velocity:
    xarmlink3_v = (motionTransforms-> fr_xarmlink3_X_fr_xarmlink2) * xarmlink2_v;
    xarmlink3_v(iit::rbd::AZ) += qd(XARMJOINT3);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(xarmlink3_v, vcross);
    xarmlink3_c = vcross.col(iit::rbd::AZ) * qd(XARMJOINT3);
    
    //  - The bias force term:
    xarmlink3_p += iit::rbd::vxIv(xarmlink3_v, xarmlink3_AI);
    
    // + Link xarmlink4
    //  - The spatial velocity:
    xarmlink4_v = (motionTransforms-> fr_xarmlink4_X_fr_xarmlink3) * xarmlink3_v;
    xarmlink4_v(iit::rbd::AZ) += qd(XARMJOINT4);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(xarmlink4_v, vcross);
    xarmlink4_c = vcross.col(iit::rbd::AZ) * qd(XARMJOINT4);
    
    //  - The bias force term:
    xarmlink4_p += iit::rbd::vxIv(xarmlink4_v, xarmlink4_AI);
    
    // + Link xarmlink5
    //  - The spatial velocity:
    xarmlink5_v = (motionTransforms-> fr_xarmlink5_X_fr_xarmlink4) * xarmlink4_v;
    xarmlink5_v(iit::rbd::AZ) += qd(XARMJOINT5);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(xarmlink5_v, vcross);
    xarmlink5_c = vcross.col(iit::rbd::AZ) * qd(XARMJOINT5);
    
    //  - The bias force term:
    xarmlink5_p += iit::rbd::vxIv(xarmlink5_v, xarmlink5_AI);
    
    // + Link xarmlink6
    //  - The spatial velocity:
    xarmlink6_v = (motionTransforms-> fr_xarmlink6_X_fr_xarmlink5) * xarmlink5_v;
    xarmlink6_v(iit::rbd::AZ) += qd(XARMJOINT6);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(xarmlink6_v, vcross);
    xarmlink6_c = vcross.col(iit::rbd::AZ) * qd(XARMJOINT6);
    
    //  - The bias force term:
    xarmlink6_p += iit::rbd::vxIv(xarmlink6_v, xarmlink6_AI);
    
    // + The floating base body
    base_link_p += iit::rbd::vxIv(base_link_v, base_link_AI);
    
    // ---------------------- SECOND PASS ---------------------- //
    Matrix66S IaB;
    Force pa;
    
    // + Link xarmlink6
    xarmlink6_u = tau(XARMJOINT6) - xarmlink6_p(iit::rbd::AZ);
    xarmlink6_U = xarmlink6_AI.col(iit::rbd::AZ);
    xarmlink6_D = xarmlink6_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(xarmlink6_AI, xarmlink6_U, xarmlink6_D, Ia_r);  // same as: Ia_r = xarmlink6_AI - xarmlink6_U/xarmlink6_D * xarmlink6_U.transpose();
    pa = xarmlink6_p + Ia_r * xarmlink6_c + xarmlink6_U * xarmlink6_u/xarmlink6_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_xarmlink6_X_fr_xarmlink5, IaB);
    xarmlink5_AI += IaB;
    xarmlink5_p += (motionTransforms-> fr_xarmlink6_X_fr_xarmlink5).transpose() * pa;
    
    // + Link xarmlink5
    xarmlink5_u = tau(XARMJOINT5) - xarmlink5_p(iit::rbd::AZ);
    xarmlink5_U = xarmlink5_AI.col(iit::rbd::AZ);
    xarmlink5_D = xarmlink5_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(xarmlink5_AI, xarmlink5_U, xarmlink5_D, Ia_r);  // same as: Ia_r = xarmlink5_AI - xarmlink5_U/xarmlink5_D * xarmlink5_U.transpose();
    pa = xarmlink5_p + Ia_r * xarmlink5_c + xarmlink5_U * xarmlink5_u/xarmlink5_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_xarmlink5_X_fr_xarmlink4, IaB);
    xarmlink4_AI += IaB;
    xarmlink4_p += (motionTransforms-> fr_xarmlink5_X_fr_xarmlink4).transpose() * pa;
    
    // + Link xarmlink4
    xarmlink4_u = tau(XARMJOINT4) - xarmlink4_p(iit::rbd::AZ);
    xarmlink4_U = xarmlink4_AI.col(iit::rbd::AZ);
    xarmlink4_D = xarmlink4_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(xarmlink4_AI, xarmlink4_U, xarmlink4_D, Ia_r);  // same as: Ia_r = xarmlink4_AI - xarmlink4_U/xarmlink4_D * xarmlink4_U.transpose();
    pa = xarmlink4_p + Ia_r * xarmlink4_c + xarmlink4_U * xarmlink4_u/xarmlink4_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_xarmlink4_X_fr_xarmlink3, IaB);
    xarmlink3_AI += IaB;
    xarmlink3_p += (motionTransforms-> fr_xarmlink4_X_fr_xarmlink3).transpose() * pa;
    
    // + Link xarmlink3
    xarmlink3_u = tau(XARMJOINT3) - xarmlink3_p(iit::rbd::AZ);
    xarmlink3_U = xarmlink3_AI.col(iit::rbd::AZ);
    xarmlink3_D = xarmlink3_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(xarmlink3_AI, xarmlink3_U, xarmlink3_D, Ia_r);  // same as: Ia_r = xarmlink3_AI - xarmlink3_U/xarmlink3_D * xarmlink3_U.transpose();
    pa = xarmlink3_p + Ia_r * xarmlink3_c + xarmlink3_U * xarmlink3_u/xarmlink3_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_xarmlink3_X_fr_xarmlink2, IaB);
    xarmlink2_AI += IaB;
    xarmlink2_p += (motionTransforms-> fr_xarmlink3_X_fr_xarmlink2).transpose() * pa;
    
    // + Link xarmlink2
    xarmlink2_u = tau(XARMJOINT2) - xarmlink2_p(iit::rbd::AZ);
    xarmlink2_U = xarmlink2_AI.col(iit::rbd::AZ);
    xarmlink2_D = xarmlink2_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(xarmlink2_AI, xarmlink2_U, xarmlink2_D, Ia_r);  // same as: Ia_r = xarmlink2_AI - xarmlink2_U/xarmlink2_D * xarmlink2_U.transpose();
    pa = xarmlink2_p + Ia_r * xarmlink2_c + xarmlink2_U * xarmlink2_u/xarmlink2_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_xarmlink2_X_fr_xarmlink1, IaB);
    xarmlink1_AI += IaB;
    xarmlink1_p += (motionTransforms-> fr_xarmlink2_X_fr_xarmlink1).transpose() * pa;
    
    // + Link xarmlink1
    xarmlink1_u = tau(XARMJOINT1) - xarmlink1_p(iit::rbd::AZ);
    xarmlink1_U = xarmlink1_AI.col(iit::rbd::AZ);
    xarmlink1_D = xarmlink1_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(xarmlink1_AI, xarmlink1_U, xarmlink1_D, Ia_r);  // same as: Ia_r = xarmlink1_AI - xarmlink1_U/xarmlink1_D * xarmlink1_U.transpose();
    pa = xarmlink1_p + Ia_r * xarmlink1_c + xarmlink1_U * xarmlink1_u/xarmlink1_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_xarmlink1_X_fr_base_link, IaB);
    base_link_AI += IaB;
    base_link_p += (motionTransforms-> fr_xarmlink1_X_fr_base_link).transpose() * pa;
    
    // + The acceleration of the floating base base_link, without gravity
    base_link_a = - TRAIT::solve(base_link_AI, base_link_p);  // base_link_a = - IA^-1 * base_link_p
    
    // ---------------------- THIRD PASS ---------------------- //
    xarmlink1_a = (motionTransforms-> fr_xarmlink1_X_fr_base_link) * base_link_a + xarmlink1_c;
    qdd(XARMJOINT1) = (xarmlink1_u - xarmlink1_U.dot(xarmlink1_a)) / xarmlink1_D;
    xarmlink1_a(iit::rbd::AZ) += qdd(XARMJOINT1);
    
    xarmlink2_a = (motionTransforms-> fr_xarmlink2_X_fr_xarmlink1) * xarmlink1_a + xarmlink2_c;
    qdd(XARMJOINT2) = (xarmlink2_u - xarmlink2_U.dot(xarmlink2_a)) / xarmlink2_D;
    xarmlink2_a(iit::rbd::AZ) += qdd(XARMJOINT2);
    
    xarmlink3_a = (motionTransforms-> fr_xarmlink3_X_fr_xarmlink2) * xarmlink2_a + xarmlink3_c;
    qdd(XARMJOINT3) = (xarmlink3_u - xarmlink3_U.dot(xarmlink3_a)) / xarmlink3_D;
    xarmlink3_a(iit::rbd::AZ) += qdd(XARMJOINT3);
    
    xarmlink4_a = (motionTransforms-> fr_xarmlink4_X_fr_xarmlink3) * xarmlink3_a + xarmlink4_c;
    qdd(XARMJOINT4) = (xarmlink4_u - xarmlink4_U.dot(xarmlink4_a)) / xarmlink4_D;
    xarmlink4_a(iit::rbd::AZ) += qdd(XARMJOINT4);
    
    xarmlink5_a = (motionTransforms-> fr_xarmlink5_X_fr_xarmlink4) * xarmlink4_a + xarmlink5_c;
    qdd(XARMJOINT5) = (xarmlink5_u - xarmlink5_U.dot(xarmlink5_a)) / xarmlink5_D;
    xarmlink5_a(iit::rbd::AZ) += qdd(XARMJOINT5);
    
    xarmlink6_a = (motionTransforms-> fr_xarmlink6_X_fr_xarmlink5) * xarmlink5_a + xarmlink6_c;
    qdd(XARMJOINT6) = (xarmlink6_u - xarmlink6_U.dot(xarmlink6_a)) / xarmlink6_D;
    xarmlink6_a(iit::rbd::AZ) += qdd(XARMJOINT6);
    
    
    // + Add gravity to the acceleration of the floating base
    base_link_a += g;
}
