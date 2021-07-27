#include <iit/rbd/utils.h>
#include "miscellaneous.h"

using namespace armstone::rcg;

Vector3 armstone::rcg::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const HomogeneousTransforms& ht)
{
    Vector3 tmpSum(Vector3::Zero());

    tmpSum += inertiaProps.getCOM_base_link_footprint() * inertiaProps.getMass_base_link_footprint();

    HomogeneousTransforms::MatrixType tmpX(HomogeneousTransforms::MatrixType::Identity());
    HomogeneousTransforms::MatrixType base_X_motor_wheel_joint_bl_chain;
    HomogeneousTransforms::MatrixType base_X_motor_wheel_joint_br_chain;
    HomogeneousTransforms::MatrixType base_X_motor_wheel_joint_fl_chain;
    HomogeneousTransforms::MatrixType base_X_motor_wheel_joint_fr_chain;
    HomogeneousTransforms::MatrixType base_X_xarmjoint1_chain;
    
    
    base_X_motor_wheel_joint_bl_chain = tmpX * ht.fr_base_link_footprint_X_fr_omniwheel_bl;
    tmpSum += inertiaProps.getMass_omniwheel_bl() *
            ( iit::rbd::Utils::transform(base_X_motor_wheel_joint_bl_chain, inertiaProps.getCOM_omniwheel_bl()));
    
    base_X_motor_wheel_joint_br_chain = tmpX * ht.fr_base_link_footprint_X_fr_omniwheel_br;
    tmpSum += inertiaProps.getMass_omniwheel_br() *
            ( iit::rbd::Utils::transform(base_X_motor_wheel_joint_br_chain, inertiaProps.getCOM_omniwheel_br()));
    
    base_X_motor_wheel_joint_fl_chain = tmpX * ht.fr_base_link_footprint_X_fr_omniwheel_fl;
    tmpSum += inertiaProps.getMass_omniwheel_fl() *
            ( iit::rbd::Utils::transform(base_X_motor_wheel_joint_fl_chain, inertiaProps.getCOM_omniwheel_fl()));
    
    base_X_motor_wheel_joint_fr_chain = tmpX * ht.fr_base_link_footprint_X_fr_omniwheel_fr;
    tmpSum += inertiaProps.getMass_omniwheel_fr() *
            ( iit::rbd::Utils::transform(base_X_motor_wheel_joint_fr_chain, inertiaProps.getCOM_omniwheel_fr()));
    
    base_X_xarmjoint1_chain = tmpX * ht.fr_base_link_footprint_X_fr_xarmlink1;
    tmpSum += inertiaProps.getMass_xarmlink1() *
            ( iit::rbd::Utils::transform(base_X_xarmjoint1_chain, inertiaProps.getCOM_xarmlink1()));
    
    base_X_xarmjoint1_chain = base_X_xarmjoint1_chain * ht.fr_xarmlink1_X_fr_xarmlink2;
    tmpSum += inertiaProps.getMass_xarmlink2() *
            ( iit::rbd::Utils::transform(base_X_xarmjoint1_chain, inertiaProps.getCOM_xarmlink2()));
    
    base_X_xarmjoint1_chain = base_X_xarmjoint1_chain * ht.fr_xarmlink2_X_fr_xarmlink3;
    tmpSum += inertiaProps.getMass_xarmlink3() *
            ( iit::rbd::Utils::transform(base_X_xarmjoint1_chain, inertiaProps.getCOM_xarmlink3()));
    
    base_X_xarmjoint1_chain = base_X_xarmjoint1_chain * ht.fr_xarmlink3_X_fr_xarmlink4;
    tmpSum += inertiaProps.getMass_xarmlink4() *
            ( iit::rbd::Utils::transform(base_X_xarmjoint1_chain, inertiaProps.getCOM_xarmlink4()));
    
    base_X_xarmjoint1_chain = base_X_xarmjoint1_chain * ht.fr_xarmlink4_X_fr_xarmlink5;
    tmpSum += inertiaProps.getMass_xarmlink5() *
            ( iit::rbd::Utils::transform(base_X_xarmjoint1_chain, inertiaProps.getCOM_xarmlink5()));
    
    base_X_xarmjoint1_chain = base_X_xarmjoint1_chain * ht.fr_xarmlink5_X_fr_xarmlink6;
    tmpSum += inertiaProps.getMass_xarmlink6() *
            ( iit::rbd::Utils::transform(base_X_xarmjoint1_chain, inertiaProps.getCOM_xarmlink6()));
    

    return tmpSum / inertiaProps.getTotalMass();
}

Vector3 armstone::rcg::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const JointState& q,
    HomogeneousTransforms& ht)
{
    // First updates the coordinate transforms that will be used by the routine
    ht.fr_base_link_footprint_X_fr_omniwheel_bl(q);
    ht.fr_base_link_footprint_X_fr_omniwheel_br(q);
    ht.fr_base_link_footprint_X_fr_omniwheel_fl(q);
    ht.fr_base_link_footprint_X_fr_omniwheel_fr(q);
    ht.fr_base_link_footprint_X_fr_xarmlink1(q);
    ht.fr_xarmlink1_X_fr_xarmlink2(q);
    ht.fr_xarmlink2_X_fr_xarmlink3(q);
    ht.fr_xarmlink3_X_fr_xarmlink4(q);
    ht.fr_xarmlink4_X_fr_xarmlink5(q);
    ht.fr_xarmlink5_X_fr_xarmlink6(q);

    // The actual calculus
    return getWholeBodyCOM(inertiaProps, ht);
}
