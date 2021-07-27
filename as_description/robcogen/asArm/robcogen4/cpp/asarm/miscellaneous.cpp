#include <iit/rbd/utils.h>
#include "miscellaneous.h"

using namespace iit::asArm;
using namespace iit::asArm::dyn;

iit::rbd::Vector3d iit::asArm::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const HomogeneousTransforms& ht)
{
    iit::rbd::Vector3d tmpSum(iit::rbd::Vector3d::Zero());

    tmpSum += inertiaProps.getCOM_base_link() * inertiaProps.getMass_base_link();

    HomogeneousTransforms::MatrixType tmpX(HomogeneousTransforms::MatrixType::Identity());
    tmpX = tmpX * ht.fr_base_link_X_fr_xarmlink1;
    tmpSum += inertiaProps.getMass_xarmlink1() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_xarmlink1()));
    
    tmpX = tmpX * ht.fr_xarmlink1_X_fr_xarmlink2;
    tmpSum += inertiaProps.getMass_xarmlink2() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_xarmlink2()));
    
    tmpX = tmpX * ht.fr_xarmlink2_X_fr_xarmlink3;
    tmpSum += inertiaProps.getMass_xarmlink3() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_xarmlink3()));
    
    tmpX = tmpX * ht.fr_xarmlink3_X_fr_xarmlink4;
    tmpSum += inertiaProps.getMass_xarmlink4() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_xarmlink4()));
    
    tmpX = tmpX * ht.fr_xarmlink4_X_fr_xarmlink5;
    tmpSum += inertiaProps.getMass_xarmlink5() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_xarmlink5()));
    
    tmpX = tmpX * ht.fr_xarmlink5_X_fr_xarmlink6;
    tmpSum += inertiaProps.getMass_xarmlink6() *
            ( iit::rbd::Utils::transform(tmpX, inertiaProps.getCOM_xarmlink6()));
    

    return tmpSum / inertiaProps.getTotalMass();
}

iit::rbd::Vector3d iit::asArm::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const JointState& q,
    HomogeneousTransforms& ht)
{
    // First updates the coordinate transforms that will be used by the routine
    ht.fr_base_link_X_fr_xarmlink1(q);
    ht.fr_xarmlink1_X_fr_xarmlink2(q);
    ht.fr_xarmlink2_X_fr_xarmlink3(q);
    ht.fr_xarmlink3_X_fr_xarmlink4(q);
    ht.fr_xarmlink4_X_fr_xarmlink5(q);
    ht.fr_xarmlink5_X_fr_xarmlink6(q);

    // The actual calculus
    return getWholeBodyCOM(inertiaProps, ht);
}
