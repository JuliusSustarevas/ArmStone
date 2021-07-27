#include "transforms.h"
#include "jsim.h"

#include <iit/rbd/robcogen_commons.h>

using namespace iit::rbd;

//Implementation of default constructor
armstone::rcg::JSIM::JSIM(InertiaProperties& inertiaProperties, ForceTransforms& forceTransforms) :
    linkInertias(inertiaProperties),
    frcTransf( &forceTransforms ),
    omniwheel_fl_Ic(linkInertias.getTensor_omniwheel_fl()),
    omniwheel_fr_Ic(linkInertias.getTensor_omniwheel_fr()),
    omniwheel_bl_Ic(linkInertias.getTensor_omniwheel_bl()),
    omniwheel_br_Ic(linkInertias.getTensor_omniwheel_br()),
    xarmlink6_Ic(linkInertias.getTensor_xarmlink6())
{
    //Initialize the matrix itself
    this->setZero();
}

#define DATA operator()
#define Fcol(j) (block<6,1>(0,(j)+6))
#define F(i,j) DATA((i),(j)+6)
const armstone::rcg::JSIM& armstone::rcg::JSIM::update(const JointState& state) {

    // Precomputes only once the coordinate transforms:
    frcTransf -> fr_xarmlink5_X_fr_xarmlink6(state);
    frcTransf -> fr_xarmlink4_X_fr_xarmlink5(state);
    frcTransf -> fr_xarmlink3_X_fr_xarmlink4(state);
    frcTransf -> fr_xarmlink2_X_fr_xarmlink3(state);
    frcTransf -> fr_xarmlink1_X_fr_xarmlink2(state);
    frcTransf -> fr_base_link_footprint_X_fr_xarmlink1(state);
    frcTransf -> fr_base_link_footprint_X_fr_omniwheel_br(state);
    frcTransf -> fr_base_link_footprint_X_fr_omniwheel_bl(state);
    frcTransf -> fr_base_link_footprint_X_fr_omniwheel_fr(state);
    frcTransf -> fr_base_link_footprint_X_fr_omniwheel_fl(state);

    // Initializes the composite inertia tensors
    base_link_footprint_Ic = linkInertias.getTensor_base_link_footprint();
    xarmlink1_Ic = linkInertias.getTensor_xarmlink1();
    xarmlink2_Ic = linkInertias.getTensor_xarmlink2();
    xarmlink3_Ic = linkInertias.getTensor_xarmlink3();
    xarmlink4_Ic = linkInertias.getTensor_xarmlink4();
    xarmlink5_Ic = linkInertias.getTensor_xarmlink5();

    // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration

    // Link xarmlink6:
    iit::rbd::transformInertia<Scalar>(xarmlink6_Ic, frcTransf -> fr_xarmlink5_X_fr_xarmlink6, Ic_spare);
    xarmlink5_Ic += Ic_spare;

    Fcol(XARMJOINT6) = xarmlink6_Ic.col(AZ);
    DATA(XARMJOINT6+6, XARMJOINT6+6) = Fcol(XARMJOINT6)(AZ);

    Fcol(XARMJOINT6) = frcTransf -> fr_xarmlink5_X_fr_xarmlink6 * Fcol(XARMJOINT6);
    DATA(XARMJOINT6+6, XARMJOINT5+6) = F(AZ,XARMJOINT6);
    DATA(XARMJOINT5+6, XARMJOINT6+6) = DATA(XARMJOINT6+6, XARMJOINT5+6);
    Fcol(XARMJOINT6) = frcTransf -> fr_xarmlink4_X_fr_xarmlink5 * Fcol(XARMJOINT6);
    DATA(XARMJOINT6+6, XARMJOINT4+6) = F(AZ,XARMJOINT6);
    DATA(XARMJOINT4+6, XARMJOINT6+6) = DATA(XARMJOINT6+6, XARMJOINT4+6);
    Fcol(XARMJOINT6) = frcTransf -> fr_xarmlink3_X_fr_xarmlink4 * Fcol(XARMJOINT6);
    DATA(XARMJOINT6+6, XARMJOINT3+6) = F(AZ,XARMJOINT6);
    DATA(XARMJOINT3+6, XARMJOINT6+6) = DATA(XARMJOINT6+6, XARMJOINT3+6);
    Fcol(XARMJOINT6) = frcTransf -> fr_xarmlink2_X_fr_xarmlink3 * Fcol(XARMJOINT6);
    DATA(XARMJOINT6+6, XARMJOINT2+6) = F(AZ,XARMJOINT6);
    DATA(XARMJOINT2+6, XARMJOINT6+6) = DATA(XARMJOINT6+6, XARMJOINT2+6);
    Fcol(XARMJOINT6) = frcTransf -> fr_xarmlink1_X_fr_xarmlink2 * Fcol(XARMJOINT6);
    DATA(XARMJOINT6+6, XARMJOINT1+6) = F(AZ,XARMJOINT6);
    DATA(XARMJOINT1+6, XARMJOINT6+6) = DATA(XARMJOINT6+6, XARMJOINT1+6);
    Fcol(XARMJOINT6) = frcTransf -> fr_base_link_footprint_X_fr_xarmlink1 * Fcol(XARMJOINT6);

    // Link xarmlink5:
    iit::rbd::transformInertia<Scalar>(xarmlink5_Ic, frcTransf -> fr_xarmlink4_X_fr_xarmlink5, Ic_spare);
    xarmlink4_Ic += Ic_spare;

    Fcol(XARMJOINT5) = xarmlink5_Ic.col(AZ);
    DATA(XARMJOINT5+6, XARMJOINT5+6) = Fcol(XARMJOINT5)(AZ);

    Fcol(XARMJOINT5) = frcTransf -> fr_xarmlink4_X_fr_xarmlink5 * Fcol(XARMJOINT5);
    DATA(XARMJOINT5+6, XARMJOINT4+6) = F(AZ,XARMJOINT5);
    DATA(XARMJOINT4+6, XARMJOINT5+6) = DATA(XARMJOINT5+6, XARMJOINT4+6);
    Fcol(XARMJOINT5) = frcTransf -> fr_xarmlink3_X_fr_xarmlink4 * Fcol(XARMJOINT5);
    DATA(XARMJOINT5+6, XARMJOINT3+6) = F(AZ,XARMJOINT5);
    DATA(XARMJOINT3+6, XARMJOINT5+6) = DATA(XARMJOINT5+6, XARMJOINT3+6);
    Fcol(XARMJOINT5) = frcTransf -> fr_xarmlink2_X_fr_xarmlink3 * Fcol(XARMJOINT5);
    DATA(XARMJOINT5+6, XARMJOINT2+6) = F(AZ,XARMJOINT5);
    DATA(XARMJOINT2+6, XARMJOINT5+6) = DATA(XARMJOINT5+6, XARMJOINT2+6);
    Fcol(XARMJOINT5) = frcTransf -> fr_xarmlink1_X_fr_xarmlink2 * Fcol(XARMJOINT5);
    DATA(XARMJOINT5+6, XARMJOINT1+6) = F(AZ,XARMJOINT5);
    DATA(XARMJOINT1+6, XARMJOINT5+6) = DATA(XARMJOINT5+6, XARMJOINT1+6);
    Fcol(XARMJOINT5) = frcTransf -> fr_base_link_footprint_X_fr_xarmlink1 * Fcol(XARMJOINT5);

    // Link xarmlink4:
    iit::rbd::transformInertia<Scalar>(xarmlink4_Ic, frcTransf -> fr_xarmlink3_X_fr_xarmlink4, Ic_spare);
    xarmlink3_Ic += Ic_spare;

    Fcol(XARMJOINT4) = xarmlink4_Ic.col(AZ);
    DATA(XARMJOINT4+6, XARMJOINT4+6) = Fcol(XARMJOINT4)(AZ);

    Fcol(XARMJOINT4) = frcTransf -> fr_xarmlink3_X_fr_xarmlink4 * Fcol(XARMJOINT4);
    DATA(XARMJOINT4+6, XARMJOINT3+6) = F(AZ,XARMJOINT4);
    DATA(XARMJOINT3+6, XARMJOINT4+6) = DATA(XARMJOINT4+6, XARMJOINT3+6);
    Fcol(XARMJOINT4) = frcTransf -> fr_xarmlink2_X_fr_xarmlink3 * Fcol(XARMJOINT4);
    DATA(XARMJOINT4+6, XARMJOINT2+6) = F(AZ,XARMJOINT4);
    DATA(XARMJOINT2+6, XARMJOINT4+6) = DATA(XARMJOINT4+6, XARMJOINT2+6);
    Fcol(XARMJOINT4) = frcTransf -> fr_xarmlink1_X_fr_xarmlink2 * Fcol(XARMJOINT4);
    DATA(XARMJOINT4+6, XARMJOINT1+6) = F(AZ,XARMJOINT4);
    DATA(XARMJOINT1+6, XARMJOINT4+6) = DATA(XARMJOINT4+6, XARMJOINT1+6);
    Fcol(XARMJOINT4) = frcTransf -> fr_base_link_footprint_X_fr_xarmlink1 * Fcol(XARMJOINT4);

    // Link xarmlink3:
    iit::rbd::transformInertia<Scalar>(xarmlink3_Ic, frcTransf -> fr_xarmlink2_X_fr_xarmlink3, Ic_spare);
    xarmlink2_Ic += Ic_spare;

    Fcol(XARMJOINT3) = xarmlink3_Ic.col(AZ);
    DATA(XARMJOINT3+6, XARMJOINT3+6) = Fcol(XARMJOINT3)(AZ);

    Fcol(XARMJOINT3) = frcTransf -> fr_xarmlink2_X_fr_xarmlink3 * Fcol(XARMJOINT3);
    DATA(XARMJOINT3+6, XARMJOINT2+6) = F(AZ,XARMJOINT3);
    DATA(XARMJOINT2+6, XARMJOINT3+6) = DATA(XARMJOINT3+6, XARMJOINT2+6);
    Fcol(XARMJOINT3) = frcTransf -> fr_xarmlink1_X_fr_xarmlink2 * Fcol(XARMJOINT3);
    DATA(XARMJOINT3+6, XARMJOINT1+6) = F(AZ,XARMJOINT3);
    DATA(XARMJOINT1+6, XARMJOINT3+6) = DATA(XARMJOINT3+6, XARMJOINT1+6);
    Fcol(XARMJOINT3) = frcTransf -> fr_base_link_footprint_X_fr_xarmlink1 * Fcol(XARMJOINT3);

    // Link xarmlink2:
    iit::rbd::transformInertia<Scalar>(xarmlink2_Ic, frcTransf -> fr_xarmlink1_X_fr_xarmlink2, Ic_spare);
    xarmlink1_Ic += Ic_spare;

    Fcol(XARMJOINT2) = xarmlink2_Ic.col(AZ);
    DATA(XARMJOINT2+6, XARMJOINT2+6) = Fcol(XARMJOINT2)(AZ);

    Fcol(XARMJOINT2) = frcTransf -> fr_xarmlink1_X_fr_xarmlink2 * Fcol(XARMJOINT2);
    DATA(XARMJOINT2+6, XARMJOINT1+6) = F(AZ,XARMJOINT2);
    DATA(XARMJOINT1+6, XARMJOINT2+6) = DATA(XARMJOINT2+6, XARMJOINT1+6);
    Fcol(XARMJOINT2) = frcTransf -> fr_base_link_footprint_X_fr_xarmlink1 * Fcol(XARMJOINT2);

    // Link xarmlink1:
    iit::rbd::transformInertia<Scalar>(xarmlink1_Ic, frcTransf -> fr_base_link_footprint_X_fr_xarmlink1, Ic_spare);
    base_link_footprint_Ic += Ic_spare;

    Fcol(XARMJOINT1) = xarmlink1_Ic.col(AZ);
    DATA(XARMJOINT1+6, XARMJOINT1+6) = Fcol(XARMJOINT1)(AZ);

    Fcol(XARMJOINT1) = frcTransf -> fr_base_link_footprint_X_fr_xarmlink1 * Fcol(XARMJOINT1);

    // Link omniwheel_br:
    iit::rbd::transformInertia<Scalar>(omniwheel_br_Ic, frcTransf -> fr_base_link_footprint_X_fr_omniwheel_br, Ic_spare);
    base_link_footprint_Ic += Ic_spare;

    Fcol(MOTOR_WHEEL_JOINT_BR) = omniwheel_br_Ic.col(AZ);
    DATA(MOTOR_WHEEL_JOINT_BR+6, MOTOR_WHEEL_JOINT_BR+6) = Fcol(MOTOR_WHEEL_JOINT_BR)(AZ);

    Fcol(MOTOR_WHEEL_JOINT_BR) = frcTransf -> fr_base_link_footprint_X_fr_omniwheel_br * Fcol(MOTOR_WHEEL_JOINT_BR);

    // Link omniwheel_bl:
    iit::rbd::transformInertia<Scalar>(omniwheel_bl_Ic, frcTransf -> fr_base_link_footprint_X_fr_omniwheel_bl, Ic_spare);
    base_link_footprint_Ic += Ic_spare;

    Fcol(MOTOR_WHEEL_JOINT_BL) = omniwheel_bl_Ic.col(AZ);
    DATA(MOTOR_WHEEL_JOINT_BL+6, MOTOR_WHEEL_JOINT_BL+6) = Fcol(MOTOR_WHEEL_JOINT_BL)(AZ);

    Fcol(MOTOR_WHEEL_JOINT_BL) = frcTransf -> fr_base_link_footprint_X_fr_omniwheel_bl * Fcol(MOTOR_WHEEL_JOINT_BL);

    // Link omniwheel_fr:
    iit::rbd::transformInertia<Scalar>(omniwheel_fr_Ic, frcTransf -> fr_base_link_footprint_X_fr_omniwheel_fr, Ic_spare);
    base_link_footprint_Ic += Ic_spare;

    Fcol(MOTOR_WHEEL_JOINT_FR) = omniwheel_fr_Ic.col(AZ);
    DATA(MOTOR_WHEEL_JOINT_FR+6, MOTOR_WHEEL_JOINT_FR+6) = Fcol(MOTOR_WHEEL_JOINT_FR)(AZ);

    Fcol(MOTOR_WHEEL_JOINT_FR) = frcTransf -> fr_base_link_footprint_X_fr_omniwheel_fr * Fcol(MOTOR_WHEEL_JOINT_FR);

    // Link omniwheel_fl:
    iit::rbd::transformInertia<Scalar>(omniwheel_fl_Ic, frcTransf -> fr_base_link_footprint_X_fr_omniwheel_fl, Ic_spare);
    base_link_footprint_Ic += Ic_spare;

    Fcol(MOTOR_WHEEL_JOINT_FL) = omniwheel_fl_Ic.col(AZ);
    DATA(MOTOR_WHEEL_JOINT_FL+6, MOTOR_WHEEL_JOINT_FL+6) = Fcol(MOTOR_WHEEL_JOINT_FL)(AZ);

    Fcol(MOTOR_WHEEL_JOINT_FL) = frcTransf -> fr_base_link_footprint_X_fr_omniwheel_fl * Fcol(MOTOR_WHEEL_JOINT_FL);

    // Copies the upper-right block into the lower-left block, after transposing
    block<10, 6>(6,0) = (block<6, 10>(0,6)).transpose();
    // The composite-inertia of the whole robot is the upper-left quadrant of the JSIM
    block<6,6>(0,0) = base_link_footprint_Ic;
    return *this;
}

#undef DATA
#undef F

void armstone::rcg::JSIM::computeL() {
    L = this -> triangularView<Eigen::Lower>();
    // Joint xarmjoint6, index 9 :
    L(9, 9) = ScalarTraits::sqrt(L(9, 9));
    L(9, 8) = L(9, 8) / L(9, 9);
    L(9, 7) = L(9, 7) / L(9, 9);
    L(9, 6) = L(9, 6) / L(9, 9);
    L(9, 5) = L(9, 5) / L(9, 9);
    L(9, 4) = L(9, 4) / L(9, 9);
    L(8, 8) = L(8, 8) - L(9, 8) * L(9, 8);
    L(8, 7) = L(8, 7) - L(9, 8) * L(9, 7);
    L(8, 6) = L(8, 6) - L(9, 8) * L(9, 6);
    L(8, 5) = L(8, 5) - L(9, 8) * L(9, 5);
    L(8, 4) = L(8, 4) - L(9, 8) * L(9, 4);
    L(7, 7) = L(7, 7) - L(9, 7) * L(9, 7);
    L(7, 6) = L(7, 6) - L(9, 7) * L(9, 6);
    L(7, 5) = L(7, 5) - L(9, 7) * L(9, 5);
    L(7, 4) = L(7, 4) - L(9, 7) * L(9, 4);
    L(6, 6) = L(6, 6) - L(9, 6) * L(9, 6);
    L(6, 5) = L(6, 5) - L(9, 6) * L(9, 5);
    L(6, 4) = L(6, 4) - L(9, 6) * L(9, 4);
    L(5, 5) = L(5, 5) - L(9, 5) * L(9, 5);
    L(5, 4) = L(5, 4) - L(9, 5) * L(9, 4);
    L(4, 4) = L(4, 4) - L(9, 4) * L(9, 4);
    
    // Joint xarmjoint5, index 8 :
    L(8, 8) = ScalarTraits::sqrt(L(8, 8));
    L(8, 7) = L(8, 7) / L(8, 8);
    L(8, 6) = L(8, 6) / L(8, 8);
    L(8, 5) = L(8, 5) / L(8, 8);
    L(8, 4) = L(8, 4) / L(8, 8);
    L(7, 7) = L(7, 7) - L(8, 7) * L(8, 7);
    L(7, 6) = L(7, 6) - L(8, 7) * L(8, 6);
    L(7, 5) = L(7, 5) - L(8, 7) * L(8, 5);
    L(7, 4) = L(7, 4) - L(8, 7) * L(8, 4);
    L(6, 6) = L(6, 6) - L(8, 6) * L(8, 6);
    L(6, 5) = L(6, 5) - L(8, 6) * L(8, 5);
    L(6, 4) = L(6, 4) - L(8, 6) * L(8, 4);
    L(5, 5) = L(5, 5) - L(8, 5) * L(8, 5);
    L(5, 4) = L(5, 4) - L(8, 5) * L(8, 4);
    L(4, 4) = L(4, 4) - L(8, 4) * L(8, 4);
    
    // Joint xarmjoint4, index 7 :
    L(7, 7) = ScalarTraits::sqrt(L(7, 7));
    L(7, 6) = L(7, 6) / L(7, 7);
    L(7, 5) = L(7, 5) / L(7, 7);
    L(7, 4) = L(7, 4) / L(7, 7);
    L(6, 6) = L(6, 6) - L(7, 6) * L(7, 6);
    L(6, 5) = L(6, 5) - L(7, 6) * L(7, 5);
    L(6, 4) = L(6, 4) - L(7, 6) * L(7, 4);
    L(5, 5) = L(5, 5) - L(7, 5) * L(7, 5);
    L(5, 4) = L(5, 4) - L(7, 5) * L(7, 4);
    L(4, 4) = L(4, 4) - L(7, 4) * L(7, 4);
    
    // Joint xarmjoint3, index 6 :
    L(6, 6) = ScalarTraits::sqrt(L(6, 6));
    L(6, 5) = L(6, 5) / L(6, 6);
    L(6, 4) = L(6, 4) / L(6, 6);
    L(5, 5) = L(5, 5) - L(6, 5) * L(6, 5);
    L(5, 4) = L(5, 4) - L(6, 5) * L(6, 4);
    L(4, 4) = L(4, 4) - L(6, 4) * L(6, 4);
    
    // Joint xarmjoint2, index 5 :
    L(5, 5) = ScalarTraits::sqrt(L(5, 5));
    L(5, 4) = L(5, 4) / L(5, 5);
    L(4, 4) = L(4, 4) - L(5, 4) * L(5, 4);
    
    // Joint xarmjoint1, index 4 :
    L(4, 4) = ScalarTraits::sqrt(L(4, 4));
    
    // Joint motor_wheel_joint_br, index 3 :
    L(3, 3) = ScalarTraits::sqrt(L(3, 3));
    
    // Joint motor_wheel_joint_bl, index 2 :
    L(2, 2) = ScalarTraits::sqrt(L(2, 2));
    
    // Joint motor_wheel_joint_fr, index 1 :
    L(1, 1) = ScalarTraits::sqrt(L(1, 1));
    
    // Joint motor_wheel_joint_fl, index 0 :
    L(0, 0) = ScalarTraits::sqrt(L(0, 0));
    
}

void armstone::rcg::JSIM::computeInverse() {
    computeLInverse();

    inverse(0, 0) =  + (Linv(0, 0) * Linv(0, 0));
    inverse(1, 1) =  + (Linv(1, 1) * Linv(1, 1));
    inverse(2, 2) =  + (Linv(2, 2) * Linv(2, 2));
    inverse(3, 3) =  + (Linv(3, 3) * Linv(3, 3));
    inverse(4, 4) =  + (Linv(4, 4) * Linv(4, 4));
    inverse(5, 5) =  + (Linv(5, 4) * Linv(5, 4)) + (Linv(5, 5) * Linv(5, 5));
    inverse(5, 4) =  + (Linv(5, 4) * Linv(4, 4));
    inverse(4, 5) = inverse(5, 4);
    inverse(6, 6) =  + (Linv(6, 4) * Linv(6, 4)) + (Linv(6, 5) * Linv(6, 5)) + (Linv(6, 6) * Linv(6, 6));
    inverse(6, 5) =  + (Linv(6, 4) * Linv(5, 4)) + (Linv(6, 5) * Linv(5, 5));
    inverse(5, 6) = inverse(6, 5);
    inverse(6, 4) =  + (Linv(6, 4) * Linv(4, 4));
    inverse(4, 6) = inverse(6, 4);
    inverse(7, 7) =  + (Linv(7, 4) * Linv(7, 4)) + (Linv(7, 5) * Linv(7, 5)) + (Linv(7, 6) * Linv(7, 6)) + (Linv(7, 7) * Linv(7, 7));
    inverse(7, 6) =  + (Linv(7, 4) * Linv(6, 4)) + (Linv(7, 5) * Linv(6, 5)) + (Linv(7, 6) * Linv(6, 6));
    inverse(6, 7) = inverse(7, 6);
    inverse(7, 5) =  + (Linv(7, 4) * Linv(5, 4)) + (Linv(7, 5) * Linv(5, 5));
    inverse(5, 7) = inverse(7, 5);
    inverse(7, 4) =  + (Linv(7, 4) * Linv(4, 4));
    inverse(4, 7) = inverse(7, 4);
    inverse(8, 8) =  + (Linv(8, 4) * Linv(8, 4)) + (Linv(8, 5) * Linv(8, 5)) + (Linv(8, 6) * Linv(8, 6)) + (Linv(8, 7) * Linv(8, 7)) + (Linv(8, 8) * Linv(8, 8));
    inverse(8, 7) =  + (Linv(8, 4) * Linv(7, 4)) + (Linv(8, 5) * Linv(7, 5)) + (Linv(8, 6) * Linv(7, 6)) + (Linv(8, 7) * Linv(7, 7));
    inverse(7, 8) = inverse(8, 7);
    inverse(8, 6) =  + (Linv(8, 4) * Linv(6, 4)) + (Linv(8, 5) * Linv(6, 5)) + (Linv(8, 6) * Linv(6, 6));
    inverse(6, 8) = inverse(8, 6);
    inverse(8, 5) =  + (Linv(8, 4) * Linv(5, 4)) + (Linv(8, 5) * Linv(5, 5));
    inverse(5, 8) = inverse(8, 5);
    inverse(8, 4) =  + (Linv(8, 4) * Linv(4, 4));
    inverse(4, 8) = inverse(8, 4);
    inverse(9, 9) =  + (Linv(9, 4) * Linv(9, 4)) + (Linv(9, 5) * Linv(9, 5)) + (Linv(9, 6) * Linv(9, 6)) + (Linv(9, 7) * Linv(9, 7)) + (Linv(9, 8) * Linv(9, 8)) + (Linv(9, 9) * Linv(9, 9));
    inverse(9, 8) =  + (Linv(9, 4) * Linv(8, 4)) + (Linv(9, 5) * Linv(8, 5)) + (Linv(9, 6) * Linv(8, 6)) + (Linv(9, 7) * Linv(8, 7)) + (Linv(9, 8) * Linv(8, 8));
    inverse(8, 9) = inverse(9, 8);
    inverse(9, 7) =  + (Linv(9, 4) * Linv(7, 4)) + (Linv(9, 5) * Linv(7, 5)) + (Linv(9, 6) * Linv(7, 6)) + (Linv(9, 7) * Linv(7, 7));
    inverse(7, 9) = inverse(9, 7);
    inverse(9, 6) =  + (Linv(9, 4) * Linv(6, 4)) + (Linv(9, 5) * Linv(6, 5)) + (Linv(9, 6) * Linv(6, 6));
    inverse(6, 9) = inverse(9, 6);
    inverse(9, 5) =  + (Linv(9, 4) * Linv(5, 4)) + (Linv(9, 5) * Linv(5, 5));
    inverse(5, 9) = inverse(9, 5);
    inverse(9, 4) =  + (Linv(9, 4) * Linv(4, 4));
    inverse(4, 9) = inverse(9, 4);
}

void armstone::rcg::JSIM::computeLInverse() {
    //assumes L has been computed already
    Linv(0, 0) = 1 / L(0, 0);
    Linv(1, 1) = 1 / L(1, 1);
    Linv(2, 2) = 1 / L(2, 2);
    Linv(3, 3) = 1 / L(3, 3);
    Linv(4, 4) = 1 / L(4, 4);
    Linv(5, 5) = 1 / L(5, 5);
    Linv(6, 6) = 1 / L(6, 6);
    Linv(7, 7) = 1 / L(7, 7);
    Linv(8, 8) = 1 / L(8, 8);
    Linv(9, 9) = 1 / L(9, 9);
    Linv(5, 4) = - Linv(4, 4) * ((Linv(5, 5) * L(5, 4)) + 0);
    Linv(6, 5) = - Linv(5, 5) * ((Linv(6, 6) * L(6, 5)) + 0);
    Linv(6, 4) = - Linv(4, 4) * ((Linv(6, 5) * L(5, 4)) + (Linv(6, 6) * L(6, 4)) + 0);
    Linv(7, 6) = - Linv(6, 6) * ((Linv(7, 7) * L(7, 6)) + 0);
    Linv(7, 5) = - Linv(5, 5) * ((Linv(7, 6) * L(6, 5)) + (Linv(7, 7) * L(7, 5)) + 0);
    Linv(7, 4) = - Linv(4, 4) * ((Linv(7, 5) * L(5, 4)) + (Linv(7, 6) * L(6, 4)) + (Linv(7, 7) * L(7, 4)) + 0);
    Linv(8, 7) = - Linv(7, 7) * ((Linv(8, 8) * L(8, 7)) + 0);
    Linv(8, 6) = - Linv(6, 6) * ((Linv(8, 7) * L(7, 6)) + (Linv(8, 8) * L(8, 6)) + 0);
    Linv(8, 5) = - Linv(5, 5) * ((Linv(8, 6) * L(6, 5)) + (Linv(8, 7) * L(7, 5)) + (Linv(8, 8) * L(8, 5)) + 0);
    Linv(8, 4) = - Linv(4, 4) * ((Linv(8, 5) * L(5, 4)) + (Linv(8, 6) * L(6, 4)) + (Linv(8, 7) * L(7, 4)) + (Linv(8, 8) * L(8, 4)) + 0);
    Linv(9, 8) = - Linv(8, 8) * ((Linv(9, 9) * L(9, 8)) + 0);
    Linv(9, 7) = - Linv(7, 7) * ((Linv(9, 8) * L(8, 7)) + (Linv(9, 9) * L(9, 7)) + 0);
    Linv(9, 6) = - Linv(6, 6) * ((Linv(9, 7) * L(7, 6)) + (Linv(9, 8) * L(8, 6)) + (Linv(9, 9) * L(9, 6)) + 0);
    Linv(9, 5) = - Linv(5, 5) * ((Linv(9, 6) * L(6, 5)) + (Linv(9, 7) * L(7, 5)) + (Linv(9, 8) * L(8, 5)) + (Linv(9, 9) * L(9, 5)) + 0);
    Linv(9, 4) = - Linv(4, 4) * ((Linv(9, 5) * L(5, 4)) + (Linv(9, 6) * L(6, 4)) + (Linv(9, 7) * L(7, 4)) + (Linv(9, 8) * L(8, 4)) + (Linv(9, 9) * L(9, 4)) + 0);
}
