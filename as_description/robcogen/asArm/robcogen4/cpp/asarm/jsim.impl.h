

//Implementation of default constructor
template <typename TRAIT>
iit::asArm::dyn::tpl::JSIM<TRAIT>::JSIM(IProperties& inertiaProperties, FTransforms& forceTransforms) :
    linkInertias(inertiaProperties),
    frcTransf( &forceTransforms ),
    xarmlink6_Ic(linkInertias.getTensor_xarmlink6())
{
    //Initialize the matrix itself
    this->setZero();
}

#define DATA tpl::JSIM<TRAIT>::operator()
#define Fcol(j) (tpl::JSIM<TRAIT>:: template block<6,1>(0,(j)+6))
#define F(i,j) DATA((i),(j)+6)

template <typename TRAIT>
const typename iit::asArm::dyn::tpl::JSIM<TRAIT>& iit::asArm::dyn::tpl::JSIM<TRAIT>::update(const JointState& state) {

    // Precomputes only once the coordinate transforms:
    frcTransf -> fr_xarmlink5_X_fr_xarmlink6(state);
    frcTransf -> fr_xarmlink4_X_fr_xarmlink5(state);
    frcTransf -> fr_xarmlink3_X_fr_xarmlink4(state);
    frcTransf -> fr_xarmlink2_X_fr_xarmlink3(state);
    frcTransf -> fr_xarmlink1_X_fr_xarmlink2(state);
    frcTransf -> fr_base_link_X_fr_xarmlink1(state);

    // Initializes the composite inertia tensors
    base_link_Ic = linkInertias.getTensor_base_link();
    xarmlink1_Ic = linkInertias.getTensor_xarmlink1();
    xarmlink2_Ic = linkInertias.getTensor_xarmlink2();
    xarmlink3_Ic = linkInertias.getTensor_xarmlink3();
    xarmlink4_Ic = linkInertias.getTensor_xarmlink4();
    xarmlink5_Ic = linkInertias.getTensor_xarmlink5();

    // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration

    // Link xarmlink6:
    iit::rbd::transformInertia<Scalar>(xarmlink6_Ic, frcTransf -> fr_xarmlink5_X_fr_xarmlink6, Ic_spare);
    xarmlink5_Ic += Ic_spare;

    Fcol(XARMJOINT6) = xarmlink6_Ic.col(iit::rbd::AZ);
    DATA(XARMJOINT6+6, XARMJOINT6+6) = Fcol(XARMJOINT6)(iit::rbd::AZ);

    Fcol(XARMJOINT6) = frcTransf -> fr_xarmlink5_X_fr_xarmlink6 * Fcol(XARMJOINT6);
    DATA(XARMJOINT6+6, XARMJOINT5+6) = F(iit::rbd::AZ,XARMJOINT6);
    DATA(XARMJOINT5+6, XARMJOINT6+6) = DATA(XARMJOINT6+6, XARMJOINT5+6);
    Fcol(XARMJOINT6) = frcTransf -> fr_xarmlink4_X_fr_xarmlink5 * Fcol(XARMJOINT6);
    DATA(XARMJOINT6+6, XARMJOINT4+6) = F(iit::rbd::AZ,XARMJOINT6);
    DATA(XARMJOINT4+6, XARMJOINT6+6) = DATA(XARMJOINT6+6, XARMJOINT4+6);
    Fcol(XARMJOINT6) = frcTransf -> fr_xarmlink3_X_fr_xarmlink4 * Fcol(XARMJOINT6);
    DATA(XARMJOINT6+6, XARMJOINT3+6) = F(iit::rbd::AZ,XARMJOINT6);
    DATA(XARMJOINT3+6, XARMJOINT6+6) = DATA(XARMJOINT6+6, XARMJOINT3+6);
    Fcol(XARMJOINT6) = frcTransf -> fr_xarmlink2_X_fr_xarmlink3 * Fcol(XARMJOINT6);
    DATA(XARMJOINT6+6, XARMJOINT2+6) = F(iit::rbd::AZ,XARMJOINT6);
    DATA(XARMJOINT2+6, XARMJOINT6+6) = DATA(XARMJOINT6+6, XARMJOINT2+6);
    Fcol(XARMJOINT6) = frcTransf -> fr_xarmlink1_X_fr_xarmlink2 * Fcol(XARMJOINT6);
    DATA(XARMJOINT6+6, XARMJOINT1+6) = F(iit::rbd::AZ,XARMJOINT6);
    DATA(XARMJOINT1+6, XARMJOINT6+6) = DATA(XARMJOINT6+6, XARMJOINT1+6);
    Fcol(XARMJOINT6) = frcTransf -> fr_base_link_X_fr_xarmlink1 * Fcol(XARMJOINT6);

    // Link xarmlink5:
    iit::rbd::transformInertia<Scalar>(xarmlink5_Ic, frcTransf -> fr_xarmlink4_X_fr_xarmlink5, Ic_spare);
    xarmlink4_Ic += Ic_spare;

    Fcol(XARMJOINT5) = xarmlink5_Ic.col(iit::rbd::AZ);
    DATA(XARMJOINT5+6, XARMJOINT5+6) = Fcol(XARMJOINT5)(iit::rbd::AZ);

    Fcol(XARMJOINT5) = frcTransf -> fr_xarmlink4_X_fr_xarmlink5 * Fcol(XARMJOINT5);
    DATA(XARMJOINT5+6, XARMJOINT4+6) = F(iit::rbd::AZ,XARMJOINT5);
    DATA(XARMJOINT4+6, XARMJOINT5+6) = DATA(XARMJOINT5+6, XARMJOINT4+6);
    Fcol(XARMJOINT5) = frcTransf -> fr_xarmlink3_X_fr_xarmlink4 * Fcol(XARMJOINT5);
    DATA(XARMJOINT5+6, XARMJOINT3+6) = F(iit::rbd::AZ,XARMJOINT5);
    DATA(XARMJOINT3+6, XARMJOINT5+6) = DATA(XARMJOINT5+6, XARMJOINT3+6);
    Fcol(XARMJOINT5) = frcTransf -> fr_xarmlink2_X_fr_xarmlink3 * Fcol(XARMJOINT5);
    DATA(XARMJOINT5+6, XARMJOINT2+6) = F(iit::rbd::AZ,XARMJOINT5);
    DATA(XARMJOINT2+6, XARMJOINT5+6) = DATA(XARMJOINT5+6, XARMJOINT2+6);
    Fcol(XARMJOINT5) = frcTransf -> fr_xarmlink1_X_fr_xarmlink2 * Fcol(XARMJOINT5);
    DATA(XARMJOINT5+6, XARMJOINT1+6) = F(iit::rbd::AZ,XARMJOINT5);
    DATA(XARMJOINT1+6, XARMJOINT5+6) = DATA(XARMJOINT5+6, XARMJOINT1+6);
    Fcol(XARMJOINT5) = frcTransf -> fr_base_link_X_fr_xarmlink1 * Fcol(XARMJOINT5);

    // Link xarmlink4:
    iit::rbd::transformInertia<Scalar>(xarmlink4_Ic, frcTransf -> fr_xarmlink3_X_fr_xarmlink4, Ic_spare);
    xarmlink3_Ic += Ic_spare;

    Fcol(XARMJOINT4) = xarmlink4_Ic.col(iit::rbd::AZ);
    DATA(XARMJOINT4+6, XARMJOINT4+6) = Fcol(XARMJOINT4)(iit::rbd::AZ);

    Fcol(XARMJOINT4) = frcTransf -> fr_xarmlink3_X_fr_xarmlink4 * Fcol(XARMJOINT4);
    DATA(XARMJOINT4+6, XARMJOINT3+6) = F(iit::rbd::AZ,XARMJOINT4);
    DATA(XARMJOINT3+6, XARMJOINT4+6) = DATA(XARMJOINT4+6, XARMJOINT3+6);
    Fcol(XARMJOINT4) = frcTransf -> fr_xarmlink2_X_fr_xarmlink3 * Fcol(XARMJOINT4);
    DATA(XARMJOINT4+6, XARMJOINT2+6) = F(iit::rbd::AZ,XARMJOINT4);
    DATA(XARMJOINT2+6, XARMJOINT4+6) = DATA(XARMJOINT4+6, XARMJOINT2+6);
    Fcol(XARMJOINT4) = frcTransf -> fr_xarmlink1_X_fr_xarmlink2 * Fcol(XARMJOINT4);
    DATA(XARMJOINT4+6, XARMJOINT1+6) = F(iit::rbd::AZ,XARMJOINT4);
    DATA(XARMJOINT1+6, XARMJOINT4+6) = DATA(XARMJOINT4+6, XARMJOINT1+6);
    Fcol(XARMJOINT4) = frcTransf -> fr_base_link_X_fr_xarmlink1 * Fcol(XARMJOINT4);

    // Link xarmlink3:
    iit::rbd::transformInertia<Scalar>(xarmlink3_Ic, frcTransf -> fr_xarmlink2_X_fr_xarmlink3, Ic_spare);
    xarmlink2_Ic += Ic_spare;

    Fcol(XARMJOINT3) = xarmlink3_Ic.col(iit::rbd::AZ);
    DATA(XARMJOINT3+6, XARMJOINT3+6) = Fcol(XARMJOINT3)(iit::rbd::AZ);

    Fcol(XARMJOINT3) = frcTransf -> fr_xarmlink2_X_fr_xarmlink3 * Fcol(XARMJOINT3);
    DATA(XARMJOINT3+6, XARMJOINT2+6) = F(iit::rbd::AZ,XARMJOINT3);
    DATA(XARMJOINT2+6, XARMJOINT3+6) = DATA(XARMJOINT3+6, XARMJOINT2+6);
    Fcol(XARMJOINT3) = frcTransf -> fr_xarmlink1_X_fr_xarmlink2 * Fcol(XARMJOINT3);
    DATA(XARMJOINT3+6, XARMJOINT1+6) = F(iit::rbd::AZ,XARMJOINT3);
    DATA(XARMJOINT1+6, XARMJOINT3+6) = DATA(XARMJOINT3+6, XARMJOINT1+6);
    Fcol(XARMJOINT3) = frcTransf -> fr_base_link_X_fr_xarmlink1 * Fcol(XARMJOINT3);

    // Link xarmlink2:
    iit::rbd::transformInertia<Scalar>(xarmlink2_Ic, frcTransf -> fr_xarmlink1_X_fr_xarmlink2, Ic_spare);
    xarmlink1_Ic += Ic_spare;

    Fcol(XARMJOINT2) = xarmlink2_Ic.col(iit::rbd::AZ);
    DATA(XARMJOINT2+6, XARMJOINT2+6) = Fcol(XARMJOINT2)(iit::rbd::AZ);

    Fcol(XARMJOINT2) = frcTransf -> fr_xarmlink1_X_fr_xarmlink2 * Fcol(XARMJOINT2);
    DATA(XARMJOINT2+6, XARMJOINT1+6) = F(iit::rbd::AZ,XARMJOINT2);
    DATA(XARMJOINT1+6, XARMJOINT2+6) = DATA(XARMJOINT2+6, XARMJOINT1+6);
    Fcol(XARMJOINT2) = frcTransf -> fr_base_link_X_fr_xarmlink1 * Fcol(XARMJOINT2);

    // Link xarmlink1:
    iit::rbd::transformInertia<Scalar>(xarmlink1_Ic, frcTransf -> fr_base_link_X_fr_xarmlink1, Ic_spare);
    base_link_Ic += Ic_spare;

    Fcol(XARMJOINT1) = xarmlink1_Ic.col(iit::rbd::AZ);
    DATA(XARMJOINT1+6, XARMJOINT1+6) = Fcol(XARMJOINT1)(iit::rbd::AZ);

    Fcol(XARMJOINT1) = frcTransf -> fr_base_link_X_fr_xarmlink1 * Fcol(XARMJOINT1);

    // Copies the upper-right block into the lower-left block, after transposing
    JSIM<TRAIT>:: template block<6, 6>(6,0) = (JSIM<TRAIT>:: template block<6, 6>(0,6)).transpose();
    // The composite-inertia of the whole robot is the upper-left quadrant of the JSIM
    JSIM<TRAIT>:: template block<6,6>(0,0) = base_link_Ic;
    return *this;
}

#undef DATA
#undef F

template <typename TRAIT>
void iit::asArm::dyn::tpl::JSIM<TRAIT>::computeL() {
    L = this -> template triangularView<Eigen::Lower>();
    // Joint xarmjoint6, index 5 :
    L(5, 5) = std::sqrt(L(5, 5));
    L(5, 4) = L(5, 4) / L(5, 5);
    L(5, 3) = L(5, 3) / L(5, 5);
    L(5, 2) = L(5, 2) / L(5, 5);
    L(5, 1) = L(5, 1) / L(5, 5);
    L(5, 0) = L(5, 0) / L(5, 5);
    L(4, 4) = L(4, 4) - L(5, 4) * L(5, 4);
    L(4, 3) = L(4, 3) - L(5, 4) * L(5, 3);
    L(4, 2) = L(4, 2) - L(5, 4) * L(5, 2);
    L(4, 1) = L(4, 1) - L(5, 4) * L(5, 1);
    L(4, 0) = L(4, 0) - L(5, 4) * L(5, 0);
    L(3, 3) = L(3, 3) - L(5, 3) * L(5, 3);
    L(3, 2) = L(3, 2) - L(5, 3) * L(5, 2);
    L(3, 1) = L(3, 1) - L(5, 3) * L(5, 1);
    L(3, 0) = L(3, 0) - L(5, 3) * L(5, 0);
    L(2, 2) = L(2, 2) - L(5, 2) * L(5, 2);
    L(2, 1) = L(2, 1) - L(5, 2) * L(5, 1);
    L(2, 0) = L(2, 0) - L(5, 2) * L(5, 0);
    L(1, 1) = L(1, 1) - L(5, 1) * L(5, 1);
    L(1, 0) = L(1, 0) - L(5, 1) * L(5, 0);
    L(0, 0) = L(0, 0) - L(5, 0) * L(5, 0);
    
    // Joint xarmjoint5, index 4 :
    L(4, 4) = std::sqrt(L(4, 4));
    L(4, 3) = L(4, 3) / L(4, 4);
    L(4, 2) = L(4, 2) / L(4, 4);
    L(4, 1) = L(4, 1) / L(4, 4);
    L(4, 0) = L(4, 0) / L(4, 4);
    L(3, 3) = L(3, 3) - L(4, 3) * L(4, 3);
    L(3, 2) = L(3, 2) - L(4, 3) * L(4, 2);
    L(3, 1) = L(3, 1) - L(4, 3) * L(4, 1);
    L(3, 0) = L(3, 0) - L(4, 3) * L(4, 0);
    L(2, 2) = L(2, 2) - L(4, 2) * L(4, 2);
    L(2, 1) = L(2, 1) - L(4, 2) * L(4, 1);
    L(2, 0) = L(2, 0) - L(4, 2) * L(4, 0);
    L(1, 1) = L(1, 1) - L(4, 1) * L(4, 1);
    L(1, 0) = L(1, 0) - L(4, 1) * L(4, 0);
    L(0, 0) = L(0, 0) - L(4, 0) * L(4, 0);
    
    // Joint xarmjoint4, index 3 :
    L(3, 3) = std::sqrt(L(3, 3));
    L(3, 2) = L(3, 2) / L(3, 3);
    L(3, 1) = L(3, 1) / L(3, 3);
    L(3, 0) = L(3, 0) / L(3, 3);
    L(2, 2) = L(2, 2) - L(3, 2) * L(3, 2);
    L(2, 1) = L(2, 1) - L(3, 2) * L(3, 1);
    L(2, 0) = L(2, 0) - L(3, 2) * L(3, 0);
    L(1, 1) = L(1, 1) - L(3, 1) * L(3, 1);
    L(1, 0) = L(1, 0) - L(3, 1) * L(3, 0);
    L(0, 0) = L(0, 0) - L(3, 0) * L(3, 0);
    
    // Joint xarmjoint3, index 2 :
    L(2, 2) = std::sqrt(L(2, 2));
    L(2, 1) = L(2, 1) / L(2, 2);
    L(2, 0) = L(2, 0) / L(2, 2);
    L(1, 1) = L(1, 1) - L(2, 1) * L(2, 1);
    L(1, 0) = L(1, 0) - L(2, 1) * L(2, 0);
    L(0, 0) = L(0, 0) - L(2, 0) * L(2, 0);
    
    // Joint xarmjoint2, index 1 :
    L(1, 1) = std::sqrt(L(1, 1));
    L(1, 0) = L(1, 0) / L(1, 1);
    L(0, 0) = L(0, 0) - L(1, 0) * L(1, 0);
    
    // Joint xarmjoint1, index 0 :
    L(0, 0) = std::sqrt(L(0, 0));
    
}

template <typename TRAIT>
void iit::asArm::dyn::tpl::JSIM<TRAIT>::computeInverse() {
    computeLInverse();

    inverse(0, 0) =  + (Linv(0, 0) * Linv(0, 0));
    inverse(1, 1) =  + (Linv(1, 0) * Linv(1, 0)) + (Linv(1, 1) * Linv(1, 1));
    inverse(1, 0) =  + (Linv(1, 0) * Linv(0, 0));
    inverse(0, 1) = inverse(1, 0);
    inverse(2, 2) =  + (Linv(2, 0) * Linv(2, 0)) + (Linv(2, 1) * Linv(2, 1)) + (Linv(2, 2) * Linv(2, 2));
    inverse(2, 1) =  + (Linv(2, 0) * Linv(1, 0)) + (Linv(2, 1) * Linv(1, 1));
    inverse(1, 2) = inverse(2, 1);
    inverse(2, 0) =  + (Linv(2, 0) * Linv(0, 0));
    inverse(0, 2) = inverse(2, 0);
    inverse(3, 3) =  + (Linv(3, 0) * Linv(3, 0)) + (Linv(3, 1) * Linv(3, 1)) + (Linv(3, 2) * Linv(3, 2)) + (Linv(3, 3) * Linv(3, 3));
    inverse(3, 2) =  + (Linv(3, 0) * Linv(2, 0)) + (Linv(3, 1) * Linv(2, 1)) + (Linv(3, 2) * Linv(2, 2));
    inverse(2, 3) = inverse(3, 2);
    inverse(3, 1) =  + (Linv(3, 0) * Linv(1, 0)) + (Linv(3, 1) * Linv(1, 1));
    inverse(1, 3) = inverse(3, 1);
    inverse(3, 0) =  + (Linv(3, 0) * Linv(0, 0));
    inverse(0, 3) = inverse(3, 0);
    inverse(4, 4) =  + (Linv(4, 0) * Linv(4, 0)) + (Linv(4, 1) * Linv(4, 1)) + (Linv(4, 2) * Linv(4, 2)) + (Linv(4, 3) * Linv(4, 3)) + (Linv(4, 4) * Linv(4, 4));
    inverse(4, 3) =  + (Linv(4, 0) * Linv(3, 0)) + (Linv(4, 1) * Linv(3, 1)) + (Linv(4, 2) * Linv(3, 2)) + (Linv(4, 3) * Linv(3, 3));
    inverse(3, 4) = inverse(4, 3);
    inverse(4, 2) =  + (Linv(4, 0) * Linv(2, 0)) + (Linv(4, 1) * Linv(2, 1)) + (Linv(4, 2) * Linv(2, 2));
    inverse(2, 4) = inverse(4, 2);
    inverse(4, 1) =  + (Linv(4, 0) * Linv(1, 0)) + (Linv(4, 1) * Linv(1, 1));
    inverse(1, 4) = inverse(4, 1);
    inverse(4, 0) =  + (Linv(4, 0) * Linv(0, 0));
    inverse(0, 4) = inverse(4, 0);
    inverse(5, 5) =  + (Linv(5, 0) * Linv(5, 0)) + (Linv(5, 1) * Linv(5, 1)) + (Linv(5, 2) * Linv(5, 2)) + (Linv(5, 3) * Linv(5, 3)) + (Linv(5, 4) * Linv(5, 4)) + (Linv(5, 5) * Linv(5, 5));
    inverse(5, 4) =  + (Linv(5, 0) * Linv(4, 0)) + (Linv(5, 1) * Linv(4, 1)) + (Linv(5, 2) * Linv(4, 2)) + (Linv(5, 3) * Linv(4, 3)) + (Linv(5, 4) * Linv(4, 4));
    inverse(4, 5) = inverse(5, 4);
    inverse(5, 3) =  + (Linv(5, 0) * Linv(3, 0)) + (Linv(5, 1) * Linv(3, 1)) + (Linv(5, 2) * Linv(3, 2)) + (Linv(5, 3) * Linv(3, 3));
    inverse(3, 5) = inverse(5, 3);
    inverse(5, 2) =  + (Linv(5, 0) * Linv(2, 0)) + (Linv(5, 1) * Linv(2, 1)) + (Linv(5, 2) * Linv(2, 2));
    inverse(2, 5) = inverse(5, 2);
    inverse(5, 1) =  + (Linv(5, 0) * Linv(1, 0)) + (Linv(5, 1) * Linv(1, 1));
    inverse(1, 5) = inverse(5, 1);
    inverse(5, 0) =  + (Linv(5, 0) * Linv(0, 0));
    inverse(0, 5) = inverse(5, 0);
}

template <typename TRAIT>
void iit::asArm::dyn::tpl::JSIM<TRAIT>::computeLInverse() {
    //assumes L has been computed already
    Linv(0, 0) = 1 / L(0, 0);
    Linv(1, 1) = 1 / L(1, 1);
    Linv(2, 2) = 1 / L(2, 2);
    Linv(3, 3) = 1 / L(3, 3);
    Linv(4, 4) = 1 / L(4, 4);
    Linv(5, 5) = 1 / L(5, 5);
    Linv(1, 0) = - Linv(0, 0) * ((Linv(1, 1) * L(1, 0)) + 0);
    Linv(2, 1) = - Linv(1, 1) * ((Linv(2, 2) * L(2, 1)) + 0);
    Linv(2, 0) = - Linv(0, 0) * ((Linv(2, 1) * L(1, 0)) + (Linv(2, 2) * L(2, 0)) + 0);
    Linv(3, 2) = - Linv(2, 2) * ((Linv(3, 3) * L(3, 2)) + 0);
    Linv(3, 1) = - Linv(1, 1) * ((Linv(3, 2) * L(2, 1)) + (Linv(3, 3) * L(3, 1)) + 0);
    Linv(3, 0) = - Linv(0, 0) * ((Linv(3, 1) * L(1, 0)) + (Linv(3, 2) * L(2, 0)) + (Linv(3, 3) * L(3, 0)) + 0);
    Linv(4, 3) = - Linv(3, 3) * ((Linv(4, 4) * L(4, 3)) + 0);
    Linv(4, 2) = - Linv(2, 2) * ((Linv(4, 3) * L(3, 2)) + (Linv(4, 4) * L(4, 2)) + 0);
    Linv(4, 1) = - Linv(1, 1) * ((Linv(4, 2) * L(2, 1)) + (Linv(4, 3) * L(3, 1)) + (Linv(4, 4) * L(4, 1)) + 0);
    Linv(4, 0) = - Linv(0, 0) * ((Linv(4, 1) * L(1, 0)) + (Linv(4, 2) * L(2, 0)) + (Linv(4, 3) * L(3, 0)) + (Linv(4, 4) * L(4, 0)) + 0);
    Linv(5, 4) = - Linv(4, 4) * ((Linv(5, 5) * L(5, 4)) + 0);
    Linv(5, 3) = - Linv(3, 3) * ((Linv(5, 4) * L(4, 3)) + (Linv(5, 5) * L(5, 3)) + 0);
    Linv(5, 2) = - Linv(2, 2) * ((Linv(5, 3) * L(3, 2)) + (Linv(5, 4) * L(4, 2)) + (Linv(5, 5) * L(5, 2)) + 0);
    Linv(5, 1) = - Linv(1, 1) * ((Linv(5, 2) * L(2, 1)) + (Linv(5, 3) * L(3, 1)) + (Linv(5, 4) * L(4, 1)) + (Linv(5, 5) * L(5, 1)) + 0);
    Linv(5, 0) = - Linv(0, 0) * ((Linv(5, 1) * L(1, 0)) + (Linv(5, 2) * L(2, 0)) + (Linv(5, 3) * L(3, 0)) + (Linv(5, 4) * L(4, 0)) + (Linv(5, 5) * L(5, 0)) + 0);
}

