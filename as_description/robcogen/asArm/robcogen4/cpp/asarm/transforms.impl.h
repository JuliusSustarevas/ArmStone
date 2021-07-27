
// Constructors
template <typename TRAIT>
iit::asArm::tpl::MotionTransforms<TRAIT>::MotionTransforms
    ()
     :
    fr_xarmlink1_X_fr_base_link(),
    fr_base_link_X_fr_xarmlink1(),
    fr_xarmlink2_X_fr_xarmlink1(),
    fr_xarmlink1_X_fr_xarmlink2(),
    fr_xarmlink3_X_fr_xarmlink2(),
    fr_xarmlink2_X_fr_xarmlink3(),
    fr_xarmlink4_X_fr_xarmlink3(),
    fr_xarmlink3_X_fr_xarmlink4(),
    fr_xarmlink5_X_fr_xarmlink4(),
    fr_xarmlink4_X_fr_xarmlink5(),
    fr_xarmlink6_X_fr_xarmlink5(),
    fr_xarmlink5_X_fr_xarmlink6()
{
    updateParameters();
}
template <typename TRAIT>
void iit::asArm::tpl::MotionTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::asArm::tpl::ForceTransforms<TRAIT>::ForceTransforms
    ()
     :
    fr_xarmlink1_X_fr_base_link(),
    fr_base_link_X_fr_xarmlink1(),
    fr_xarmlink2_X_fr_xarmlink1(),
    fr_xarmlink1_X_fr_xarmlink2(),
    fr_xarmlink3_X_fr_xarmlink2(),
    fr_xarmlink2_X_fr_xarmlink3(),
    fr_xarmlink4_X_fr_xarmlink3(),
    fr_xarmlink3_X_fr_xarmlink4(),
    fr_xarmlink5_X_fr_xarmlink4(),
    fr_xarmlink4_X_fr_xarmlink5(),
    fr_xarmlink6_X_fr_xarmlink5(),
    fr_xarmlink5_X_fr_xarmlink6()
{
    updateParameters();
}
template <typename TRAIT>
void iit::asArm::tpl::ForceTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::asArm::tpl::HomogeneousTransforms<TRAIT>::HomogeneousTransforms
    ()
     :
    fr_xarmlink1_X_fr_base_link(),
    fr_base_link_X_fr_xarmlink1(),
    fr_xarmlink2_X_fr_xarmlink1(),
    fr_xarmlink1_X_fr_xarmlink2(),
    fr_xarmlink3_X_fr_xarmlink2(),
    fr_xarmlink2_X_fr_xarmlink3(),
    fr_xarmlink4_X_fr_xarmlink3(),
    fr_xarmlink3_X_fr_xarmlink4(),
    fr_xarmlink5_X_fr_xarmlink4(),
    fr_xarmlink4_X_fr_xarmlink5(),
    fr_xarmlink6_X_fr_xarmlink5(),
    fr_xarmlink5_X_fr_xarmlink6()
{
    updateParameters();
}
template <typename TRAIT>
void iit::asArm::tpl::HomogeneousTransforms<TRAIT>::updateParameters() {
}

template <typename TRAIT>
iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_xarmlink1_X_fr_base_link::Type_fr_xarmlink1_X_fr_base_link()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,5) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = - 0.175;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_xarmlink1_X_fr_base_link& iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_xarmlink1_X_fr_base_link::update(const JState& q) {
    Scalar s_q_xarmjoint1_;
    Scalar c_q_xarmjoint1_;
    
    s_q_xarmjoint1_ = TRAIT::sin( q(XARMJOINT1));
    c_q_xarmjoint1_ = TRAIT::cos( q(XARMJOINT1));
    
    (*this)(0,0) =  c_q_xarmjoint1_;
    (*this)(0,1) =  s_q_xarmjoint1_;
    (*this)(1,0) = - s_q_xarmjoint1_;
    (*this)(1,1) =  c_q_xarmjoint1_;
    (*this)(3,0) = (- 0.214 *  s_q_xarmjoint1_);
    (*this)(3,1) = ( 0.214 *  c_q_xarmjoint1_);
    (*this)(3,2) = ( 0.175 *  s_q_xarmjoint1_);
    (*this)(3,3) =  c_q_xarmjoint1_;
    (*this)(3,4) =  s_q_xarmjoint1_;
    (*this)(4,0) = (- 0.214 *  c_q_xarmjoint1_);
    (*this)(4,1) = (- 0.214 *  s_q_xarmjoint1_);
    (*this)(4,2) = ( 0.175 *  c_q_xarmjoint1_);
    (*this)(4,3) = - s_q_xarmjoint1_;
    (*this)(4,4) =  c_q_xarmjoint1_;
    return *this;
}
template <typename TRAIT>
iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_base_link_X_fr_xarmlink1::Type_fr_base_link_X_fr_xarmlink1()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,2) = - 0.175;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_base_link_X_fr_xarmlink1& iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_base_link_X_fr_xarmlink1::update(const JState& q) {
    Scalar s_q_xarmjoint1_;
    Scalar c_q_xarmjoint1_;
    
    s_q_xarmjoint1_ = TRAIT::sin( q(XARMJOINT1));
    c_q_xarmjoint1_ = TRAIT::cos( q(XARMJOINT1));
    
    (*this)(0,0) =  c_q_xarmjoint1_;
    (*this)(0,1) = - s_q_xarmjoint1_;
    (*this)(1,0) =  s_q_xarmjoint1_;
    (*this)(1,1) =  c_q_xarmjoint1_;
    (*this)(3,0) = (- 0.214 *  s_q_xarmjoint1_);
    (*this)(3,1) = (- 0.214 *  c_q_xarmjoint1_);
    (*this)(3,3) =  c_q_xarmjoint1_;
    (*this)(3,4) = - s_q_xarmjoint1_;
    (*this)(4,0) = ( 0.214 *  c_q_xarmjoint1_);
    (*this)(4,1) = (- 0.214 *  s_q_xarmjoint1_);
    (*this)(4,3) =  s_q_xarmjoint1_;
    (*this)(4,4) =  c_q_xarmjoint1_;
    (*this)(5,0) = ( 0.175 *  s_q_xarmjoint1_);
    (*this)(5,1) = ( 0.175 *  c_q_xarmjoint1_);
    return *this;
}
template <typename TRAIT>
iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_xarmlink2_X_fr_xarmlink1::Type_fr_xarmlink2_X_fr_xarmlink1()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0.999999;
    (*this)(2,2) = - 3.6732E-6;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0.999999;
    (*this)(5,5) = - 3.6732E-6;
}
template <typename TRAIT>
const typename iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_xarmlink2_X_fr_xarmlink1& iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_xarmlink2_X_fr_xarmlink1::update(const JState& q) {
    Scalar s_q_xarmjoint2_;
    Scalar c_q_xarmjoint2_;
    
    s_q_xarmjoint2_ = TRAIT::sin( q(XARMJOINT2));
    c_q_xarmjoint2_ = TRAIT::cos( q(XARMJOINT2));
    
    (*this)(0,0) =  c_q_xarmjoint2_;
    (*this)(0,1) = (- 3.6732E-6 *  s_q_xarmjoint2_);
    (*this)(0,2) = (- 0.999999 *  s_q_xarmjoint2_);
    (*this)(1,0) = - s_q_xarmjoint2_;
    (*this)(1,1) = (- 3.6732E-6 *  c_q_xarmjoint2_);
    (*this)(1,2) = (- 0.999999 *  c_q_xarmjoint2_);
    (*this)(3,3) =  c_q_xarmjoint2_;
    (*this)(3,4) = (- 3.6732E-6 *  s_q_xarmjoint2_);
    (*this)(3,5) = (- 0.999999 *  s_q_xarmjoint2_);
    (*this)(4,3) = - s_q_xarmjoint2_;
    (*this)(4,4) = (- 3.6732E-6 *  c_q_xarmjoint2_);
    (*this)(4,5) = (- 0.999999 *  c_q_xarmjoint2_);
    return *this;
}
template <typename TRAIT>
iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_xarmlink1_X_fr_xarmlink2::Type_fr_xarmlink1_X_fr_xarmlink2()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0.999999;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = - 3.6732E-6;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0.999999;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = - 3.6732E-6;
}
template <typename TRAIT>
const typename iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_xarmlink1_X_fr_xarmlink2& iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_xarmlink1_X_fr_xarmlink2::update(const JState& q) {
    Scalar s_q_xarmjoint2_;
    Scalar c_q_xarmjoint2_;
    
    s_q_xarmjoint2_ = TRAIT::sin( q(XARMJOINT2));
    c_q_xarmjoint2_ = TRAIT::cos( q(XARMJOINT2));
    
    (*this)(0,0) =  c_q_xarmjoint2_;
    (*this)(0,1) = - s_q_xarmjoint2_;
    (*this)(1,0) = (- 3.6732E-6 *  s_q_xarmjoint2_);
    (*this)(1,1) = (- 3.6732E-6 *  c_q_xarmjoint2_);
    (*this)(2,0) = (- 0.999999 *  s_q_xarmjoint2_);
    (*this)(2,1) = (- 0.999999 *  c_q_xarmjoint2_);
    (*this)(3,3) =  c_q_xarmjoint2_;
    (*this)(3,4) = - s_q_xarmjoint2_;
    (*this)(4,3) = (- 3.6732E-6 *  s_q_xarmjoint2_);
    (*this)(4,4) = (- 3.6732E-6 *  c_q_xarmjoint2_);
    (*this)(5,3) = (- 0.999999 *  s_q_xarmjoint2_);
    (*this)(5,4) = (- 0.999999 *  c_q_xarmjoint2_);
    return *this;
}
template <typename TRAIT>
iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_xarmlink3_X_fr_xarmlink2::Type_fr_xarmlink3_X_fr_xarmlink2()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = - 0.2845;
    (*this)(5,1) = - 0.0535;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_xarmlink3_X_fr_xarmlink2& iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_xarmlink3_X_fr_xarmlink2::update(const JState& q) {
    Scalar s_q_xarmjoint3_;
    Scalar c_q_xarmjoint3_;
    
    s_q_xarmjoint3_ = TRAIT::sin( q(XARMJOINT3));
    c_q_xarmjoint3_ = TRAIT::cos( q(XARMJOINT3));
    
    (*this)(0,0) =  c_q_xarmjoint3_;
    (*this)(0,1) =  s_q_xarmjoint3_;
    (*this)(1,0) = - s_q_xarmjoint3_;
    (*this)(1,1) =  c_q_xarmjoint3_;
    (*this)(3,2) = (( 0.0535 *  s_q_xarmjoint3_) + ( 0.2845 *  c_q_xarmjoint3_));
    (*this)(3,3) =  c_q_xarmjoint3_;
    (*this)(3,4) =  s_q_xarmjoint3_;
    (*this)(4,2) = (( 0.0535 *  c_q_xarmjoint3_) - ( 0.2845 *  s_q_xarmjoint3_));
    (*this)(4,3) = - s_q_xarmjoint3_;
    (*this)(4,4) =  c_q_xarmjoint3_;
    return *this;
}
template <typename TRAIT>
iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_xarmlink2_X_fr_xarmlink3::Type_fr_xarmlink2_X_fr_xarmlink3()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = - 0.2845;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = - 0.0535;
    (*this)(4,5) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_xarmlink2_X_fr_xarmlink3& iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_xarmlink2_X_fr_xarmlink3::update(const JState& q) {
    Scalar s_q_xarmjoint3_;
    Scalar c_q_xarmjoint3_;
    
    s_q_xarmjoint3_ = TRAIT::sin( q(XARMJOINT3));
    c_q_xarmjoint3_ = TRAIT::cos( q(XARMJOINT3));
    
    (*this)(0,0) =  c_q_xarmjoint3_;
    (*this)(0,1) = - s_q_xarmjoint3_;
    (*this)(1,0) =  s_q_xarmjoint3_;
    (*this)(1,1) =  c_q_xarmjoint3_;
    (*this)(3,3) =  c_q_xarmjoint3_;
    (*this)(3,4) = - s_q_xarmjoint3_;
    (*this)(4,3) =  s_q_xarmjoint3_;
    (*this)(4,4) =  c_q_xarmjoint3_;
    (*this)(5,0) = (( 0.0535 *  s_q_xarmjoint3_) + ( 0.2845 *  c_q_xarmjoint3_));
    (*this)(5,1) = (( 0.0535 *  c_q_xarmjoint3_) - ( 0.2845 *  s_q_xarmjoint3_));
    return *this;
}
template <typename TRAIT>
iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_xarmlink4_X_fr_xarmlink3::Type_fr_xarmlink4_X_fr_xarmlink3()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0.999999;
    (*this)(2,2) = - 3.6732E-6;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,0) = - 1.25807E-6;
    (*this)(5,1) = 2.84673E-7;
    (*this)(5,2) = 0.0774999;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0.999999;
    (*this)(5,5) = - 3.6732E-6;
}
template <typename TRAIT>
const typename iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_xarmlink4_X_fr_xarmlink3& iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_xarmlink4_X_fr_xarmlink3::update(const JState& q) {
    Scalar s_q_xarmjoint4_;
    Scalar c_q_xarmjoint4_;
    
    s_q_xarmjoint4_ = TRAIT::sin( q(XARMJOINT4));
    c_q_xarmjoint4_ = TRAIT::cos( q(XARMJOINT4));
    
    (*this)(0,0) =  c_q_xarmjoint4_;
    (*this)(0,1) = (- 3.6732E-6 *  s_q_xarmjoint4_);
    (*this)(0,2) = (- 0.999999 *  s_q_xarmjoint4_);
    (*this)(1,0) = - s_q_xarmjoint4_;
    (*this)(1,1) = (- 3.6732E-6 *  c_q_xarmjoint4_);
    (*this)(1,2) = (- 0.999999 *  c_q_xarmjoint4_);
    (*this)(3,0) = (- 0.342499 *  s_q_xarmjoint4_);
    (*this)(3,1) = ( 0.0774999 *  s_q_xarmjoint4_);
    (*this)(3,2) = ((- 2.84673E-7 *  s_q_xarmjoint4_) - ( 0.3425 *  c_q_xarmjoint4_));
    (*this)(3,3) =  c_q_xarmjoint4_;
    (*this)(3,4) = (- 3.6732E-6 *  s_q_xarmjoint4_);
    (*this)(3,5) = (- 0.999999 *  s_q_xarmjoint4_);
    (*this)(4,0) = (- 0.342499 *  c_q_xarmjoint4_);
    (*this)(4,1) = ( 0.0774999 *  c_q_xarmjoint4_);
    (*this)(4,2) = (( 0.3425 *  s_q_xarmjoint4_) - ( 2.84673E-7 *  c_q_xarmjoint4_));
    (*this)(4,3) = - s_q_xarmjoint4_;
    (*this)(4,4) = (- 3.6732E-6 *  c_q_xarmjoint4_);
    (*this)(4,5) = (- 0.999999 *  c_q_xarmjoint4_);
    return *this;
}
template <typename TRAIT>
iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_xarmlink3_X_fr_xarmlink4::Type_fr_xarmlink3_X_fr_xarmlink4()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0.999999;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = - 3.6732E-6;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,2) = - 1.25807E-6;
    (*this)(3,5) = 0;
    (*this)(4,2) = 2.84673E-7;
    (*this)(4,5) = 0.999999;
    (*this)(5,2) = 0.0774999;
    (*this)(5,5) = - 3.6732E-6;
}
template <typename TRAIT>
const typename iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_xarmlink3_X_fr_xarmlink4& iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_xarmlink3_X_fr_xarmlink4::update(const JState& q) {
    Scalar s_q_xarmjoint4_;
    Scalar c_q_xarmjoint4_;
    
    s_q_xarmjoint4_ = TRAIT::sin( q(XARMJOINT4));
    c_q_xarmjoint4_ = TRAIT::cos( q(XARMJOINT4));
    
    (*this)(0,0) =  c_q_xarmjoint4_;
    (*this)(0,1) = - s_q_xarmjoint4_;
    (*this)(1,0) = (- 3.6732E-6 *  s_q_xarmjoint4_);
    (*this)(1,1) = (- 3.6732E-6 *  c_q_xarmjoint4_);
    (*this)(2,0) = (- 0.999999 *  s_q_xarmjoint4_);
    (*this)(2,1) = (- 0.999999 *  c_q_xarmjoint4_);
    (*this)(3,0) = (- 0.342499 *  s_q_xarmjoint4_);
    (*this)(3,1) = (- 0.342499 *  c_q_xarmjoint4_);
    (*this)(3,3) =  c_q_xarmjoint4_;
    (*this)(3,4) = - s_q_xarmjoint4_;
    (*this)(4,0) = ( 0.0774999 *  s_q_xarmjoint4_);
    (*this)(4,1) = ( 0.0774999 *  c_q_xarmjoint4_);
    (*this)(4,3) = (- 3.6732E-6 *  s_q_xarmjoint4_);
    (*this)(4,4) = (- 3.6732E-6 *  c_q_xarmjoint4_);
    (*this)(5,0) = ((- 2.84673E-7 *  s_q_xarmjoint4_) - ( 0.3425 *  c_q_xarmjoint4_));
    (*this)(5,1) = (( 0.3425 *  s_q_xarmjoint4_) - ( 2.84673E-7 *  c_q_xarmjoint4_));
    (*this)(5,3) = (- 0.999999 *  s_q_xarmjoint4_);
    (*this)(5,4) = (- 0.999999 *  c_q_xarmjoint4_);
    return *this;
}
template <typename TRAIT>
iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_xarmlink5_X_fr_xarmlink4::Type_fr_xarmlink5_X_fr_xarmlink4()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 0.999999;
    (*this)(2,2) = - 3.6732E-6;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = - 0.999999;
    (*this)(5,5) = - 3.6732E-6;
}
template <typename TRAIT>
const typename iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_xarmlink5_X_fr_xarmlink4& iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_xarmlink5_X_fr_xarmlink4::update(const JState& q) {
    Scalar s_q_xarmjoint5_;
    Scalar c_q_xarmjoint5_;
    
    s_q_xarmjoint5_ = TRAIT::sin( q(XARMJOINT5));
    c_q_xarmjoint5_ = TRAIT::cos( q(XARMJOINT5));
    
    (*this)(0,0) =  c_q_xarmjoint5_;
    (*this)(0,1) = (- 3.6732E-6 *  s_q_xarmjoint5_);
    (*this)(0,2) = ( 0.999999 *  s_q_xarmjoint5_);
    (*this)(1,0) = - s_q_xarmjoint5_;
    (*this)(1,1) = (- 3.6732E-6 *  c_q_xarmjoint5_);
    (*this)(1,2) = ( 0.999999 *  c_q_xarmjoint5_);
    (*this)(3,3) =  c_q_xarmjoint5_;
    (*this)(3,4) = (- 3.6732E-6 *  s_q_xarmjoint5_);
    (*this)(3,5) = ( 0.999999 *  s_q_xarmjoint5_);
    (*this)(4,3) = - s_q_xarmjoint5_;
    (*this)(4,4) = (- 3.6732E-6 *  c_q_xarmjoint5_);
    (*this)(4,5) = ( 0.999999 *  c_q_xarmjoint5_);
    return *this;
}
template <typename TRAIT>
iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_xarmlink4_X_fr_xarmlink5::Type_fr_xarmlink4_X_fr_xarmlink5()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = - 0.999999;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = - 3.6732E-6;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = - 0.999999;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = - 3.6732E-6;
}
template <typename TRAIT>
const typename iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_xarmlink4_X_fr_xarmlink5& iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_xarmlink4_X_fr_xarmlink5::update(const JState& q) {
    Scalar s_q_xarmjoint5_;
    Scalar c_q_xarmjoint5_;
    
    s_q_xarmjoint5_ = TRAIT::sin( q(XARMJOINT5));
    c_q_xarmjoint5_ = TRAIT::cos( q(XARMJOINT5));
    
    (*this)(0,0) =  c_q_xarmjoint5_;
    (*this)(0,1) = - s_q_xarmjoint5_;
    (*this)(1,0) = (- 3.6732E-6 *  s_q_xarmjoint5_);
    (*this)(1,1) = (- 3.6732E-6 *  c_q_xarmjoint5_);
    (*this)(2,0) = ( 0.999999 *  s_q_xarmjoint5_);
    (*this)(2,1) = ( 0.999999 *  c_q_xarmjoint5_);
    (*this)(3,3) =  c_q_xarmjoint5_;
    (*this)(3,4) = - s_q_xarmjoint5_;
    (*this)(4,3) = (- 3.6732E-6 *  s_q_xarmjoint5_);
    (*this)(4,4) = (- 3.6732E-6 *  c_q_xarmjoint5_);
    (*this)(5,3) = ( 0.999999 *  s_q_xarmjoint5_);
    (*this)(5,4) = ( 0.999999 *  c_q_xarmjoint5_);
    return *this;
}
template <typename TRAIT>
iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_xarmlink6_X_fr_xarmlink5::Type_fr_xarmlink6_X_fr_xarmlink5()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0.999999;
    (*this)(2,2) = - 3.6732E-6;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(5,0) = - 3.563E-7;
    (*this)(5,1) = 2.79163E-7;
    (*this)(5,2) = 0.0759999;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0.999999;
    (*this)(5,5) = - 3.6732E-6;
}
template <typename TRAIT>
const typename iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_xarmlink6_X_fr_xarmlink5& iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_xarmlink6_X_fr_xarmlink5::update(const JState& q) {
    Scalar s_q_xarmjoint6_;
    Scalar c_q_xarmjoint6_;
    
    s_q_xarmjoint6_ = TRAIT::sin( q(XARMJOINT6));
    c_q_xarmjoint6_ = TRAIT::cos( q(XARMJOINT6));
    
    (*this)(0,0) =  c_q_xarmjoint6_;
    (*this)(0,1) = (- 3.6732E-6 *  s_q_xarmjoint6_);
    (*this)(0,2) = (- 0.999999 *  s_q_xarmjoint6_);
    (*this)(1,0) = - s_q_xarmjoint6_;
    (*this)(1,1) = (- 3.6732E-6 *  c_q_xarmjoint6_);
    (*this)(1,2) = (- 0.999999 *  c_q_xarmjoint6_);
    (*this)(3,0) = (- 0.0969999 *  s_q_xarmjoint6_);
    (*this)(3,1) = ( 0.0759999 *  s_q_xarmjoint6_);
    (*this)(3,2) = ((- 2.79163E-7 *  s_q_xarmjoint6_) - ( 0.097 *  c_q_xarmjoint6_));
    (*this)(3,3) =  c_q_xarmjoint6_;
    (*this)(3,4) = (- 3.6732E-6 *  s_q_xarmjoint6_);
    (*this)(3,5) = (- 0.999999 *  s_q_xarmjoint6_);
    (*this)(4,0) = (- 0.0969999 *  c_q_xarmjoint6_);
    (*this)(4,1) = ( 0.0759999 *  c_q_xarmjoint6_);
    (*this)(4,2) = (( 0.097 *  s_q_xarmjoint6_) - ( 2.79163E-7 *  c_q_xarmjoint6_));
    (*this)(4,3) = - s_q_xarmjoint6_;
    (*this)(4,4) = (- 3.6732E-6 *  c_q_xarmjoint6_);
    (*this)(4,5) = (- 0.999999 *  c_q_xarmjoint6_);
    return *this;
}
template <typename TRAIT>
iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_xarmlink5_X_fr_xarmlink6::Type_fr_xarmlink5_X_fr_xarmlink6()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0.999999;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = - 3.6732E-6;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,2) = - 3.563E-7;
    (*this)(3,5) = 0;
    (*this)(4,2) = 2.79163E-7;
    (*this)(4,5) = 0.999999;
    (*this)(5,2) = 0.0759999;
    (*this)(5,5) = - 3.6732E-6;
}
template <typename TRAIT>
const typename iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_xarmlink5_X_fr_xarmlink6& iit::asArm::tpl::MotionTransforms<TRAIT>::Type_fr_xarmlink5_X_fr_xarmlink6::update(const JState& q) {
    Scalar s_q_xarmjoint6_;
    Scalar c_q_xarmjoint6_;
    
    s_q_xarmjoint6_ = TRAIT::sin( q(XARMJOINT6));
    c_q_xarmjoint6_ = TRAIT::cos( q(XARMJOINT6));
    
    (*this)(0,0) =  c_q_xarmjoint6_;
    (*this)(0,1) = - s_q_xarmjoint6_;
    (*this)(1,0) = (- 3.6732E-6 *  s_q_xarmjoint6_);
    (*this)(1,1) = (- 3.6732E-6 *  c_q_xarmjoint6_);
    (*this)(2,0) = (- 0.999999 *  s_q_xarmjoint6_);
    (*this)(2,1) = (- 0.999999 *  c_q_xarmjoint6_);
    (*this)(3,0) = (- 0.0969999 *  s_q_xarmjoint6_);
    (*this)(3,1) = (- 0.0969999 *  c_q_xarmjoint6_);
    (*this)(3,3) =  c_q_xarmjoint6_;
    (*this)(3,4) = - s_q_xarmjoint6_;
    (*this)(4,0) = ( 0.0759999 *  s_q_xarmjoint6_);
    (*this)(4,1) = ( 0.0759999 *  c_q_xarmjoint6_);
    (*this)(4,3) = (- 3.6732E-6 *  s_q_xarmjoint6_);
    (*this)(4,4) = (- 3.6732E-6 *  c_q_xarmjoint6_);
    (*this)(5,0) = ((- 2.79163E-7 *  s_q_xarmjoint6_) - ( 0.097 *  c_q_xarmjoint6_));
    (*this)(5,1) = (( 0.097 *  s_q_xarmjoint6_) - ( 2.79163E-7 *  c_q_xarmjoint6_));
    (*this)(5,3) = (- 0.999999 *  s_q_xarmjoint6_);
    (*this)(5,4) = (- 0.999999 *  c_q_xarmjoint6_);
    return *this;
}

template <typename TRAIT>
iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_xarmlink1_X_fr_base_link::Type_fr_xarmlink1_X_fr_base_link()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(2,4) = - 0.175;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_xarmlink1_X_fr_base_link& iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_xarmlink1_X_fr_base_link::update(const JState& q) {
    Scalar s_q_xarmjoint1_;
    Scalar c_q_xarmjoint1_;
    
    s_q_xarmjoint1_ = TRAIT::sin( q(XARMJOINT1));
    c_q_xarmjoint1_ = TRAIT::cos( q(XARMJOINT1));
    
    (*this)(0,0) =  c_q_xarmjoint1_;
    (*this)(0,1) =  s_q_xarmjoint1_;
    (*this)(0,3) = (- 0.214 *  s_q_xarmjoint1_);
    (*this)(0,4) = ( 0.214 *  c_q_xarmjoint1_);
    (*this)(0,5) = ( 0.175 *  s_q_xarmjoint1_);
    (*this)(1,0) = - s_q_xarmjoint1_;
    (*this)(1,1) =  c_q_xarmjoint1_;
    (*this)(1,3) = (- 0.214 *  c_q_xarmjoint1_);
    (*this)(1,4) = (- 0.214 *  s_q_xarmjoint1_);
    (*this)(1,5) = ( 0.175 *  c_q_xarmjoint1_);
    (*this)(3,3) =  c_q_xarmjoint1_;
    (*this)(3,4) =  s_q_xarmjoint1_;
    (*this)(4,3) = - s_q_xarmjoint1_;
    (*this)(4,4) =  c_q_xarmjoint1_;
    return *this;
}
template <typename TRAIT>
iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_base_link_X_fr_xarmlink1::Type_fr_base_link_X_fr_xarmlink1()
{
    (*this)(0,2) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0;
    (*this)(1,5) = - 0.175;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_base_link_X_fr_xarmlink1& iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_base_link_X_fr_xarmlink1::update(const JState& q) {
    Scalar s_q_xarmjoint1_;
    Scalar c_q_xarmjoint1_;
    
    s_q_xarmjoint1_ = TRAIT::sin( q(XARMJOINT1));
    c_q_xarmjoint1_ = TRAIT::cos( q(XARMJOINT1));
    
    (*this)(0,0) =  c_q_xarmjoint1_;
    (*this)(0,1) = - s_q_xarmjoint1_;
    (*this)(0,3) = (- 0.214 *  s_q_xarmjoint1_);
    (*this)(0,4) = (- 0.214 *  c_q_xarmjoint1_);
    (*this)(1,0) =  s_q_xarmjoint1_;
    (*this)(1,1) =  c_q_xarmjoint1_;
    (*this)(1,3) = ( 0.214 *  c_q_xarmjoint1_);
    (*this)(1,4) = (- 0.214 *  s_q_xarmjoint1_);
    (*this)(2,3) = ( 0.175 *  s_q_xarmjoint1_);
    (*this)(2,4) = ( 0.175 *  c_q_xarmjoint1_);
    (*this)(3,3) =  c_q_xarmjoint1_;
    (*this)(3,4) = - s_q_xarmjoint1_;
    (*this)(4,3) =  s_q_xarmjoint1_;
    (*this)(4,4) =  c_q_xarmjoint1_;
    return *this;
}
template <typename TRAIT>
iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_xarmlink2_X_fr_xarmlink1::Type_fr_xarmlink2_X_fr_xarmlink1()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0.999999;
    (*this)(2,2) = - 3.6732E-6;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0.999999;
    (*this)(5,5) = - 3.6732E-6;
}
template <typename TRAIT>
const typename iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_xarmlink2_X_fr_xarmlink1& iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_xarmlink2_X_fr_xarmlink1::update(const JState& q) {
    Scalar s_q_xarmjoint2_;
    Scalar c_q_xarmjoint2_;
    
    s_q_xarmjoint2_ = TRAIT::sin( q(XARMJOINT2));
    c_q_xarmjoint2_ = TRAIT::cos( q(XARMJOINT2));
    
    (*this)(0,0) =  c_q_xarmjoint2_;
    (*this)(0,1) = (- 3.6732E-6 *  s_q_xarmjoint2_);
    (*this)(0,2) = (- 0.999999 *  s_q_xarmjoint2_);
    (*this)(1,0) = - s_q_xarmjoint2_;
    (*this)(1,1) = (- 3.6732E-6 *  c_q_xarmjoint2_);
    (*this)(1,2) = (- 0.999999 *  c_q_xarmjoint2_);
    (*this)(3,3) =  c_q_xarmjoint2_;
    (*this)(3,4) = (- 3.6732E-6 *  s_q_xarmjoint2_);
    (*this)(3,5) = (- 0.999999 *  s_q_xarmjoint2_);
    (*this)(4,3) = - s_q_xarmjoint2_;
    (*this)(4,4) = (- 3.6732E-6 *  c_q_xarmjoint2_);
    (*this)(4,5) = (- 0.999999 *  c_q_xarmjoint2_);
    return *this;
}
template <typename TRAIT>
iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_xarmlink1_X_fr_xarmlink2::Type_fr_xarmlink1_X_fr_xarmlink2()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = 0.999999;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = - 3.6732E-6;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0.999999;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = - 3.6732E-6;
}
template <typename TRAIT>
const typename iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_xarmlink1_X_fr_xarmlink2& iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_xarmlink1_X_fr_xarmlink2::update(const JState& q) {
    Scalar s_q_xarmjoint2_;
    Scalar c_q_xarmjoint2_;
    
    s_q_xarmjoint2_ = TRAIT::sin( q(XARMJOINT2));
    c_q_xarmjoint2_ = TRAIT::cos( q(XARMJOINT2));
    
    (*this)(0,0) =  c_q_xarmjoint2_;
    (*this)(0,1) = - s_q_xarmjoint2_;
    (*this)(1,0) = (- 3.6732E-6 *  s_q_xarmjoint2_);
    (*this)(1,1) = (- 3.6732E-6 *  c_q_xarmjoint2_);
    (*this)(2,0) = (- 0.999999 *  s_q_xarmjoint2_);
    (*this)(2,1) = (- 0.999999 *  c_q_xarmjoint2_);
    (*this)(3,3) =  c_q_xarmjoint2_;
    (*this)(3,4) = - s_q_xarmjoint2_;
    (*this)(4,3) = (- 3.6732E-6 *  s_q_xarmjoint2_);
    (*this)(4,4) = (- 3.6732E-6 *  c_q_xarmjoint2_);
    (*this)(5,3) = (- 0.999999 *  s_q_xarmjoint2_);
    (*this)(5,4) = (- 0.999999 *  c_q_xarmjoint2_);
    return *this;
}
template <typename TRAIT>
iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_xarmlink3_X_fr_xarmlink2::Type_fr_xarmlink3_X_fr_xarmlink2()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - 0.2845;
    (*this)(2,4) = - 0.0535;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_xarmlink3_X_fr_xarmlink2& iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_xarmlink3_X_fr_xarmlink2::update(const JState& q) {
    Scalar s_q_xarmjoint3_;
    Scalar c_q_xarmjoint3_;
    
    s_q_xarmjoint3_ = TRAIT::sin( q(XARMJOINT3));
    c_q_xarmjoint3_ = TRAIT::cos( q(XARMJOINT3));
    
    (*this)(0,0) =  c_q_xarmjoint3_;
    (*this)(0,1) =  s_q_xarmjoint3_;
    (*this)(0,5) = (( 0.0535 *  s_q_xarmjoint3_) + ( 0.2845 *  c_q_xarmjoint3_));
    (*this)(1,0) = - s_q_xarmjoint3_;
    (*this)(1,1) =  c_q_xarmjoint3_;
    (*this)(1,5) = (( 0.0535 *  c_q_xarmjoint3_) - ( 0.2845 *  s_q_xarmjoint3_));
    (*this)(3,3) =  c_q_xarmjoint3_;
    (*this)(3,4) =  s_q_xarmjoint3_;
    (*this)(4,3) = - s_q_xarmjoint3_;
    (*this)(4,4) =  c_q_xarmjoint3_;
    return *this;
}
template <typename TRAIT>
iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_xarmlink2_X_fr_xarmlink3::Type_fr_xarmlink2_X_fr_xarmlink3()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = - 0.2845;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = - 0.0535;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0;
    (*this)(5,5) = 1.0;
}
template <typename TRAIT>
const typename iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_xarmlink2_X_fr_xarmlink3& iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_xarmlink2_X_fr_xarmlink3::update(const JState& q) {
    Scalar s_q_xarmjoint3_;
    Scalar c_q_xarmjoint3_;
    
    s_q_xarmjoint3_ = TRAIT::sin( q(XARMJOINT3));
    c_q_xarmjoint3_ = TRAIT::cos( q(XARMJOINT3));
    
    (*this)(0,0) =  c_q_xarmjoint3_;
    (*this)(0,1) = - s_q_xarmjoint3_;
    (*this)(1,0) =  s_q_xarmjoint3_;
    (*this)(1,1) =  c_q_xarmjoint3_;
    (*this)(2,3) = (( 0.0535 *  s_q_xarmjoint3_) + ( 0.2845 *  c_q_xarmjoint3_));
    (*this)(2,4) = (( 0.0535 *  c_q_xarmjoint3_) - ( 0.2845 *  s_q_xarmjoint3_));
    (*this)(3,3) =  c_q_xarmjoint3_;
    (*this)(3,4) = - s_q_xarmjoint3_;
    (*this)(4,3) =  s_q_xarmjoint3_;
    (*this)(4,4) =  c_q_xarmjoint3_;
    return *this;
}
template <typename TRAIT>
iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_xarmlink4_X_fr_xarmlink3::Type_fr_xarmlink4_X_fr_xarmlink3()
{
    (*this)(2,0) = 0;
    (*this)(2,1) = 0.999999;
    (*this)(2,2) = - 3.6732E-6;
    (*this)(2,3) = - 1.25807E-6;
    (*this)(2,4) = 2.84673E-7;
    (*this)(2,5) = 0.0774999;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0.999999;
    (*this)(5,5) = - 3.6732E-6;
}
template <typename TRAIT>
const typename iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_xarmlink4_X_fr_xarmlink3& iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_xarmlink4_X_fr_xarmlink3::update(const JState& q) {
    Scalar s_q_xarmjoint4_;
    Scalar c_q_xarmjoint4_;
    
    s_q_xarmjoint4_ = TRAIT::sin( q(XARMJOINT4));
    c_q_xarmjoint4_ = TRAIT::cos( q(XARMJOINT4));
    
    (*this)(0,0) =  c_q_xarmjoint4_;
    (*this)(0,1) = (- 3.6732E-6 *  s_q_xarmjoint4_);
    (*this)(0,2) = (- 0.999999 *  s_q_xarmjoint4_);
    (*this)(0,3) = (- 0.342499 *  s_q_xarmjoint4_);
    (*this)(0,4) = ( 0.0774999 *  s_q_xarmjoint4_);
    (*this)(0,5) = ((- 2.84673E-7 *  s_q_xarmjoint4_) - ( 0.3425 *  c_q_xarmjoint4_));
    (*this)(1,0) = - s_q_xarmjoint4_;
    (*this)(1,1) = (- 3.6732E-6 *  c_q_xarmjoint4_);
    (*this)(1,2) = (- 0.999999 *  c_q_xarmjoint4_);
    (*this)(1,3) = (- 0.342499 *  c_q_xarmjoint4_);
    (*this)(1,4) = ( 0.0774999 *  c_q_xarmjoint4_);
    (*this)(1,5) = (( 0.3425 *  s_q_xarmjoint4_) - ( 2.84673E-7 *  c_q_xarmjoint4_));
    (*this)(3,3) =  c_q_xarmjoint4_;
    (*this)(3,4) = (- 3.6732E-6 *  s_q_xarmjoint4_);
    (*this)(3,5) = (- 0.999999 *  s_q_xarmjoint4_);
    (*this)(4,3) = - s_q_xarmjoint4_;
    (*this)(4,4) = (- 3.6732E-6 *  c_q_xarmjoint4_);
    (*this)(4,5) = (- 0.999999 *  c_q_xarmjoint4_);
    return *this;
}
template <typename TRAIT>
iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_xarmlink3_X_fr_xarmlink4::Type_fr_xarmlink3_X_fr_xarmlink4()
{
    (*this)(0,2) = 0;
    (*this)(0,5) = - 1.25807E-6;
    (*this)(1,2) = 0.999999;
    (*this)(1,5) = 2.84673E-7;
    (*this)(2,2) = - 3.6732E-6;
    (*this)(2,5) = 0.0774999;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0.999999;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = - 3.6732E-6;
}
template <typename TRAIT>
const typename iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_xarmlink3_X_fr_xarmlink4& iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_xarmlink3_X_fr_xarmlink4::update(const JState& q) {
    Scalar s_q_xarmjoint4_;
    Scalar c_q_xarmjoint4_;
    
    s_q_xarmjoint4_ = TRAIT::sin( q(XARMJOINT4));
    c_q_xarmjoint4_ = TRAIT::cos( q(XARMJOINT4));
    
    (*this)(0,0) =  c_q_xarmjoint4_;
    (*this)(0,1) = - s_q_xarmjoint4_;
    (*this)(0,3) = (- 0.342499 *  s_q_xarmjoint4_);
    (*this)(0,4) = (- 0.342499 *  c_q_xarmjoint4_);
    (*this)(1,0) = (- 3.6732E-6 *  s_q_xarmjoint4_);
    (*this)(1,1) = (- 3.6732E-6 *  c_q_xarmjoint4_);
    (*this)(1,3) = ( 0.0774999 *  s_q_xarmjoint4_);
    (*this)(1,4) = ( 0.0774999 *  c_q_xarmjoint4_);
    (*this)(2,0) = (- 0.999999 *  s_q_xarmjoint4_);
    (*this)(2,1) = (- 0.999999 *  c_q_xarmjoint4_);
    (*this)(2,3) = ((- 2.84673E-7 *  s_q_xarmjoint4_) - ( 0.3425 *  c_q_xarmjoint4_));
    (*this)(2,4) = (( 0.3425 *  s_q_xarmjoint4_) - ( 2.84673E-7 *  c_q_xarmjoint4_));
    (*this)(3,3) =  c_q_xarmjoint4_;
    (*this)(3,4) = - s_q_xarmjoint4_;
    (*this)(4,3) = (- 3.6732E-6 *  s_q_xarmjoint4_);
    (*this)(4,4) = (- 3.6732E-6 *  c_q_xarmjoint4_);
    (*this)(5,3) = (- 0.999999 *  s_q_xarmjoint4_);
    (*this)(5,4) = (- 0.999999 *  c_q_xarmjoint4_);
    return *this;
}
template <typename TRAIT>
iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_xarmlink5_X_fr_xarmlink4::Type_fr_xarmlink5_X_fr_xarmlink4()
{
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 0.999999;
    (*this)(2,2) = - 3.6732E-6;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = - 0.999999;
    (*this)(5,5) = - 3.6732E-6;
}
template <typename TRAIT>
const typename iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_xarmlink5_X_fr_xarmlink4& iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_xarmlink5_X_fr_xarmlink4::update(const JState& q) {
    Scalar s_q_xarmjoint5_;
    Scalar c_q_xarmjoint5_;
    
    s_q_xarmjoint5_ = TRAIT::sin( q(XARMJOINT5));
    c_q_xarmjoint5_ = TRAIT::cos( q(XARMJOINT5));
    
    (*this)(0,0) =  c_q_xarmjoint5_;
    (*this)(0,1) = (- 3.6732E-6 *  s_q_xarmjoint5_);
    (*this)(0,2) = ( 0.999999 *  s_q_xarmjoint5_);
    (*this)(1,0) = - s_q_xarmjoint5_;
    (*this)(1,1) = (- 3.6732E-6 *  c_q_xarmjoint5_);
    (*this)(1,2) = ( 0.999999 *  c_q_xarmjoint5_);
    (*this)(3,3) =  c_q_xarmjoint5_;
    (*this)(3,4) = (- 3.6732E-6 *  s_q_xarmjoint5_);
    (*this)(3,5) = ( 0.999999 *  s_q_xarmjoint5_);
    (*this)(4,3) = - s_q_xarmjoint5_;
    (*this)(4,4) = (- 3.6732E-6 *  c_q_xarmjoint5_);
    (*this)(4,5) = ( 0.999999 *  c_q_xarmjoint5_);
    return *this;
}
template <typename TRAIT>
iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_xarmlink4_X_fr_xarmlink5::Type_fr_xarmlink4_X_fr_xarmlink5()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(0,4) = 0;
    (*this)(0,5) = 0;
    (*this)(1,2) = - 0.999999;
    (*this)(1,3) = 0;
    (*this)(1,4) = 0;
    (*this)(1,5) = 0;
    (*this)(2,2) = - 3.6732E-6;
    (*this)(2,3) = 0;
    (*this)(2,4) = 0;
    (*this)(2,5) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = - 0.999999;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = - 3.6732E-6;
}
template <typename TRAIT>
const typename iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_xarmlink4_X_fr_xarmlink5& iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_xarmlink4_X_fr_xarmlink5::update(const JState& q) {
    Scalar s_q_xarmjoint5_;
    Scalar c_q_xarmjoint5_;
    
    s_q_xarmjoint5_ = TRAIT::sin( q(XARMJOINT5));
    c_q_xarmjoint5_ = TRAIT::cos( q(XARMJOINT5));
    
    (*this)(0,0) =  c_q_xarmjoint5_;
    (*this)(0,1) = - s_q_xarmjoint5_;
    (*this)(1,0) = (- 3.6732E-6 *  s_q_xarmjoint5_);
    (*this)(1,1) = (- 3.6732E-6 *  c_q_xarmjoint5_);
    (*this)(2,0) = ( 0.999999 *  s_q_xarmjoint5_);
    (*this)(2,1) = ( 0.999999 *  c_q_xarmjoint5_);
    (*this)(3,3) =  c_q_xarmjoint5_;
    (*this)(3,4) = - s_q_xarmjoint5_;
    (*this)(4,3) = (- 3.6732E-6 *  s_q_xarmjoint5_);
    (*this)(4,4) = (- 3.6732E-6 *  c_q_xarmjoint5_);
    (*this)(5,3) = ( 0.999999 *  s_q_xarmjoint5_);
    (*this)(5,4) = ( 0.999999 *  c_q_xarmjoint5_);
    return *this;
}
template <typename TRAIT>
iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_xarmlink6_X_fr_xarmlink5::Type_fr_xarmlink6_X_fr_xarmlink5()
{
    (*this)(2,0) = 0;
    (*this)(2,1) = 0.999999;
    (*this)(2,2) = - 3.6732E-6;
    (*this)(2,3) = - 3.563E-7;
    (*this)(2,4) = 2.79163E-7;
    (*this)(2,5) = 0.0759999;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,3) = 0;
    (*this)(5,4) = 0.999999;
    (*this)(5,5) = - 3.6732E-6;
}
template <typename TRAIT>
const typename iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_xarmlink6_X_fr_xarmlink5& iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_xarmlink6_X_fr_xarmlink5::update(const JState& q) {
    Scalar s_q_xarmjoint6_;
    Scalar c_q_xarmjoint6_;
    
    s_q_xarmjoint6_ = TRAIT::sin( q(XARMJOINT6));
    c_q_xarmjoint6_ = TRAIT::cos( q(XARMJOINT6));
    
    (*this)(0,0) =  c_q_xarmjoint6_;
    (*this)(0,1) = (- 3.6732E-6 *  s_q_xarmjoint6_);
    (*this)(0,2) = (- 0.999999 *  s_q_xarmjoint6_);
    (*this)(0,3) = (- 0.0969999 *  s_q_xarmjoint6_);
    (*this)(0,4) = ( 0.0759999 *  s_q_xarmjoint6_);
    (*this)(0,5) = ((- 2.79163E-7 *  s_q_xarmjoint6_) - ( 0.097 *  c_q_xarmjoint6_));
    (*this)(1,0) = - s_q_xarmjoint6_;
    (*this)(1,1) = (- 3.6732E-6 *  c_q_xarmjoint6_);
    (*this)(1,2) = (- 0.999999 *  c_q_xarmjoint6_);
    (*this)(1,3) = (- 0.0969999 *  c_q_xarmjoint6_);
    (*this)(1,4) = ( 0.0759999 *  c_q_xarmjoint6_);
    (*this)(1,5) = (( 0.097 *  s_q_xarmjoint6_) - ( 2.79163E-7 *  c_q_xarmjoint6_));
    (*this)(3,3) =  c_q_xarmjoint6_;
    (*this)(3,4) = (- 3.6732E-6 *  s_q_xarmjoint6_);
    (*this)(3,5) = (- 0.999999 *  s_q_xarmjoint6_);
    (*this)(4,3) = - s_q_xarmjoint6_;
    (*this)(4,4) = (- 3.6732E-6 *  c_q_xarmjoint6_);
    (*this)(4,5) = (- 0.999999 *  c_q_xarmjoint6_);
    return *this;
}
template <typename TRAIT>
iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_xarmlink5_X_fr_xarmlink6::Type_fr_xarmlink5_X_fr_xarmlink6()
{
    (*this)(0,2) = 0;
    (*this)(0,5) = - 3.563E-7;
    (*this)(1,2) = 0.999999;
    (*this)(1,5) = 2.79163E-7;
    (*this)(2,2) = - 3.6732E-6;
    (*this)(2,5) = 0.0759999;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,5) = 0;
    (*this)(4,0) = 0;
    (*this)(4,1) = 0;
    (*this)(4,2) = 0;
    (*this)(4,5) = 0.999999;
    (*this)(5,0) = 0;
    (*this)(5,1) = 0;
    (*this)(5,2) = 0;
    (*this)(5,5) = - 3.6732E-6;
}
template <typename TRAIT>
const typename iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_xarmlink5_X_fr_xarmlink6& iit::asArm::tpl::ForceTransforms<TRAIT>::Type_fr_xarmlink5_X_fr_xarmlink6::update(const JState& q) {
    Scalar s_q_xarmjoint6_;
    Scalar c_q_xarmjoint6_;
    
    s_q_xarmjoint6_ = TRAIT::sin( q(XARMJOINT6));
    c_q_xarmjoint6_ = TRAIT::cos( q(XARMJOINT6));
    
    (*this)(0,0) =  c_q_xarmjoint6_;
    (*this)(0,1) = - s_q_xarmjoint6_;
    (*this)(0,3) = (- 0.0969999 *  s_q_xarmjoint6_);
    (*this)(0,4) = (- 0.0969999 *  c_q_xarmjoint6_);
    (*this)(1,0) = (- 3.6732E-6 *  s_q_xarmjoint6_);
    (*this)(1,1) = (- 3.6732E-6 *  c_q_xarmjoint6_);
    (*this)(1,3) = ( 0.0759999 *  s_q_xarmjoint6_);
    (*this)(1,4) = ( 0.0759999 *  c_q_xarmjoint6_);
    (*this)(2,0) = (- 0.999999 *  s_q_xarmjoint6_);
    (*this)(2,1) = (- 0.999999 *  c_q_xarmjoint6_);
    (*this)(2,3) = ((- 2.79163E-7 *  s_q_xarmjoint6_) - ( 0.097 *  c_q_xarmjoint6_));
    (*this)(2,4) = (( 0.097 *  s_q_xarmjoint6_) - ( 2.79163E-7 *  c_q_xarmjoint6_));
    (*this)(3,3) =  c_q_xarmjoint6_;
    (*this)(3,4) = - s_q_xarmjoint6_;
    (*this)(4,3) = (- 3.6732E-6 *  s_q_xarmjoint6_);
    (*this)(4,4) = (- 3.6732E-6 *  c_q_xarmjoint6_);
    (*this)(5,3) = (- 0.999999 *  s_q_xarmjoint6_);
    (*this)(5,4) = (- 0.999999 *  c_q_xarmjoint6_);
    return *this;
}

template <typename TRAIT>
iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_xarmlink1_X_fr_base_link::Type_fr_xarmlink1_X_fr_base_link()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - 0.214;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_xarmlink1_X_fr_base_link& iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_xarmlink1_X_fr_base_link::update(const JState& q) {
    Scalar s_q_xarmjoint1_;
    Scalar c_q_xarmjoint1_;
    
    s_q_xarmjoint1_ = TRAIT::sin( q(XARMJOINT1));
    c_q_xarmjoint1_ = TRAIT::cos( q(XARMJOINT1));
    
    (*this)(0,0) =  c_q_xarmjoint1_;
    (*this)(0,1) =  s_q_xarmjoint1_;
    (*this)(0,3) = (- 0.175 *  c_q_xarmjoint1_);
    (*this)(1,0) = - s_q_xarmjoint1_;
    (*this)(1,1) =  c_q_xarmjoint1_;
    (*this)(1,3) = ( 0.175 *  s_q_xarmjoint1_);
    return *this;
}
template <typename TRAIT>
iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_link_X_fr_xarmlink1::Type_fr_base_link_X_fr_xarmlink1()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.175;
    (*this)(1,2) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.214;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_link_X_fr_xarmlink1& iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_base_link_X_fr_xarmlink1::update(const JState& q) {
    Scalar s_q_xarmjoint1_;
    Scalar c_q_xarmjoint1_;
    
    s_q_xarmjoint1_ = TRAIT::sin( q(XARMJOINT1));
    c_q_xarmjoint1_ = TRAIT::cos( q(XARMJOINT1));
    
    (*this)(0,0) =  c_q_xarmjoint1_;
    (*this)(0,1) = - s_q_xarmjoint1_;
    (*this)(1,0) =  s_q_xarmjoint1_;
    (*this)(1,1) =  c_q_xarmjoint1_;
    return *this;
}
template <typename TRAIT>
iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_xarmlink2_X_fr_xarmlink1::Type_fr_xarmlink2_X_fr_xarmlink1()
{
    (*this)(0,3) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0.999999;
    (*this)(2,2) = - 3.6732E-6;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_xarmlink2_X_fr_xarmlink1& iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_xarmlink2_X_fr_xarmlink1::update(const JState& q) {
    Scalar s_q_xarmjoint2_;
    Scalar c_q_xarmjoint2_;
    
    s_q_xarmjoint2_ = TRAIT::sin( q(XARMJOINT2));
    c_q_xarmjoint2_ = TRAIT::cos( q(XARMJOINT2));
    
    (*this)(0,0) =  c_q_xarmjoint2_;
    (*this)(0,1) = (- 3.6732E-6 *  s_q_xarmjoint2_);
    (*this)(0,2) = (- 0.999999 *  s_q_xarmjoint2_);
    (*this)(1,0) = - s_q_xarmjoint2_;
    (*this)(1,1) = (- 3.6732E-6 *  c_q_xarmjoint2_);
    (*this)(1,2) = (- 0.999999 *  c_q_xarmjoint2_);
    return *this;
}
template <typename TRAIT>
iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_xarmlink1_X_fr_xarmlink2::Type_fr_xarmlink1_X_fr_xarmlink2()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,2) = 0.999999;
    (*this)(1,3) = 0;
    (*this)(2,2) = - 3.6732E-6;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_xarmlink1_X_fr_xarmlink2& iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_xarmlink1_X_fr_xarmlink2::update(const JState& q) {
    Scalar s_q_xarmjoint2_;
    Scalar c_q_xarmjoint2_;
    
    s_q_xarmjoint2_ = TRAIT::sin( q(XARMJOINT2));
    c_q_xarmjoint2_ = TRAIT::cos( q(XARMJOINT2));
    
    (*this)(0,0) =  c_q_xarmjoint2_;
    (*this)(0,1) = - s_q_xarmjoint2_;
    (*this)(1,0) = (- 3.6732E-6 *  s_q_xarmjoint2_);
    (*this)(1,1) = (- 3.6732E-6 *  c_q_xarmjoint2_);
    (*this)(2,0) = (- 0.999999 *  s_q_xarmjoint2_);
    (*this)(2,1) = (- 0.999999 *  c_q_xarmjoint2_);
    return *this;
}
template <typename TRAIT>
iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_xarmlink3_X_fr_xarmlink2::Type_fr_xarmlink3_X_fr_xarmlink2()
{
    (*this)(0,2) = 0;
    (*this)(1,2) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_xarmlink3_X_fr_xarmlink2& iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_xarmlink3_X_fr_xarmlink2::update(const JState& q) {
    Scalar s_q_xarmjoint3_;
    Scalar c_q_xarmjoint3_;
    
    s_q_xarmjoint3_ = TRAIT::sin( q(XARMJOINT3));
    c_q_xarmjoint3_ = TRAIT::cos( q(XARMJOINT3));
    
    (*this)(0,0) =  c_q_xarmjoint3_;
    (*this)(0,1) =  s_q_xarmjoint3_;
    (*this)(0,3) = (( 0.2845 *  s_q_xarmjoint3_) - ( 0.0535 *  c_q_xarmjoint3_));
    (*this)(1,0) = - s_q_xarmjoint3_;
    (*this)(1,1) =  c_q_xarmjoint3_;
    (*this)(1,3) = (( 0.0535 *  s_q_xarmjoint3_) + ( 0.2845 *  c_q_xarmjoint3_));
    return *this;
}
template <typename TRAIT>
iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_xarmlink2_X_fr_xarmlink3::Type_fr_xarmlink2_X_fr_xarmlink3()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.0535;
    (*this)(1,2) = 0;
    (*this)(1,3) = - 0.2845;
    (*this)(2,0) = 0;
    (*this)(2,1) = 0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_xarmlink2_X_fr_xarmlink3& iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_xarmlink2_X_fr_xarmlink3::update(const JState& q) {
    Scalar s_q_xarmjoint3_;
    Scalar c_q_xarmjoint3_;
    
    s_q_xarmjoint3_ = TRAIT::sin( q(XARMJOINT3));
    c_q_xarmjoint3_ = TRAIT::cos( q(XARMJOINT3));
    
    (*this)(0,0) =  c_q_xarmjoint3_;
    (*this)(0,1) = - s_q_xarmjoint3_;
    (*this)(1,0) =  s_q_xarmjoint3_;
    (*this)(1,1) =  c_q_xarmjoint3_;
    return *this;
}
template <typename TRAIT>
iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_xarmlink4_X_fr_xarmlink3::Type_fr_xarmlink4_X_fr_xarmlink3()
{
    (*this)(2,0) = 0;
    (*this)(2,1) = 0.999999;
    (*this)(2,2) = - 3.6732E-6;
    (*this)(2,3) = - 0.342499;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_xarmlink4_X_fr_xarmlink3& iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_xarmlink4_X_fr_xarmlink3::update(const JState& q) {
    Scalar s_q_xarmjoint4_;
    Scalar c_q_xarmjoint4_;
    
    s_q_xarmjoint4_ = TRAIT::sin( q(XARMJOINT4));
    c_q_xarmjoint4_ = TRAIT::cos( q(XARMJOINT4));
    
    (*this)(0,0) =  c_q_xarmjoint4_;
    (*this)(0,1) = (- 3.6732E-6 *  s_q_xarmjoint4_);
    (*this)(0,2) = (- 0.999999 *  s_q_xarmjoint4_);
    (*this)(0,3) = (( 1.25807E-6 *  s_q_xarmjoint4_) - ( 0.0775 *  c_q_xarmjoint4_));
    (*this)(1,0) = - s_q_xarmjoint4_;
    (*this)(1,1) = (- 3.6732E-6 *  c_q_xarmjoint4_);
    (*this)(1,2) = (- 0.999999 *  c_q_xarmjoint4_);
    (*this)(1,3) = (( 0.0775 *  s_q_xarmjoint4_) + ( 1.25807E-6 *  c_q_xarmjoint4_));
    return *this;
}
template <typename TRAIT>
iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_xarmlink3_X_fr_xarmlink4::Type_fr_xarmlink3_X_fr_xarmlink4()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.0775;
    (*this)(1,2) = 0.999999;
    (*this)(1,3) = 0.3425;
    (*this)(2,2) = - 3.6732E-6;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_xarmlink3_X_fr_xarmlink4& iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_xarmlink3_X_fr_xarmlink4::update(const JState& q) {
    Scalar s_q_xarmjoint4_;
    Scalar c_q_xarmjoint4_;
    
    s_q_xarmjoint4_ = TRAIT::sin( q(XARMJOINT4));
    c_q_xarmjoint4_ = TRAIT::cos( q(XARMJOINT4));
    
    (*this)(0,0) =  c_q_xarmjoint4_;
    (*this)(0,1) = - s_q_xarmjoint4_;
    (*this)(1,0) = (- 3.6732E-6 *  s_q_xarmjoint4_);
    (*this)(1,1) = (- 3.6732E-6 *  c_q_xarmjoint4_);
    (*this)(2,0) = (- 0.999999 *  s_q_xarmjoint4_);
    (*this)(2,1) = (- 0.999999 *  c_q_xarmjoint4_);
    return *this;
}
template <typename TRAIT>
iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_xarmlink5_X_fr_xarmlink4::Type_fr_xarmlink5_X_fr_xarmlink4()
{
    (*this)(0,3) = 0;
    (*this)(1,3) = 0;
    (*this)(2,0) = 0;
    (*this)(2,1) = - 0.999999;
    (*this)(2,2) = - 3.6732E-6;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_xarmlink5_X_fr_xarmlink4& iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_xarmlink5_X_fr_xarmlink4::update(const JState& q) {
    Scalar s_q_xarmjoint5_;
    Scalar c_q_xarmjoint5_;
    
    s_q_xarmjoint5_ = TRAIT::sin( q(XARMJOINT5));
    c_q_xarmjoint5_ = TRAIT::cos( q(XARMJOINT5));
    
    (*this)(0,0) =  c_q_xarmjoint5_;
    (*this)(0,1) = (- 3.6732E-6 *  s_q_xarmjoint5_);
    (*this)(0,2) = ( 0.999999 *  s_q_xarmjoint5_);
    (*this)(1,0) = - s_q_xarmjoint5_;
    (*this)(1,1) = (- 3.6732E-6 *  c_q_xarmjoint5_);
    (*this)(1,2) = ( 0.999999 *  c_q_xarmjoint5_);
    return *this;
}
template <typename TRAIT>
iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_xarmlink4_X_fr_xarmlink5::Type_fr_xarmlink4_X_fr_xarmlink5()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0;
    (*this)(1,2) = - 0.999999;
    (*this)(1,3) = 0;
    (*this)(2,2) = - 3.6732E-6;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1;
}
template <typename TRAIT>
const typename iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_xarmlink4_X_fr_xarmlink5& iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_xarmlink4_X_fr_xarmlink5::update(const JState& q) {
    Scalar s_q_xarmjoint5_;
    Scalar c_q_xarmjoint5_;
    
    s_q_xarmjoint5_ = TRAIT::sin( q(XARMJOINT5));
    c_q_xarmjoint5_ = TRAIT::cos( q(XARMJOINT5));
    
    (*this)(0,0) =  c_q_xarmjoint5_;
    (*this)(0,1) = - s_q_xarmjoint5_;
    (*this)(1,0) = (- 3.6732E-6 *  s_q_xarmjoint5_);
    (*this)(1,1) = (- 3.6732E-6 *  c_q_xarmjoint5_);
    (*this)(2,0) = ( 0.999999 *  s_q_xarmjoint5_);
    (*this)(2,1) = ( 0.999999 *  c_q_xarmjoint5_);
    return *this;
}
template <typename TRAIT>
iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_xarmlink6_X_fr_xarmlink5::Type_fr_xarmlink6_X_fr_xarmlink5()
{
    (*this)(2,0) = 0;
    (*this)(2,1) = 0.999999;
    (*this)(2,2) = - 3.6732E-6;
    (*this)(2,3) = - 0.0969999;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_xarmlink6_X_fr_xarmlink5& iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_xarmlink6_X_fr_xarmlink5::update(const JState& q) {
    Scalar s_q_xarmjoint6_;
    Scalar c_q_xarmjoint6_;
    
    s_q_xarmjoint6_ = TRAIT::sin( q(XARMJOINT6));
    c_q_xarmjoint6_ = TRAIT::cos( q(XARMJOINT6));
    
    (*this)(0,0) =  c_q_xarmjoint6_;
    (*this)(0,1) = (- 3.6732E-6 *  s_q_xarmjoint6_);
    (*this)(0,2) = (- 0.999999 *  s_q_xarmjoint6_);
    (*this)(0,3) = (( 3.563E-7 *  s_q_xarmjoint6_) - ( 0.076 *  c_q_xarmjoint6_));
    (*this)(1,0) = - s_q_xarmjoint6_;
    (*this)(1,1) = (- 3.6732E-6 *  c_q_xarmjoint6_);
    (*this)(1,2) = (- 0.999999 *  c_q_xarmjoint6_);
    (*this)(1,3) = (( 0.076 *  s_q_xarmjoint6_) + ( 3.563E-7 *  c_q_xarmjoint6_));
    return *this;
}
template <typename TRAIT>
iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_xarmlink5_X_fr_xarmlink6::Type_fr_xarmlink5_X_fr_xarmlink6()
{
    (*this)(0,2) = 0;
    (*this)(0,3) = 0.076;
    (*this)(1,2) = 0.999999;
    (*this)(1,3) = 0.097;
    (*this)(2,2) = - 3.6732E-6;
    (*this)(2,3) = 0;
    (*this)(3,0) = 0;
    (*this)(3,1) = 0;
    (*this)(3,2) = 0;
    (*this)(3,3) = 1.0;
}
template <typename TRAIT>
const typename iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_xarmlink5_X_fr_xarmlink6& iit::asArm::tpl::HomogeneousTransforms<TRAIT>::Type_fr_xarmlink5_X_fr_xarmlink6::update(const JState& q) {
    Scalar s_q_xarmjoint6_;
    Scalar c_q_xarmjoint6_;
    
    s_q_xarmjoint6_ = TRAIT::sin( q(XARMJOINT6));
    c_q_xarmjoint6_ = TRAIT::cos( q(XARMJOINT6));
    
    (*this)(0,0) =  c_q_xarmjoint6_;
    (*this)(0,1) = - s_q_xarmjoint6_;
    (*this)(1,0) = (- 3.6732E-6 *  s_q_xarmjoint6_);
    (*this)(1,1) = (- 3.6732E-6 *  c_q_xarmjoint6_);
    (*this)(2,0) = (- 0.999999 *  s_q_xarmjoint6_);
    (*this)(2,1) = (- 0.999999 *  c_q_xarmjoint6_);
    return *this;
}

