#include "transforms.h"

using namespace armstone::rcg;

// Constructors

MotionTransforms::MotionTransforms()
 :     fr_omniwheel_fl_X_fr_base_link_footprint(),
    fr_base_link_footprint_X_fr_omniwheel_fl(),
    fr_omniwheel_fr_X_fr_base_link_footprint(),
    fr_base_link_footprint_X_fr_omniwheel_fr(),
    fr_omniwheel_bl_X_fr_base_link_footprint(),
    fr_base_link_footprint_X_fr_omniwheel_bl(),
    fr_omniwheel_br_X_fr_base_link_footprint(),
    fr_base_link_footprint_X_fr_omniwheel_br(),
    fr_xarmlink1_X_fr_base_link_footprint(),
    fr_base_link_footprint_X_fr_xarmlink1(),
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
{}
void MotionTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles)
{
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

ForceTransforms::ForceTransforms()
 :     fr_omniwheel_fl_X_fr_base_link_footprint(),
    fr_base_link_footprint_X_fr_omniwheel_fl(),
    fr_omniwheel_fr_X_fr_base_link_footprint(),
    fr_base_link_footprint_X_fr_omniwheel_fr(),
    fr_omniwheel_bl_X_fr_base_link_footprint(),
    fr_base_link_footprint_X_fr_omniwheel_bl(),
    fr_omniwheel_br_X_fr_base_link_footprint(),
    fr_base_link_footprint_X_fr_omniwheel_br(),
    fr_xarmlink1_X_fr_base_link_footprint(),
    fr_base_link_footprint_X_fr_xarmlink1(),
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
{}
void ForceTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles)
{
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

HomogeneousTransforms::HomogeneousTransforms()
 :     fr_omniwheel_fl_X_fr_base_link_footprint(),
    fr_base_link_footprint_X_fr_omniwheel_fl(),
    fr_omniwheel_fr_X_fr_base_link_footprint(),
    fr_base_link_footprint_X_fr_omniwheel_fr(),
    fr_omniwheel_bl_X_fr_base_link_footprint(),
    fr_base_link_footprint_X_fr_omniwheel_bl(),
    fr_omniwheel_br_X_fr_base_link_footprint(),
    fr_base_link_footprint_X_fr_omniwheel_br(),
    fr_xarmlink1_X_fr_base_link_footprint(),
    fr_base_link_footprint_X_fr_xarmlink1(),
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
{}
void HomogeneousTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles)
{
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

MotionTransforms::Type_fr_omniwheel_fl_X_fr_base_link_footprint::Type_fr_omniwheel_fl_X_fr_base_link_footprint()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(2,1) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,0) = - 0.707106 *  tz_motor_wheel_joint_fl;    // Maxima DSL: -0.707106*_k__tz_motor_wheel_joint_fl
    (*this)(5,1) =  0.707106 *  tz_motor_wheel_joint_fl;    // Maxima DSL: 0.707106*_k__tz_motor_wheel_joint_fl
    (*this)(5,2) = - 0.5 * (( 1.41421 *  ty_motor_wheel_joint_fl)-( 1.41421 *  tx_motor_wheel_joint_fl));    // Maxima DSL: -0.5*(1.41421*_k__ty_motor_wheel_joint_fl-1.41421*_k__tx_motor_wheel_joint_fl)
    (*this)(5,3) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(5,4) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_omniwheel_fl_X_fr_base_link_footprint& MotionTransforms::Type_fr_omniwheel_fl_X_fr_base_link_footprint::update(const state_t& q)
{
    Scalar sin_q_motor_wheel_joint_fl  = ScalarTraits::sin( q(MOTOR_WHEEL_JOINT_FL) );
    Scalar cos_q_motor_wheel_joint_fl  = ScalarTraits::cos( q(MOTOR_WHEEL_JOINT_FL) );
    (*this)(0,0) = - 0.5 * (sin_q_motor_wheel_joint_fl-cos_q_motor_wheel_joint_fl);
    (*this)(0,1) =  0.5 * (sin_q_motor_wheel_joint_fl-cos_q_motor_wheel_joint_fl);
    (*this)(0,2) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fl)+( 1.41421 * cos_q_motor_wheel_joint_fl));
    (*this)(1,0) = - 0.5 * (sin_q_motor_wheel_joint_fl+cos_q_motor_wheel_joint_fl);
    (*this)(1,1) =  0.5 * (sin_q_motor_wheel_joint_fl+cos_q_motor_wheel_joint_fl);
    (*this)(1,2) =  0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fl)-( 1.41421 * cos_q_motor_wheel_joint_fl));
    (*this)(3,0) = - 0.5 * ((( tz_motor_wheel_joint_fl+( 1.41421 *  ty_motor_wheel_joint_fl)) * sin_q_motor_wheel_joint_fl)+((( 1.41421 *  ty_motor_wheel_joint_fl)- tz_motor_wheel_joint_fl) * cos_q_motor_wheel_joint_fl));
    (*this)(3,1) = - 0.5 * ((( tz_motor_wheel_joint_fl-( 1.41421 *  tx_motor_wheel_joint_fl)) * sin_q_motor_wheel_joint_fl)+((- tz_motor_wheel_joint_fl-( 1.41421 *  tx_motor_wheel_joint_fl)) * cos_q_motor_wheel_joint_fl));
    (*this)(3,2) =  0.5 * ((( ty_motor_wheel_joint_fl+ tx_motor_wheel_joint_fl) * sin_q_motor_wheel_joint_fl)+((- ty_motor_wheel_joint_fl- tx_motor_wheel_joint_fl) * cos_q_motor_wheel_joint_fl));
    (*this)(3,3) = - 0.5 * (sin_q_motor_wheel_joint_fl-cos_q_motor_wheel_joint_fl);
    (*this)(3,4) =  0.5 * (sin_q_motor_wheel_joint_fl-cos_q_motor_wheel_joint_fl);
    (*this)(3,5) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fl)+( 1.41421 * cos_q_motor_wheel_joint_fl));
    (*this)(4,0) = - 0.5 * ((( tz_motor_wheel_joint_fl-( 1.41421 *  ty_motor_wheel_joint_fl)) * sin_q_motor_wheel_joint_fl)+(( tz_motor_wheel_joint_fl+( 1.41421 *  ty_motor_wheel_joint_fl)) * cos_q_motor_wheel_joint_fl));
    (*this)(4,1) = - 0.5 * ((( tz_motor_wheel_joint_fl+( 1.41421 *  tx_motor_wheel_joint_fl)) * sin_q_motor_wheel_joint_fl)+(( tz_motor_wheel_joint_fl-( 1.41421 *  tx_motor_wheel_joint_fl)) * cos_q_motor_wheel_joint_fl));
    (*this)(4,2) =  0.5 * ((( ty_motor_wheel_joint_fl+ tx_motor_wheel_joint_fl) * sin_q_motor_wheel_joint_fl)+(( ty_motor_wheel_joint_fl+ tx_motor_wheel_joint_fl) * cos_q_motor_wheel_joint_fl));
    (*this)(4,3) = - 0.5 * (sin_q_motor_wheel_joint_fl+cos_q_motor_wheel_joint_fl);
    (*this)(4,4) =  0.5 * (sin_q_motor_wheel_joint_fl+cos_q_motor_wheel_joint_fl);
    (*this)(4,5) =  0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fl)-( 1.41421 * cos_q_motor_wheel_joint_fl));
    return *this;
}
MotionTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_fl::Type_fr_base_link_footprint_X_fr_omniwheel_fl()
{
    (*this)(0,2) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = - 0.707106 *  tz_motor_wheel_joint_fl;    // Maxima DSL: -0.707106*_k__tz_motor_wheel_joint_fl
    (*this)(3,5) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(4,2) =  0.707106 *  tz_motor_wheel_joint_fl;    // Maxima DSL: 0.707106*_k__tz_motor_wheel_joint_fl
    (*this)(4,5) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(5,2) = - 0.5 * (( 1.41421 *  ty_motor_wheel_joint_fl)-( 1.41421 *  tx_motor_wheel_joint_fl));    // Maxima DSL: -0.5*(1.41421*_k__ty_motor_wheel_joint_fl-1.41421*_k__tx_motor_wheel_joint_fl)
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_fl& MotionTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_fl::update(const state_t& q)
{
    Scalar sin_q_motor_wheel_joint_fl  = ScalarTraits::sin( q(MOTOR_WHEEL_JOINT_FL) );
    Scalar cos_q_motor_wheel_joint_fl  = ScalarTraits::cos( q(MOTOR_WHEEL_JOINT_FL) );
    (*this)(0,0) = - 0.5 * (sin_q_motor_wheel_joint_fl-cos_q_motor_wheel_joint_fl);
    (*this)(0,1) = - 0.5 * (sin_q_motor_wheel_joint_fl+cos_q_motor_wheel_joint_fl);
    (*this)(1,0) =  0.5 * (sin_q_motor_wheel_joint_fl-cos_q_motor_wheel_joint_fl);
    (*this)(1,1) =  0.5 * (sin_q_motor_wheel_joint_fl+cos_q_motor_wheel_joint_fl);
    (*this)(2,0) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fl)+( 1.41421 * cos_q_motor_wheel_joint_fl));
    (*this)(2,1) =  0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fl)-( 1.41421 * cos_q_motor_wheel_joint_fl));
    (*this)(3,0) = - 0.5 * ((( tz_motor_wheel_joint_fl+( 1.41421 *  ty_motor_wheel_joint_fl)) * sin_q_motor_wheel_joint_fl)+((( 1.41421 *  ty_motor_wheel_joint_fl)- tz_motor_wheel_joint_fl) * cos_q_motor_wheel_joint_fl));
    (*this)(3,1) = - 0.5 * ((( tz_motor_wheel_joint_fl-( 1.41421 *  ty_motor_wheel_joint_fl)) * sin_q_motor_wheel_joint_fl)+(( tz_motor_wheel_joint_fl+( 1.41421 *  ty_motor_wheel_joint_fl)) * cos_q_motor_wheel_joint_fl));
    (*this)(3,3) = - 0.5 * (sin_q_motor_wheel_joint_fl-cos_q_motor_wheel_joint_fl);
    (*this)(3,4) = - 0.5 * (sin_q_motor_wheel_joint_fl+cos_q_motor_wheel_joint_fl);
    (*this)(4,0) = - 0.5 * ((( tz_motor_wheel_joint_fl-( 1.41421 *  tx_motor_wheel_joint_fl)) * sin_q_motor_wheel_joint_fl)+((- tz_motor_wheel_joint_fl-( 1.41421 *  tx_motor_wheel_joint_fl)) * cos_q_motor_wheel_joint_fl));
    (*this)(4,1) = - 0.5 * ((( tz_motor_wheel_joint_fl+( 1.41421 *  tx_motor_wheel_joint_fl)) * sin_q_motor_wheel_joint_fl)+(( tz_motor_wheel_joint_fl-( 1.41421 *  tx_motor_wheel_joint_fl)) * cos_q_motor_wheel_joint_fl));
    (*this)(4,3) =  0.5 * (sin_q_motor_wheel_joint_fl-cos_q_motor_wheel_joint_fl);
    (*this)(4,4) =  0.5 * (sin_q_motor_wheel_joint_fl+cos_q_motor_wheel_joint_fl);
    (*this)(5,0) =  0.5 * ((( ty_motor_wheel_joint_fl+ tx_motor_wheel_joint_fl) * sin_q_motor_wheel_joint_fl)+((- ty_motor_wheel_joint_fl- tx_motor_wheel_joint_fl) * cos_q_motor_wheel_joint_fl));
    (*this)(5,1) =  0.5 * ((( ty_motor_wheel_joint_fl+ tx_motor_wheel_joint_fl) * sin_q_motor_wheel_joint_fl)+(( ty_motor_wheel_joint_fl+ tx_motor_wheel_joint_fl) * cos_q_motor_wheel_joint_fl));
    (*this)(5,3) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fl)+( 1.41421 * cos_q_motor_wheel_joint_fl));
    (*this)(5,4) =  0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fl)-( 1.41421 * cos_q_motor_wheel_joint_fl));
    return *this;
}
MotionTransforms::Type_fr_omniwheel_fr_X_fr_base_link_footprint::Type_fr_omniwheel_fr_X_fr_base_link_footprint()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = - 0.707106;    // Maxima DSL: -0.707106
    (*this)(2,1) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,0) = - 0.707106 *  tz_motor_wheel_joint_fr;    // Maxima DSL: -0.707106*_k__tz_motor_wheel_joint_fr
    (*this)(5,1) = - 0.707106 *  tz_motor_wheel_joint_fr;    // Maxima DSL: -0.707106*_k__tz_motor_wheel_joint_fr
    (*this)(5,2) =  0.5 * (( 1.41421 *  ty_motor_wheel_joint_fr)+( 1.41421 *  tx_motor_wheel_joint_fr));    // Maxima DSL: 0.5*(1.41421*_k__ty_motor_wheel_joint_fr+1.41421*_k__tx_motor_wheel_joint_fr)
    (*this)(5,3) = - 0.707106;    // Maxima DSL: -0.707106
    (*this)(5,4) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_omniwheel_fr_X_fr_base_link_footprint& MotionTransforms::Type_fr_omniwheel_fr_X_fr_base_link_footprint::update(const state_t& q)
{
    Scalar sin_q_motor_wheel_joint_fr  = ScalarTraits::sin( q(MOTOR_WHEEL_JOINT_FR) );
    Scalar cos_q_motor_wheel_joint_fr  = ScalarTraits::cos( q(MOTOR_WHEEL_JOINT_FR) );
    (*this)(0,0) =  0.5 * (sin_q_motor_wheel_joint_fr+cos_q_motor_wheel_joint_fr);
    (*this)(0,1) =  0.5 * (sin_q_motor_wheel_joint_fr+cos_q_motor_wheel_joint_fr);
    (*this)(0,2) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fr)-( 1.41421 * cos_q_motor_wheel_joint_fr));
    (*this)(1,0) = - 0.5 * (sin_q_motor_wheel_joint_fr-cos_q_motor_wheel_joint_fr);
    (*this)(1,1) = - 0.5 * (sin_q_motor_wheel_joint_fr-cos_q_motor_wheel_joint_fr);
    (*this)(1,2) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fr)+( 1.41421 * cos_q_motor_wheel_joint_fr));
    (*this)(3,0) = - 0.5 * ((( tz_motor_wheel_joint_fr+( 1.41421 *  ty_motor_wheel_joint_fr)) * sin_q_motor_wheel_joint_fr)+(( tz_motor_wheel_joint_fr-( 1.41421 *  ty_motor_wheel_joint_fr)) * cos_q_motor_wheel_joint_fr));
    (*this)(3,1) =  0.5 * ((( tz_motor_wheel_joint_fr+( 1.41421 *  tx_motor_wheel_joint_fr)) * sin_q_motor_wheel_joint_fr)+(( tz_motor_wheel_joint_fr-( 1.41421 *  tx_motor_wheel_joint_fr)) * cos_q_motor_wheel_joint_fr));
    (*this)(3,2) = - 0.5 * ((( ty_motor_wheel_joint_fr- tx_motor_wheel_joint_fr) * sin_q_motor_wheel_joint_fr)+(( ty_motor_wheel_joint_fr- tx_motor_wheel_joint_fr) * cos_q_motor_wheel_joint_fr));
    (*this)(3,3) =  0.5 * (sin_q_motor_wheel_joint_fr+cos_q_motor_wheel_joint_fr);
    (*this)(3,4) =  0.5 * (sin_q_motor_wheel_joint_fr+cos_q_motor_wheel_joint_fr);
    (*this)(3,5) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fr)-( 1.41421 * cos_q_motor_wheel_joint_fr));
    (*this)(4,0) =  0.5 * ((( tz_motor_wheel_joint_fr-( 1.41421 *  ty_motor_wheel_joint_fr)) * sin_q_motor_wheel_joint_fr)+((- tz_motor_wheel_joint_fr-( 1.41421 *  ty_motor_wheel_joint_fr)) * cos_q_motor_wheel_joint_fr));
    (*this)(4,1) = - 0.5 * ((( tz_motor_wheel_joint_fr-( 1.41421 *  tx_motor_wheel_joint_fr)) * sin_q_motor_wheel_joint_fr)+((- tz_motor_wheel_joint_fr-( 1.41421 *  tx_motor_wheel_joint_fr)) * cos_q_motor_wheel_joint_fr));
    (*this)(4,2) =  0.5 * ((( ty_motor_wheel_joint_fr- tx_motor_wheel_joint_fr) * sin_q_motor_wheel_joint_fr)+(( tx_motor_wheel_joint_fr- ty_motor_wheel_joint_fr) * cos_q_motor_wheel_joint_fr));
    (*this)(4,3) = - 0.5 * (sin_q_motor_wheel_joint_fr-cos_q_motor_wheel_joint_fr);
    (*this)(4,4) = - 0.5 * (sin_q_motor_wheel_joint_fr-cos_q_motor_wheel_joint_fr);
    (*this)(4,5) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fr)+( 1.41421 * cos_q_motor_wheel_joint_fr));
    return *this;
}
MotionTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_fr::Type_fr_base_link_footprint_X_fr_omniwheel_fr()
{
    (*this)(0,2) = - 0.707106;    // Maxima DSL: -0.707106
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = - 0.707106 *  tz_motor_wheel_joint_fr;    // Maxima DSL: -0.707106*_k__tz_motor_wheel_joint_fr
    (*this)(3,5) = - 0.707106;    // Maxima DSL: -0.707106
    (*this)(4,2) = - 0.707106 *  tz_motor_wheel_joint_fr;    // Maxima DSL: -0.707106*_k__tz_motor_wheel_joint_fr
    (*this)(4,5) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(5,2) =  0.5 * (( 1.41421 *  ty_motor_wheel_joint_fr)+( 1.41421 *  tx_motor_wheel_joint_fr));    // Maxima DSL: 0.5*(1.41421*_k__ty_motor_wheel_joint_fr+1.41421*_k__tx_motor_wheel_joint_fr)
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_fr& MotionTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_fr::update(const state_t& q)
{
    Scalar sin_q_motor_wheel_joint_fr  = ScalarTraits::sin( q(MOTOR_WHEEL_JOINT_FR) );
    Scalar cos_q_motor_wheel_joint_fr  = ScalarTraits::cos( q(MOTOR_WHEEL_JOINT_FR) );
    (*this)(0,0) =  0.5 * (sin_q_motor_wheel_joint_fr+cos_q_motor_wheel_joint_fr);
    (*this)(0,1) = - 0.5 * (sin_q_motor_wheel_joint_fr-cos_q_motor_wheel_joint_fr);
    (*this)(1,0) =  0.5 * (sin_q_motor_wheel_joint_fr+cos_q_motor_wheel_joint_fr);
    (*this)(1,1) = - 0.5 * (sin_q_motor_wheel_joint_fr-cos_q_motor_wheel_joint_fr);
    (*this)(2,0) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fr)-( 1.41421 * cos_q_motor_wheel_joint_fr));
    (*this)(2,1) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fr)+( 1.41421 * cos_q_motor_wheel_joint_fr));
    (*this)(3,0) = - 0.5 * ((( tz_motor_wheel_joint_fr+( 1.41421 *  ty_motor_wheel_joint_fr)) * sin_q_motor_wheel_joint_fr)+(( tz_motor_wheel_joint_fr-( 1.41421 *  ty_motor_wheel_joint_fr)) * cos_q_motor_wheel_joint_fr));
    (*this)(3,1) =  0.5 * ((( tz_motor_wheel_joint_fr-( 1.41421 *  ty_motor_wheel_joint_fr)) * sin_q_motor_wheel_joint_fr)+((- tz_motor_wheel_joint_fr-( 1.41421 *  ty_motor_wheel_joint_fr)) * cos_q_motor_wheel_joint_fr));
    (*this)(3,3) =  0.5 * (sin_q_motor_wheel_joint_fr+cos_q_motor_wheel_joint_fr);
    (*this)(3,4) = - 0.5 * (sin_q_motor_wheel_joint_fr-cos_q_motor_wheel_joint_fr);
    (*this)(4,0) =  0.5 * ((( tz_motor_wheel_joint_fr+( 1.41421 *  tx_motor_wheel_joint_fr)) * sin_q_motor_wheel_joint_fr)+(( tz_motor_wheel_joint_fr-( 1.41421 *  tx_motor_wheel_joint_fr)) * cos_q_motor_wheel_joint_fr));
    (*this)(4,1) = - 0.5 * ((( tz_motor_wheel_joint_fr-( 1.41421 *  tx_motor_wheel_joint_fr)) * sin_q_motor_wheel_joint_fr)+((- tz_motor_wheel_joint_fr-( 1.41421 *  tx_motor_wheel_joint_fr)) * cos_q_motor_wheel_joint_fr));
    (*this)(4,3) =  0.5 * (sin_q_motor_wheel_joint_fr+cos_q_motor_wheel_joint_fr);
    (*this)(4,4) = - 0.5 * (sin_q_motor_wheel_joint_fr-cos_q_motor_wheel_joint_fr);
    (*this)(5,0) = - 0.5 * ((( ty_motor_wheel_joint_fr- tx_motor_wheel_joint_fr) * sin_q_motor_wheel_joint_fr)+(( ty_motor_wheel_joint_fr- tx_motor_wheel_joint_fr) * cos_q_motor_wheel_joint_fr));
    (*this)(5,1) =  0.5 * ((( ty_motor_wheel_joint_fr- tx_motor_wheel_joint_fr) * sin_q_motor_wheel_joint_fr)+(( tx_motor_wheel_joint_fr- ty_motor_wheel_joint_fr) * cos_q_motor_wheel_joint_fr));
    (*this)(5,3) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fr)-( 1.41421 * cos_q_motor_wheel_joint_fr));
    (*this)(5,4) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fr)+( 1.41421 * cos_q_motor_wheel_joint_fr));
    return *this;
}
MotionTransforms::Type_fr_omniwheel_bl_X_fr_base_link_footprint::Type_fr_omniwheel_bl_X_fr_base_link_footprint()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = - 0.707106;    // Maxima DSL: -0.707106
    (*this)(2,1) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,0) = - 0.707106 *  tz_motor_wheel_joint_bl;    // Maxima DSL: -0.707106*_k__tz_motor_wheel_joint_bl
    (*this)(5,1) = - 0.707106 *  tz_motor_wheel_joint_bl;    // Maxima DSL: -0.707106*_k__tz_motor_wheel_joint_bl
    (*this)(5,2) =  0.5 * (( 1.41421 *  ty_motor_wheel_joint_bl)+( 1.41421 *  tx_motor_wheel_joint_bl));    // Maxima DSL: 0.5*(1.41421*_k__ty_motor_wheel_joint_bl+1.41421*_k__tx_motor_wheel_joint_bl)
    (*this)(5,3) = - 0.707106;    // Maxima DSL: -0.707106
    (*this)(5,4) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_omniwheel_bl_X_fr_base_link_footprint& MotionTransforms::Type_fr_omniwheel_bl_X_fr_base_link_footprint::update(const state_t& q)
{
    Scalar sin_q_motor_wheel_joint_bl  = ScalarTraits::sin( q(MOTOR_WHEEL_JOINT_BL) );
    Scalar cos_q_motor_wheel_joint_bl  = ScalarTraits::cos( q(MOTOR_WHEEL_JOINT_BL) );
    (*this)(0,0) =  0.5 * (sin_q_motor_wheel_joint_bl+cos_q_motor_wheel_joint_bl);
    (*this)(0,1) =  0.5 * (sin_q_motor_wheel_joint_bl+cos_q_motor_wheel_joint_bl);
    (*this)(0,2) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_bl)-( 1.41421 * cos_q_motor_wheel_joint_bl));
    (*this)(1,0) = - 0.5 * (sin_q_motor_wheel_joint_bl-cos_q_motor_wheel_joint_bl);
    (*this)(1,1) = - 0.5 * (sin_q_motor_wheel_joint_bl-cos_q_motor_wheel_joint_bl);
    (*this)(1,2) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_bl)+( 1.41421 * cos_q_motor_wheel_joint_bl));
    (*this)(3,0) = - 0.5 * ((( tz_motor_wheel_joint_bl+( 1.41421 *  ty_motor_wheel_joint_bl)) * sin_q_motor_wheel_joint_bl)+(( tz_motor_wheel_joint_bl-( 1.41421 *  ty_motor_wheel_joint_bl)) * cos_q_motor_wheel_joint_bl));
    (*this)(3,1) =  0.5 * ((( tz_motor_wheel_joint_bl+( 1.41421 *  tx_motor_wheel_joint_bl)) * sin_q_motor_wheel_joint_bl)+(( tz_motor_wheel_joint_bl-( 1.41421 *  tx_motor_wheel_joint_bl)) * cos_q_motor_wheel_joint_bl));
    (*this)(3,2) = - 0.5 * ((( ty_motor_wheel_joint_bl- tx_motor_wheel_joint_bl) * sin_q_motor_wheel_joint_bl)+(( ty_motor_wheel_joint_bl- tx_motor_wheel_joint_bl) * cos_q_motor_wheel_joint_bl));
    (*this)(3,3) =  0.5 * (sin_q_motor_wheel_joint_bl+cos_q_motor_wheel_joint_bl);
    (*this)(3,4) =  0.5 * (sin_q_motor_wheel_joint_bl+cos_q_motor_wheel_joint_bl);
    (*this)(3,5) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_bl)-( 1.41421 * cos_q_motor_wheel_joint_bl));
    (*this)(4,0) =  0.5 * ((( tz_motor_wheel_joint_bl-( 1.41421 *  ty_motor_wheel_joint_bl)) * sin_q_motor_wheel_joint_bl)+((- tz_motor_wheel_joint_bl-( 1.41421 *  ty_motor_wheel_joint_bl)) * cos_q_motor_wheel_joint_bl));
    (*this)(4,1) = - 0.5 * ((( tz_motor_wheel_joint_bl-( 1.41421 *  tx_motor_wheel_joint_bl)) * sin_q_motor_wheel_joint_bl)+((- tz_motor_wheel_joint_bl-( 1.41421 *  tx_motor_wheel_joint_bl)) * cos_q_motor_wheel_joint_bl));
    (*this)(4,2) =  0.5 * ((( ty_motor_wheel_joint_bl- tx_motor_wheel_joint_bl) * sin_q_motor_wheel_joint_bl)+(( tx_motor_wheel_joint_bl- ty_motor_wheel_joint_bl) * cos_q_motor_wheel_joint_bl));
    (*this)(4,3) = - 0.5 * (sin_q_motor_wheel_joint_bl-cos_q_motor_wheel_joint_bl);
    (*this)(4,4) = - 0.5 * (sin_q_motor_wheel_joint_bl-cos_q_motor_wheel_joint_bl);
    (*this)(4,5) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_bl)+( 1.41421 * cos_q_motor_wheel_joint_bl));
    return *this;
}
MotionTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_bl::Type_fr_base_link_footprint_X_fr_omniwheel_bl()
{
    (*this)(0,2) = - 0.707106;    // Maxima DSL: -0.707106
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = - 0.707106 *  tz_motor_wheel_joint_bl;    // Maxima DSL: -0.707106*_k__tz_motor_wheel_joint_bl
    (*this)(3,5) = - 0.707106;    // Maxima DSL: -0.707106
    (*this)(4,2) = - 0.707106 *  tz_motor_wheel_joint_bl;    // Maxima DSL: -0.707106*_k__tz_motor_wheel_joint_bl
    (*this)(4,5) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(5,2) =  0.5 * (( 1.41421 *  ty_motor_wheel_joint_bl)+( 1.41421 *  tx_motor_wheel_joint_bl));    // Maxima DSL: 0.5*(1.41421*_k__ty_motor_wheel_joint_bl+1.41421*_k__tx_motor_wheel_joint_bl)
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_bl& MotionTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_bl::update(const state_t& q)
{
    Scalar sin_q_motor_wheel_joint_bl  = ScalarTraits::sin( q(MOTOR_WHEEL_JOINT_BL) );
    Scalar cos_q_motor_wheel_joint_bl  = ScalarTraits::cos( q(MOTOR_WHEEL_JOINT_BL) );
    (*this)(0,0) =  0.5 * (sin_q_motor_wheel_joint_bl+cos_q_motor_wheel_joint_bl);
    (*this)(0,1) = - 0.5 * (sin_q_motor_wheel_joint_bl-cos_q_motor_wheel_joint_bl);
    (*this)(1,0) =  0.5 * (sin_q_motor_wheel_joint_bl+cos_q_motor_wheel_joint_bl);
    (*this)(1,1) = - 0.5 * (sin_q_motor_wheel_joint_bl-cos_q_motor_wheel_joint_bl);
    (*this)(2,0) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_bl)-( 1.41421 * cos_q_motor_wheel_joint_bl));
    (*this)(2,1) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_bl)+( 1.41421 * cos_q_motor_wheel_joint_bl));
    (*this)(3,0) = - 0.5 * ((( tz_motor_wheel_joint_bl+( 1.41421 *  ty_motor_wheel_joint_bl)) * sin_q_motor_wheel_joint_bl)+(( tz_motor_wheel_joint_bl-( 1.41421 *  ty_motor_wheel_joint_bl)) * cos_q_motor_wheel_joint_bl));
    (*this)(3,1) =  0.5 * ((( tz_motor_wheel_joint_bl-( 1.41421 *  ty_motor_wheel_joint_bl)) * sin_q_motor_wheel_joint_bl)+((- tz_motor_wheel_joint_bl-( 1.41421 *  ty_motor_wheel_joint_bl)) * cos_q_motor_wheel_joint_bl));
    (*this)(3,3) =  0.5 * (sin_q_motor_wheel_joint_bl+cos_q_motor_wheel_joint_bl);
    (*this)(3,4) = - 0.5 * (sin_q_motor_wheel_joint_bl-cos_q_motor_wheel_joint_bl);
    (*this)(4,0) =  0.5 * ((( tz_motor_wheel_joint_bl+( 1.41421 *  tx_motor_wheel_joint_bl)) * sin_q_motor_wheel_joint_bl)+(( tz_motor_wheel_joint_bl-( 1.41421 *  tx_motor_wheel_joint_bl)) * cos_q_motor_wheel_joint_bl));
    (*this)(4,1) = - 0.5 * ((( tz_motor_wheel_joint_bl-( 1.41421 *  tx_motor_wheel_joint_bl)) * sin_q_motor_wheel_joint_bl)+((- tz_motor_wheel_joint_bl-( 1.41421 *  tx_motor_wheel_joint_bl)) * cos_q_motor_wheel_joint_bl));
    (*this)(4,3) =  0.5 * (sin_q_motor_wheel_joint_bl+cos_q_motor_wheel_joint_bl);
    (*this)(4,4) = - 0.5 * (sin_q_motor_wheel_joint_bl-cos_q_motor_wheel_joint_bl);
    (*this)(5,0) = - 0.5 * ((( ty_motor_wheel_joint_bl- tx_motor_wheel_joint_bl) * sin_q_motor_wheel_joint_bl)+(( ty_motor_wheel_joint_bl- tx_motor_wheel_joint_bl) * cos_q_motor_wheel_joint_bl));
    (*this)(5,1) =  0.5 * ((( ty_motor_wheel_joint_bl- tx_motor_wheel_joint_bl) * sin_q_motor_wheel_joint_bl)+(( tx_motor_wheel_joint_bl- ty_motor_wheel_joint_bl) * cos_q_motor_wheel_joint_bl));
    (*this)(5,3) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_bl)-( 1.41421 * cos_q_motor_wheel_joint_bl));
    (*this)(5,4) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_bl)+( 1.41421 * cos_q_motor_wheel_joint_bl));
    return *this;
}
MotionTransforms::Type_fr_omniwheel_br_X_fr_base_link_footprint::Type_fr_omniwheel_br_X_fr_base_link_footprint()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(2,1) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,0) = - 0.707106 *  tz_motor_wheel_joint_br;    // Maxima DSL: -0.707106*_k__tz_motor_wheel_joint_br
    (*this)(5,1) =  0.707106 *  tz_motor_wheel_joint_br;    // Maxima DSL: 0.707106*_k__tz_motor_wheel_joint_br
    (*this)(5,2) = - 0.5 * (( 1.41421 *  ty_motor_wheel_joint_br)-( 1.41421 *  tx_motor_wheel_joint_br));    // Maxima DSL: -0.5*(1.41421*_k__ty_motor_wheel_joint_br-1.41421*_k__tx_motor_wheel_joint_br)
    (*this)(5,3) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(5,4) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_omniwheel_br_X_fr_base_link_footprint& MotionTransforms::Type_fr_omniwheel_br_X_fr_base_link_footprint::update(const state_t& q)
{
    Scalar sin_q_motor_wheel_joint_br  = ScalarTraits::sin( q(MOTOR_WHEEL_JOINT_BR) );
    Scalar cos_q_motor_wheel_joint_br  = ScalarTraits::cos( q(MOTOR_WHEEL_JOINT_BR) );
    (*this)(0,0) = - 0.5 * (sin_q_motor_wheel_joint_br-cos_q_motor_wheel_joint_br);
    (*this)(0,1) =  0.5 * (sin_q_motor_wheel_joint_br-cos_q_motor_wheel_joint_br);
    (*this)(0,2) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_br)+( 1.41421 * cos_q_motor_wheel_joint_br));
    (*this)(1,0) = - 0.5 * (sin_q_motor_wheel_joint_br+cos_q_motor_wheel_joint_br);
    (*this)(1,1) =  0.5 * (sin_q_motor_wheel_joint_br+cos_q_motor_wheel_joint_br);
    (*this)(1,2) =  0.5 * (( 1.41421 * sin_q_motor_wheel_joint_br)-( 1.41421 * cos_q_motor_wheel_joint_br));
    (*this)(3,0) = - 0.5 * ((( tz_motor_wheel_joint_br+( 1.41421 *  ty_motor_wheel_joint_br)) * sin_q_motor_wheel_joint_br)+((( 1.41421 *  ty_motor_wheel_joint_br)- tz_motor_wheel_joint_br) * cos_q_motor_wheel_joint_br));
    (*this)(3,1) = - 0.5 * ((( tz_motor_wheel_joint_br-( 1.41421 *  tx_motor_wheel_joint_br)) * sin_q_motor_wheel_joint_br)+((- tz_motor_wheel_joint_br-( 1.41421 *  tx_motor_wheel_joint_br)) * cos_q_motor_wheel_joint_br));
    (*this)(3,2) =  0.5 * ((( ty_motor_wheel_joint_br+ tx_motor_wheel_joint_br) * sin_q_motor_wheel_joint_br)+((- ty_motor_wheel_joint_br- tx_motor_wheel_joint_br) * cos_q_motor_wheel_joint_br));
    (*this)(3,3) = - 0.5 * (sin_q_motor_wheel_joint_br-cos_q_motor_wheel_joint_br);
    (*this)(3,4) =  0.5 * (sin_q_motor_wheel_joint_br-cos_q_motor_wheel_joint_br);
    (*this)(3,5) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_br)+( 1.41421 * cos_q_motor_wheel_joint_br));
    (*this)(4,0) = - 0.5 * ((( tz_motor_wheel_joint_br-( 1.41421 *  ty_motor_wheel_joint_br)) * sin_q_motor_wheel_joint_br)+(( tz_motor_wheel_joint_br+( 1.41421 *  ty_motor_wheel_joint_br)) * cos_q_motor_wheel_joint_br));
    (*this)(4,1) = - 0.5 * ((( tz_motor_wheel_joint_br+( 1.41421 *  tx_motor_wheel_joint_br)) * sin_q_motor_wheel_joint_br)+(( tz_motor_wheel_joint_br-( 1.41421 *  tx_motor_wheel_joint_br)) * cos_q_motor_wheel_joint_br));
    (*this)(4,2) =  0.5 * ((( ty_motor_wheel_joint_br+ tx_motor_wheel_joint_br) * sin_q_motor_wheel_joint_br)+(( ty_motor_wheel_joint_br+ tx_motor_wheel_joint_br) * cos_q_motor_wheel_joint_br));
    (*this)(4,3) = - 0.5 * (sin_q_motor_wheel_joint_br+cos_q_motor_wheel_joint_br);
    (*this)(4,4) =  0.5 * (sin_q_motor_wheel_joint_br+cos_q_motor_wheel_joint_br);
    (*this)(4,5) =  0.5 * (( 1.41421 * sin_q_motor_wheel_joint_br)-( 1.41421 * cos_q_motor_wheel_joint_br));
    return *this;
}
MotionTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_br::Type_fr_base_link_footprint_X_fr_omniwheel_br()
{
    (*this)(0,2) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = - 0.707106 *  tz_motor_wheel_joint_br;    // Maxima DSL: -0.707106*_k__tz_motor_wheel_joint_br
    (*this)(3,5) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(4,2) =  0.707106 *  tz_motor_wheel_joint_br;    // Maxima DSL: 0.707106*_k__tz_motor_wheel_joint_br
    (*this)(4,5) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(5,2) = - 0.5 * (( 1.41421 *  ty_motor_wheel_joint_br)-( 1.41421 *  tx_motor_wheel_joint_br));    // Maxima DSL: -0.5*(1.41421*_k__ty_motor_wheel_joint_br-1.41421*_k__tx_motor_wheel_joint_br)
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_br& MotionTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_br::update(const state_t& q)
{
    Scalar sin_q_motor_wheel_joint_br  = ScalarTraits::sin( q(MOTOR_WHEEL_JOINT_BR) );
    Scalar cos_q_motor_wheel_joint_br  = ScalarTraits::cos( q(MOTOR_WHEEL_JOINT_BR) );
    (*this)(0,0) = - 0.5 * (sin_q_motor_wheel_joint_br-cos_q_motor_wheel_joint_br);
    (*this)(0,1) = - 0.5 * (sin_q_motor_wheel_joint_br+cos_q_motor_wheel_joint_br);
    (*this)(1,0) =  0.5 * (sin_q_motor_wheel_joint_br-cos_q_motor_wheel_joint_br);
    (*this)(1,1) =  0.5 * (sin_q_motor_wheel_joint_br+cos_q_motor_wheel_joint_br);
    (*this)(2,0) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_br)+( 1.41421 * cos_q_motor_wheel_joint_br));
    (*this)(2,1) =  0.5 * (( 1.41421 * sin_q_motor_wheel_joint_br)-( 1.41421 * cos_q_motor_wheel_joint_br));
    (*this)(3,0) = - 0.5 * ((( tz_motor_wheel_joint_br+( 1.41421 *  ty_motor_wheel_joint_br)) * sin_q_motor_wheel_joint_br)+((( 1.41421 *  ty_motor_wheel_joint_br)- tz_motor_wheel_joint_br) * cos_q_motor_wheel_joint_br));
    (*this)(3,1) = - 0.5 * ((( tz_motor_wheel_joint_br-( 1.41421 *  ty_motor_wheel_joint_br)) * sin_q_motor_wheel_joint_br)+(( tz_motor_wheel_joint_br+( 1.41421 *  ty_motor_wheel_joint_br)) * cos_q_motor_wheel_joint_br));
    (*this)(3,3) = - 0.5 * (sin_q_motor_wheel_joint_br-cos_q_motor_wheel_joint_br);
    (*this)(3,4) = - 0.5 * (sin_q_motor_wheel_joint_br+cos_q_motor_wheel_joint_br);
    (*this)(4,0) = - 0.5 * ((( tz_motor_wheel_joint_br-( 1.41421 *  tx_motor_wheel_joint_br)) * sin_q_motor_wheel_joint_br)+((- tz_motor_wheel_joint_br-( 1.41421 *  tx_motor_wheel_joint_br)) * cos_q_motor_wheel_joint_br));
    (*this)(4,1) = - 0.5 * ((( tz_motor_wheel_joint_br+( 1.41421 *  tx_motor_wheel_joint_br)) * sin_q_motor_wheel_joint_br)+(( tz_motor_wheel_joint_br-( 1.41421 *  tx_motor_wheel_joint_br)) * cos_q_motor_wheel_joint_br));
    (*this)(4,3) =  0.5 * (sin_q_motor_wheel_joint_br-cos_q_motor_wheel_joint_br);
    (*this)(4,4) =  0.5 * (sin_q_motor_wheel_joint_br+cos_q_motor_wheel_joint_br);
    (*this)(5,0) =  0.5 * ((( ty_motor_wheel_joint_br+ tx_motor_wheel_joint_br) * sin_q_motor_wheel_joint_br)+((- ty_motor_wheel_joint_br- tx_motor_wheel_joint_br) * cos_q_motor_wheel_joint_br));
    (*this)(5,1) =  0.5 * ((( ty_motor_wheel_joint_br+ tx_motor_wheel_joint_br) * sin_q_motor_wheel_joint_br)+(( ty_motor_wheel_joint_br+ tx_motor_wheel_joint_br) * cos_q_motor_wheel_joint_br));
    (*this)(5,3) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_br)+( 1.41421 * cos_q_motor_wheel_joint_br));
    (*this)(5,4) =  0.5 * (( 1.41421 * sin_q_motor_wheel_joint_br)-( 1.41421 * cos_q_motor_wheel_joint_br));
    return *this;
}
MotionTransforms::Type_fr_xarmlink1_X_fr_base_link_footprint::Type_fr_xarmlink1_X_fr_base_link_footprint()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = - tx_xarmjoint1;    // Maxima DSL: -_k__tx_xarmjoint1
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_xarmlink1_X_fr_base_link_footprint& MotionTransforms::Type_fr_xarmlink1_X_fr_base_link_footprint::update(const state_t& q)
{
    Scalar sin_q_xarmjoint1  = ScalarTraits::sin( q(XARMJOINT1) );
    Scalar cos_q_xarmjoint1  = ScalarTraits::cos( q(XARMJOINT1) );
    (*this)(0,0) = cos_q_xarmjoint1;
    (*this)(0,1) = sin_q_xarmjoint1;
    (*this)(1,0) = -sin_q_xarmjoint1;
    (*this)(1,1) = cos_q_xarmjoint1;
    (*this)(3,0) = - tz_xarmjoint1 * sin_q_xarmjoint1;
    (*this)(3,1) =  tz_xarmjoint1 * cos_q_xarmjoint1;
    (*this)(3,2) =  tx_xarmjoint1 * sin_q_xarmjoint1;
    (*this)(3,3) = cos_q_xarmjoint1;
    (*this)(3,4) = sin_q_xarmjoint1;
    (*this)(4,0) = - tz_xarmjoint1 * cos_q_xarmjoint1;
    (*this)(4,1) = - tz_xarmjoint1 * sin_q_xarmjoint1;
    (*this)(4,2) =  tx_xarmjoint1 * cos_q_xarmjoint1;
    (*this)(4,3) = -sin_q_xarmjoint1;
    (*this)(4,4) = cos_q_xarmjoint1;
    return *this;
}
MotionTransforms::Type_fr_base_link_footprint_X_fr_xarmlink1::Type_fr_base_link_footprint_X_fr_xarmlink1()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,2) = - tx_xarmjoint1;    // Maxima DSL: -_k__tx_xarmjoint1
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_base_link_footprint_X_fr_xarmlink1& MotionTransforms::Type_fr_base_link_footprint_X_fr_xarmlink1::update(const state_t& q)
{
    Scalar sin_q_xarmjoint1  = ScalarTraits::sin( q(XARMJOINT1) );
    Scalar cos_q_xarmjoint1  = ScalarTraits::cos( q(XARMJOINT1) );
    (*this)(0,0) = cos_q_xarmjoint1;
    (*this)(0,1) = -sin_q_xarmjoint1;
    (*this)(1,0) = sin_q_xarmjoint1;
    (*this)(1,1) = cos_q_xarmjoint1;
    (*this)(3,0) = - tz_xarmjoint1 * sin_q_xarmjoint1;
    (*this)(3,1) = - tz_xarmjoint1 * cos_q_xarmjoint1;
    (*this)(3,3) = cos_q_xarmjoint1;
    (*this)(3,4) = -sin_q_xarmjoint1;
    (*this)(4,0) =  tz_xarmjoint1 * cos_q_xarmjoint1;
    (*this)(4,1) = - tz_xarmjoint1 * sin_q_xarmjoint1;
    (*this)(4,3) = sin_q_xarmjoint1;
    (*this)(4,4) = cos_q_xarmjoint1;
    (*this)(5,0) =  tx_xarmjoint1 * sin_q_xarmjoint1;
    (*this)(5,1) =  tx_xarmjoint1 * cos_q_xarmjoint1;
    return *this;
}
MotionTransforms::Type_fr_xarmlink2_X_fr_xarmlink1::Type_fr_xarmlink2_X_fr_xarmlink1()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_xarmjoint2;    // Maxima DSL: -sin(_k__rx_xarmjoint2)
    (*this)(2,2) = cos_rx_xarmjoint2;    // Maxima DSL: cos(_k__rx_xarmjoint2)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -sin_rx_xarmjoint2;    // Maxima DSL: -sin(_k__rx_xarmjoint2)
    (*this)(5,5) = cos_rx_xarmjoint2;    // Maxima DSL: cos(_k__rx_xarmjoint2)
}

const MotionTransforms::Type_fr_xarmlink2_X_fr_xarmlink1& MotionTransforms::Type_fr_xarmlink2_X_fr_xarmlink1::update(const state_t& q)
{
    Scalar sin_q_xarmjoint2  = ScalarTraits::sin( q(XARMJOINT2) );
    Scalar cos_q_xarmjoint2  = ScalarTraits::cos( q(XARMJOINT2) );
    (*this)(0,0) = cos_q_xarmjoint2;
    (*this)(0,1) = cos_rx_xarmjoint2 * sin_q_xarmjoint2;
    (*this)(0,2) = sin_rx_xarmjoint2 * sin_q_xarmjoint2;
    (*this)(1,0) = -sin_q_xarmjoint2;
    (*this)(1,1) = cos_rx_xarmjoint2 * cos_q_xarmjoint2;
    (*this)(1,2) = sin_rx_xarmjoint2 * cos_q_xarmjoint2;
    (*this)(3,3) = cos_q_xarmjoint2;
    (*this)(3,4) = cos_rx_xarmjoint2 * sin_q_xarmjoint2;
    (*this)(3,5) = sin_rx_xarmjoint2 * sin_q_xarmjoint2;
    (*this)(4,3) = -sin_q_xarmjoint2;
    (*this)(4,4) = cos_rx_xarmjoint2 * cos_q_xarmjoint2;
    (*this)(4,5) = sin_rx_xarmjoint2 * cos_q_xarmjoint2;
    return *this;
}
MotionTransforms::Type_fr_xarmlink1_X_fr_xarmlink2::Type_fr_xarmlink1_X_fr_xarmlink2()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = -sin_rx_xarmjoint2;    // Maxima DSL: -sin(_k__rx_xarmjoint2)
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = cos_rx_xarmjoint2;    // Maxima DSL: cos(_k__rx_xarmjoint2)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = -sin_rx_xarmjoint2;    // Maxima DSL: -sin(_k__rx_xarmjoint2)
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = cos_rx_xarmjoint2;    // Maxima DSL: cos(_k__rx_xarmjoint2)
}

const MotionTransforms::Type_fr_xarmlink1_X_fr_xarmlink2& MotionTransforms::Type_fr_xarmlink1_X_fr_xarmlink2::update(const state_t& q)
{
    Scalar sin_q_xarmjoint2  = ScalarTraits::sin( q(XARMJOINT2) );
    Scalar cos_q_xarmjoint2  = ScalarTraits::cos( q(XARMJOINT2) );
    (*this)(0,0) = cos_q_xarmjoint2;
    (*this)(0,1) = -sin_q_xarmjoint2;
    (*this)(1,0) = cos_rx_xarmjoint2 * sin_q_xarmjoint2;
    (*this)(1,1) = cos_rx_xarmjoint2 * cos_q_xarmjoint2;
    (*this)(2,0) = sin_rx_xarmjoint2 * sin_q_xarmjoint2;
    (*this)(2,1) = sin_rx_xarmjoint2 * cos_q_xarmjoint2;
    (*this)(3,3) = cos_q_xarmjoint2;
    (*this)(3,4) = -sin_q_xarmjoint2;
    (*this)(4,3) = cos_rx_xarmjoint2 * sin_q_xarmjoint2;
    (*this)(4,4) = cos_rx_xarmjoint2 * cos_q_xarmjoint2;
    (*this)(5,3) = sin_rx_xarmjoint2 * sin_q_xarmjoint2;
    (*this)(5,4) = sin_rx_xarmjoint2 * cos_q_xarmjoint2;
    return *this;
}
MotionTransforms::Type_fr_xarmlink3_X_fr_xarmlink2::Type_fr_xarmlink3_X_fr_xarmlink2()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) =  ty_xarmjoint3;    // Maxima DSL: _k__ty_xarmjoint3
    (*this)(5,1) = - tx_xarmjoint3;    // Maxima DSL: -_k__tx_xarmjoint3
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_xarmlink3_X_fr_xarmlink2& MotionTransforms::Type_fr_xarmlink3_X_fr_xarmlink2::update(const state_t& q)
{
    Scalar sin_q_xarmjoint3  = ScalarTraits::sin( q(XARMJOINT3) );
    Scalar cos_q_xarmjoint3  = ScalarTraits::cos( q(XARMJOINT3) );
    (*this)(0,0) = cos_q_xarmjoint3;
    (*this)(0,1) = sin_q_xarmjoint3;
    (*this)(1,0) = -sin_q_xarmjoint3;
    (*this)(1,1) = cos_q_xarmjoint3;
    (*this)(3,2) = ( tx_xarmjoint3 * sin_q_xarmjoint3)-( ty_xarmjoint3 * cos_q_xarmjoint3);
    (*this)(3,3) = cos_q_xarmjoint3;
    (*this)(3,4) = sin_q_xarmjoint3;
    (*this)(4,2) = ( ty_xarmjoint3 * sin_q_xarmjoint3)+( tx_xarmjoint3 * cos_q_xarmjoint3);
    (*this)(4,3) = -sin_q_xarmjoint3;
    (*this)(4,4) = cos_q_xarmjoint3;
    return *this;
}
MotionTransforms::Type_fr_xarmlink2_X_fr_xarmlink3::Type_fr_xarmlink2_X_fr_xarmlink3()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) =  ty_xarmjoint3;    // Maxima DSL: _k__ty_xarmjoint3
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = - tx_xarmjoint3;    // Maxima DSL: -_k__tx_xarmjoint3
    (*this)(4,5) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_xarmlink2_X_fr_xarmlink3& MotionTransforms::Type_fr_xarmlink2_X_fr_xarmlink3::update(const state_t& q)
{
    Scalar sin_q_xarmjoint3  = ScalarTraits::sin( q(XARMJOINT3) );
    Scalar cos_q_xarmjoint3  = ScalarTraits::cos( q(XARMJOINT3) );
    (*this)(0,0) = cos_q_xarmjoint3;
    (*this)(0,1) = -sin_q_xarmjoint3;
    (*this)(1,0) = sin_q_xarmjoint3;
    (*this)(1,1) = cos_q_xarmjoint3;
    (*this)(3,3) = cos_q_xarmjoint3;
    (*this)(3,4) = -sin_q_xarmjoint3;
    (*this)(4,3) = sin_q_xarmjoint3;
    (*this)(4,4) = cos_q_xarmjoint3;
    (*this)(5,0) = ( tx_xarmjoint3 * sin_q_xarmjoint3)-( ty_xarmjoint3 * cos_q_xarmjoint3);
    (*this)(5,1) = ( ty_xarmjoint3 * sin_q_xarmjoint3)+( tx_xarmjoint3 * cos_q_xarmjoint3);
    return *this;
}
MotionTransforms::Type_fr_xarmlink4_X_fr_xarmlink3::Type_fr_xarmlink4_X_fr_xarmlink3()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_xarmjoint4;    // Maxima DSL: -sin(_k__rx_xarmjoint4)
    (*this)(2,2) = cos_rx_xarmjoint4;    // Maxima DSL: cos(_k__rx_xarmjoint4)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,0) =  ty_xarmjoint4 * cos_rx_xarmjoint4;    // Maxima DSL: _k__ty_xarmjoint4*cos(_k__rx_xarmjoint4)
    (*this)(5,1) = - tx_xarmjoint4 * cos_rx_xarmjoint4;    // Maxima DSL: -_k__tx_xarmjoint4*cos(_k__rx_xarmjoint4)
    (*this)(5,2) = - tx_xarmjoint4 * sin_rx_xarmjoint4;    // Maxima DSL: -_k__tx_xarmjoint4*sin(_k__rx_xarmjoint4)
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -sin_rx_xarmjoint4;    // Maxima DSL: -sin(_k__rx_xarmjoint4)
    (*this)(5,5) = cos_rx_xarmjoint4;    // Maxima DSL: cos(_k__rx_xarmjoint4)
}

const MotionTransforms::Type_fr_xarmlink4_X_fr_xarmlink3& MotionTransforms::Type_fr_xarmlink4_X_fr_xarmlink3::update(const state_t& q)
{
    Scalar sin_q_xarmjoint4  = ScalarTraits::sin( q(XARMJOINT4) );
    Scalar cos_q_xarmjoint4  = ScalarTraits::cos( q(XARMJOINT4) );
    (*this)(0,0) = cos_q_xarmjoint4;
    (*this)(0,1) = cos_rx_xarmjoint4 * sin_q_xarmjoint4;
    (*this)(0,2) = sin_rx_xarmjoint4 * sin_q_xarmjoint4;
    (*this)(1,0) = -sin_q_xarmjoint4;
    (*this)(1,1) = cos_rx_xarmjoint4 * cos_q_xarmjoint4;
    (*this)(1,2) = sin_rx_xarmjoint4 * cos_q_xarmjoint4;
    (*this)(3,0) =  ty_xarmjoint4 * sin_rx_xarmjoint4 * sin_q_xarmjoint4;
    (*this)(3,1) = - tx_xarmjoint4 * sin_rx_xarmjoint4 * sin_q_xarmjoint4;
    (*this)(3,2) = ( tx_xarmjoint4 * cos_rx_xarmjoint4 * sin_q_xarmjoint4)-( ty_xarmjoint4 * cos_q_xarmjoint4);
    (*this)(3,3) = cos_q_xarmjoint4;
    (*this)(3,4) = cos_rx_xarmjoint4 * sin_q_xarmjoint4;
    (*this)(3,5) = sin_rx_xarmjoint4 * sin_q_xarmjoint4;
    (*this)(4,0) =  ty_xarmjoint4 * sin_rx_xarmjoint4 * cos_q_xarmjoint4;
    (*this)(4,1) = - tx_xarmjoint4 * sin_rx_xarmjoint4 * cos_q_xarmjoint4;
    (*this)(4,2) = ( ty_xarmjoint4 * sin_q_xarmjoint4)+( tx_xarmjoint4 * cos_rx_xarmjoint4 * cos_q_xarmjoint4);
    (*this)(4,3) = -sin_q_xarmjoint4;
    (*this)(4,4) = cos_rx_xarmjoint4 * cos_q_xarmjoint4;
    (*this)(4,5) = sin_rx_xarmjoint4 * cos_q_xarmjoint4;
    return *this;
}
MotionTransforms::Type_fr_xarmlink3_X_fr_xarmlink4::Type_fr_xarmlink3_X_fr_xarmlink4()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = -sin_rx_xarmjoint4;    // Maxima DSL: -sin(_k__rx_xarmjoint4)
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = cos_rx_xarmjoint4;    // Maxima DSL: cos(_k__rx_xarmjoint4)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) =  ty_xarmjoint4 * cos_rx_xarmjoint4;    // Maxima DSL: _k__ty_xarmjoint4*cos(_k__rx_xarmjoint4)
    (*this)(3,5) = 0.0;
    (*this)(4,2) = - tx_xarmjoint4 * cos_rx_xarmjoint4;    // Maxima DSL: -_k__tx_xarmjoint4*cos(_k__rx_xarmjoint4)
    (*this)(4,5) = -sin_rx_xarmjoint4;    // Maxima DSL: -sin(_k__rx_xarmjoint4)
    (*this)(5,2) = - tx_xarmjoint4 * sin_rx_xarmjoint4;    // Maxima DSL: -_k__tx_xarmjoint4*sin(_k__rx_xarmjoint4)
    (*this)(5,5) = cos_rx_xarmjoint4;    // Maxima DSL: cos(_k__rx_xarmjoint4)
}

const MotionTransforms::Type_fr_xarmlink3_X_fr_xarmlink4& MotionTransforms::Type_fr_xarmlink3_X_fr_xarmlink4::update(const state_t& q)
{
    Scalar sin_q_xarmjoint4  = ScalarTraits::sin( q(XARMJOINT4) );
    Scalar cos_q_xarmjoint4  = ScalarTraits::cos( q(XARMJOINT4) );
    (*this)(0,0) = cos_q_xarmjoint4;
    (*this)(0,1) = -sin_q_xarmjoint4;
    (*this)(1,0) = cos_rx_xarmjoint4 * sin_q_xarmjoint4;
    (*this)(1,1) = cos_rx_xarmjoint4 * cos_q_xarmjoint4;
    (*this)(2,0) = sin_rx_xarmjoint4 * sin_q_xarmjoint4;
    (*this)(2,1) = sin_rx_xarmjoint4 * cos_q_xarmjoint4;
    (*this)(3,0) =  ty_xarmjoint4 * sin_rx_xarmjoint4 * sin_q_xarmjoint4;
    (*this)(3,1) =  ty_xarmjoint4 * sin_rx_xarmjoint4 * cos_q_xarmjoint4;
    (*this)(3,3) = cos_q_xarmjoint4;
    (*this)(3,4) = -sin_q_xarmjoint4;
    (*this)(4,0) = - tx_xarmjoint4 * sin_rx_xarmjoint4 * sin_q_xarmjoint4;
    (*this)(4,1) = - tx_xarmjoint4 * sin_rx_xarmjoint4 * cos_q_xarmjoint4;
    (*this)(4,3) = cos_rx_xarmjoint4 * sin_q_xarmjoint4;
    (*this)(4,4) = cos_rx_xarmjoint4 * cos_q_xarmjoint4;
    (*this)(5,0) = ( tx_xarmjoint4 * cos_rx_xarmjoint4 * sin_q_xarmjoint4)-( ty_xarmjoint4 * cos_q_xarmjoint4);
    (*this)(5,1) = ( ty_xarmjoint4 * sin_q_xarmjoint4)+( tx_xarmjoint4 * cos_rx_xarmjoint4 * cos_q_xarmjoint4);
    (*this)(5,3) = sin_rx_xarmjoint4 * sin_q_xarmjoint4;
    (*this)(5,4) = sin_rx_xarmjoint4 * cos_q_xarmjoint4;
    return *this;
}
MotionTransforms::Type_fr_xarmlink5_X_fr_xarmlink4::Type_fr_xarmlink5_X_fr_xarmlink4()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_xarmjoint5;    // Maxima DSL: -sin(_k__rx_xarmjoint5)
    (*this)(2,2) = cos_rx_xarmjoint5;    // Maxima DSL: cos(_k__rx_xarmjoint5)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -sin_rx_xarmjoint5;    // Maxima DSL: -sin(_k__rx_xarmjoint5)
    (*this)(5,5) = cos_rx_xarmjoint5;    // Maxima DSL: cos(_k__rx_xarmjoint5)
}

const MotionTransforms::Type_fr_xarmlink5_X_fr_xarmlink4& MotionTransforms::Type_fr_xarmlink5_X_fr_xarmlink4::update(const state_t& q)
{
    Scalar sin_q_xarmjoint5  = ScalarTraits::sin( q(XARMJOINT5) );
    Scalar cos_q_xarmjoint5  = ScalarTraits::cos( q(XARMJOINT5) );
    (*this)(0,0) = cos_q_xarmjoint5;
    (*this)(0,1) = cos_rx_xarmjoint5 * sin_q_xarmjoint5;
    (*this)(0,2) = sin_rx_xarmjoint5 * sin_q_xarmjoint5;
    (*this)(1,0) = -sin_q_xarmjoint5;
    (*this)(1,1) = cos_rx_xarmjoint5 * cos_q_xarmjoint5;
    (*this)(1,2) = sin_rx_xarmjoint5 * cos_q_xarmjoint5;
    (*this)(3,3) = cos_q_xarmjoint5;
    (*this)(3,4) = cos_rx_xarmjoint5 * sin_q_xarmjoint5;
    (*this)(3,5) = sin_rx_xarmjoint5 * sin_q_xarmjoint5;
    (*this)(4,3) = -sin_q_xarmjoint5;
    (*this)(4,4) = cos_rx_xarmjoint5 * cos_q_xarmjoint5;
    (*this)(4,5) = sin_rx_xarmjoint5 * cos_q_xarmjoint5;
    return *this;
}
MotionTransforms::Type_fr_xarmlink4_X_fr_xarmlink5::Type_fr_xarmlink4_X_fr_xarmlink5()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = -sin_rx_xarmjoint5;    // Maxima DSL: -sin(_k__rx_xarmjoint5)
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = cos_rx_xarmjoint5;    // Maxima DSL: cos(_k__rx_xarmjoint5)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = -sin_rx_xarmjoint5;    // Maxima DSL: -sin(_k__rx_xarmjoint5)
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = cos_rx_xarmjoint5;    // Maxima DSL: cos(_k__rx_xarmjoint5)
}

const MotionTransforms::Type_fr_xarmlink4_X_fr_xarmlink5& MotionTransforms::Type_fr_xarmlink4_X_fr_xarmlink5::update(const state_t& q)
{
    Scalar sin_q_xarmjoint5  = ScalarTraits::sin( q(XARMJOINT5) );
    Scalar cos_q_xarmjoint5  = ScalarTraits::cos( q(XARMJOINT5) );
    (*this)(0,0) = cos_q_xarmjoint5;
    (*this)(0,1) = -sin_q_xarmjoint5;
    (*this)(1,0) = cos_rx_xarmjoint5 * sin_q_xarmjoint5;
    (*this)(1,1) = cos_rx_xarmjoint5 * cos_q_xarmjoint5;
    (*this)(2,0) = sin_rx_xarmjoint5 * sin_q_xarmjoint5;
    (*this)(2,1) = sin_rx_xarmjoint5 * cos_q_xarmjoint5;
    (*this)(3,3) = cos_q_xarmjoint5;
    (*this)(3,4) = -sin_q_xarmjoint5;
    (*this)(4,3) = cos_rx_xarmjoint5 * sin_q_xarmjoint5;
    (*this)(4,4) = cos_rx_xarmjoint5 * cos_q_xarmjoint5;
    (*this)(5,3) = sin_rx_xarmjoint5 * sin_q_xarmjoint5;
    (*this)(5,4) = sin_rx_xarmjoint5 * cos_q_xarmjoint5;
    return *this;
}
MotionTransforms::Type_fr_xarmlink6_X_fr_xarmlink5::Type_fr_xarmlink6_X_fr_xarmlink5()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_xarmjoint6;    // Maxima DSL: -sin(_k__rx_xarmjoint6)
    (*this)(2,2) = cos_rx_xarmjoint6;    // Maxima DSL: cos(_k__rx_xarmjoint6)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(5,0) =  ty_xarmjoint6 * cos_rx_xarmjoint6;    // Maxima DSL: _k__ty_xarmjoint6*cos(_k__rx_xarmjoint6)
    (*this)(5,1) = - tx_xarmjoint6 * cos_rx_xarmjoint6;    // Maxima DSL: -_k__tx_xarmjoint6*cos(_k__rx_xarmjoint6)
    (*this)(5,2) = - tx_xarmjoint6 * sin_rx_xarmjoint6;    // Maxima DSL: -_k__tx_xarmjoint6*sin(_k__rx_xarmjoint6)
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -sin_rx_xarmjoint6;    // Maxima DSL: -sin(_k__rx_xarmjoint6)
    (*this)(5,5) = cos_rx_xarmjoint6;    // Maxima DSL: cos(_k__rx_xarmjoint6)
}

const MotionTransforms::Type_fr_xarmlink6_X_fr_xarmlink5& MotionTransforms::Type_fr_xarmlink6_X_fr_xarmlink5::update(const state_t& q)
{
    Scalar sin_q_xarmjoint6  = ScalarTraits::sin( q(XARMJOINT6) );
    Scalar cos_q_xarmjoint6  = ScalarTraits::cos( q(XARMJOINT6) );
    (*this)(0,0) = cos_q_xarmjoint6;
    (*this)(0,1) = cos_rx_xarmjoint6 * sin_q_xarmjoint6;
    (*this)(0,2) = sin_rx_xarmjoint6 * sin_q_xarmjoint6;
    (*this)(1,0) = -sin_q_xarmjoint6;
    (*this)(1,1) = cos_rx_xarmjoint6 * cos_q_xarmjoint6;
    (*this)(1,2) = sin_rx_xarmjoint6 * cos_q_xarmjoint6;
    (*this)(3,0) =  ty_xarmjoint6 * sin_rx_xarmjoint6 * sin_q_xarmjoint6;
    (*this)(3,1) = - tx_xarmjoint6 * sin_rx_xarmjoint6 * sin_q_xarmjoint6;
    (*this)(3,2) = ( tx_xarmjoint6 * cos_rx_xarmjoint6 * sin_q_xarmjoint6)-( ty_xarmjoint6 * cos_q_xarmjoint6);
    (*this)(3,3) = cos_q_xarmjoint6;
    (*this)(3,4) = cos_rx_xarmjoint6 * sin_q_xarmjoint6;
    (*this)(3,5) = sin_rx_xarmjoint6 * sin_q_xarmjoint6;
    (*this)(4,0) =  ty_xarmjoint6 * sin_rx_xarmjoint6 * cos_q_xarmjoint6;
    (*this)(4,1) = - tx_xarmjoint6 * sin_rx_xarmjoint6 * cos_q_xarmjoint6;
    (*this)(4,2) = ( ty_xarmjoint6 * sin_q_xarmjoint6)+( tx_xarmjoint6 * cos_rx_xarmjoint6 * cos_q_xarmjoint6);
    (*this)(4,3) = -sin_q_xarmjoint6;
    (*this)(4,4) = cos_rx_xarmjoint6 * cos_q_xarmjoint6;
    (*this)(4,5) = sin_rx_xarmjoint6 * cos_q_xarmjoint6;
    return *this;
}
MotionTransforms::Type_fr_xarmlink5_X_fr_xarmlink6::Type_fr_xarmlink5_X_fr_xarmlink6()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = -sin_rx_xarmjoint6;    // Maxima DSL: -sin(_k__rx_xarmjoint6)
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = cos_rx_xarmjoint6;    // Maxima DSL: cos(_k__rx_xarmjoint6)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) =  ty_xarmjoint6 * cos_rx_xarmjoint6;    // Maxima DSL: _k__ty_xarmjoint6*cos(_k__rx_xarmjoint6)
    (*this)(3,5) = 0.0;
    (*this)(4,2) = - tx_xarmjoint6 * cos_rx_xarmjoint6;    // Maxima DSL: -_k__tx_xarmjoint6*cos(_k__rx_xarmjoint6)
    (*this)(4,5) = -sin_rx_xarmjoint6;    // Maxima DSL: -sin(_k__rx_xarmjoint6)
    (*this)(5,2) = - tx_xarmjoint6 * sin_rx_xarmjoint6;    // Maxima DSL: -_k__tx_xarmjoint6*sin(_k__rx_xarmjoint6)
    (*this)(5,5) = cos_rx_xarmjoint6;    // Maxima DSL: cos(_k__rx_xarmjoint6)
}

const MotionTransforms::Type_fr_xarmlink5_X_fr_xarmlink6& MotionTransforms::Type_fr_xarmlink5_X_fr_xarmlink6::update(const state_t& q)
{
    Scalar sin_q_xarmjoint6  = ScalarTraits::sin( q(XARMJOINT6) );
    Scalar cos_q_xarmjoint6  = ScalarTraits::cos( q(XARMJOINT6) );
    (*this)(0,0) = cos_q_xarmjoint6;
    (*this)(0,1) = -sin_q_xarmjoint6;
    (*this)(1,0) = cos_rx_xarmjoint6 * sin_q_xarmjoint6;
    (*this)(1,1) = cos_rx_xarmjoint6 * cos_q_xarmjoint6;
    (*this)(2,0) = sin_rx_xarmjoint6 * sin_q_xarmjoint6;
    (*this)(2,1) = sin_rx_xarmjoint6 * cos_q_xarmjoint6;
    (*this)(3,0) =  ty_xarmjoint6 * sin_rx_xarmjoint6 * sin_q_xarmjoint6;
    (*this)(3,1) =  ty_xarmjoint6 * sin_rx_xarmjoint6 * cos_q_xarmjoint6;
    (*this)(3,3) = cos_q_xarmjoint6;
    (*this)(3,4) = -sin_q_xarmjoint6;
    (*this)(4,0) = - tx_xarmjoint6 * sin_rx_xarmjoint6 * sin_q_xarmjoint6;
    (*this)(4,1) = - tx_xarmjoint6 * sin_rx_xarmjoint6 * cos_q_xarmjoint6;
    (*this)(4,3) = cos_rx_xarmjoint6 * sin_q_xarmjoint6;
    (*this)(4,4) = cos_rx_xarmjoint6 * cos_q_xarmjoint6;
    (*this)(5,0) = ( tx_xarmjoint6 * cos_rx_xarmjoint6 * sin_q_xarmjoint6)-( ty_xarmjoint6 * cos_q_xarmjoint6);
    (*this)(5,1) = ( ty_xarmjoint6 * sin_q_xarmjoint6)+( tx_xarmjoint6 * cos_rx_xarmjoint6 * cos_q_xarmjoint6);
    (*this)(5,3) = sin_rx_xarmjoint6 * sin_q_xarmjoint6;
    (*this)(5,4) = sin_rx_xarmjoint6 * cos_q_xarmjoint6;
    return *this;
}

ForceTransforms::Type_fr_omniwheel_fl_X_fr_base_link_footprint::Type_fr_omniwheel_fl_X_fr_base_link_footprint()
{
    (*this)(2,0) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(2,1) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - 0.707106 *  tz_motor_wheel_joint_fl;    // Maxima DSL: -0.707106*_k__tz_motor_wheel_joint_fl
    (*this)(2,4) =  0.707106 *  tz_motor_wheel_joint_fl;    // Maxima DSL: 0.707106*_k__tz_motor_wheel_joint_fl
    (*this)(2,5) = - 0.5 * (( 1.41421 *  ty_motor_wheel_joint_fl)-( 1.41421 *  tx_motor_wheel_joint_fl));    // Maxima DSL: -0.5*(1.41421*_k__ty_motor_wheel_joint_fl-1.41421*_k__tx_motor_wheel_joint_fl)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(5,4) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_omniwheel_fl_X_fr_base_link_footprint& ForceTransforms::Type_fr_omniwheel_fl_X_fr_base_link_footprint::update(const state_t& q)
{
    Scalar sin_q_motor_wheel_joint_fl  = ScalarTraits::sin( q(MOTOR_WHEEL_JOINT_FL) );
    Scalar cos_q_motor_wheel_joint_fl  = ScalarTraits::cos( q(MOTOR_WHEEL_JOINT_FL) );
    (*this)(0,0) = - 0.5 * (sin_q_motor_wheel_joint_fl-cos_q_motor_wheel_joint_fl);
    (*this)(0,1) =  0.5 * (sin_q_motor_wheel_joint_fl-cos_q_motor_wheel_joint_fl);
    (*this)(0,2) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fl)+( 1.41421 * cos_q_motor_wheel_joint_fl));
    (*this)(0,3) = - 0.5 * ((( tz_motor_wheel_joint_fl+( 1.41421 *  ty_motor_wheel_joint_fl)) * sin_q_motor_wheel_joint_fl)+((( 1.41421 *  ty_motor_wheel_joint_fl)- tz_motor_wheel_joint_fl) * cos_q_motor_wheel_joint_fl));
    (*this)(0,4) = - 0.5 * ((( tz_motor_wheel_joint_fl-( 1.41421 *  tx_motor_wheel_joint_fl)) * sin_q_motor_wheel_joint_fl)+((- tz_motor_wheel_joint_fl-( 1.41421 *  tx_motor_wheel_joint_fl)) * cos_q_motor_wheel_joint_fl));
    (*this)(0,5) =  0.5 * ((( ty_motor_wheel_joint_fl+ tx_motor_wheel_joint_fl) * sin_q_motor_wheel_joint_fl)+((- ty_motor_wheel_joint_fl- tx_motor_wheel_joint_fl) * cos_q_motor_wheel_joint_fl));
    (*this)(1,0) = - 0.5 * (sin_q_motor_wheel_joint_fl+cos_q_motor_wheel_joint_fl);
    (*this)(1,1) =  0.5 * (sin_q_motor_wheel_joint_fl+cos_q_motor_wheel_joint_fl);
    (*this)(1,2) =  0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fl)-( 1.41421 * cos_q_motor_wheel_joint_fl));
    (*this)(1,3) = - 0.5 * ((( tz_motor_wheel_joint_fl-( 1.41421 *  ty_motor_wheel_joint_fl)) * sin_q_motor_wheel_joint_fl)+(( tz_motor_wheel_joint_fl+( 1.41421 *  ty_motor_wheel_joint_fl)) * cos_q_motor_wheel_joint_fl));
    (*this)(1,4) = - 0.5 * ((( tz_motor_wheel_joint_fl+( 1.41421 *  tx_motor_wheel_joint_fl)) * sin_q_motor_wheel_joint_fl)+(( tz_motor_wheel_joint_fl-( 1.41421 *  tx_motor_wheel_joint_fl)) * cos_q_motor_wheel_joint_fl));
    (*this)(1,5) =  0.5 * ((( ty_motor_wheel_joint_fl+ tx_motor_wheel_joint_fl) * sin_q_motor_wheel_joint_fl)+(( ty_motor_wheel_joint_fl+ tx_motor_wheel_joint_fl) * cos_q_motor_wheel_joint_fl));
    (*this)(3,3) = - 0.5 * (sin_q_motor_wheel_joint_fl-cos_q_motor_wheel_joint_fl);
    (*this)(3,4) =  0.5 * (sin_q_motor_wheel_joint_fl-cos_q_motor_wheel_joint_fl);
    (*this)(3,5) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fl)+( 1.41421 * cos_q_motor_wheel_joint_fl));
    (*this)(4,3) = - 0.5 * (sin_q_motor_wheel_joint_fl+cos_q_motor_wheel_joint_fl);
    (*this)(4,4) =  0.5 * (sin_q_motor_wheel_joint_fl+cos_q_motor_wheel_joint_fl);
    (*this)(4,5) =  0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fl)-( 1.41421 * cos_q_motor_wheel_joint_fl));
    return *this;
}
ForceTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_fl::Type_fr_base_link_footprint_X_fr_omniwheel_fl()
{
    (*this)(0,2) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(0,5) = - 0.707106 *  tz_motor_wheel_joint_fl;    // Maxima DSL: -0.707106*_k__tz_motor_wheel_joint_fl
    (*this)(1,2) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(1,5) =  0.707106 *  tz_motor_wheel_joint_fl;    // Maxima DSL: 0.707106*_k__tz_motor_wheel_joint_fl
    (*this)(2,2) = 0.0;
    (*this)(2,5) = - 0.5 * (( 1.41421 *  ty_motor_wheel_joint_fl)-( 1.41421 *  tx_motor_wheel_joint_fl));    // Maxima DSL: -0.5*(1.41421*_k__ty_motor_wheel_joint_fl-1.41421*_k__tx_motor_wheel_joint_fl)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_fl& ForceTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_fl::update(const state_t& q)
{
    Scalar sin_q_motor_wheel_joint_fl  = ScalarTraits::sin( q(MOTOR_WHEEL_JOINT_FL) );
    Scalar cos_q_motor_wheel_joint_fl  = ScalarTraits::cos( q(MOTOR_WHEEL_JOINT_FL) );
    (*this)(0,0) = - 0.5 * (sin_q_motor_wheel_joint_fl-cos_q_motor_wheel_joint_fl);
    (*this)(0,1) = - 0.5 * (sin_q_motor_wheel_joint_fl+cos_q_motor_wheel_joint_fl);
    (*this)(0,3) = - 0.5 * ((( tz_motor_wheel_joint_fl+( 1.41421 *  ty_motor_wheel_joint_fl)) * sin_q_motor_wheel_joint_fl)+((( 1.41421 *  ty_motor_wheel_joint_fl)- tz_motor_wheel_joint_fl) * cos_q_motor_wheel_joint_fl));
    (*this)(0,4) = - 0.5 * ((( tz_motor_wheel_joint_fl-( 1.41421 *  ty_motor_wheel_joint_fl)) * sin_q_motor_wheel_joint_fl)+(( tz_motor_wheel_joint_fl+( 1.41421 *  ty_motor_wheel_joint_fl)) * cos_q_motor_wheel_joint_fl));
    (*this)(1,0) =  0.5 * (sin_q_motor_wheel_joint_fl-cos_q_motor_wheel_joint_fl);
    (*this)(1,1) =  0.5 * (sin_q_motor_wheel_joint_fl+cos_q_motor_wheel_joint_fl);
    (*this)(1,3) = - 0.5 * ((( tz_motor_wheel_joint_fl-( 1.41421 *  tx_motor_wheel_joint_fl)) * sin_q_motor_wheel_joint_fl)+((- tz_motor_wheel_joint_fl-( 1.41421 *  tx_motor_wheel_joint_fl)) * cos_q_motor_wheel_joint_fl));
    (*this)(1,4) = - 0.5 * ((( tz_motor_wheel_joint_fl+( 1.41421 *  tx_motor_wheel_joint_fl)) * sin_q_motor_wheel_joint_fl)+(( tz_motor_wheel_joint_fl-( 1.41421 *  tx_motor_wheel_joint_fl)) * cos_q_motor_wheel_joint_fl));
    (*this)(2,0) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fl)+( 1.41421 * cos_q_motor_wheel_joint_fl));
    (*this)(2,1) =  0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fl)-( 1.41421 * cos_q_motor_wheel_joint_fl));
    (*this)(2,3) =  0.5 * ((( ty_motor_wheel_joint_fl+ tx_motor_wheel_joint_fl) * sin_q_motor_wheel_joint_fl)+((- ty_motor_wheel_joint_fl- tx_motor_wheel_joint_fl) * cos_q_motor_wheel_joint_fl));
    (*this)(2,4) =  0.5 * ((( ty_motor_wheel_joint_fl+ tx_motor_wheel_joint_fl) * sin_q_motor_wheel_joint_fl)+(( ty_motor_wheel_joint_fl+ tx_motor_wheel_joint_fl) * cos_q_motor_wheel_joint_fl));
    (*this)(3,3) = - 0.5 * (sin_q_motor_wheel_joint_fl-cos_q_motor_wheel_joint_fl);
    (*this)(3,4) = - 0.5 * (sin_q_motor_wheel_joint_fl+cos_q_motor_wheel_joint_fl);
    (*this)(4,3) =  0.5 * (sin_q_motor_wheel_joint_fl-cos_q_motor_wheel_joint_fl);
    (*this)(4,4) =  0.5 * (sin_q_motor_wheel_joint_fl+cos_q_motor_wheel_joint_fl);
    (*this)(5,3) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fl)+( 1.41421 * cos_q_motor_wheel_joint_fl));
    (*this)(5,4) =  0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fl)-( 1.41421 * cos_q_motor_wheel_joint_fl));
    return *this;
}
ForceTransforms::Type_fr_omniwheel_fr_X_fr_base_link_footprint::Type_fr_omniwheel_fr_X_fr_base_link_footprint()
{
    (*this)(2,0) = - 0.707106;    // Maxima DSL: -0.707106
    (*this)(2,1) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - 0.707106 *  tz_motor_wheel_joint_fr;    // Maxima DSL: -0.707106*_k__tz_motor_wheel_joint_fr
    (*this)(2,4) = - 0.707106 *  tz_motor_wheel_joint_fr;    // Maxima DSL: -0.707106*_k__tz_motor_wheel_joint_fr
    (*this)(2,5) =  0.5 * (( 1.41421 *  ty_motor_wheel_joint_fr)+( 1.41421 *  tx_motor_wheel_joint_fr));    // Maxima DSL: 0.5*(1.41421*_k__ty_motor_wheel_joint_fr+1.41421*_k__tx_motor_wheel_joint_fr)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = - 0.707106;    // Maxima DSL: -0.707106
    (*this)(5,4) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_omniwheel_fr_X_fr_base_link_footprint& ForceTransforms::Type_fr_omniwheel_fr_X_fr_base_link_footprint::update(const state_t& q)
{
    Scalar sin_q_motor_wheel_joint_fr  = ScalarTraits::sin( q(MOTOR_WHEEL_JOINT_FR) );
    Scalar cos_q_motor_wheel_joint_fr  = ScalarTraits::cos( q(MOTOR_WHEEL_JOINT_FR) );
    (*this)(0,0) =  0.5 * (sin_q_motor_wheel_joint_fr+cos_q_motor_wheel_joint_fr);
    (*this)(0,1) =  0.5 * (sin_q_motor_wheel_joint_fr+cos_q_motor_wheel_joint_fr);
    (*this)(0,2) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fr)-( 1.41421 * cos_q_motor_wheel_joint_fr));
    (*this)(0,3) = - 0.5 * ((( tz_motor_wheel_joint_fr+( 1.41421 *  ty_motor_wheel_joint_fr)) * sin_q_motor_wheel_joint_fr)+(( tz_motor_wheel_joint_fr-( 1.41421 *  ty_motor_wheel_joint_fr)) * cos_q_motor_wheel_joint_fr));
    (*this)(0,4) =  0.5 * ((( tz_motor_wheel_joint_fr+( 1.41421 *  tx_motor_wheel_joint_fr)) * sin_q_motor_wheel_joint_fr)+(( tz_motor_wheel_joint_fr-( 1.41421 *  tx_motor_wheel_joint_fr)) * cos_q_motor_wheel_joint_fr));
    (*this)(0,5) = - 0.5 * ((( ty_motor_wheel_joint_fr- tx_motor_wheel_joint_fr) * sin_q_motor_wheel_joint_fr)+(( ty_motor_wheel_joint_fr- tx_motor_wheel_joint_fr) * cos_q_motor_wheel_joint_fr));
    (*this)(1,0) = - 0.5 * (sin_q_motor_wheel_joint_fr-cos_q_motor_wheel_joint_fr);
    (*this)(1,1) = - 0.5 * (sin_q_motor_wheel_joint_fr-cos_q_motor_wheel_joint_fr);
    (*this)(1,2) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fr)+( 1.41421 * cos_q_motor_wheel_joint_fr));
    (*this)(1,3) =  0.5 * ((( tz_motor_wheel_joint_fr-( 1.41421 *  ty_motor_wheel_joint_fr)) * sin_q_motor_wheel_joint_fr)+((- tz_motor_wheel_joint_fr-( 1.41421 *  ty_motor_wheel_joint_fr)) * cos_q_motor_wheel_joint_fr));
    (*this)(1,4) = - 0.5 * ((( tz_motor_wheel_joint_fr-( 1.41421 *  tx_motor_wheel_joint_fr)) * sin_q_motor_wheel_joint_fr)+((- tz_motor_wheel_joint_fr-( 1.41421 *  tx_motor_wheel_joint_fr)) * cos_q_motor_wheel_joint_fr));
    (*this)(1,5) =  0.5 * ((( ty_motor_wheel_joint_fr- tx_motor_wheel_joint_fr) * sin_q_motor_wheel_joint_fr)+(( tx_motor_wheel_joint_fr- ty_motor_wheel_joint_fr) * cos_q_motor_wheel_joint_fr));
    (*this)(3,3) =  0.5 * (sin_q_motor_wheel_joint_fr+cos_q_motor_wheel_joint_fr);
    (*this)(3,4) =  0.5 * (sin_q_motor_wheel_joint_fr+cos_q_motor_wheel_joint_fr);
    (*this)(3,5) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fr)-( 1.41421 * cos_q_motor_wheel_joint_fr));
    (*this)(4,3) = - 0.5 * (sin_q_motor_wheel_joint_fr-cos_q_motor_wheel_joint_fr);
    (*this)(4,4) = - 0.5 * (sin_q_motor_wheel_joint_fr-cos_q_motor_wheel_joint_fr);
    (*this)(4,5) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fr)+( 1.41421 * cos_q_motor_wheel_joint_fr));
    return *this;
}
ForceTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_fr::Type_fr_base_link_footprint_X_fr_omniwheel_fr()
{
    (*this)(0,2) = - 0.707106;    // Maxima DSL: -0.707106
    (*this)(0,5) = - 0.707106 *  tz_motor_wheel_joint_fr;    // Maxima DSL: -0.707106*_k__tz_motor_wheel_joint_fr
    (*this)(1,2) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(1,5) = - 0.707106 *  tz_motor_wheel_joint_fr;    // Maxima DSL: -0.707106*_k__tz_motor_wheel_joint_fr
    (*this)(2,2) = 0.0;
    (*this)(2,5) =  0.5 * (( 1.41421 *  ty_motor_wheel_joint_fr)+( 1.41421 *  tx_motor_wheel_joint_fr));    // Maxima DSL: 0.5*(1.41421*_k__ty_motor_wheel_joint_fr+1.41421*_k__tx_motor_wheel_joint_fr)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = - 0.707106;    // Maxima DSL: -0.707106
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_fr& ForceTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_fr::update(const state_t& q)
{
    Scalar sin_q_motor_wheel_joint_fr  = ScalarTraits::sin( q(MOTOR_WHEEL_JOINT_FR) );
    Scalar cos_q_motor_wheel_joint_fr  = ScalarTraits::cos( q(MOTOR_WHEEL_JOINT_FR) );
    (*this)(0,0) =  0.5 * (sin_q_motor_wheel_joint_fr+cos_q_motor_wheel_joint_fr);
    (*this)(0,1) = - 0.5 * (sin_q_motor_wheel_joint_fr-cos_q_motor_wheel_joint_fr);
    (*this)(0,3) = - 0.5 * ((( tz_motor_wheel_joint_fr+( 1.41421 *  ty_motor_wheel_joint_fr)) * sin_q_motor_wheel_joint_fr)+(( tz_motor_wheel_joint_fr-( 1.41421 *  ty_motor_wheel_joint_fr)) * cos_q_motor_wheel_joint_fr));
    (*this)(0,4) =  0.5 * ((( tz_motor_wheel_joint_fr-( 1.41421 *  ty_motor_wheel_joint_fr)) * sin_q_motor_wheel_joint_fr)+((- tz_motor_wheel_joint_fr-( 1.41421 *  ty_motor_wheel_joint_fr)) * cos_q_motor_wheel_joint_fr));
    (*this)(1,0) =  0.5 * (sin_q_motor_wheel_joint_fr+cos_q_motor_wheel_joint_fr);
    (*this)(1,1) = - 0.5 * (sin_q_motor_wheel_joint_fr-cos_q_motor_wheel_joint_fr);
    (*this)(1,3) =  0.5 * ((( tz_motor_wheel_joint_fr+( 1.41421 *  tx_motor_wheel_joint_fr)) * sin_q_motor_wheel_joint_fr)+(( tz_motor_wheel_joint_fr-( 1.41421 *  tx_motor_wheel_joint_fr)) * cos_q_motor_wheel_joint_fr));
    (*this)(1,4) = - 0.5 * ((( tz_motor_wheel_joint_fr-( 1.41421 *  tx_motor_wheel_joint_fr)) * sin_q_motor_wheel_joint_fr)+((- tz_motor_wheel_joint_fr-( 1.41421 *  tx_motor_wheel_joint_fr)) * cos_q_motor_wheel_joint_fr));
    (*this)(2,0) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fr)-( 1.41421 * cos_q_motor_wheel_joint_fr));
    (*this)(2,1) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fr)+( 1.41421 * cos_q_motor_wheel_joint_fr));
    (*this)(2,3) = - 0.5 * ((( ty_motor_wheel_joint_fr- tx_motor_wheel_joint_fr) * sin_q_motor_wheel_joint_fr)+(( ty_motor_wheel_joint_fr- tx_motor_wheel_joint_fr) * cos_q_motor_wheel_joint_fr));
    (*this)(2,4) =  0.5 * ((( ty_motor_wheel_joint_fr- tx_motor_wheel_joint_fr) * sin_q_motor_wheel_joint_fr)+(( tx_motor_wheel_joint_fr- ty_motor_wheel_joint_fr) * cos_q_motor_wheel_joint_fr));
    (*this)(3,3) =  0.5 * (sin_q_motor_wheel_joint_fr+cos_q_motor_wheel_joint_fr);
    (*this)(3,4) = - 0.5 * (sin_q_motor_wheel_joint_fr-cos_q_motor_wheel_joint_fr);
    (*this)(4,3) =  0.5 * (sin_q_motor_wheel_joint_fr+cos_q_motor_wheel_joint_fr);
    (*this)(4,4) = - 0.5 * (sin_q_motor_wheel_joint_fr-cos_q_motor_wheel_joint_fr);
    (*this)(5,3) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fr)-( 1.41421 * cos_q_motor_wheel_joint_fr));
    (*this)(5,4) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fr)+( 1.41421 * cos_q_motor_wheel_joint_fr));
    return *this;
}
ForceTransforms::Type_fr_omniwheel_bl_X_fr_base_link_footprint::Type_fr_omniwheel_bl_X_fr_base_link_footprint()
{
    (*this)(2,0) = - 0.707106;    // Maxima DSL: -0.707106
    (*this)(2,1) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - 0.707106 *  tz_motor_wheel_joint_bl;    // Maxima DSL: -0.707106*_k__tz_motor_wheel_joint_bl
    (*this)(2,4) = - 0.707106 *  tz_motor_wheel_joint_bl;    // Maxima DSL: -0.707106*_k__tz_motor_wheel_joint_bl
    (*this)(2,5) =  0.5 * (( 1.41421 *  ty_motor_wheel_joint_bl)+( 1.41421 *  tx_motor_wheel_joint_bl));    // Maxima DSL: 0.5*(1.41421*_k__ty_motor_wheel_joint_bl+1.41421*_k__tx_motor_wheel_joint_bl)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = - 0.707106;    // Maxima DSL: -0.707106
    (*this)(5,4) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_omniwheel_bl_X_fr_base_link_footprint& ForceTransforms::Type_fr_omniwheel_bl_X_fr_base_link_footprint::update(const state_t& q)
{
    Scalar sin_q_motor_wheel_joint_bl  = ScalarTraits::sin( q(MOTOR_WHEEL_JOINT_BL) );
    Scalar cos_q_motor_wheel_joint_bl  = ScalarTraits::cos( q(MOTOR_WHEEL_JOINT_BL) );
    (*this)(0,0) =  0.5 * (sin_q_motor_wheel_joint_bl+cos_q_motor_wheel_joint_bl);
    (*this)(0,1) =  0.5 * (sin_q_motor_wheel_joint_bl+cos_q_motor_wheel_joint_bl);
    (*this)(0,2) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_bl)-( 1.41421 * cos_q_motor_wheel_joint_bl));
    (*this)(0,3) = - 0.5 * ((( tz_motor_wheel_joint_bl+( 1.41421 *  ty_motor_wheel_joint_bl)) * sin_q_motor_wheel_joint_bl)+(( tz_motor_wheel_joint_bl-( 1.41421 *  ty_motor_wheel_joint_bl)) * cos_q_motor_wheel_joint_bl));
    (*this)(0,4) =  0.5 * ((( tz_motor_wheel_joint_bl+( 1.41421 *  tx_motor_wheel_joint_bl)) * sin_q_motor_wheel_joint_bl)+(( tz_motor_wheel_joint_bl-( 1.41421 *  tx_motor_wheel_joint_bl)) * cos_q_motor_wheel_joint_bl));
    (*this)(0,5) = - 0.5 * ((( ty_motor_wheel_joint_bl- tx_motor_wheel_joint_bl) * sin_q_motor_wheel_joint_bl)+(( ty_motor_wheel_joint_bl- tx_motor_wheel_joint_bl) * cos_q_motor_wheel_joint_bl));
    (*this)(1,0) = - 0.5 * (sin_q_motor_wheel_joint_bl-cos_q_motor_wheel_joint_bl);
    (*this)(1,1) = - 0.5 * (sin_q_motor_wheel_joint_bl-cos_q_motor_wheel_joint_bl);
    (*this)(1,2) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_bl)+( 1.41421 * cos_q_motor_wheel_joint_bl));
    (*this)(1,3) =  0.5 * ((( tz_motor_wheel_joint_bl-( 1.41421 *  ty_motor_wheel_joint_bl)) * sin_q_motor_wheel_joint_bl)+((- tz_motor_wheel_joint_bl-( 1.41421 *  ty_motor_wheel_joint_bl)) * cos_q_motor_wheel_joint_bl));
    (*this)(1,4) = - 0.5 * ((( tz_motor_wheel_joint_bl-( 1.41421 *  tx_motor_wheel_joint_bl)) * sin_q_motor_wheel_joint_bl)+((- tz_motor_wheel_joint_bl-( 1.41421 *  tx_motor_wheel_joint_bl)) * cos_q_motor_wheel_joint_bl));
    (*this)(1,5) =  0.5 * ((( ty_motor_wheel_joint_bl- tx_motor_wheel_joint_bl) * sin_q_motor_wheel_joint_bl)+(( tx_motor_wheel_joint_bl- ty_motor_wheel_joint_bl) * cos_q_motor_wheel_joint_bl));
    (*this)(3,3) =  0.5 * (sin_q_motor_wheel_joint_bl+cos_q_motor_wheel_joint_bl);
    (*this)(3,4) =  0.5 * (sin_q_motor_wheel_joint_bl+cos_q_motor_wheel_joint_bl);
    (*this)(3,5) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_bl)-( 1.41421 * cos_q_motor_wheel_joint_bl));
    (*this)(4,3) = - 0.5 * (sin_q_motor_wheel_joint_bl-cos_q_motor_wheel_joint_bl);
    (*this)(4,4) = - 0.5 * (sin_q_motor_wheel_joint_bl-cos_q_motor_wheel_joint_bl);
    (*this)(4,5) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_bl)+( 1.41421 * cos_q_motor_wheel_joint_bl));
    return *this;
}
ForceTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_bl::Type_fr_base_link_footprint_X_fr_omniwheel_bl()
{
    (*this)(0,2) = - 0.707106;    // Maxima DSL: -0.707106
    (*this)(0,5) = - 0.707106 *  tz_motor_wheel_joint_bl;    // Maxima DSL: -0.707106*_k__tz_motor_wheel_joint_bl
    (*this)(1,2) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(1,5) = - 0.707106 *  tz_motor_wheel_joint_bl;    // Maxima DSL: -0.707106*_k__tz_motor_wheel_joint_bl
    (*this)(2,2) = 0.0;
    (*this)(2,5) =  0.5 * (( 1.41421 *  ty_motor_wheel_joint_bl)+( 1.41421 *  tx_motor_wheel_joint_bl));    // Maxima DSL: 0.5*(1.41421*_k__ty_motor_wheel_joint_bl+1.41421*_k__tx_motor_wheel_joint_bl)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = - 0.707106;    // Maxima DSL: -0.707106
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_bl& ForceTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_bl::update(const state_t& q)
{
    Scalar sin_q_motor_wheel_joint_bl  = ScalarTraits::sin( q(MOTOR_WHEEL_JOINT_BL) );
    Scalar cos_q_motor_wheel_joint_bl  = ScalarTraits::cos( q(MOTOR_WHEEL_JOINT_BL) );
    (*this)(0,0) =  0.5 * (sin_q_motor_wheel_joint_bl+cos_q_motor_wheel_joint_bl);
    (*this)(0,1) = - 0.5 * (sin_q_motor_wheel_joint_bl-cos_q_motor_wheel_joint_bl);
    (*this)(0,3) = - 0.5 * ((( tz_motor_wheel_joint_bl+( 1.41421 *  ty_motor_wheel_joint_bl)) * sin_q_motor_wheel_joint_bl)+(( tz_motor_wheel_joint_bl-( 1.41421 *  ty_motor_wheel_joint_bl)) * cos_q_motor_wheel_joint_bl));
    (*this)(0,4) =  0.5 * ((( tz_motor_wheel_joint_bl-( 1.41421 *  ty_motor_wheel_joint_bl)) * sin_q_motor_wheel_joint_bl)+((- tz_motor_wheel_joint_bl-( 1.41421 *  ty_motor_wheel_joint_bl)) * cos_q_motor_wheel_joint_bl));
    (*this)(1,0) =  0.5 * (sin_q_motor_wheel_joint_bl+cos_q_motor_wheel_joint_bl);
    (*this)(1,1) = - 0.5 * (sin_q_motor_wheel_joint_bl-cos_q_motor_wheel_joint_bl);
    (*this)(1,3) =  0.5 * ((( tz_motor_wheel_joint_bl+( 1.41421 *  tx_motor_wheel_joint_bl)) * sin_q_motor_wheel_joint_bl)+(( tz_motor_wheel_joint_bl-( 1.41421 *  tx_motor_wheel_joint_bl)) * cos_q_motor_wheel_joint_bl));
    (*this)(1,4) = - 0.5 * ((( tz_motor_wheel_joint_bl-( 1.41421 *  tx_motor_wheel_joint_bl)) * sin_q_motor_wheel_joint_bl)+((- tz_motor_wheel_joint_bl-( 1.41421 *  tx_motor_wheel_joint_bl)) * cos_q_motor_wheel_joint_bl));
    (*this)(2,0) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_bl)-( 1.41421 * cos_q_motor_wheel_joint_bl));
    (*this)(2,1) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_bl)+( 1.41421 * cos_q_motor_wheel_joint_bl));
    (*this)(2,3) = - 0.5 * ((( ty_motor_wheel_joint_bl- tx_motor_wheel_joint_bl) * sin_q_motor_wheel_joint_bl)+(( ty_motor_wheel_joint_bl- tx_motor_wheel_joint_bl) * cos_q_motor_wheel_joint_bl));
    (*this)(2,4) =  0.5 * ((( ty_motor_wheel_joint_bl- tx_motor_wheel_joint_bl) * sin_q_motor_wheel_joint_bl)+(( tx_motor_wheel_joint_bl- ty_motor_wheel_joint_bl) * cos_q_motor_wheel_joint_bl));
    (*this)(3,3) =  0.5 * (sin_q_motor_wheel_joint_bl+cos_q_motor_wheel_joint_bl);
    (*this)(3,4) = - 0.5 * (sin_q_motor_wheel_joint_bl-cos_q_motor_wheel_joint_bl);
    (*this)(4,3) =  0.5 * (sin_q_motor_wheel_joint_bl+cos_q_motor_wheel_joint_bl);
    (*this)(4,4) = - 0.5 * (sin_q_motor_wheel_joint_bl-cos_q_motor_wheel_joint_bl);
    (*this)(5,3) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_bl)-( 1.41421 * cos_q_motor_wheel_joint_bl));
    (*this)(5,4) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_bl)+( 1.41421 * cos_q_motor_wheel_joint_bl));
    return *this;
}
ForceTransforms::Type_fr_omniwheel_br_X_fr_base_link_footprint::Type_fr_omniwheel_br_X_fr_base_link_footprint()
{
    (*this)(2,0) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(2,1) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - 0.707106 *  tz_motor_wheel_joint_br;    // Maxima DSL: -0.707106*_k__tz_motor_wheel_joint_br
    (*this)(2,4) =  0.707106 *  tz_motor_wheel_joint_br;    // Maxima DSL: 0.707106*_k__tz_motor_wheel_joint_br
    (*this)(2,5) = - 0.5 * (( 1.41421 *  ty_motor_wheel_joint_br)-( 1.41421 *  tx_motor_wheel_joint_br));    // Maxima DSL: -0.5*(1.41421*_k__ty_motor_wheel_joint_br-1.41421*_k__tx_motor_wheel_joint_br)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(5,4) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_omniwheel_br_X_fr_base_link_footprint& ForceTransforms::Type_fr_omniwheel_br_X_fr_base_link_footprint::update(const state_t& q)
{
    Scalar sin_q_motor_wheel_joint_br  = ScalarTraits::sin( q(MOTOR_WHEEL_JOINT_BR) );
    Scalar cos_q_motor_wheel_joint_br  = ScalarTraits::cos( q(MOTOR_WHEEL_JOINT_BR) );
    (*this)(0,0) = - 0.5 * (sin_q_motor_wheel_joint_br-cos_q_motor_wheel_joint_br);
    (*this)(0,1) =  0.5 * (sin_q_motor_wheel_joint_br-cos_q_motor_wheel_joint_br);
    (*this)(0,2) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_br)+( 1.41421 * cos_q_motor_wheel_joint_br));
    (*this)(0,3) = - 0.5 * ((( tz_motor_wheel_joint_br+( 1.41421 *  ty_motor_wheel_joint_br)) * sin_q_motor_wheel_joint_br)+((( 1.41421 *  ty_motor_wheel_joint_br)- tz_motor_wheel_joint_br) * cos_q_motor_wheel_joint_br));
    (*this)(0,4) = - 0.5 * ((( tz_motor_wheel_joint_br-( 1.41421 *  tx_motor_wheel_joint_br)) * sin_q_motor_wheel_joint_br)+((- tz_motor_wheel_joint_br-( 1.41421 *  tx_motor_wheel_joint_br)) * cos_q_motor_wheel_joint_br));
    (*this)(0,5) =  0.5 * ((( ty_motor_wheel_joint_br+ tx_motor_wheel_joint_br) * sin_q_motor_wheel_joint_br)+((- ty_motor_wheel_joint_br- tx_motor_wheel_joint_br) * cos_q_motor_wheel_joint_br));
    (*this)(1,0) = - 0.5 * (sin_q_motor_wheel_joint_br+cos_q_motor_wheel_joint_br);
    (*this)(1,1) =  0.5 * (sin_q_motor_wheel_joint_br+cos_q_motor_wheel_joint_br);
    (*this)(1,2) =  0.5 * (( 1.41421 * sin_q_motor_wheel_joint_br)-( 1.41421 * cos_q_motor_wheel_joint_br));
    (*this)(1,3) = - 0.5 * ((( tz_motor_wheel_joint_br-( 1.41421 *  ty_motor_wheel_joint_br)) * sin_q_motor_wheel_joint_br)+(( tz_motor_wheel_joint_br+( 1.41421 *  ty_motor_wheel_joint_br)) * cos_q_motor_wheel_joint_br));
    (*this)(1,4) = - 0.5 * ((( tz_motor_wheel_joint_br+( 1.41421 *  tx_motor_wheel_joint_br)) * sin_q_motor_wheel_joint_br)+(( tz_motor_wheel_joint_br-( 1.41421 *  tx_motor_wheel_joint_br)) * cos_q_motor_wheel_joint_br));
    (*this)(1,5) =  0.5 * ((( ty_motor_wheel_joint_br+ tx_motor_wheel_joint_br) * sin_q_motor_wheel_joint_br)+(( ty_motor_wheel_joint_br+ tx_motor_wheel_joint_br) * cos_q_motor_wheel_joint_br));
    (*this)(3,3) = - 0.5 * (sin_q_motor_wheel_joint_br-cos_q_motor_wheel_joint_br);
    (*this)(3,4) =  0.5 * (sin_q_motor_wheel_joint_br-cos_q_motor_wheel_joint_br);
    (*this)(3,5) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_br)+( 1.41421 * cos_q_motor_wheel_joint_br));
    (*this)(4,3) = - 0.5 * (sin_q_motor_wheel_joint_br+cos_q_motor_wheel_joint_br);
    (*this)(4,4) =  0.5 * (sin_q_motor_wheel_joint_br+cos_q_motor_wheel_joint_br);
    (*this)(4,5) =  0.5 * (( 1.41421 * sin_q_motor_wheel_joint_br)-( 1.41421 * cos_q_motor_wheel_joint_br));
    return *this;
}
ForceTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_br::Type_fr_base_link_footprint_X_fr_omniwheel_br()
{
    (*this)(0,2) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(0,5) = - 0.707106 *  tz_motor_wheel_joint_br;    // Maxima DSL: -0.707106*_k__tz_motor_wheel_joint_br
    (*this)(1,2) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(1,5) =  0.707106 *  tz_motor_wheel_joint_br;    // Maxima DSL: 0.707106*_k__tz_motor_wheel_joint_br
    (*this)(2,2) = 0.0;
    (*this)(2,5) = - 0.5 * (( 1.41421 *  ty_motor_wheel_joint_br)-( 1.41421 *  tx_motor_wheel_joint_br));    // Maxima DSL: -0.5*(1.41421*_k__ty_motor_wheel_joint_br-1.41421*_k__tx_motor_wheel_joint_br)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_br& ForceTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_br::update(const state_t& q)
{
    Scalar sin_q_motor_wheel_joint_br  = ScalarTraits::sin( q(MOTOR_WHEEL_JOINT_BR) );
    Scalar cos_q_motor_wheel_joint_br  = ScalarTraits::cos( q(MOTOR_WHEEL_JOINT_BR) );
    (*this)(0,0) = - 0.5 * (sin_q_motor_wheel_joint_br-cos_q_motor_wheel_joint_br);
    (*this)(0,1) = - 0.5 * (sin_q_motor_wheel_joint_br+cos_q_motor_wheel_joint_br);
    (*this)(0,3) = - 0.5 * ((( tz_motor_wheel_joint_br+( 1.41421 *  ty_motor_wheel_joint_br)) * sin_q_motor_wheel_joint_br)+((( 1.41421 *  ty_motor_wheel_joint_br)- tz_motor_wheel_joint_br) * cos_q_motor_wheel_joint_br));
    (*this)(0,4) = - 0.5 * ((( tz_motor_wheel_joint_br-( 1.41421 *  ty_motor_wheel_joint_br)) * sin_q_motor_wheel_joint_br)+(( tz_motor_wheel_joint_br+( 1.41421 *  ty_motor_wheel_joint_br)) * cos_q_motor_wheel_joint_br));
    (*this)(1,0) =  0.5 * (sin_q_motor_wheel_joint_br-cos_q_motor_wheel_joint_br);
    (*this)(1,1) =  0.5 * (sin_q_motor_wheel_joint_br+cos_q_motor_wheel_joint_br);
    (*this)(1,3) = - 0.5 * ((( tz_motor_wheel_joint_br-( 1.41421 *  tx_motor_wheel_joint_br)) * sin_q_motor_wheel_joint_br)+((- tz_motor_wheel_joint_br-( 1.41421 *  tx_motor_wheel_joint_br)) * cos_q_motor_wheel_joint_br));
    (*this)(1,4) = - 0.5 * ((( tz_motor_wheel_joint_br+( 1.41421 *  tx_motor_wheel_joint_br)) * sin_q_motor_wheel_joint_br)+(( tz_motor_wheel_joint_br-( 1.41421 *  tx_motor_wheel_joint_br)) * cos_q_motor_wheel_joint_br));
    (*this)(2,0) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_br)+( 1.41421 * cos_q_motor_wheel_joint_br));
    (*this)(2,1) =  0.5 * (( 1.41421 * sin_q_motor_wheel_joint_br)-( 1.41421 * cos_q_motor_wheel_joint_br));
    (*this)(2,3) =  0.5 * ((( ty_motor_wheel_joint_br+ tx_motor_wheel_joint_br) * sin_q_motor_wheel_joint_br)+((- ty_motor_wheel_joint_br- tx_motor_wheel_joint_br) * cos_q_motor_wheel_joint_br));
    (*this)(2,4) =  0.5 * ((( ty_motor_wheel_joint_br+ tx_motor_wheel_joint_br) * sin_q_motor_wheel_joint_br)+(( ty_motor_wheel_joint_br+ tx_motor_wheel_joint_br) * cos_q_motor_wheel_joint_br));
    (*this)(3,3) = - 0.5 * (sin_q_motor_wheel_joint_br-cos_q_motor_wheel_joint_br);
    (*this)(3,4) = - 0.5 * (sin_q_motor_wheel_joint_br+cos_q_motor_wheel_joint_br);
    (*this)(4,3) =  0.5 * (sin_q_motor_wheel_joint_br-cos_q_motor_wheel_joint_br);
    (*this)(4,4) =  0.5 * (sin_q_motor_wheel_joint_br+cos_q_motor_wheel_joint_br);
    (*this)(5,3) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_br)+( 1.41421 * cos_q_motor_wheel_joint_br));
    (*this)(5,4) =  0.5 * (( 1.41421 * sin_q_motor_wheel_joint_br)-( 1.41421 * cos_q_motor_wheel_joint_br));
    return *this;
}
ForceTransforms::Type_fr_xarmlink1_X_fr_base_link_footprint::Type_fr_xarmlink1_X_fr_base_link_footprint()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = - tx_xarmjoint1;    // Maxima DSL: -_k__tx_xarmjoint1
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_xarmlink1_X_fr_base_link_footprint& ForceTransforms::Type_fr_xarmlink1_X_fr_base_link_footprint::update(const state_t& q)
{
    Scalar sin_q_xarmjoint1  = ScalarTraits::sin( q(XARMJOINT1) );
    Scalar cos_q_xarmjoint1  = ScalarTraits::cos( q(XARMJOINT1) );
    (*this)(0,0) = cos_q_xarmjoint1;
    (*this)(0,1) = sin_q_xarmjoint1;
    (*this)(0,3) = - tz_xarmjoint1 * sin_q_xarmjoint1;
    (*this)(0,4) =  tz_xarmjoint1 * cos_q_xarmjoint1;
    (*this)(0,5) =  tx_xarmjoint1 * sin_q_xarmjoint1;
    (*this)(1,0) = -sin_q_xarmjoint1;
    (*this)(1,1) = cos_q_xarmjoint1;
    (*this)(1,3) = - tz_xarmjoint1 * cos_q_xarmjoint1;
    (*this)(1,4) = - tz_xarmjoint1 * sin_q_xarmjoint1;
    (*this)(1,5) =  tx_xarmjoint1 * cos_q_xarmjoint1;
    (*this)(3,3) = cos_q_xarmjoint1;
    (*this)(3,4) = sin_q_xarmjoint1;
    (*this)(4,3) = -sin_q_xarmjoint1;
    (*this)(4,4) = cos_q_xarmjoint1;
    return *this;
}
ForceTransforms::Type_fr_base_link_footprint_X_fr_xarmlink1::Type_fr_base_link_footprint_X_fr_xarmlink1()
{
    (*this)(0,2) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,5) = - tx_xarmjoint1;    // Maxima DSL: -_k__tx_xarmjoint1
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_base_link_footprint_X_fr_xarmlink1& ForceTransforms::Type_fr_base_link_footprint_X_fr_xarmlink1::update(const state_t& q)
{
    Scalar sin_q_xarmjoint1  = ScalarTraits::sin( q(XARMJOINT1) );
    Scalar cos_q_xarmjoint1  = ScalarTraits::cos( q(XARMJOINT1) );
    (*this)(0,0) = cos_q_xarmjoint1;
    (*this)(0,1) = -sin_q_xarmjoint1;
    (*this)(0,3) = - tz_xarmjoint1 * sin_q_xarmjoint1;
    (*this)(0,4) = - tz_xarmjoint1 * cos_q_xarmjoint1;
    (*this)(1,0) = sin_q_xarmjoint1;
    (*this)(1,1) = cos_q_xarmjoint1;
    (*this)(1,3) =  tz_xarmjoint1 * cos_q_xarmjoint1;
    (*this)(1,4) = - tz_xarmjoint1 * sin_q_xarmjoint1;
    (*this)(2,3) =  tx_xarmjoint1 * sin_q_xarmjoint1;
    (*this)(2,4) =  tx_xarmjoint1 * cos_q_xarmjoint1;
    (*this)(3,3) = cos_q_xarmjoint1;
    (*this)(3,4) = -sin_q_xarmjoint1;
    (*this)(4,3) = sin_q_xarmjoint1;
    (*this)(4,4) = cos_q_xarmjoint1;
    return *this;
}
ForceTransforms::Type_fr_xarmlink2_X_fr_xarmlink1::Type_fr_xarmlink2_X_fr_xarmlink1()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_xarmjoint2;    // Maxima DSL: -sin(_k__rx_xarmjoint2)
    (*this)(2,2) = cos_rx_xarmjoint2;    // Maxima DSL: cos(_k__rx_xarmjoint2)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -sin_rx_xarmjoint2;    // Maxima DSL: -sin(_k__rx_xarmjoint2)
    (*this)(5,5) = cos_rx_xarmjoint2;    // Maxima DSL: cos(_k__rx_xarmjoint2)
}

const ForceTransforms::Type_fr_xarmlink2_X_fr_xarmlink1& ForceTransforms::Type_fr_xarmlink2_X_fr_xarmlink1::update(const state_t& q)
{
    Scalar sin_q_xarmjoint2  = ScalarTraits::sin( q(XARMJOINT2) );
    Scalar cos_q_xarmjoint2  = ScalarTraits::cos( q(XARMJOINT2) );
    (*this)(0,0) = cos_q_xarmjoint2;
    (*this)(0,1) = cos_rx_xarmjoint2 * sin_q_xarmjoint2;
    (*this)(0,2) = sin_rx_xarmjoint2 * sin_q_xarmjoint2;
    (*this)(1,0) = -sin_q_xarmjoint2;
    (*this)(1,1) = cos_rx_xarmjoint2 * cos_q_xarmjoint2;
    (*this)(1,2) = sin_rx_xarmjoint2 * cos_q_xarmjoint2;
    (*this)(3,3) = cos_q_xarmjoint2;
    (*this)(3,4) = cos_rx_xarmjoint2 * sin_q_xarmjoint2;
    (*this)(3,5) = sin_rx_xarmjoint2 * sin_q_xarmjoint2;
    (*this)(4,3) = -sin_q_xarmjoint2;
    (*this)(4,4) = cos_rx_xarmjoint2 * cos_q_xarmjoint2;
    (*this)(4,5) = sin_rx_xarmjoint2 * cos_q_xarmjoint2;
    return *this;
}
ForceTransforms::Type_fr_xarmlink1_X_fr_xarmlink2::Type_fr_xarmlink1_X_fr_xarmlink2()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = -sin_rx_xarmjoint2;    // Maxima DSL: -sin(_k__rx_xarmjoint2)
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = cos_rx_xarmjoint2;    // Maxima DSL: cos(_k__rx_xarmjoint2)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = -sin_rx_xarmjoint2;    // Maxima DSL: -sin(_k__rx_xarmjoint2)
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = cos_rx_xarmjoint2;    // Maxima DSL: cos(_k__rx_xarmjoint2)
}

const ForceTransforms::Type_fr_xarmlink1_X_fr_xarmlink2& ForceTransforms::Type_fr_xarmlink1_X_fr_xarmlink2::update(const state_t& q)
{
    Scalar sin_q_xarmjoint2  = ScalarTraits::sin( q(XARMJOINT2) );
    Scalar cos_q_xarmjoint2  = ScalarTraits::cos( q(XARMJOINT2) );
    (*this)(0,0) = cos_q_xarmjoint2;
    (*this)(0,1) = -sin_q_xarmjoint2;
    (*this)(1,0) = cos_rx_xarmjoint2 * sin_q_xarmjoint2;
    (*this)(1,1) = cos_rx_xarmjoint2 * cos_q_xarmjoint2;
    (*this)(2,0) = sin_rx_xarmjoint2 * sin_q_xarmjoint2;
    (*this)(2,1) = sin_rx_xarmjoint2 * cos_q_xarmjoint2;
    (*this)(3,3) = cos_q_xarmjoint2;
    (*this)(3,4) = -sin_q_xarmjoint2;
    (*this)(4,3) = cos_rx_xarmjoint2 * sin_q_xarmjoint2;
    (*this)(4,4) = cos_rx_xarmjoint2 * cos_q_xarmjoint2;
    (*this)(5,3) = sin_rx_xarmjoint2 * sin_q_xarmjoint2;
    (*this)(5,4) = sin_rx_xarmjoint2 * cos_q_xarmjoint2;
    return *this;
}
ForceTransforms::Type_fr_xarmlink3_X_fr_xarmlink2::Type_fr_xarmlink3_X_fr_xarmlink2()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  ty_xarmjoint3;    // Maxima DSL: _k__ty_xarmjoint3
    (*this)(2,4) = - tx_xarmjoint3;    // Maxima DSL: -_k__tx_xarmjoint3
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_xarmlink3_X_fr_xarmlink2& ForceTransforms::Type_fr_xarmlink3_X_fr_xarmlink2::update(const state_t& q)
{
    Scalar sin_q_xarmjoint3  = ScalarTraits::sin( q(XARMJOINT3) );
    Scalar cos_q_xarmjoint3  = ScalarTraits::cos( q(XARMJOINT3) );
    (*this)(0,0) = cos_q_xarmjoint3;
    (*this)(0,1) = sin_q_xarmjoint3;
    (*this)(0,5) = ( tx_xarmjoint3 * sin_q_xarmjoint3)-( ty_xarmjoint3 * cos_q_xarmjoint3);
    (*this)(1,0) = -sin_q_xarmjoint3;
    (*this)(1,1) = cos_q_xarmjoint3;
    (*this)(1,5) = ( ty_xarmjoint3 * sin_q_xarmjoint3)+( tx_xarmjoint3 * cos_q_xarmjoint3);
    (*this)(3,3) = cos_q_xarmjoint3;
    (*this)(3,4) = sin_q_xarmjoint3;
    (*this)(4,3) = -sin_q_xarmjoint3;
    (*this)(4,4) = cos_q_xarmjoint3;
    return *this;
}
ForceTransforms::Type_fr_xarmlink2_X_fr_xarmlink3::Type_fr_xarmlink2_X_fr_xarmlink3()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) =  ty_xarmjoint3;    // Maxima DSL: _k__ty_xarmjoint3
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = - tx_xarmjoint3;    // Maxima DSL: -_k__tx_xarmjoint3
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_xarmlink2_X_fr_xarmlink3& ForceTransforms::Type_fr_xarmlink2_X_fr_xarmlink3::update(const state_t& q)
{
    Scalar sin_q_xarmjoint3  = ScalarTraits::sin( q(XARMJOINT3) );
    Scalar cos_q_xarmjoint3  = ScalarTraits::cos( q(XARMJOINT3) );
    (*this)(0,0) = cos_q_xarmjoint3;
    (*this)(0,1) = -sin_q_xarmjoint3;
    (*this)(1,0) = sin_q_xarmjoint3;
    (*this)(1,1) = cos_q_xarmjoint3;
    (*this)(2,3) = ( tx_xarmjoint3 * sin_q_xarmjoint3)-( ty_xarmjoint3 * cos_q_xarmjoint3);
    (*this)(2,4) = ( ty_xarmjoint3 * sin_q_xarmjoint3)+( tx_xarmjoint3 * cos_q_xarmjoint3);
    (*this)(3,3) = cos_q_xarmjoint3;
    (*this)(3,4) = -sin_q_xarmjoint3;
    (*this)(4,3) = sin_q_xarmjoint3;
    (*this)(4,4) = cos_q_xarmjoint3;
    return *this;
}
ForceTransforms::Type_fr_xarmlink4_X_fr_xarmlink3::Type_fr_xarmlink4_X_fr_xarmlink3()
{
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_xarmjoint4;    // Maxima DSL: -sin(_k__rx_xarmjoint4)
    (*this)(2,2) = cos_rx_xarmjoint4;    // Maxima DSL: cos(_k__rx_xarmjoint4)
    (*this)(2,3) =  ty_xarmjoint4 * cos_rx_xarmjoint4;    // Maxima DSL: _k__ty_xarmjoint4*cos(_k__rx_xarmjoint4)
    (*this)(2,4) = - tx_xarmjoint4 * cos_rx_xarmjoint4;    // Maxima DSL: -_k__tx_xarmjoint4*cos(_k__rx_xarmjoint4)
    (*this)(2,5) = - tx_xarmjoint4 * sin_rx_xarmjoint4;    // Maxima DSL: -_k__tx_xarmjoint4*sin(_k__rx_xarmjoint4)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -sin_rx_xarmjoint4;    // Maxima DSL: -sin(_k__rx_xarmjoint4)
    (*this)(5,5) = cos_rx_xarmjoint4;    // Maxima DSL: cos(_k__rx_xarmjoint4)
}

const ForceTransforms::Type_fr_xarmlink4_X_fr_xarmlink3& ForceTransforms::Type_fr_xarmlink4_X_fr_xarmlink3::update(const state_t& q)
{
    Scalar sin_q_xarmjoint4  = ScalarTraits::sin( q(XARMJOINT4) );
    Scalar cos_q_xarmjoint4  = ScalarTraits::cos( q(XARMJOINT4) );
    (*this)(0,0) = cos_q_xarmjoint4;
    (*this)(0,1) = cos_rx_xarmjoint4 * sin_q_xarmjoint4;
    (*this)(0,2) = sin_rx_xarmjoint4 * sin_q_xarmjoint4;
    (*this)(0,3) =  ty_xarmjoint4 * sin_rx_xarmjoint4 * sin_q_xarmjoint4;
    (*this)(0,4) = - tx_xarmjoint4 * sin_rx_xarmjoint4 * sin_q_xarmjoint4;
    (*this)(0,5) = ( tx_xarmjoint4 * cos_rx_xarmjoint4 * sin_q_xarmjoint4)-( ty_xarmjoint4 * cos_q_xarmjoint4);
    (*this)(1,0) = -sin_q_xarmjoint4;
    (*this)(1,1) = cos_rx_xarmjoint4 * cos_q_xarmjoint4;
    (*this)(1,2) = sin_rx_xarmjoint4 * cos_q_xarmjoint4;
    (*this)(1,3) =  ty_xarmjoint4 * sin_rx_xarmjoint4 * cos_q_xarmjoint4;
    (*this)(1,4) = - tx_xarmjoint4 * sin_rx_xarmjoint4 * cos_q_xarmjoint4;
    (*this)(1,5) = ( ty_xarmjoint4 * sin_q_xarmjoint4)+( tx_xarmjoint4 * cos_rx_xarmjoint4 * cos_q_xarmjoint4);
    (*this)(3,3) = cos_q_xarmjoint4;
    (*this)(3,4) = cos_rx_xarmjoint4 * sin_q_xarmjoint4;
    (*this)(3,5) = sin_rx_xarmjoint4 * sin_q_xarmjoint4;
    (*this)(4,3) = -sin_q_xarmjoint4;
    (*this)(4,4) = cos_rx_xarmjoint4 * cos_q_xarmjoint4;
    (*this)(4,5) = sin_rx_xarmjoint4 * cos_q_xarmjoint4;
    return *this;
}
ForceTransforms::Type_fr_xarmlink3_X_fr_xarmlink4::Type_fr_xarmlink3_X_fr_xarmlink4()
{
    (*this)(0,2) = 0.0;
    (*this)(0,5) =  ty_xarmjoint4 * cos_rx_xarmjoint4;    // Maxima DSL: _k__ty_xarmjoint4*cos(_k__rx_xarmjoint4)
    (*this)(1,2) = -sin_rx_xarmjoint4;    // Maxima DSL: -sin(_k__rx_xarmjoint4)
    (*this)(1,5) = - tx_xarmjoint4 * cos_rx_xarmjoint4;    // Maxima DSL: -_k__tx_xarmjoint4*cos(_k__rx_xarmjoint4)
    (*this)(2,2) = cos_rx_xarmjoint4;    // Maxima DSL: cos(_k__rx_xarmjoint4)
    (*this)(2,5) = - tx_xarmjoint4 * sin_rx_xarmjoint4;    // Maxima DSL: -_k__tx_xarmjoint4*sin(_k__rx_xarmjoint4)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = -sin_rx_xarmjoint4;    // Maxima DSL: -sin(_k__rx_xarmjoint4)
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = cos_rx_xarmjoint4;    // Maxima DSL: cos(_k__rx_xarmjoint4)
}

const ForceTransforms::Type_fr_xarmlink3_X_fr_xarmlink4& ForceTransforms::Type_fr_xarmlink3_X_fr_xarmlink4::update(const state_t& q)
{
    Scalar sin_q_xarmjoint4  = ScalarTraits::sin( q(XARMJOINT4) );
    Scalar cos_q_xarmjoint4  = ScalarTraits::cos( q(XARMJOINT4) );
    (*this)(0,0) = cos_q_xarmjoint4;
    (*this)(0,1) = -sin_q_xarmjoint4;
    (*this)(0,3) =  ty_xarmjoint4 * sin_rx_xarmjoint4 * sin_q_xarmjoint4;
    (*this)(0,4) =  ty_xarmjoint4 * sin_rx_xarmjoint4 * cos_q_xarmjoint4;
    (*this)(1,0) = cos_rx_xarmjoint4 * sin_q_xarmjoint4;
    (*this)(1,1) = cos_rx_xarmjoint4 * cos_q_xarmjoint4;
    (*this)(1,3) = - tx_xarmjoint4 * sin_rx_xarmjoint4 * sin_q_xarmjoint4;
    (*this)(1,4) = - tx_xarmjoint4 * sin_rx_xarmjoint4 * cos_q_xarmjoint4;
    (*this)(2,0) = sin_rx_xarmjoint4 * sin_q_xarmjoint4;
    (*this)(2,1) = sin_rx_xarmjoint4 * cos_q_xarmjoint4;
    (*this)(2,3) = ( tx_xarmjoint4 * cos_rx_xarmjoint4 * sin_q_xarmjoint4)-( ty_xarmjoint4 * cos_q_xarmjoint4);
    (*this)(2,4) = ( ty_xarmjoint4 * sin_q_xarmjoint4)+( tx_xarmjoint4 * cos_rx_xarmjoint4 * cos_q_xarmjoint4);
    (*this)(3,3) = cos_q_xarmjoint4;
    (*this)(3,4) = -sin_q_xarmjoint4;
    (*this)(4,3) = cos_rx_xarmjoint4 * sin_q_xarmjoint4;
    (*this)(4,4) = cos_rx_xarmjoint4 * cos_q_xarmjoint4;
    (*this)(5,3) = sin_rx_xarmjoint4 * sin_q_xarmjoint4;
    (*this)(5,4) = sin_rx_xarmjoint4 * cos_q_xarmjoint4;
    return *this;
}
ForceTransforms::Type_fr_xarmlink5_X_fr_xarmlink4::Type_fr_xarmlink5_X_fr_xarmlink4()
{
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_xarmjoint5;    // Maxima DSL: -sin(_k__rx_xarmjoint5)
    (*this)(2,2) = cos_rx_xarmjoint5;    // Maxima DSL: cos(_k__rx_xarmjoint5)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -sin_rx_xarmjoint5;    // Maxima DSL: -sin(_k__rx_xarmjoint5)
    (*this)(5,5) = cos_rx_xarmjoint5;    // Maxima DSL: cos(_k__rx_xarmjoint5)
}

const ForceTransforms::Type_fr_xarmlink5_X_fr_xarmlink4& ForceTransforms::Type_fr_xarmlink5_X_fr_xarmlink4::update(const state_t& q)
{
    Scalar sin_q_xarmjoint5  = ScalarTraits::sin( q(XARMJOINT5) );
    Scalar cos_q_xarmjoint5  = ScalarTraits::cos( q(XARMJOINT5) );
    (*this)(0,0) = cos_q_xarmjoint5;
    (*this)(0,1) = cos_rx_xarmjoint5 * sin_q_xarmjoint5;
    (*this)(0,2) = sin_rx_xarmjoint5 * sin_q_xarmjoint5;
    (*this)(1,0) = -sin_q_xarmjoint5;
    (*this)(1,1) = cos_rx_xarmjoint5 * cos_q_xarmjoint5;
    (*this)(1,2) = sin_rx_xarmjoint5 * cos_q_xarmjoint5;
    (*this)(3,3) = cos_q_xarmjoint5;
    (*this)(3,4) = cos_rx_xarmjoint5 * sin_q_xarmjoint5;
    (*this)(3,5) = sin_rx_xarmjoint5 * sin_q_xarmjoint5;
    (*this)(4,3) = -sin_q_xarmjoint5;
    (*this)(4,4) = cos_rx_xarmjoint5 * cos_q_xarmjoint5;
    (*this)(4,5) = sin_rx_xarmjoint5 * cos_q_xarmjoint5;
    return *this;
}
ForceTransforms::Type_fr_xarmlink4_X_fr_xarmlink5::Type_fr_xarmlink4_X_fr_xarmlink5()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,2) = -sin_rx_xarmjoint5;    // Maxima DSL: -sin(_k__rx_xarmjoint5)
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = cos_rx_xarmjoint5;    // Maxima DSL: cos(_k__rx_xarmjoint5)
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = -sin_rx_xarmjoint5;    // Maxima DSL: -sin(_k__rx_xarmjoint5)
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = cos_rx_xarmjoint5;    // Maxima DSL: cos(_k__rx_xarmjoint5)
}

const ForceTransforms::Type_fr_xarmlink4_X_fr_xarmlink5& ForceTransforms::Type_fr_xarmlink4_X_fr_xarmlink5::update(const state_t& q)
{
    Scalar sin_q_xarmjoint5  = ScalarTraits::sin( q(XARMJOINT5) );
    Scalar cos_q_xarmjoint5  = ScalarTraits::cos( q(XARMJOINT5) );
    (*this)(0,0) = cos_q_xarmjoint5;
    (*this)(0,1) = -sin_q_xarmjoint5;
    (*this)(1,0) = cos_rx_xarmjoint5 * sin_q_xarmjoint5;
    (*this)(1,1) = cos_rx_xarmjoint5 * cos_q_xarmjoint5;
    (*this)(2,0) = sin_rx_xarmjoint5 * sin_q_xarmjoint5;
    (*this)(2,1) = sin_rx_xarmjoint5 * cos_q_xarmjoint5;
    (*this)(3,3) = cos_q_xarmjoint5;
    (*this)(3,4) = -sin_q_xarmjoint5;
    (*this)(4,3) = cos_rx_xarmjoint5 * sin_q_xarmjoint5;
    (*this)(4,4) = cos_rx_xarmjoint5 * cos_q_xarmjoint5;
    (*this)(5,3) = sin_rx_xarmjoint5 * sin_q_xarmjoint5;
    (*this)(5,4) = sin_rx_xarmjoint5 * cos_q_xarmjoint5;
    return *this;
}
ForceTransforms::Type_fr_xarmlink6_X_fr_xarmlink5::Type_fr_xarmlink6_X_fr_xarmlink5()
{
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_xarmjoint6;    // Maxima DSL: -sin(_k__rx_xarmjoint6)
    (*this)(2,2) = cos_rx_xarmjoint6;    // Maxima DSL: cos(_k__rx_xarmjoint6)
    (*this)(2,3) =  ty_xarmjoint6 * cos_rx_xarmjoint6;    // Maxima DSL: _k__ty_xarmjoint6*cos(_k__rx_xarmjoint6)
    (*this)(2,4) = - tx_xarmjoint6 * cos_rx_xarmjoint6;    // Maxima DSL: -_k__tx_xarmjoint6*cos(_k__rx_xarmjoint6)
    (*this)(2,5) = - tx_xarmjoint6 * sin_rx_xarmjoint6;    // Maxima DSL: -_k__tx_xarmjoint6*sin(_k__rx_xarmjoint6)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -sin_rx_xarmjoint6;    // Maxima DSL: -sin(_k__rx_xarmjoint6)
    (*this)(5,5) = cos_rx_xarmjoint6;    // Maxima DSL: cos(_k__rx_xarmjoint6)
}

const ForceTransforms::Type_fr_xarmlink6_X_fr_xarmlink5& ForceTransforms::Type_fr_xarmlink6_X_fr_xarmlink5::update(const state_t& q)
{
    Scalar sin_q_xarmjoint6  = ScalarTraits::sin( q(XARMJOINT6) );
    Scalar cos_q_xarmjoint6  = ScalarTraits::cos( q(XARMJOINT6) );
    (*this)(0,0) = cos_q_xarmjoint6;
    (*this)(0,1) = cos_rx_xarmjoint6 * sin_q_xarmjoint6;
    (*this)(0,2) = sin_rx_xarmjoint6 * sin_q_xarmjoint6;
    (*this)(0,3) =  ty_xarmjoint6 * sin_rx_xarmjoint6 * sin_q_xarmjoint6;
    (*this)(0,4) = - tx_xarmjoint6 * sin_rx_xarmjoint6 * sin_q_xarmjoint6;
    (*this)(0,5) = ( tx_xarmjoint6 * cos_rx_xarmjoint6 * sin_q_xarmjoint6)-( ty_xarmjoint6 * cos_q_xarmjoint6);
    (*this)(1,0) = -sin_q_xarmjoint6;
    (*this)(1,1) = cos_rx_xarmjoint6 * cos_q_xarmjoint6;
    (*this)(1,2) = sin_rx_xarmjoint6 * cos_q_xarmjoint6;
    (*this)(1,3) =  ty_xarmjoint6 * sin_rx_xarmjoint6 * cos_q_xarmjoint6;
    (*this)(1,4) = - tx_xarmjoint6 * sin_rx_xarmjoint6 * cos_q_xarmjoint6;
    (*this)(1,5) = ( ty_xarmjoint6 * sin_q_xarmjoint6)+( tx_xarmjoint6 * cos_rx_xarmjoint6 * cos_q_xarmjoint6);
    (*this)(3,3) = cos_q_xarmjoint6;
    (*this)(3,4) = cos_rx_xarmjoint6 * sin_q_xarmjoint6;
    (*this)(3,5) = sin_rx_xarmjoint6 * sin_q_xarmjoint6;
    (*this)(4,3) = -sin_q_xarmjoint6;
    (*this)(4,4) = cos_rx_xarmjoint6 * cos_q_xarmjoint6;
    (*this)(4,5) = sin_rx_xarmjoint6 * cos_q_xarmjoint6;
    return *this;
}
ForceTransforms::Type_fr_xarmlink5_X_fr_xarmlink6::Type_fr_xarmlink5_X_fr_xarmlink6()
{
    (*this)(0,2) = 0.0;
    (*this)(0,5) =  ty_xarmjoint6 * cos_rx_xarmjoint6;    // Maxima DSL: _k__ty_xarmjoint6*cos(_k__rx_xarmjoint6)
    (*this)(1,2) = -sin_rx_xarmjoint6;    // Maxima DSL: -sin(_k__rx_xarmjoint6)
    (*this)(1,5) = - tx_xarmjoint6 * cos_rx_xarmjoint6;    // Maxima DSL: -_k__tx_xarmjoint6*cos(_k__rx_xarmjoint6)
    (*this)(2,2) = cos_rx_xarmjoint6;    // Maxima DSL: cos(_k__rx_xarmjoint6)
    (*this)(2,5) = - tx_xarmjoint6 * sin_rx_xarmjoint6;    // Maxima DSL: -_k__tx_xarmjoint6*sin(_k__rx_xarmjoint6)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,5) = -sin_rx_xarmjoint6;    // Maxima DSL: -sin(_k__rx_xarmjoint6)
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = cos_rx_xarmjoint6;    // Maxima DSL: cos(_k__rx_xarmjoint6)
}

const ForceTransforms::Type_fr_xarmlink5_X_fr_xarmlink6& ForceTransforms::Type_fr_xarmlink5_X_fr_xarmlink6::update(const state_t& q)
{
    Scalar sin_q_xarmjoint6  = ScalarTraits::sin( q(XARMJOINT6) );
    Scalar cos_q_xarmjoint6  = ScalarTraits::cos( q(XARMJOINT6) );
    (*this)(0,0) = cos_q_xarmjoint6;
    (*this)(0,1) = -sin_q_xarmjoint6;
    (*this)(0,3) =  ty_xarmjoint6 * sin_rx_xarmjoint6 * sin_q_xarmjoint6;
    (*this)(0,4) =  ty_xarmjoint6 * sin_rx_xarmjoint6 * cos_q_xarmjoint6;
    (*this)(1,0) = cos_rx_xarmjoint6 * sin_q_xarmjoint6;
    (*this)(1,1) = cos_rx_xarmjoint6 * cos_q_xarmjoint6;
    (*this)(1,3) = - tx_xarmjoint6 * sin_rx_xarmjoint6 * sin_q_xarmjoint6;
    (*this)(1,4) = - tx_xarmjoint6 * sin_rx_xarmjoint6 * cos_q_xarmjoint6;
    (*this)(2,0) = sin_rx_xarmjoint6 * sin_q_xarmjoint6;
    (*this)(2,1) = sin_rx_xarmjoint6 * cos_q_xarmjoint6;
    (*this)(2,3) = ( tx_xarmjoint6 * cos_rx_xarmjoint6 * sin_q_xarmjoint6)-( ty_xarmjoint6 * cos_q_xarmjoint6);
    (*this)(2,4) = ( ty_xarmjoint6 * sin_q_xarmjoint6)+( tx_xarmjoint6 * cos_rx_xarmjoint6 * cos_q_xarmjoint6);
    (*this)(3,3) = cos_q_xarmjoint6;
    (*this)(3,4) = -sin_q_xarmjoint6;
    (*this)(4,3) = cos_rx_xarmjoint6 * sin_q_xarmjoint6;
    (*this)(4,4) = cos_rx_xarmjoint6 * cos_q_xarmjoint6;
    (*this)(5,3) = sin_rx_xarmjoint6 * sin_q_xarmjoint6;
    (*this)(5,4) = sin_rx_xarmjoint6 * cos_q_xarmjoint6;
    return *this;
}

HomogeneousTransforms::Type_fr_omniwheel_fl_X_fr_base_link_footprint::Type_fr_omniwheel_fl_X_fr_base_link_footprint()
{
    (*this)(2,0) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(2,1) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - 0.5 * (( 1.41421 *  ty_motor_wheel_joint_fl)+( 1.41421 *  tx_motor_wheel_joint_fl));    // Maxima DSL: -0.5*(1.41421*_k__ty_motor_wheel_joint_fl+1.41421*_k__tx_motor_wheel_joint_fl)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_omniwheel_fl_X_fr_base_link_footprint& HomogeneousTransforms::Type_fr_omniwheel_fl_X_fr_base_link_footprint::update(const state_t& q)
{
    Scalar sin_q_motor_wheel_joint_fl  = ScalarTraits::sin( q(MOTOR_WHEEL_JOINT_FL) );
    Scalar cos_q_motor_wheel_joint_fl  = ScalarTraits::cos( q(MOTOR_WHEEL_JOINT_FL) );
    (*this)(0,0) = - 0.5 * (sin_q_motor_wheel_joint_fl-cos_q_motor_wheel_joint_fl);
    (*this)(0,1) =  0.5 * (sin_q_motor_wheel_joint_fl-cos_q_motor_wheel_joint_fl);
    (*this)(0,2) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fl)+( 1.41421 * cos_q_motor_wheel_joint_fl));
    (*this)(0,3) =  0.5 * (((( 1.41421 *  tz_motor_wheel_joint_fl)- ty_motor_wheel_joint_fl+ tx_motor_wheel_joint_fl) * sin_q_motor_wheel_joint_fl)+((( 1.41421 *  tz_motor_wheel_joint_fl)+ ty_motor_wheel_joint_fl- tx_motor_wheel_joint_fl) * cos_q_motor_wheel_joint_fl));
    (*this)(1,0) = - 0.5 * (sin_q_motor_wheel_joint_fl+cos_q_motor_wheel_joint_fl);
    (*this)(1,1) =  0.5 * (sin_q_motor_wheel_joint_fl+cos_q_motor_wheel_joint_fl);
    (*this)(1,2) =  0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fl)-( 1.41421 * cos_q_motor_wheel_joint_fl));
    (*this)(1,3) = - 0.5 * (((( 1.41421 *  tz_motor_wheel_joint_fl)+ ty_motor_wheel_joint_fl- tx_motor_wheel_joint_fl) * sin_q_motor_wheel_joint_fl)+(((- 1.41421 *  tz_motor_wheel_joint_fl)+ ty_motor_wheel_joint_fl- tx_motor_wheel_joint_fl) * cos_q_motor_wheel_joint_fl));
    return *this;
}
HomogeneousTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_fl::Type_fr_base_link_footprint_X_fr_omniwheel_fl()
{
    (*this)(0,2) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(0,3) =  tx_motor_wheel_joint_fl;    // Maxima DSL: _k__tx_motor_wheel_joint_fl
    (*this)(1,2) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(1,3) =  ty_motor_wheel_joint_fl;    // Maxima DSL: _k__ty_motor_wheel_joint_fl
    (*this)(2,2) = 0.0;
    (*this)(2,3) =  tz_motor_wheel_joint_fl;    // Maxima DSL: _k__tz_motor_wheel_joint_fl
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_fl& HomogeneousTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_fl::update(const state_t& q)
{
    Scalar sin_q_motor_wheel_joint_fl  = ScalarTraits::sin( q(MOTOR_WHEEL_JOINT_FL) );
    Scalar cos_q_motor_wheel_joint_fl  = ScalarTraits::cos( q(MOTOR_WHEEL_JOINT_FL) );
    (*this)(0,0) = - 0.5 * (sin_q_motor_wheel_joint_fl-cos_q_motor_wheel_joint_fl);
    (*this)(0,1) = - 0.5 * (sin_q_motor_wheel_joint_fl+cos_q_motor_wheel_joint_fl);
    (*this)(1,0) =  0.5 * (sin_q_motor_wheel_joint_fl-cos_q_motor_wheel_joint_fl);
    (*this)(1,1) =  0.5 * (sin_q_motor_wheel_joint_fl+cos_q_motor_wheel_joint_fl);
    (*this)(2,0) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fl)+( 1.41421 * cos_q_motor_wheel_joint_fl));
    (*this)(2,1) =  0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fl)-( 1.41421 * cos_q_motor_wheel_joint_fl));
    return *this;
}
HomogeneousTransforms::Type_fr_omniwheel_fr_X_fr_base_link_footprint::Type_fr_omniwheel_fr_X_fr_base_link_footprint()
{
    (*this)(2,0) = - 0.707106;    // Maxima DSL: -0.707106
    (*this)(2,1) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - 0.5 * (( 1.41421 *  ty_motor_wheel_joint_fr)-( 1.41421 *  tx_motor_wheel_joint_fr));    // Maxima DSL: -0.5*(1.41421*_k__ty_motor_wheel_joint_fr-1.41421*_k__tx_motor_wheel_joint_fr)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_omniwheel_fr_X_fr_base_link_footprint& HomogeneousTransforms::Type_fr_omniwheel_fr_X_fr_base_link_footprint::update(const state_t& q)
{
    Scalar sin_q_motor_wheel_joint_fr  = ScalarTraits::sin( q(MOTOR_WHEEL_JOINT_FR) );
    Scalar cos_q_motor_wheel_joint_fr  = ScalarTraits::cos( q(MOTOR_WHEEL_JOINT_FR) );
    (*this)(0,0) =  0.5 * (sin_q_motor_wheel_joint_fr+cos_q_motor_wheel_joint_fr);
    (*this)(0,1) =  0.5 * (sin_q_motor_wheel_joint_fr+cos_q_motor_wheel_joint_fr);
    (*this)(0,2) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fr)-( 1.41421 * cos_q_motor_wheel_joint_fr));
    (*this)(0,3) =  0.5 * (((( 1.41421 *  tz_motor_wheel_joint_fr)- ty_motor_wheel_joint_fr- tx_motor_wheel_joint_fr) * sin_q_motor_wheel_joint_fr)+(((- 1.41421 *  tz_motor_wheel_joint_fr)- ty_motor_wheel_joint_fr- tx_motor_wheel_joint_fr) * cos_q_motor_wheel_joint_fr));
    (*this)(1,0) = - 0.5 * (sin_q_motor_wheel_joint_fr-cos_q_motor_wheel_joint_fr);
    (*this)(1,1) = - 0.5 * (sin_q_motor_wheel_joint_fr-cos_q_motor_wheel_joint_fr);
    (*this)(1,2) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fr)+( 1.41421 * cos_q_motor_wheel_joint_fr));
    (*this)(1,3) =  0.5 * (((( 1.41421 *  tz_motor_wheel_joint_fr)+ ty_motor_wheel_joint_fr+ tx_motor_wheel_joint_fr) * sin_q_motor_wheel_joint_fr)+((( 1.41421 *  tz_motor_wheel_joint_fr)- ty_motor_wheel_joint_fr- tx_motor_wheel_joint_fr) * cos_q_motor_wheel_joint_fr));
    return *this;
}
HomogeneousTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_fr::Type_fr_base_link_footprint_X_fr_omniwheel_fr()
{
    (*this)(0,2) = - 0.707106;    // Maxima DSL: -0.707106
    (*this)(0,3) =  tx_motor_wheel_joint_fr;    // Maxima DSL: _k__tx_motor_wheel_joint_fr
    (*this)(1,2) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(1,3) =  ty_motor_wheel_joint_fr;    // Maxima DSL: _k__ty_motor_wheel_joint_fr
    (*this)(2,2) = 0.0;
    (*this)(2,3) =  tz_motor_wheel_joint_fr;    // Maxima DSL: _k__tz_motor_wheel_joint_fr
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_fr& HomogeneousTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_fr::update(const state_t& q)
{
    Scalar sin_q_motor_wheel_joint_fr  = ScalarTraits::sin( q(MOTOR_WHEEL_JOINT_FR) );
    Scalar cos_q_motor_wheel_joint_fr  = ScalarTraits::cos( q(MOTOR_WHEEL_JOINT_FR) );
    (*this)(0,0) =  0.5 * (sin_q_motor_wheel_joint_fr+cos_q_motor_wheel_joint_fr);
    (*this)(0,1) = - 0.5 * (sin_q_motor_wheel_joint_fr-cos_q_motor_wheel_joint_fr);
    (*this)(1,0) =  0.5 * (sin_q_motor_wheel_joint_fr+cos_q_motor_wheel_joint_fr);
    (*this)(1,1) = - 0.5 * (sin_q_motor_wheel_joint_fr-cos_q_motor_wheel_joint_fr);
    (*this)(2,0) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fr)-( 1.41421 * cos_q_motor_wheel_joint_fr));
    (*this)(2,1) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_fr)+( 1.41421 * cos_q_motor_wheel_joint_fr));
    return *this;
}
HomogeneousTransforms::Type_fr_omniwheel_bl_X_fr_base_link_footprint::Type_fr_omniwheel_bl_X_fr_base_link_footprint()
{
    (*this)(2,0) = - 0.707106;    // Maxima DSL: -0.707106
    (*this)(2,1) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - 0.5 * (( 1.41421 *  ty_motor_wheel_joint_bl)-( 1.41421 *  tx_motor_wheel_joint_bl));    // Maxima DSL: -0.5*(1.41421*_k__ty_motor_wheel_joint_bl-1.41421*_k__tx_motor_wheel_joint_bl)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_omniwheel_bl_X_fr_base_link_footprint& HomogeneousTransforms::Type_fr_omniwheel_bl_X_fr_base_link_footprint::update(const state_t& q)
{
    Scalar sin_q_motor_wheel_joint_bl  = ScalarTraits::sin( q(MOTOR_WHEEL_JOINT_BL) );
    Scalar cos_q_motor_wheel_joint_bl  = ScalarTraits::cos( q(MOTOR_WHEEL_JOINT_BL) );
    (*this)(0,0) =  0.5 * (sin_q_motor_wheel_joint_bl+cos_q_motor_wheel_joint_bl);
    (*this)(0,1) =  0.5 * (sin_q_motor_wheel_joint_bl+cos_q_motor_wheel_joint_bl);
    (*this)(0,2) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_bl)-( 1.41421 * cos_q_motor_wheel_joint_bl));
    (*this)(0,3) =  0.5 * (((( 1.41421 *  tz_motor_wheel_joint_bl)- ty_motor_wheel_joint_bl- tx_motor_wheel_joint_bl) * sin_q_motor_wheel_joint_bl)+(((- 1.41421 *  tz_motor_wheel_joint_bl)- ty_motor_wheel_joint_bl- tx_motor_wheel_joint_bl) * cos_q_motor_wheel_joint_bl));
    (*this)(1,0) = - 0.5 * (sin_q_motor_wheel_joint_bl-cos_q_motor_wheel_joint_bl);
    (*this)(1,1) = - 0.5 * (sin_q_motor_wheel_joint_bl-cos_q_motor_wheel_joint_bl);
    (*this)(1,2) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_bl)+( 1.41421 * cos_q_motor_wheel_joint_bl));
    (*this)(1,3) =  0.5 * (((( 1.41421 *  tz_motor_wheel_joint_bl)+ ty_motor_wheel_joint_bl+ tx_motor_wheel_joint_bl) * sin_q_motor_wheel_joint_bl)+((( 1.41421 *  tz_motor_wheel_joint_bl)- ty_motor_wheel_joint_bl- tx_motor_wheel_joint_bl) * cos_q_motor_wheel_joint_bl));
    return *this;
}
HomogeneousTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_bl::Type_fr_base_link_footprint_X_fr_omniwheel_bl()
{
    (*this)(0,2) = - 0.707106;    // Maxima DSL: -0.707106
    (*this)(0,3) =  tx_motor_wheel_joint_bl;    // Maxima DSL: _k__tx_motor_wheel_joint_bl
    (*this)(1,2) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(1,3) =  ty_motor_wheel_joint_bl;    // Maxima DSL: _k__ty_motor_wheel_joint_bl
    (*this)(2,2) = 0.0;
    (*this)(2,3) =  tz_motor_wheel_joint_bl;    // Maxima DSL: _k__tz_motor_wheel_joint_bl
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_bl& HomogeneousTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_bl::update(const state_t& q)
{
    Scalar sin_q_motor_wheel_joint_bl  = ScalarTraits::sin( q(MOTOR_WHEEL_JOINT_BL) );
    Scalar cos_q_motor_wheel_joint_bl  = ScalarTraits::cos( q(MOTOR_WHEEL_JOINT_BL) );
    (*this)(0,0) =  0.5 * (sin_q_motor_wheel_joint_bl+cos_q_motor_wheel_joint_bl);
    (*this)(0,1) = - 0.5 * (sin_q_motor_wheel_joint_bl-cos_q_motor_wheel_joint_bl);
    (*this)(1,0) =  0.5 * (sin_q_motor_wheel_joint_bl+cos_q_motor_wheel_joint_bl);
    (*this)(1,1) = - 0.5 * (sin_q_motor_wheel_joint_bl-cos_q_motor_wheel_joint_bl);
    (*this)(2,0) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_bl)-( 1.41421 * cos_q_motor_wheel_joint_bl));
    (*this)(2,1) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_bl)+( 1.41421 * cos_q_motor_wheel_joint_bl));
    return *this;
}
HomogeneousTransforms::Type_fr_omniwheel_br_X_fr_base_link_footprint::Type_fr_omniwheel_br_X_fr_base_link_footprint()
{
    (*this)(2,0) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(2,1) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - 0.5 * (( 1.41421 *  ty_motor_wheel_joint_br)+( 1.41421 *  tx_motor_wheel_joint_br));    // Maxima DSL: -0.5*(1.41421*_k__ty_motor_wheel_joint_br+1.41421*_k__tx_motor_wheel_joint_br)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_omniwheel_br_X_fr_base_link_footprint& HomogeneousTransforms::Type_fr_omniwheel_br_X_fr_base_link_footprint::update(const state_t& q)
{
    Scalar sin_q_motor_wheel_joint_br  = ScalarTraits::sin( q(MOTOR_WHEEL_JOINT_BR) );
    Scalar cos_q_motor_wheel_joint_br  = ScalarTraits::cos( q(MOTOR_WHEEL_JOINT_BR) );
    (*this)(0,0) = - 0.5 * (sin_q_motor_wheel_joint_br-cos_q_motor_wheel_joint_br);
    (*this)(0,1) =  0.5 * (sin_q_motor_wheel_joint_br-cos_q_motor_wheel_joint_br);
    (*this)(0,2) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_br)+( 1.41421 * cos_q_motor_wheel_joint_br));
    (*this)(0,3) =  0.5 * (((( 1.41421 *  tz_motor_wheel_joint_br)- ty_motor_wheel_joint_br+ tx_motor_wheel_joint_br) * sin_q_motor_wheel_joint_br)+((( 1.41421 *  tz_motor_wheel_joint_br)+ ty_motor_wheel_joint_br- tx_motor_wheel_joint_br) * cos_q_motor_wheel_joint_br));
    (*this)(1,0) = - 0.5 * (sin_q_motor_wheel_joint_br+cos_q_motor_wheel_joint_br);
    (*this)(1,1) =  0.5 * (sin_q_motor_wheel_joint_br+cos_q_motor_wheel_joint_br);
    (*this)(1,2) =  0.5 * (( 1.41421 * sin_q_motor_wheel_joint_br)-( 1.41421 * cos_q_motor_wheel_joint_br));
    (*this)(1,3) = - 0.5 * (((( 1.41421 *  tz_motor_wheel_joint_br)+ ty_motor_wheel_joint_br- tx_motor_wheel_joint_br) * sin_q_motor_wheel_joint_br)+(((- 1.41421 *  tz_motor_wheel_joint_br)+ ty_motor_wheel_joint_br- tx_motor_wheel_joint_br) * cos_q_motor_wheel_joint_br));
    return *this;
}
HomogeneousTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_br::Type_fr_base_link_footprint_X_fr_omniwheel_br()
{
    (*this)(0,2) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(0,3) =  tx_motor_wheel_joint_br;    // Maxima DSL: _k__tx_motor_wheel_joint_br
    (*this)(1,2) =  0.707106;    // Maxima DSL: 0.707106
    (*this)(1,3) =  ty_motor_wheel_joint_br;    // Maxima DSL: _k__ty_motor_wheel_joint_br
    (*this)(2,2) = 0.0;
    (*this)(2,3) =  tz_motor_wheel_joint_br;    // Maxima DSL: _k__tz_motor_wheel_joint_br
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_br& HomogeneousTransforms::Type_fr_base_link_footprint_X_fr_omniwheel_br::update(const state_t& q)
{
    Scalar sin_q_motor_wheel_joint_br  = ScalarTraits::sin( q(MOTOR_WHEEL_JOINT_BR) );
    Scalar cos_q_motor_wheel_joint_br  = ScalarTraits::cos( q(MOTOR_WHEEL_JOINT_BR) );
    (*this)(0,0) = - 0.5 * (sin_q_motor_wheel_joint_br-cos_q_motor_wheel_joint_br);
    (*this)(0,1) = - 0.5 * (sin_q_motor_wheel_joint_br+cos_q_motor_wheel_joint_br);
    (*this)(1,0) =  0.5 * (sin_q_motor_wheel_joint_br-cos_q_motor_wheel_joint_br);
    (*this)(1,1) =  0.5 * (sin_q_motor_wheel_joint_br+cos_q_motor_wheel_joint_br);
    (*this)(2,0) = - 0.5 * (( 1.41421 * sin_q_motor_wheel_joint_br)+( 1.41421 * cos_q_motor_wheel_joint_br));
    (*this)(2,1) =  0.5 * (( 1.41421 * sin_q_motor_wheel_joint_br)-( 1.41421 * cos_q_motor_wheel_joint_br));
    return *this;
}
HomogeneousTransforms::Type_fr_xarmlink1_X_fr_base_link_footprint::Type_fr_xarmlink1_X_fr_base_link_footprint()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - tz_xarmjoint1;    // Maxima DSL: -_k__tz_xarmjoint1
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_xarmlink1_X_fr_base_link_footprint& HomogeneousTransforms::Type_fr_xarmlink1_X_fr_base_link_footprint::update(const state_t& q)
{
    Scalar sin_q_xarmjoint1  = ScalarTraits::sin( q(XARMJOINT1) );
    Scalar cos_q_xarmjoint1  = ScalarTraits::cos( q(XARMJOINT1) );
    (*this)(0,0) = cos_q_xarmjoint1;
    (*this)(0,1) = sin_q_xarmjoint1;
    (*this)(0,3) = - tx_xarmjoint1 * cos_q_xarmjoint1;
    (*this)(1,0) = -sin_q_xarmjoint1;
    (*this)(1,1) = cos_q_xarmjoint1;
    (*this)(1,3) =  tx_xarmjoint1 * sin_q_xarmjoint1;
    return *this;
}
HomogeneousTransforms::Type_fr_base_link_footprint_X_fr_xarmlink1::Type_fr_base_link_footprint_X_fr_xarmlink1()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_xarmjoint1;    // Maxima DSL: _k__tx_xarmjoint1
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  tz_xarmjoint1;    // Maxima DSL: _k__tz_xarmjoint1
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_link_footprint_X_fr_xarmlink1& HomogeneousTransforms::Type_fr_base_link_footprint_X_fr_xarmlink1::update(const state_t& q)
{
    Scalar sin_q_xarmjoint1  = ScalarTraits::sin( q(XARMJOINT1) );
    Scalar cos_q_xarmjoint1  = ScalarTraits::cos( q(XARMJOINT1) );
    (*this)(0,0) = cos_q_xarmjoint1;
    (*this)(0,1) = -sin_q_xarmjoint1;
    (*this)(1,0) = sin_q_xarmjoint1;
    (*this)(1,1) = cos_q_xarmjoint1;
    return *this;
}
HomogeneousTransforms::Type_fr_xarmlink2_X_fr_xarmlink1::Type_fr_xarmlink2_X_fr_xarmlink1()
{
    (*this)(0,3) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_xarmjoint2;    // Maxima DSL: -sin(_k__rx_xarmjoint2)
    (*this)(2,2) = cos_rx_xarmjoint2;    // Maxima DSL: cos(_k__rx_xarmjoint2)
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_xarmlink2_X_fr_xarmlink1& HomogeneousTransforms::Type_fr_xarmlink2_X_fr_xarmlink1::update(const state_t& q)
{
    Scalar sin_q_xarmjoint2  = ScalarTraits::sin( q(XARMJOINT2) );
    Scalar cos_q_xarmjoint2  = ScalarTraits::cos( q(XARMJOINT2) );
    (*this)(0,0) = cos_q_xarmjoint2;
    (*this)(0,1) = cos_rx_xarmjoint2 * sin_q_xarmjoint2;
    (*this)(0,2) = sin_rx_xarmjoint2 * sin_q_xarmjoint2;
    (*this)(1,0) = -sin_q_xarmjoint2;
    (*this)(1,1) = cos_rx_xarmjoint2 * cos_q_xarmjoint2;
    (*this)(1,2) = sin_rx_xarmjoint2 * cos_q_xarmjoint2;
    return *this;
}
HomogeneousTransforms::Type_fr_xarmlink1_X_fr_xarmlink2::Type_fr_xarmlink1_X_fr_xarmlink2()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,2) = -sin_rx_xarmjoint2;    // Maxima DSL: -sin(_k__rx_xarmjoint2)
    (*this)(1,3) = 0.0;
    (*this)(2,2) = cos_rx_xarmjoint2;    // Maxima DSL: cos(_k__rx_xarmjoint2)
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_xarmlink1_X_fr_xarmlink2& HomogeneousTransforms::Type_fr_xarmlink1_X_fr_xarmlink2::update(const state_t& q)
{
    Scalar sin_q_xarmjoint2  = ScalarTraits::sin( q(XARMJOINT2) );
    Scalar cos_q_xarmjoint2  = ScalarTraits::cos( q(XARMJOINT2) );
    (*this)(0,0) = cos_q_xarmjoint2;
    (*this)(0,1) = -sin_q_xarmjoint2;
    (*this)(1,0) = cos_rx_xarmjoint2 * sin_q_xarmjoint2;
    (*this)(1,1) = cos_rx_xarmjoint2 * cos_q_xarmjoint2;
    (*this)(2,0) = sin_rx_xarmjoint2 * sin_q_xarmjoint2;
    (*this)(2,1) = sin_rx_xarmjoint2 * cos_q_xarmjoint2;
    return *this;
}
HomogeneousTransforms::Type_fr_xarmlink3_X_fr_xarmlink2::Type_fr_xarmlink3_X_fr_xarmlink2()
{
    (*this)(0,2) = 0.0;
    (*this)(1,2) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_xarmlink3_X_fr_xarmlink2& HomogeneousTransforms::Type_fr_xarmlink3_X_fr_xarmlink2::update(const state_t& q)
{
    Scalar sin_q_xarmjoint3  = ScalarTraits::sin( q(XARMJOINT3) );
    Scalar cos_q_xarmjoint3  = ScalarTraits::cos( q(XARMJOINT3) );
    (*this)(0,0) = cos_q_xarmjoint3;
    (*this)(0,1) = sin_q_xarmjoint3;
    (*this)(0,3) = (- ty_xarmjoint3 * sin_q_xarmjoint3)-( tx_xarmjoint3 * cos_q_xarmjoint3);
    (*this)(1,0) = -sin_q_xarmjoint3;
    (*this)(1,1) = cos_q_xarmjoint3;
    (*this)(1,3) = ( tx_xarmjoint3 * sin_q_xarmjoint3)-( ty_xarmjoint3 * cos_q_xarmjoint3);
    return *this;
}
HomogeneousTransforms::Type_fr_xarmlink2_X_fr_xarmlink3::Type_fr_xarmlink2_X_fr_xarmlink3()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_xarmjoint3;    // Maxima DSL: _k__tx_xarmjoint3
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_xarmjoint3;    // Maxima DSL: _k__ty_xarmjoint3
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_xarmlink2_X_fr_xarmlink3& HomogeneousTransforms::Type_fr_xarmlink2_X_fr_xarmlink3::update(const state_t& q)
{
    Scalar sin_q_xarmjoint3  = ScalarTraits::sin( q(XARMJOINT3) );
    Scalar cos_q_xarmjoint3  = ScalarTraits::cos( q(XARMJOINT3) );
    (*this)(0,0) = cos_q_xarmjoint3;
    (*this)(0,1) = -sin_q_xarmjoint3;
    (*this)(1,0) = sin_q_xarmjoint3;
    (*this)(1,1) = cos_q_xarmjoint3;
    return *this;
}
HomogeneousTransforms::Type_fr_xarmlink4_X_fr_xarmlink3::Type_fr_xarmlink4_X_fr_xarmlink3()
{
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_xarmjoint4;    // Maxima DSL: -sin(_k__rx_xarmjoint4)
    (*this)(2,2) = cos_rx_xarmjoint4;    // Maxima DSL: cos(_k__rx_xarmjoint4)
    (*this)(2,3) =  ty_xarmjoint4 * sin_rx_xarmjoint4;    // Maxima DSL: _k__ty_xarmjoint4*sin(_k__rx_xarmjoint4)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_xarmlink4_X_fr_xarmlink3& HomogeneousTransforms::Type_fr_xarmlink4_X_fr_xarmlink3::update(const state_t& q)
{
    Scalar sin_q_xarmjoint4  = ScalarTraits::sin( q(XARMJOINT4) );
    Scalar cos_q_xarmjoint4  = ScalarTraits::cos( q(XARMJOINT4) );
    (*this)(0,0) = cos_q_xarmjoint4;
    (*this)(0,1) = cos_rx_xarmjoint4 * sin_q_xarmjoint4;
    (*this)(0,2) = sin_rx_xarmjoint4 * sin_q_xarmjoint4;
    (*this)(0,3) = (- ty_xarmjoint4 * cos_rx_xarmjoint4 * sin_q_xarmjoint4)-( tx_xarmjoint4 * cos_q_xarmjoint4);
    (*this)(1,0) = -sin_q_xarmjoint4;
    (*this)(1,1) = cos_rx_xarmjoint4 * cos_q_xarmjoint4;
    (*this)(1,2) = sin_rx_xarmjoint4 * cos_q_xarmjoint4;
    (*this)(1,3) = ( tx_xarmjoint4 * sin_q_xarmjoint4)-( ty_xarmjoint4 * cos_rx_xarmjoint4 * cos_q_xarmjoint4);
    return *this;
}
HomogeneousTransforms::Type_fr_xarmlink3_X_fr_xarmlink4::Type_fr_xarmlink3_X_fr_xarmlink4()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_xarmjoint4;    // Maxima DSL: _k__tx_xarmjoint4
    (*this)(1,2) = -sin_rx_xarmjoint4;    // Maxima DSL: -sin(_k__rx_xarmjoint4)
    (*this)(1,3) =  ty_xarmjoint4;    // Maxima DSL: _k__ty_xarmjoint4
    (*this)(2,2) = cos_rx_xarmjoint4;    // Maxima DSL: cos(_k__rx_xarmjoint4)
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_xarmlink3_X_fr_xarmlink4& HomogeneousTransforms::Type_fr_xarmlink3_X_fr_xarmlink4::update(const state_t& q)
{
    Scalar sin_q_xarmjoint4  = ScalarTraits::sin( q(XARMJOINT4) );
    Scalar cos_q_xarmjoint4  = ScalarTraits::cos( q(XARMJOINT4) );
    (*this)(0,0) = cos_q_xarmjoint4;
    (*this)(0,1) = -sin_q_xarmjoint4;
    (*this)(1,0) = cos_rx_xarmjoint4 * sin_q_xarmjoint4;
    (*this)(1,1) = cos_rx_xarmjoint4 * cos_q_xarmjoint4;
    (*this)(2,0) = sin_rx_xarmjoint4 * sin_q_xarmjoint4;
    (*this)(2,1) = sin_rx_xarmjoint4 * cos_q_xarmjoint4;
    return *this;
}
HomogeneousTransforms::Type_fr_xarmlink5_X_fr_xarmlink4::Type_fr_xarmlink5_X_fr_xarmlink4()
{
    (*this)(0,3) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_xarmjoint5;    // Maxima DSL: -sin(_k__rx_xarmjoint5)
    (*this)(2,2) = cos_rx_xarmjoint5;    // Maxima DSL: cos(_k__rx_xarmjoint5)
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_xarmlink5_X_fr_xarmlink4& HomogeneousTransforms::Type_fr_xarmlink5_X_fr_xarmlink4::update(const state_t& q)
{
    Scalar sin_q_xarmjoint5  = ScalarTraits::sin( q(XARMJOINT5) );
    Scalar cos_q_xarmjoint5  = ScalarTraits::cos( q(XARMJOINT5) );
    (*this)(0,0) = cos_q_xarmjoint5;
    (*this)(0,1) = cos_rx_xarmjoint5 * sin_q_xarmjoint5;
    (*this)(0,2) = sin_rx_xarmjoint5 * sin_q_xarmjoint5;
    (*this)(1,0) = -sin_q_xarmjoint5;
    (*this)(1,1) = cos_rx_xarmjoint5 * cos_q_xarmjoint5;
    (*this)(1,2) = sin_rx_xarmjoint5 * cos_q_xarmjoint5;
    return *this;
}
HomogeneousTransforms::Type_fr_xarmlink4_X_fr_xarmlink5::Type_fr_xarmlink4_X_fr_xarmlink5()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,2) = -sin_rx_xarmjoint5;    // Maxima DSL: -sin(_k__rx_xarmjoint5)
    (*this)(1,3) = 0.0;
    (*this)(2,2) = cos_rx_xarmjoint5;    // Maxima DSL: cos(_k__rx_xarmjoint5)
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_xarmlink4_X_fr_xarmlink5& HomogeneousTransforms::Type_fr_xarmlink4_X_fr_xarmlink5::update(const state_t& q)
{
    Scalar sin_q_xarmjoint5  = ScalarTraits::sin( q(XARMJOINT5) );
    Scalar cos_q_xarmjoint5  = ScalarTraits::cos( q(XARMJOINT5) );
    (*this)(0,0) = cos_q_xarmjoint5;
    (*this)(0,1) = -sin_q_xarmjoint5;
    (*this)(1,0) = cos_rx_xarmjoint5 * sin_q_xarmjoint5;
    (*this)(1,1) = cos_rx_xarmjoint5 * cos_q_xarmjoint5;
    (*this)(2,0) = sin_rx_xarmjoint5 * sin_q_xarmjoint5;
    (*this)(2,1) = sin_rx_xarmjoint5 * cos_q_xarmjoint5;
    return *this;
}
HomogeneousTransforms::Type_fr_xarmlink6_X_fr_xarmlink5::Type_fr_xarmlink6_X_fr_xarmlink5()
{
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -sin_rx_xarmjoint6;    // Maxima DSL: -sin(_k__rx_xarmjoint6)
    (*this)(2,2) = cos_rx_xarmjoint6;    // Maxima DSL: cos(_k__rx_xarmjoint6)
    (*this)(2,3) =  ty_xarmjoint6 * sin_rx_xarmjoint6;    // Maxima DSL: _k__ty_xarmjoint6*sin(_k__rx_xarmjoint6)
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_xarmlink6_X_fr_xarmlink5& HomogeneousTransforms::Type_fr_xarmlink6_X_fr_xarmlink5::update(const state_t& q)
{
    Scalar sin_q_xarmjoint6  = ScalarTraits::sin( q(XARMJOINT6) );
    Scalar cos_q_xarmjoint6  = ScalarTraits::cos( q(XARMJOINT6) );
    (*this)(0,0) = cos_q_xarmjoint6;
    (*this)(0,1) = cos_rx_xarmjoint6 * sin_q_xarmjoint6;
    (*this)(0,2) = sin_rx_xarmjoint6 * sin_q_xarmjoint6;
    (*this)(0,3) = (- ty_xarmjoint6 * cos_rx_xarmjoint6 * sin_q_xarmjoint6)-( tx_xarmjoint6 * cos_q_xarmjoint6);
    (*this)(1,0) = -sin_q_xarmjoint6;
    (*this)(1,1) = cos_rx_xarmjoint6 * cos_q_xarmjoint6;
    (*this)(1,2) = sin_rx_xarmjoint6 * cos_q_xarmjoint6;
    (*this)(1,3) = ( tx_xarmjoint6 * sin_q_xarmjoint6)-( ty_xarmjoint6 * cos_rx_xarmjoint6 * cos_q_xarmjoint6);
    return *this;
}
HomogeneousTransforms::Type_fr_xarmlink5_X_fr_xarmlink6::Type_fr_xarmlink5_X_fr_xarmlink6()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_xarmjoint6;    // Maxima DSL: _k__tx_xarmjoint6
    (*this)(1,2) = -sin_rx_xarmjoint6;    // Maxima DSL: -sin(_k__rx_xarmjoint6)
    (*this)(1,3) =  ty_xarmjoint6;    // Maxima DSL: _k__ty_xarmjoint6
    (*this)(2,2) = cos_rx_xarmjoint6;    // Maxima DSL: cos(_k__rx_xarmjoint6)
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_xarmlink5_X_fr_xarmlink6& HomogeneousTransforms::Type_fr_xarmlink5_X_fr_xarmlink6::update(const state_t& q)
{
    Scalar sin_q_xarmjoint6  = ScalarTraits::sin( q(XARMJOINT6) );
    Scalar cos_q_xarmjoint6  = ScalarTraits::cos( q(XARMJOINT6) );
    (*this)(0,0) = cos_q_xarmjoint6;
    (*this)(0,1) = -sin_q_xarmjoint6;
    (*this)(1,0) = cos_rx_xarmjoint6 * sin_q_xarmjoint6;
    (*this)(1,1) = cos_rx_xarmjoint6 * cos_q_xarmjoint6;
    (*this)(2,0) = sin_rx_xarmjoint6 * sin_q_xarmjoint6;
    (*this)(2,1) = sin_rx_xarmjoint6 * cos_q_xarmjoint6;
    return *this;
}

