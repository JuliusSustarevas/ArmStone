#ifndef ARMSTONE_TRANSFORMS_H_
#define ARMSTONE_TRANSFORMS_H_

#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include "model_constants.h"
#include "kinematics_parameters.h"

namespace armstone {
namespace rcg {

struct Parameters
{
    struct AngleFuncValues {
        AngleFuncValues() {
            update();
        }

        void update()
        {
        }
    };

    Params_lengths lengths;
    Params_angles angles;
    AngleFuncValues trig = AngleFuncValues();
};

// The type of the "vector" with the status of the variables
typedef JointState state_t;

template<class M>
using TransformMotion = iit::rbd::SpatialTransformBase<state_t, M>;

template<class M>
using TransformForce = iit::rbd::SpatialTransformBase<state_t, M>;

template<class M>
using TransformHomogeneous = iit::rbd::HomogeneousTransformBase<state_t, M>;

/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial motion vectors.
 */
class MotionTransforms
{
public:
    class Dummy {};
    typedef TransformMotion<Dummy>::MatrixType MatrixType;

    struct Type_fr_omniwheel_fl_X_fr_base_link_footprint : public TransformMotion<Type_fr_omniwheel_fl_X_fr_base_link_footprint>
    {
        Type_fr_omniwheel_fl_X_fr_base_link_footprint();
        const Type_fr_omniwheel_fl_X_fr_base_link_footprint& update(const state_t&);
    };
    
    struct Type_fr_base_link_footprint_X_fr_omniwheel_fl : public TransformMotion<Type_fr_base_link_footprint_X_fr_omniwheel_fl>
    {
        Type_fr_base_link_footprint_X_fr_omniwheel_fl();
        const Type_fr_base_link_footprint_X_fr_omniwheel_fl& update(const state_t&);
    };
    
    struct Type_fr_omniwheel_fr_X_fr_base_link_footprint : public TransformMotion<Type_fr_omniwheel_fr_X_fr_base_link_footprint>
    {
        Type_fr_omniwheel_fr_X_fr_base_link_footprint();
        const Type_fr_omniwheel_fr_X_fr_base_link_footprint& update(const state_t&);
    };
    
    struct Type_fr_base_link_footprint_X_fr_omniwheel_fr : public TransformMotion<Type_fr_base_link_footprint_X_fr_omniwheel_fr>
    {
        Type_fr_base_link_footprint_X_fr_omniwheel_fr();
        const Type_fr_base_link_footprint_X_fr_omniwheel_fr& update(const state_t&);
    };
    
    struct Type_fr_omniwheel_bl_X_fr_base_link_footprint : public TransformMotion<Type_fr_omniwheel_bl_X_fr_base_link_footprint>
    {
        Type_fr_omniwheel_bl_X_fr_base_link_footprint();
        const Type_fr_omniwheel_bl_X_fr_base_link_footprint& update(const state_t&);
    };
    
    struct Type_fr_base_link_footprint_X_fr_omniwheel_bl : public TransformMotion<Type_fr_base_link_footprint_X_fr_omniwheel_bl>
    {
        Type_fr_base_link_footprint_X_fr_omniwheel_bl();
        const Type_fr_base_link_footprint_X_fr_omniwheel_bl& update(const state_t&);
    };
    
    struct Type_fr_omniwheel_br_X_fr_base_link_footprint : public TransformMotion<Type_fr_omniwheel_br_X_fr_base_link_footprint>
    {
        Type_fr_omniwheel_br_X_fr_base_link_footprint();
        const Type_fr_omniwheel_br_X_fr_base_link_footprint& update(const state_t&);
    };
    
    struct Type_fr_base_link_footprint_X_fr_omniwheel_br : public TransformMotion<Type_fr_base_link_footprint_X_fr_omniwheel_br>
    {
        Type_fr_base_link_footprint_X_fr_omniwheel_br();
        const Type_fr_base_link_footprint_X_fr_omniwheel_br& update(const state_t&);
    };
    
    struct Type_fr_xarmlink1_X_fr_base_link_footprint : public TransformMotion<Type_fr_xarmlink1_X_fr_base_link_footprint>
    {
        Type_fr_xarmlink1_X_fr_base_link_footprint();
        const Type_fr_xarmlink1_X_fr_base_link_footprint& update(const state_t&);
    };
    
    struct Type_fr_base_link_footprint_X_fr_xarmlink1 : public TransformMotion<Type_fr_base_link_footprint_X_fr_xarmlink1>
    {
        Type_fr_base_link_footprint_X_fr_xarmlink1();
        const Type_fr_base_link_footprint_X_fr_xarmlink1& update(const state_t&);
    };
    
    struct Type_fr_xarmlink2_X_fr_xarmlink1 : public TransformMotion<Type_fr_xarmlink2_X_fr_xarmlink1>
    {
        Type_fr_xarmlink2_X_fr_xarmlink1();
        const Type_fr_xarmlink2_X_fr_xarmlink1& update(const state_t&);
    };
    
    struct Type_fr_xarmlink1_X_fr_xarmlink2 : public TransformMotion<Type_fr_xarmlink1_X_fr_xarmlink2>
    {
        Type_fr_xarmlink1_X_fr_xarmlink2();
        const Type_fr_xarmlink1_X_fr_xarmlink2& update(const state_t&);
    };
    
    struct Type_fr_xarmlink3_X_fr_xarmlink2 : public TransformMotion<Type_fr_xarmlink3_X_fr_xarmlink2>
    {
        Type_fr_xarmlink3_X_fr_xarmlink2();
        const Type_fr_xarmlink3_X_fr_xarmlink2& update(const state_t&);
    };
    
    struct Type_fr_xarmlink2_X_fr_xarmlink3 : public TransformMotion<Type_fr_xarmlink2_X_fr_xarmlink3>
    {
        Type_fr_xarmlink2_X_fr_xarmlink3();
        const Type_fr_xarmlink2_X_fr_xarmlink3& update(const state_t&);
    };
    
    struct Type_fr_xarmlink4_X_fr_xarmlink3 : public TransformMotion<Type_fr_xarmlink4_X_fr_xarmlink3>
    {
        Type_fr_xarmlink4_X_fr_xarmlink3();
        const Type_fr_xarmlink4_X_fr_xarmlink3& update(const state_t&);
    };
    
    struct Type_fr_xarmlink3_X_fr_xarmlink4 : public TransformMotion<Type_fr_xarmlink3_X_fr_xarmlink4>
    {
        Type_fr_xarmlink3_X_fr_xarmlink4();
        const Type_fr_xarmlink3_X_fr_xarmlink4& update(const state_t&);
    };
    
    struct Type_fr_xarmlink5_X_fr_xarmlink4 : public TransformMotion<Type_fr_xarmlink5_X_fr_xarmlink4>
    {
        Type_fr_xarmlink5_X_fr_xarmlink4();
        const Type_fr_xarmlink5_X_fr_xarmlink4& update(const state_t&);
    };
    
    struct Type_fr_xarmlink4_X_fr_xarmlink5 : public TransformMotion<Type_fr_xarmlink4_X_fr_xarmlink5>
    {
        Type_fr_xarmlink4_X_fr_xarmlink5();
        const Type_fr_xarmlink4_X_fr_xarmlink5& update(const state_t&);
    };
    
    struct Type_fr_xarmlink6_X_fr_xarmlink5 : public TransformMotion<Type_fr_xarmlink6_X_fr_xarmlink5>
    {
        Type_fr_xarmlink6_X_fr_xarmlink5();
        const Type_fr_xarmlink6_X_fr_xarmlink5& update(const state_t&);
    };
    
    struct Type_fr_xarmlink5_X_fr_xarmlink6 : public TransformMotion<Type_fr_xarmlink5_X_fr_xarmlink6>
    {
        Type_fr_xarmlink5_X_fr_xarmlink6();
        const Type_fr_xarmlink5_X_fr_xarmlink6& update(const state_t&);
    };
    
public:
    MotionTransforms();
    void updateParams(const Params_lengths&, const Params_angles&);

    Type_fr_omniwheel_fl_X_fr_base_link_footprint fr_omniwheel_fl_X_fr_base_link_footprint;
    Type_fr_base_link_footprint_X_fr_omniwheel_fl fr_base_link_footprint_X_fr_omniwheel_fl;
    Type_fr_omniwheel_fr_X_fr_base_link_footprint fr_omniwheel_fr_X_fr_base_link_footprint;
    Type_fr_base_link_footprint_X_fr_omniwheel_fr fr_base_link_footprint_X_fr_omniwheel_fr;
    Type_fr_omniwheel_bl_X_fr_base_link_footprint fr_omniwheel_bl_X_fr_base_link_footprint;
    Type_fr_base_link_footprint_X_fr_omniwheel_bl fr_base_link_footprint_X_fr_omniwheel_bl;
    Type_fr_omniwheel_br_X_fr_base_link_footprint fr_omniwheel_br_X_fr_base_link_footprint;
    Type_fr_base_link_footprint_X_fr_omniwheel_br fr_base_link_footprint_X_fr_omniwheel_br;
    Type_fr_xarmlink1_X_fr_base_link_footprint fr_xarmlink1_X_fr_base_link_footprint;
    Type_fr_base_link_footprint_X_fr_xarmlink1 fr_base_link_footprint_X_fr_xarmlink1;
    Type_fr_xarmlink2_X_fr_xarmlink1 fr_xarmlink2_X_fr_xarmlink1;
    Type_fr_xarmlink1_X_fr_xarmlink2 fr_xarmlink1_X_fr_xarmlink2;
    Type_fr_xarmlink3_X_fr_xarmlink2 fr_xarmlink3_X_fr_xarmlink2;
    Type_fr_xarmlink2_X_fr_xarmlink3 fr_xarmlink2_X_fr_xarmlink3;
    Type_fr_xarmlink4_X_fr_xarmlink3 fr_xarmlink4_X_fr_xarmlink3;
    Type_fr_xarmlink3_X_fr_xarmlink4 fr_xarmlink3_X_fr_xarmlink4;
    Type_fr_xarmlink5_X_fr_xarmlink4 fr_xarmlink5_X_fr_xarmlink4;
    Type_fr_xarmlink4_X_fr_xarmlink5 fr_xarmlink4_X_fr_xarmlink5;
    Type_fr_xarmlink6_X_fr_xarmlink5 fr_xarmlink6_X_fr_xarmlink5;
    Type_fr_xarmlink5_X_fr_xarmlink6 fr_xarmlink5_X_fr_xarmlink6;

protected:
    Parameters params;

}; //class 'MotionTransforms'

/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial force vectors.
 */
class ForceTransforms
{
public:
    class Dummy {};
    typedef TransformForce<Dummy>::MatrixType MatrixType;

    struct Type_fr_omniwheel_fl_X_fr_base_link_footprint : public TransformForce<Type_fr_omniwheel_fl_X_fr_base_link_footprint>
    {
        Type_fr_omniwheel_fl_X_fr_base_link_footprint();
        const Type_fr_omniwheel_fl_X_fr_base_link_footprint& update(const state_t&);
    };
    
    struct Type_fr_base_link_footprint_X_fr_omniwheel_fl : public TransformForce<Type_fr_base_link_footprint_X_fr_omniwheel_fl>
    {
        Type_fr_base_link_footprint_X_fr_omniwheel_fl();
        const Type_fr_base_link_footprint_X_fr_omniwheel_fl& update(const state_t&);
    };
    
    struct Type_fr_omniwheel_fr_X_fr_base_link_footprint : public TransformForce<Type_fr_omniwheel_fr_X_fr_base_link_footprint>
    {
        Type_fr_omniwheel_fr_X_fr_base_link_footprint();
        const Type_fr_omniwheel_fr_X_fr_base_link_footprint& update(const state_t&);
    };
    
    struct Type_fr_base_link_footprint_X_fr_omniwheel_fr : public TransformForce<Type_fr_base_link_footprint_X_fr_omniwheel_fr>
    {
        Type_fr_base_link_footprint_X_fr_omniwheel_fr();
        const Type_fr_base_link_footprint_X_fr_omniwheel_fr& update(const state_t&);
    };
    
    struct Type_fr_omniwheel_bl_X_fr_base_link_footprint : public TransformForce<Type_fr_omniwheel_bl_X_fr_base_link_footprint>
    {
        Type_fr_omniwheel_bl_X_fr_base_link_footprint();
        const Type_fr_omniwheel_bl_X_fr_base_link_footprint& update(const state_t&);
    };
    
    struct Type_fr_base_link_footprint_X_fr_omniwheel_bl : public TransformForce<Type_fr_base_link_footprint_X_fr_omniwheel_bl>
    {
        Type_fr_base_link_footprint_X_fr_omniwheel_bl();
        const Type_fr_base_link_footprint_X_fr_omniwheel_bl& update(const state_t&);
    };
    
    struct Type_fr_omniwheel_br_X_fr_base_link_footprint : public TransformForce<Type_fr_omniwheel_br_X_fr_base_link_footprint>
    {
        Type_fr_omniwheel_br_X_fr_base_link_footprint();
        const Type_fr_omniwheel_br_X_fr_base_link_footprint& update(const state_t&);
    };
    
    struct Type_fr_base_link_footprint_X_fr_omniwheel_br : public TransformForce<Type_fr_base_link_footprint_X_fr_omniwheel_br>
    {
        Type_fr_base_link_footprint_X_fr_omniwheel_br();
        const Type_fr_base_link_footprint_X_fr_omniwheel_br& update(const state_t&);
    };
    
    struct Type_fr_xarmlink1_X_fr_base_link_footprint : public TransformForce<Type_fr_xarmlink1_X_fr_base_link_footprint>
    {
        Type_fr_xarmlink1_X_fr_base_link_footprint();
        const Type_fr_xarmlink1_X_fr_base_link_footprint& update(const state_t&);
    };
    
    struct Type_fr_base_link_footprint_X_fr_xarmlink1 : public TransformForce<Type_fr_base_link_footprint_X_fr_xarmlink1>
    {
        Type_fr_base_link_footprint_X_fr_xarmlink1();
        const Type_fr_base_link_footprint_X_fr_xarmlink1& update(const state_t&);
    };
    
    struct Type_fr_xarmlink2_X_fr_xarmlink1 : public TransformForce<Type_fr_xarmlink2_X_fr_xarmlink1>
    {
        Type_fr_xarmlink2_X_fr_xarmlink1();
        const Type_fr_xarmlink2_X_fr_xarmlink1& update(const state_t&);
    };
    
    struct Type_fr_xarmlink1_X_fr_xarmlink2 : public TransformForce<Type_fr_xarmlink1_X_fr_xarmlink2>
    {
        Type_fr_xarmlink1_X_fr_xarmlink2();
        const Type_fr_xarmlink1_X_fr_xarmlink2& update(const state_t&);
    };
    
    struct Type_fr_xarmlink3_X_fr_xarmlink2 : public TransformForce<Type_fr_xarmlink3_X_fr_xarmlink2>
    {
        Type_fr_xarmlink3_X_fr_xarmlink2();
        const Type_fr_xarmlink3_X_fr_xarmlink2& update(const state_t&);
    };
    
    struct Type_fr_xarmlink2_X_fr_xarmlink3 : public TransformForce<Type_fr_xarmlink2_X_fr_xarmlink3>
    {
        Type_fr_xarmlink2_X_fr_xarmlink3();
        const Type_fr_xarmlink2_X_fr_xarmlink3& update(const state_t&);
    };
    
    struct Type_fr_xarmlink4_X_fr_xarmlink3 : public TransformForce<Type_fr_xarmlink4_X_fr_xarmlink3>
    {
        Type_fr_xarmlink4_X_fr_xarmlink3();
        const Type_fr_xarmlink4_X_fr_xarmlink3& update(const state_t&);
    };
    
    struct Type_fr_xarmlink3_X_fr_xarmlink4 : public TransformForce<Type_fr_xarmlink3_X_fr_xarmlink4>
    {
        Type_fr_xarmlink3_X_fr_xarmlink4();
        const Type_fr_xarmlink3_X_fr_xarmlink4& update(const state_t&);
    };
    
    struct Type_fr_xarmlink5_X_fr_xarmlink4 : public TransformForce<Type_fr_xarmlink5_X_fr_xarmlink4>
    {
        Type_fr_xarmlink5_X_fr_xarmlink4();
        const Type_fr_xarmlink5_X_fr_xarmlink4& update(const state_t&);
    };
    
    struct Type_fr_xarmlink4_X_fr_xarmlink5 : public TransformForce<Type_fr_xarmlink4_X_fr_xarmlink5>
    {
        Type_fr_xarmlink4_X_fr_xarmlink5();
        const Type_fr_xarmlink4_X_fr_xarmlink5& update(const state_t&);
    };
    
    struct Type_fr_xarmlink6_X_fr_xarmlink5 : public TransformForce<Type_fr_xarmlink6_X_fr_xarmlink5>
    {
        Type_fr_xarmlink6_X_fr_xarmlink5();
        const Type_fr_xarmlink6_X_fr_xarmlink5& update(const state_t&);
    };
    
    struct Type_fr_xarmlink5_X_fr_xarmlink6 : public TransformForce<Type_fr_xarmlink5_X_fr_xarmlink6>
    {
        Type_fr_xarmlink5_X_fr_xarmlink6();
        const Type_fr_xarmlink5_X_fr_xarmlink6& update(const state_t&);
    };
    
public:
    ForceTransforms();
    void updateParams(const Params_lengths&, const Params_angles&);

    Type_fr_omniwheel_fl_X_fr_base_link_footprint fr_omniwheel_fl_X_fr_base_link_footprint;
    Type_fr_base_link_footprint_X_fr_omniwheel_fl fr_base_link_footprint_X_fr_omniwheel_fl;
    Type_fr_omniwheel_fr_X_fr_base_link_footprint fr_omniwheel_fr_X_fr_base_link_footprint;
    Type_fr_base_link_footprint_X_fr_omniwheel_fr fr_base_link_footprint_X_fr_omniwheel_fr;
    Type_fr_omniwheel_bl_X_fr_base_link_footprint fr_omniwheel_bl_X_fr_base_link_footprint;
    Type_fr_base_link_footprint_X_fr_omniwheel_bl fr_base_link_footprint_X_fr_omniwheel_bl;
    Type_fr_omniwheel_br_X_fr_base_link_footprint fr_omniwheel_br_X_fr_base_link_footprint;
    Type_fr_base_link_footprint_X_fr_omniwheel_br fr_base_link_footprint_X_fr_omniwheel_br;
    Type_fr_xarmlink1_X_fr_base_link_footprint fr_xarmlink1_X_fr_base_link_footprint;
    Type_fr_base_link_footprint_X_fr_xarmlink1 fr_base_link_footprint_X_fr_xarmlink1;
    Type_fr_xarmlink2_X_fr_xarmlink1 fr_xarmlink2_X_fr_xarmlink1;
    Type_fr_xarmlink1_X_fr_xarmlink2 fr_xarmlink1_X_fr_xarmlink2;
    Type_fr_xarmlink3_X_fr_xarmlink2 fr_xarmlink3_X_fr_xarmlink2;
    Type_fr_xarmlink2_X_fr_xarmlink3 fr_xarmlink2_X_fr_xarmlink3;
    Type_fr_xarmlink4_X_fr_xarmlink3 fr_xarmlink4_X_fr_xarmlink3;
    Type_fr_xarmlink3_X_fr_xarmlink4 fr_xarmlink3_X_fr_xarmlink4;
    Type_fr_xarmlink5_X_fr_xarmlink4 fr_xarmlink5_X_fr_xarmlink4;
    Type_fr_xarmlink4_X_fr_xarmlink5 fr_xarmlink4_X_fr_xarmlink5;
    Type_fr_xarmlink6_X_fr_xarmlink5 fr_xarmlink6_X_fr_xarmlink5;
    Type_fr_xarmlink5_X_fr_xarmlink6 fr_xarmlink5_X_fr_xarmlink6;

protected:
    Parameters params;

}; //class 'ForceTransforms'

/**
 * The class with the homogeneous (4x4) coordinates transformation
 * matrices.
 */
class HomogeneousTransforms
{
public:
    class Dummy {};
    typedef TransformHomogeneous<Dummy>::MatrixType MatrixType;

    struct Type_fr_omniwheel_fl_X_fr_base_link_footprint : public TransformHomogeneous<Type_fr_omniwheel_fl_X_fr_base_link_footprint>
    {
        Type_fr_omniwheel_fl_X_fr_base_link_footprint();
        const Type_fr_omniwheel_fl_X_fr_base_link_footprint& update(const state_t&);
    };
    
    struct Type_fr_base_link_footprint_X_fr_omniwheel_fl : public TransformHomogeneous<Type_fr_base_link_footprint_X_fr_omniwheel_fl>
    {
        Type_fr_base_link_footprint_X_fr_omniwheel_fl();
        const Type_fr_base_link_footprint_X_fr_omniwheel_fl& update(const state_t&);
    };
    
    struct Type_fr_omniwheel_fr_X_fr_base_link_footprint : public TransformHomogeneous<Type_fr_omniwheel_fr_X_fr_base_link_footprint>
    {
        Type_fr_omniwheel_fr_X_fr_base_link_footprint();
        const Type_fr_omniwheel_fr_X_fr_base_link_footprint& update(const state_t&);
    };
    
    struct Type_fr_base_link_footprint_X_fr_omniwheel_fr : public TransformHomogeneous<Type_fr_base_link_footprint_X_fr_omniwheel_fr>
    {
        Type_fr_base_link_footprint_X_fr_omniwheel_fr();
        const Type_fr_base_link_footprint_X_fr_omniwheel_fr& update(const state_t&);
    };
    
    struct Type_fr_omniwheel_bl_X_fr_base_link_footprint : public TransformHomogeneous<Type_fr_omniwheel_bl_X_fr_base_link_footprint>
    {
        Type_fr_omniwheel_bl_X_fr_base_link_footprint();
        const Type_fr_omniwheel_bl_X_fr_base_link_footprint& update(const state_t&);
    };
    
    struct Type_fr_base_link_footprint_X_fr_omniwheel_bl : public TransformHomogeneous<Type_fr_base_link_footprint_X_fr_omniwheel_bl>
    {
        Type_fr_base_link_footprint_X_fr_omniwheel_bl();
        const Type_fr_base_link_footprint_X_fr_omniwheel_bl& update(const state_t&);
    };
    
    struct Type_fr_omniwheel_br_X_fr_base_link_footprint : public TransformHomogeneous<Type_fr_omniwheel_br_X_fr_base_link_footprint>
    {
        Type_fr_omniwheel_br_X_fr_base_link_footprint();
        const Type_fr_omniwheel_br_X_fr_base_link_footprint& update(const state_t&);
    };
    
    struct Type_fr_base_link_footprint_X_fr_omniwheel_br : public TransformHomogeneous<Type_fr_base_link_footprint_X_fr_omniwheel_br>
    {
        Type_fr_base_link_footprint_X_fr_omniwheel_br();
        const Type_fr_base_link_footprint_X_fr_omniwheel_br& update(const state_t&);
    };
    
    struct Type_fr_xarmlink1_X_fr_base_link_footprint : public TransformHomogeneous<Type_fr_xarmlink1_X_fr_base_link_footprint>
    {
        Type_fr_xarmlink1_X_fr_base_link_footprint();
        const Type_fr_xarmlink1_X_fr_base_link_footprint& update(const state_t&);
    };
    
    struct Type_fr_base_link_footprint_X_fr_xarmlink1 : public TransformHomogeneous<Type_fr_base_link_footprint_X_fr_xarmlink1>
    {
        Type_fr_base_link_footprint_X_fr_xarmlink1();
        const Type_fr_base_link_footprint_X_fr_xarmlink1& update(const state_t&);
    };
    
    struct Type_fr_xarmlink2_X_fr_xarmlink1 : public TransformHomogeneous<Type_fr_xarmlink2_X_fr_xarmlink1>
    {
        Type_fr_xarmlink2_X_fr_xarmlink1();
        const Type_fr_xarmlink2_X_fr_xarmlink1& update(const state_t&);
    };
    
    struct Type_fr_xarmlink1_X_fr_xarmlink2 : public TransformHomogeneous<Type_fr_xarmlink1_X_fr_xarmlink2>
    {
        Type_fr_xarmlink1_X_fr_xarmlink2();
        const Type_fr_xarmlink1_X_fr_xarmlink2& update(const state_t&);
    };
    
    struct Type_fr_xarmlink3_X_fr_xarmlink2 : public TransformHomogeneous<Type_fr_xarmlink3_X_fr_xarmlink2>
    {
        Type_fr_xarmlink3_X_fr_xarmlink2();
        const Type_fr_xarmlink3_X_fr_xarmlink2& update(const state_t&);
    };
    
    struct Type_fr_xarmlink2_X_fr_xarmlink3 : public TransformHomogeneous<Type_fr_xarmlink2_X_fr_xarmlink3>
    {
        Type_fr_xarmlink2_X_fr_xarmlink3();
        const Type_fr_xarmlink2_X_fr_xarmlink3& update(const state_t&);
    };
    
    struct Type_fr_xarmlink4_X_fr_xarmlink3 : public TransformHomogeneous<Type_fr_xarmlink4_X_fr_xarmlink3>
    {
        Type_fr_xarmlink4_X_fr_xarmlink3();
        const Type_fr_xarmlink4_X_fr_xarmlink3& update(const state_t&);
    };
    
    struct Type_fr_xarmlink3_X_fr_xarmlink4 : public TransformHomogeneous<Type_fr_xarmlink3_X_fr_xarmlink4>
    {
        Type_fr_xarmlink3_X_fr_xarmlink4();
        const Type_fr_xarmlink3_X_fr_xarmlink4& update(const state_t&);
    };
    
    struct Type_fr_xarmlink5_X_fr_xarmlink4 : public TransformHomogeneous<Type_fr_xarmlink5_X_fr_xarmlink4>
    {
        Type_fr_xarmlink5_X_fr_xarmlink4();
        const Type_fr_xarmlink5_X_fr_xarmlink4& update(const state_t&);
    };
    
    struct Type_fr_xarmlink4_X_fr_xarmlink5 : public TransformHomogeneous<Type_fr_xarmlink4_X_fr_xarmlink5>
    {
        Type_fr_xarmlink4_X_fr_xarmlink5();
        const Type_fr_xarmlink4_X_fr_xarmlink5& update(const state_t&);
    };
    
    struct Type_fr_xarmlink6_X_fr_xarmlink5 : public TransformHomogeneous<Type_fr_xarmlink6_X_fr_xarmlink5>
    {
        Type_fr_xarmlink6_X_fr_xarmlink5();
        const Type_fr_xarmlink6_X_fr_xarmlink5& update(const state_t&);
    };
    
    struct Type_fr_xarmlink5_X_fr_xarmlink6 : public TransformHomogeneous<Type_fr_xarmlink5_X_fr_xarmlink6>
    {
        Type_fr_xarmlink5_X_fr_xarmlink6();
        const Type_fr_xarmlink5_X_fr_xarmlink6& update(const state_t&);
    };
    
public:
    HomogeneousTransforms();
    void updateParams(const Params_lengths&, const Params_angles&);

    Type_fr_omniwheel_fl_X_fr_base_link_footprint fr_omniwheel_fl_X_fr_base_link_footprint;
    Type_fr_base_link_footprint_X_fr_omniwheel_fl fr_base_link_footprint_X_fr_omniwheel_fl;
    Type_fr_omniwheel_fr_X_fr_base_link_footprint fr_omniwheel_fr_X_fr_base_link_footprint;
    Type_fr_base_link_footprint_X_fr_omniwheel_fr fr_base_link_footprint_X_fr_omniwheel_fr;
    Type_fr_omniwheel_bl_X_fr_base_link_footprint fr_omniwheel_bl_X_fr_base_link_footprint;
    Type_fr_base_link_footprint_X_fr_omniwheel_bl fr_base_link_footprint_X_fr_omniwheel_bl;
    Type_fr_omniwheel_br_X_fr_base_link_footprint fr_omniwheel_br_X_fr_base_link_footprint;
    Type_fr_base_link_footprint_X_fr_omniwheel_br fr_base_link_footprint_X_fr_omniwheel_br;
    Type_fr_xarmlink1_X_fr_base_link_footprint fr_xarmlink1_X_fr_base_link_footprint;
    Type_fr_base_link_footprint_X_fr_xarmlink1 fr_base_link_footprint_X_fr_xarmlink1;
    Type_fr_xarmlink2_X_fr_xarmlink1 fr_xarmlink2_X_fr_xarmlink1;
    Type_fr_xarmlink1_X_fr_xarmlink2 fr_xarmlink1_X_fr_xarmlink2;
    Type_fr_xarmlink3_X_fr_xarmlink2 fr_xarmlink3_X_fr_xarmlink2;
    Type_fr_xarmlink2_X_fr_xarmlink3 fr_xarmlink2_X_fr_xarmlink3;
    Type_fr_xarmlink4_X_fr_xarmlink3 fr_xarmlink4_X_fr_xarmlink3;
    Type_fr_xarmlink3_X_fr_xarmlink4 fr_xarmlink3_X_fr_xarmlink4;
    Type_fr_xarmlink5_X_fr_xarmlink4 fr_xarmlink5_X_fr_xarmlink4;
    Type_fr_xarmlink4_X_fr_xarmlink5 fr_xarmlink4_X_fr_xarmlink5;
    Type_fr_xarmlink6_X_fr_xarmlink5 fr_xarmlink6_X_fr_xarmlink5;
    Type_fr_xarmlink5_X_fr_xarmlink6 fr_xarmlink5_X_fr_xarmlink6;

protected:
    Parameters params;

}; //class 'HomogeneousTransforms'

}
}

#endif
