#ifndef ASARM_TRANSFORMS_H_
#define ASARM_TRANSFORMS_H_

#include <Eigen/Dense>
#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include <iit/rbd/traits/DoubleTrait.h>
#include "kinematics_parameters.h"

namespace iit {
namespace asArm {

template<typename SCALAR, class M>
class TransformMotion : public iit::rbd::SpatialTransformBase<tpl::JointState<SCALAR>, M> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<typename SCALAR, class M>
class TransformForce : public iit::rbd::SpatialTransformBase<tpl::JointState<SCALAR>, M> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<typename SCALAR, class M>
class TransformHomogeneous : public iit::rbd::HomogeneousTransformBase<tpl::JointState<SCALAR>, M> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

namespace tpl {


/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial motion vectors.
 */
template <typename TRAIT>
class MotionTransforms {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef typename TRAIT::Scalar Scalar;

    typedef JointState<Scalar> JState;
    class Dummy {};
    typedef typename TransformMotion<Scalar, Dummy>::MatrixType MatrixType;
public:
    class Type_fr_xarmlink1_X_fr_base_link : public TransformMotion<Scalar, Type_fr_xarmlink1_X_fr_base_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_xarmlink1_X_fr_base_link();
        const Type_fr_xarmlink1_X_fr_base_link& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_X_fr_xarmlink1 : public TransformMotion<Scalar, Type_fr_base_link_X_fr_xarmlink1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_X_fr_xarmlink1();
        const Type_fr_base_link_X_fr_xarmlink1& update(const JState&);
    protected:
    };
    
    class Type_fr_xarmlink2_X_fr_xarmlink1 : public TransformMotion<Scalar, Type_fr_xarmlink2_X_fr_xarmlink1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_xarmlink2_X_fr_xarmlink1();
        const Type_fr_xarmlink2_X_fr_xarmlink1& update(const JState&);
    protected:
    };
    
    class Type_fr_xarmlink1_X_fr_xarmlink2 : public TransformMotion<Scalar, Type_fr_xarmlink1_X_fr_xarmlink2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_xarmlink1_X_fr_xarmlink2();
        const Type_fr_xarmlink1_X_fr_xarmlink2& update(const JState&);
    protected:
    };
    
    class Type_fr_xarmlink3_X_fr_xarmlink2 : public TransformMotion<Scalar, Type_fr_xarmlink3_X_fr_xarmlink2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_xarmlink3_X_fr_xarmlink2();
        const Type_fr_xarmlink3_X_fr_xarmlink2& update(const JState&);
    protected:
    };
    
    class Type_fr_xarmlink2_X_fr_xarmlink3 : public TransformMotion<Scalar, Type_fr_xarmlink2_X_fr_xarmlink3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_xarmlink2_X_fr_xarmlink3();
        const Type_fr_xarmlink2_X_fr_xarmlink3& update(const JState&);
    protected:
    };
    
    class Type_fr_xarmlink4_X_fr_xarmlink3 : public TransformMotion<Scalar, Type_fr_xarmlink4_X_fr_xarmlink3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_xarmlink4_X_fr_xarmlink3();
        const Type_fr_xarmlink4_X_fr_xarmlink3& update(const JState&);
    protected:
    };
    
    class Type_fr_xarmlink3_X_fr_xarmlink4 : public TransformMotion<Scalar, Type_fr_xarmlink3_X_fr_xarmlink4>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_xarmlink3_X_fr_xarmlink4();
        const Type_fr_xarmlink3_X_fr_xarmlink4& update(const JState&);
    protected:
    };
    
    class Type_fr_xarmlink5_X_fr_xarmlink4 : public TransformMotion<Scalar, Type_fr_xarmlink5_X_fr_xarmlink4>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_xarmlink5_X_fr_xarmlink4();
        const Type_fr_xarmlink5_X_fr_xarmlink4& update(const JState&);
    protected:
    };
    
    class Type_fr_xarmlink4_X_fr_xarmlink5 : public TransformMotion<Scalar, Type_fr_xarmlink4_X_fr_xarmlink5>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_xarmlink4_X_fr_xarmlink5();
        const Type_fr_xarmlink4_X_fr_xarmlink5& update(const JState&);
    protected:
    };
    
    class Type_fr_xarmlink6_X_fr_xarmlink5 : public TransformMotion<Scalar, Type_fr_xarmlink6_X_fr_xarmlink5>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_xarmlink6_X_fr_xarmlink5();
        const Type_fr_xarmlink6_X_fr_xarmlink5& update(const JState&);
    protected:
    };
    
    class Type_fr_xarmlink5_X_fr_xarmlink6 : public TransformMotion<Scalar, Type_fr_xarmlink5_X_fr_xarmlink6>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_xarmlink5_X_fr_xarmlink6();
        const Type_fr_xarmlink5_X_fr_xarmlink6& update(const JState&);
    protected:
    };
    
public:
    MotionTransforms();
    void updateParameters();
    Type_fr_xarmlink1_X_fr_base_link fr_xarmlink1_X_fr_base_link;
    Type_fr_base_link_X_fr_xarmlink1 fr_base_link_X_fr_xarmlink1;
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

}; //class 'MotionTransforms'

/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial force vectors.
 */
template <typename TRAIT>
class ForceTransforms {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef typename TRAIT::Scalar Scalar;

    typedef JointState<Scalar> JState;
    class Dummy {};
    typedef typename TransformForce<Scalar, Dummy>::MatrixType MatrixType;
public:
    class Type_fr_xarmlink1_X_fr_base_link : public TransformForce<Scalar, Type_fr_xarmlink1_X_fr_base_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_xarmlink1_X_fr_base_link();
        const Type_fr_xarmlink1_X_fr_base_link& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_X_fr_xarmlink1 : public TransformForce<Scalar, Type_fr_base_link_X_fr_xarmlink1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_X_fr_xarmlink1();
        const Type_fr_base_link_X_fr_xarmlink1& update(const JState&);
    protected:
    };
    
    class Type_fr_xarmlink2_X_fr_xarmlink1 : public TransformForce<Scalar, Type_fr_xarmlink2_X_fr_xarmlink1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_xarmlink2_X_fr_xarmlink1();
        const Type_fr_xarmlink2_X_fr_xarmlink1& update(const JState&);
    protected:
    };
    
    class Type_fr_xarmlink1_X_fr_xarmlink2 : public TransformForce<Scalar, Type_fr_xarmlink1_X_fr_xarmlink2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_xarmlink1_X_fr_xarmlink2();
        const Type_fr_xarmlink1_X_fr_xarmlink2& update(const JState&);
    protected:
    };
    
    class Type_fr_xarmlink3_X_fr_xarmlink2 : public TransformForce<Scalar, Type_fr_xarmlink3_X_fr_xarmlink2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_xarmlink3_X_fr_xarmlink2();
        const Type_fr_xarmlink3_X_fr_xarmlink2& update(const JState&);
    protected:
    };
    
    class Type_fr_xarmlink2_X_fr_xarmlink3 : public TransformForce<Scalar, Type_fr_xarmlink2_X_fr_xarmlink3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_xarmlink2_X_fr_xarmlink3();
        const Type_fr_xarmlink2_X_fr_xarmlink3& update(const JState&);
    protected:
    };
    
    class Type_fr_xarmlink4_X_fr_xarmlink3 : public TransformForce<Scalar, Type_fr_xarmlink4_X_fr_xarmlink3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_xarmlink4_X_fr_xarmlink3();
        const Type_fr_xarmlink4_X_fr_xarmlink3& update(const JState&);
    protected:
    };
    
    class Type_fr_xarmlink3_X_fr_xarmlink4 : public TransformForce<Scalar, Type_fr_xarmlink3_X_fr_xarmlink4>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_xarmlink3_X_fr_xarmlink4();
        const Type_fr_xarmlink3_X_fr_xarmlink4& update(const JState&);
    protected:
    };
    
    class Type_fr_xarmlink5_X_fr_xarmlink4 : public TransformForce<Scalar, Type_fr_xarmlink5_X_fr_xarmlink4>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_xarmlink5_X_fr_xarmlink4();
        const Type_fr_xarmlink5_X_fr_xarmlink4& update(const JState&);
    protected:
    };
    
    class Type_fr_xarmlink4_X_fr_xarmlink5 : public TransformForce<Scalar, Type_fr_xarmlink4_X_fr_xarmlink5>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_xarmlink4_X_fr_xarmlink5();
        const Type_fr_xarmlink4_X_fr_xarmlink5& update(const JState&);
    protected:
    };
    
    class Type_fr_xarmlink6_X_fr_xarmlink5 : public TransformForce<Scalar, Type_fr_xarmlink6_X_fr_xarmlink5>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_xarmlink6_X_fr_xarmlink5();
        const Type_fr_xarmlink6_X_fr_xarmlink5& update(const JState&);
    protected:
    };
    
    class Type_fr_xarmlink5_X_fr_xarmlink6 : public TransformForce<Scalar, Type_fr_xarmlink5_X_fr_xarmlink6>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_xarmlink5_X_fr_xarmlink6();
        const Type_fr_xarmlink5_X_fr_xarmlink6& update(const JState&);
    protected:
    };
    
public:
    ForceTransforms();
    void updateParameters();
    Type_fr_xarmlink1_X_fr_base_link fr_xarmlink1_X_fr_base_link;
    Type_fr_base_link_X_fr_xarmlink1 fr_base_link_X_fr_xarmlink1;
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

}; //class 'ForceTransforms'

/**
 * The class with the homogeneous (4x4) coordinates transformation
 * matrices.
 */
template <typename TRAIT>
class HomogeneousTransforms {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef typename TRAIT::Scalar Scalar;

    typedef JointState<Scalar> JState;
    class Dummy {};
    typedef typename TransformHomogeneous<Scalar, Dummy>::MatrixType MatrixType;
public:
    class Type_fr_xarmlink1_X_fr_base_link : public TransformHomogeneous<Scalar, Type_fr_xarmlink1_X_fr_base_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_xarmlink1_X_fr_base_link();
        const Type_fr_xarmlink1_X_fr_base_link& update(const JState&);
    protected:
    };
    
    class Type_fr_base_link_X_fr_xarmlink1 : public TransformHomogeneous<Scalar, Type_fr_base_link_X_fr_xarmlink1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_base_link_X_fr_xarmlink1();
        const Type_fr_base_link_X_fr_xarmlink1& update(const JState&);
    protected:
    };
    
    class Type_fr_xarmlink2_X_fr_xarmlink1 : public TransformHomogeneous<Scalar, Type_fr_xarmlink2_X_fr_xarmlink1>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_xarmlink2_X_fr_xarmlink1();
        const Type_fr_xarmlink2_X_fr_xarmlink1& update(const JState&);
    protected:
    };
    
    class Type_fr_xarmlink1_X_fr_xarmlink2 : public TransformHomogeneous<Scalar, Type_fr_xarmlink1_X_fr_xarmlink2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_xarmlink1_X_fr_xarmlink2();
        const Type_fr_xarmlink1_X_fr_xarmlink2& update(const JState&);
    protected:
    };
    
    class Type_fr_xarmlink3_X_fr_xarmlink2 : public TransformHomogeneous<Scalar, Type_fr_xarmlink3_X_fr_xarmlink2>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_xarmlink3_X_fr_xarmlink2();
        const Type_fr_xarmlink3_X_fr_xarmlink2& update(const JState&);
    protected:
    };
    
    class Type_fr_xarmlink2_X_fr_xarmlink3 : public TransformHomogeneous<Scalar, Type_fr_xarmlink2_X_fr_xarmlink3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_xarmlink2_X_fr_xarmlink3();
        const Type_fr_xarmlink2_X_fr_xarmlink3& update(const JState&);
    protected:
    };
    
    class Type_fr_xarmlink4_X_fr_xarmlink3 : public TransformHomogeneous<Scalar, Type_fr_xarmlink4_X_fr_xarmlink3>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_xarmlink4_X_fr_xarmlink3();
        const Type_fr_xarmlink4_X_fr_xarmlink3& update(const JState&);
    protected:
    };
    
    class Type_fr_xarmlink3_X_fr_xarmlink4 : public TransformHomogeneous<Scalar, Type_fr_xarmlink3_X_fr_xarmlink4>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_xarmlink3_X_fr_xarmlink4();
        const Type_fr_xarmlink3_X_fr_xarmlink4& update(const JState&);
    protected:
    };
    
    class Type_fr_xarmlink5_X_fr_xarmlink4 : public TransformHomogeneous<Scalar, Type_fr_xarmlink5_X_fr_xarmlink4>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_xarmlink5_X_fr_xarmlink4();
        const Type_fr_xarmlink5_X_fr_xarmlink4& update(const JState&);
    protected:
    };
    
    class Type_fr_xarmlink4_X_fr_xarmlink5 : public TransformHomogeneous<Scalar, Type_fr_xarmlink4_X_fr_xarmlink5>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_xarmlink4_X_fr_xarmlink5();
        const Type_fr_xarmlink4_X_fr_xarmlink5& update(const JState&);
    protected:
    };
    
    class Type_fr_xarmlink6_X_fr_xarmlink5 : public TransformHomogeneous<Scalar, Type_fr_xarmlink6_X_fr_xarmlink5>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_xarmlink6_X_fr_xarmlink5();
        const Type_fr_xarmlink6_X_fr_xarmlink5& update(const JState&);
    protected:
    };
    
    class Type_fr_xarmlink5_X_fr_xarmlink6 : public TransformHomogeneous<Scalar, Type_fr_xarmlink5_X_fr_xarmlink6>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_xarmlink5_X_fr_xarmlink6();
        const Type_fr_xarmlink5_X_fr_xarmlink6& update(const JState&);
    protected:
    };
    
public:
    HomogeneousTransforms();
    void updateParameters();
    Type_fr_xarmlink1_X_fr_base_link fr_xarmlink1_X_fr_base_link;
    Type_fr_base_link_X_fr_xarmlink1 fr_base_link_X_fr_xarmlink1;
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

}; //class 'HomogeneousTransforms'

}

using MotionTransforms = tpl::MotionTransforms<rbd::DoubleTrait>;
using ForceTransforms = tpl::ForceTransforms<rbd::DoubleTrait>;
using HomogeneousTransforms = tpl::HomogeneousTransforms<rbd::DoubleTrait>;

}
}

#include "transforms.impl.h"

#endif
