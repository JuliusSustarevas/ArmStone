#ifndef IIT_ROBOT_ASARM_INERTIA_PROPERTIES_H_
#define IIT_ROBOT_ASARM_INERTIA_PROPERTIES_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>
#include <iit/rbd/traits/DoubleTrait.h>

#include "declarations.h"

namespace iit {
namespace asArm {
/**
 * This namespace encloses classes and functions related to the Dynamics
 * of the robot asArm.
 */
namespace dyn {

using InertiaMatrix = iit::rbd::InertiaMatrixDense;

namespace tpl {

template <typename TRAIT>
class InertiaProperties {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef typename TRAIT::Scalar Scalar;
        typedef iit::rbd::Core<Scalar> CoreS;
        typedef iit::rbd::tpl::InertiaMatrixDense<Scalar> IMatrix;
        typedef typename CoreS::Vector3 Vec3d;

        InertiaProperties();
        ~InertiaProperties();
        const IMatrix& getTensor_base_link() const;
        const IMatrix& getTensor_xarmlink1() const;
        const IMatrix& getTensor_xarmlink2() const;
        const IMatrix& getTensor_xarmlink3() const;
        const IMatrix& getTensor_xarmlink4() const;
        const IMatrix& getTensor_xarmlink5() const;
        const IMatrix& getTensor_xarmlink6() const;
        Scalar getMass_base_link() const;
        Scalar getMass_xarmlink1() const;
        Scalar getMass_xarmlink2() const;
        Scalar getMass_xarmlink3() const;
        Scalar getMass_xarmlink4() const;
        Scalar getMass_xarmlink5() const;
        Scalar getMass_xarmlink6() const;
        const Vec3d& getCOM_base_link() const;
        const Vec3d& getCOM_xarmlink1() const;
        const Vec3d& getCOM_xarmlink2() const;
        const Vec3d& getCOM_xarmlink3() const;
        const Vec3d& getCOM_xarmlink4() const;
        const Vec3d& getCOM_xarmlink5() const;
        const Vec3d& getCOM_xarmlink6() const;
        Scalar getTotalMass() const;

    private:

        IMatrix tensor_base_link;
        IMatrix tensor_xarmlink1;
        IMatrix tensor_xarmlink2;
        IMatrix tensor_xarmlink3;
        IMatrix tensor_xarmlink4;
        IMatrix tensor_xarmlink5;
        IMatrix tensor_xarmlink6;
        Vec3d com_base_link;
        Vec3d com_xarmlink1;
        Vec3d com_xarmlink2;
        Vec3d com_xarmlink3;
        Vec3d com_xarmlink4;
        Vec3d com_xarmlink5;
        Vec3d com_xarmlink6;
};

template <typename TRAIT>
inline InertiaProperties<TRAIT>::~InertiaProperties() {}

template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_base_link() const {
    return this->tensor_base_link;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_xarmlink1() const {
    return this->tensor_xarmlink1;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_xarmlink2() const {
    return this->tensor_xarmlink2;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_xarmlink3() const {
    return this->tensor_xarmlink3;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_xarmlink4() const {
    return this->tensor_xarmlink4;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_xarmlink5() const {
    return this->tensor_xarmlink5;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_xarmlink6() const {
    return this->tensor_xarmlink6;
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_base_link() const {
    return this->tensor_base_link.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_xarmlink1() const {
    return this->tensor_xarmlink1.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_xarmlink2() const {
    return this->tensor_xarmlink2.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_xarmlink3() const {
    return this->tensor_xarmlink3.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_xarmlink4() const {
    return this->tensor_xarmlink4.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_xarmlink5() const {
    return this->tensor_xarmlink5.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_xarmlink6() const {
    return this->tensor_xarmlink6.getMass();
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_base_link() const {
    return this->com_base_link;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_xarmlink1() const {
    return this->com_xarmlink1;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_xarmlink2() const {
    return this->com_xarmlink2;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_xarmlink3() const {
    return this->com_xarmlink3;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_xarmlink4() const {
    return this->com_xarmlink4;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_xarmlink5() const {
    return this->com_xarmlink5;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_xarmlink6() const {
    return this->com_xarmlink6;
}

template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getTotalMass() const {
    return 2.7 + 2.16 + 1.71 + 1.384 + 1.115 + 1.275 + 1.1596;
}

}

using InertiaProperties = tpl::InertiaProperties<rbd::DoubleTrait>;

}
}
}

#include "inertia_properties.impl.h"

#endif
