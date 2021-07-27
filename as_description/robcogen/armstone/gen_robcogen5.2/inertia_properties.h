#ifndef RCG_ARMSTONE_INERTIA_PROPERTIES_H_
#define RCG_ARMSTONE_INERTIA_PROPERTIES_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "model_constants.h"
#include "dynamics_parameters.h"

namespace armstone {
namespace rcg {

class InertiaProperties {
    public:
        InertiaProperties();
        ~InertiaProperties();
        const InertiaMatrix& getTensor_base_link_footprint() const;
        const InertiaMatrix& getTensor_omniwheel_fl() const;
        const InertiaMatrix& getTensor_omniwheel_fr() const;
        const InertiaMatrix& getTensor_omniwheel_bl() const;
        const InertiaMatrix& getTensor_omniwheel_br() const;
        const InertiaMatrix& getTensor_xarmlink1() const;
        const InertiaMatrix& getTensor_xarmlink2() const;
        const InertiaMatrix& getTensor_xarmlink3() const;
        const InertiaMatrix& getTensor_xarmlink4() const;
        const InertiaMatrix& getTensor_xarmlink5() const;
        const InertiaMatrix& getTensor_xarmlink6() const;
        Scalar getMass_base_link_footprint() const;
        Scalar getMass_omniwheel_fl() const;
        Scalar getMass_omniwheel_fr() const;
        Scalar getMass_omniwheel_bl() const;
        Scalar getMass_omniwheel_br() const;
        Scalar getMass_xarmlink1() const;
        Scalar getMass_xarmlink2() const;
        Scalar getMass_xarmlink3() const;
        Scalar getMass_xarmlink4() const;
        Scalar getMass_xarmlink5() const;
        Scalar getMass_xarmlink6() const;
        const Vector3& getCOM_base_link_footprint() const;
        const Vector3& getCOM_omniwheel_fl() const;
        const Vector3& getCOM_omniwheel_fr() const;
        const Vector3& getCOM_omniwheel_bl() const;
        const Vector3& getCOM_omniwheel_br() const;
        const Vector3& getCOM_xarmlink1() const;
        const Vector3& getCOM_xarmlink2() const;
        const Vector3& getCOM_xarmlink3() const;
        const Vector3& getCOM_xarmlink4() const;
        const Vector3& getCOM_xarmlink5() const;
        const Vector3& getCOM_xarmlink6() const;
        Scalar getTotalMass() const;


        /*!
         * Fresh values for the runtime parameters of the robot armstone,
         * causing the update of the inertia properties modeled by this
         * instance.
         */
        void updateParameters(const RuntimeInertiaParams&);

    private:
        RuntimeInertiaParams params;

        InertiaMatrix tensor_base_link_footprint;
        InertiaMatrix tensor_omniwheel_fl;
        InertiaMatrix tensor_omniwheel_fr;
        InertiaMatrix tensor_omniwheel_bl;
        InertiaMatrix tensor_omniwheel_br;
        InertiaMatrix tensor_xarmlink1;
        InertiaMatrix tensor_xarmlink2;
        InertiaMatrix tensor_xarmlink3;
        InertiaMatrix tensor_xarmlink4;
        InertiaMatrix tensor_xarmlink5;
        InertiaMatrix tensor_xarmlink6;
        Vector3 com_base_link_footprint;
        Vector3 com_omniwheel_fl;
        Vector3 com_omniwheel_fr;
        Vector3 com_omniwheel_bl;
        Vector3 com_omniwheel_br;
        Vector3 com_xarmlink1;
        Vector3 com_xarmlink2;
        Vector3 com_xarmlink3;
        Vector3 com_xarmlink4;
        Vector3 com_xarmlink5;
        Vector3 com_xarmlink6;
};


inline InertiaProperties::~InertiaProperties() {}

inline const InertiaMatrix& InertiaProperties::getTensor_base_link_footprint() const {
    return this->tensor_base_link_footprint;
}
inline const InertiaMatrix& InertiaProperties::getTensor_omniwheel_fl() const {
    return this->tensor_omniwheel_fl;
}
inline const InertiaMatrix& InertiaProperties::getTensor_omniwheel_fr() const {
    return this->tensor_omniwheel_fr;
}
inline const InertiaMatrix& InertiaProperties::getTensor_omniwheel_bl() const {
    return this->tensor_omniwheel_bl;
}
inline const InertiaMatrix& InertiaProperties::getTensor_omniwheel_br() const {
    return this->tensor_omniwheel_br;
}
inline const InertiaMatrix& InertiaProperties::getTensor_xarmlink1() const {
    return this->tensor_xarmlink1;
}
inline const InertiaMatrix& InertiaProperties::getTensor_xarmlink2() const {
    return this->tensor_xarmlink2;
}
inline const InertiaMatrix& InertiaProperties::getTensor_xarmlink3() const {
    return this->tensor_xarmlink3;
}
inline const InertiaMatrix& InertiaProperties::getTensor_xarmlink4() const {
    return this->tensor_xarmlink4;
}
inline const InertiaMatrix& InertiaProperties::getTensor_xarmlink5() const {
    return this->tensor_xarmlink5;
}
inline const InertiaMatrix& InertiaProperties::getTensor_xarmlink6() const {
    return this->tensor_xarmlink6;
}
inline Scalar InertiaProperties::getMass_base_link_footprint() const {
    return this->tensor_base_link_footprint.getMass();
}
inline Scalar InertiaProperties::getMass_omniwheel_fl() const {
    return this->tensor_omniwheel_fl.getMass();
}
inline Scalar InertiaProperties::getMass_omniwheel_fr() const {
    return this->tensor_omniwheel_fr.getMass();
}
inline Scalar InertiaProperties::getMass_omniwheel_bl() const {
    return this->tensor_omniwheel_bl.getMass();
}
inline Scalar InertiaProperties::getMass_omniwheel_br() const {
    return this->tensor_omniwheel_br.getMass();
}
inline Scalar InertiaProperties::getMass_xarmlink1() const {
    return this->tensor_xarmlink1.getMass();
}
inline Scalar InertiaProperties::getMass_xarmlink2() const {
    return this->tensor_xarmlink2.getMass();
}
inline Scalar InertiaProperties::getMass_xarmlink3() const {
    return this->tensor_xarmlink3.getMass();
}
inline Scalar InertiaProperties::getMass_xarmlink4() const {
    return this->tensor_xarmlink4.getMass();
}
inline Scalar InertiaProperties::getMass_xarmlink5() const {
    return this->tensor_xarmlink5.getMass();
}
inline Scalar InertiaProperties::getMass_xarmlink6() const {
    return this->tensor_xarmlink6.getMass();
}
inline const Vector3& InertiaProperties::getCOM_base_link_footprint() const {
    return this->com_base_link_footprint;
}
inline const Vector3& InertiaProperties::getCOM_omniwheel_fl() const {
    return this->com_omniwheel_fl;
}
inline const Vector3& InertiaProperties::getCOM_omniwheel_fr() const {
    return this->com_omniwheel_fr;
}
inline const Vector3& InertiaProperties::getCOM_omniwheel_bl() const {
    return this->com_omniwheel_bl;
}
inline const Vector3& InertiaProperties::getCOM_omniwheel_br() const {
    return this->com_omniwheel_br;
}
inline const Vector3& InertiaProperties::getCOM_xarmlink1() const {
    return this->com_xarmlink1;
}
inline const Vector3& InertiaProperties::getCOM_xarmlink2() const {
    return this->com_xarmlink2;
}
inline const Vector3& InertiaProperties::getCOM_xarmlink3() const {
    return this->com_xarmlink3;
}
inline const Vector3& InertiaProperties::getCOM_xarmlink4() const {
    return this->com_xarmlink4;
}
inline const Vector3& InertiaProperties::getCOM_xarmlink5() const {
    return this->com_xarmlink5;
}
inline const Vector3& InertiaProperties::getCOM_xarmlink6() const {
    return this->com_xarmlink6;
}

inline Scalar InertiaProperties::getTotalMass() const {
    return m_base_link_footprint + m_omniwheel_fl + m_omniwheel_fr + m_omniwheel_bl + m_omniwheel_br + m_xarmlink1 + m_xarmlink2 + m_xarmlink3 + m_xarmlink4 + m_xarmlink5 + m_xarmlink6;
}

}
}

#endif
