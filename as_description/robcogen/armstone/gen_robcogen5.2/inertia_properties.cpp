#include "inertia_properties.h"

using namespace std;
using namespace iit::rbd;

armstone::rcg::InertiaProperties::InertiaProperties()
{
    com_base_link_footprint = Vector3(comx_base_link_footprint,0.0,comz_base_link_footprint);
    tensor_base_link_footprint.fill(
        m_base_link_footprint,
        com_base_link_footprint,
        Utils::buildInertiaTensor<Scalar>(ix_base_link_footprint,iy_base_link_footprint,iz_base_link_footprint,ixy_base_link_footprint,ixz_base_link_footprint,iyz_base_link_footprint) );

    com_omniwheel_fl = Vector3(0.0,0.0,0.0);
    tensor_omniwheel_fl.fill(
        m_omniwheel_fl,
        com_omniwheel_fl,
        Utils::buildInertiaTensor<Scalar>(ix_omniwheel_fl,iy_omniwheel_fl,iz_omniwheel_fl,0.0,ixz_omniwheel_fl,iyz_omniwheel_fl) );

    com_omniwheel_fr = Vector3(0.0,0.0,0.0);
    tensor_omniwheel_fr.fill(
        m_omniwheel_fr,
        com_omniwheel_fr,
        Utils::buildInertiaTensor<Scalar>(ix_omniwheel_fr,iy_omniwheel_fr,iz_omniwheel_fr,0.0,ixz_omniwheel_fr,iyz_omniwheel_fr) );

    com_omniwheel_bl = Vector3(0.0,0.0,0.0);
    tensor_omniwheel_bl.fill(
        m_omniwheel_bl,
        com_omniwheel_bl,
        Utils::buildInertiaTensor<Scalar>(ix_omniwheel_bl,iy_omniwheel_bl,iz_omniwheel_bl,0.0,ixz_omniwheel_bl,iyz_omniwheel_bl) );

    com_omniwheel_br = Vector3(0.0,0.0,0.0);
    tensor_omniwheel_br.fill(
        m_omniwheel_br,
        com_omniwheel_br,
        Utils::buildInertiaTensor<Scalar>(ix_omniwheel_br,iy_omniwheel_br,iz_omniwheel_br,0.0,ixz_omniwheel_br,iyz_omniwheel_br) );

    com_xarmlink1 = Vector3(comx_xarmlink1,comy_xarmlink1,comz_xarmlink1);
    tensor_xarmlink1.fill(
        m_xarmlink1,
        com_xarmlink1,
        Utils::buildInertiaTensor<Scalar>(ix_xarmlink1,iy_xarmlink1,iz_xarmlink1,ixy_xarmlink1,ixz_xarmlink1,iyz_xarmlink1) );

    com_xarmlink2 = Vector3(comx_xarmlink2,comy_xarmlink2,comz_xarmlink2);
    tensor_xarmlink2.fill(
        m_xarmlink2,
        com_xarmlink2,
        Utils::buildInertiaTensor<Scalar>(ix_xarmlink2,iy_xarmlink2,iz_xarmlink2,ixy_xarmlink2,ixz_xarmlink2,iyz_xarmlink2) );

    com_xarmlink3 = Vector3(comx_xarmlink3,comy_xarmlink3,comz_xarmlink3);
    tensor_xarmlink3.fill(
        m_xarmlink3,
        com_xarmlink3,
        Utils::buildInertiaTensor<Scalar>(ix_xarmlink3,iy_xarmlink3,iz_xarmlink3,ixy_xarmlink3,ixz_xarmlink3,iyz_xarmlink3) );

    com_xarmlink4 = Vector3(comx_xarmlink4,comy_xarmlink4,comz_xarmlink4);
    tensor_xarmlink4.fill(
        m_xarmlink4,
        com_xarmlink4,
        Utils::buildInertiaTensor<Scalar>(ix_xarmlink4,iy_xarmlink4,iz_xarmlink4,ixy_xarmlink4,ixz_xarmlink4,iyz_xarmlink4) );

    com_xarmlink5 = Vector3(comx_xarmlink5,comy_xarmlink5,comz_xarmlink5);
    tensor_xarmlink5.fill(
        m_xarmlink5,
        com_xarmlink5,
        Utils::buildInertiaTensor<Scalar>(ix_xarmlink5,iy_xarmlink5,iz_xarmlink5,ixy_xarmlink5,ixz_xarmlink5,iyz_xarmlink5) );

    com_xarmlink6 = Vector3(0.0,comy_xarmlink6,comz_xarmlink6);
    tensor_xarmlink6.fill(
        m_xarmlink6,
        com_xarmlink6,
        Utils::buildInertiaTensor<Scalar>(ix_xarmlink6,iy_xarmlink6,iz_xarmlink6,0.0,0.0,iyz_xarmlink6) );

}


void armstone::rcg::InertiaProperties::updateParameters(const RuntimeInertiaParams& fresh)
{
    this-> params = fresh;
}
