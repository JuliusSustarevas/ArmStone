#ifndef RCG_ARMSTONE_MODEL_CONSTANTS_H_
#define RCG_ARMSTONE_MODEL_CONSTANTS_H_

#include "rbd_types.h"

/**
 * \file
 * This file contains the definitions of all the non-zero numerical
 * constants of the robot model (i.e. the numbers appearing in the
 * .kindsl file).
 *
 * Varying these values (and recompiling) is a quick & dirty
 * way to vary the kinematics/dynamics model. For a much more
 * flexible way of exploring variations of the model, consider
 * using the parametrization feature of RobCoGen (see the wiki).
 *
 * Beware of inconsistencies when changing any of the inertia
 * properties.
 */

namespace armstone {
namespace rcg {

// Do not use 'constexpr' to allow for non-literal scalar types

const Scalar rx_xarmjoint2 = -1.5707999467849731;
const Scalar sin_rx_xarmjoint2 = ScalarTraits::sin(rx_xarmjoint2);
const Scalar cos_rx_xarmjoint2 = ScalarTraits::cos(rx_xarmjoint2);
const Scalar rx_xarmjoint4 = -1.5708004236221313;
const Scalar sin_rx_xarmjoint4 = ScalarTraits::sin(rx_xarmjoint4);
const Scalar cos_rx_xarmjoint4 = ScalarTraits::cos(rx_xarmjoint4);
const Scalar rx_xarmjoint5 = 1.5707995891571045;
const Scalar sin_rx_xarmjoint5 = ScalarTraits::sin(rx_xarmjoint5);
const Scalar cos_rx_xarmjoint5 = ScalarTraits::cos(rx_xarmjoint5);
const Scalar rx_xarmjoint6 = -1.5708004236221313;
const Scalar sin_rx_xarmjoint6 = ScalarTraits::sin(rx_xarmjoint6);
const Scalar cos_rx_xarmjoint6 = ScalarTraits::cos(rx_xarmjoint6);
const Scalar tx_motor_wheel_joint_fl = 0.3215000033378601;
const Scalar ty_motor_wheel_joint_fl = 0.2659499943256378;
const Scalar tz_motor_wheel_joint_fl = 0.07599999755620956;
const Scalar tx_motor_wheel_joint_fr = 0.3215000033378601;
const Scalar ty_motor_wheel_joint_fr = -0.2659499943256378;
const Scalar tz_motor_wheel_joint_fr = 0.07599999755620956;
const Scalar tx_motor_wheel_joint_bl = -0.3215000033378601;
const Scalar ty_motor_wheel_joint_bl = 0.2659499943256378;
const Scalar tz_motor_wheel_joint_bl = 0.07599999755620956;
const Scalar tx_motor_wheel_joint_br = -0.3215000033378601;
const Scalar ty_motor_wheel_joint_br = -0.2659499943256378;
const Scalar tz_motor_wheel_joint_br = 0.07599999755620956;
const Scalar tx_xarmjoint1 = 0.17499999701976776;
const Scalar tz_xarmjoint1 = 0.40049999952316284;
const Scalar tx_xarmjoint3 = 0.05350000038743019;
const Scalar ty_xarmjoint3 = -0.28450000286102295;
const Scalar tx_xarmjoint4 = 0.07750000059604645;
const Scalar ty_xarmjoint4 = 0.3425000011920929;
const Scalar tx_xarmjoint6 = 0.07599999755620956;
const Scalar ty_xarmjoint6 = 0.09700000286102295;
const Scalar tz_fr_base_link = 0.18649999797344208;
const Scalar tz_fr_base_link_base = 0.10050000250339508;
const Scalar tx_fr_base_link_footprint_COM = -0.05270084738731384;
const Scalar tz_fr_base_link_footprint_COM = 0.16858650743961334;
const Scalar tx_fr_battery48_link = -0.23999999463558197;
const Scalar tz_fr_battery48_link = 0.16300000250339508;
const Scalar tx_fr_dc_box_link = -0.26750001311302185;
const Scalar tz_fr_dc_box_link = 0.23849999904632568;
const Scalar tx_fr_motor_gearbox_bl = -0.3215000033378601;
const Scalar ty_fr_motor_gearbox_bl = 0.15594999492168427;
const Scalar tz_fr_motor_gearbox_bl = 0.07599999755620956;
const Scalar tx_fr_motor_gearbox_br = -0.3215000033378601;
const Scalar ty_fr_motor_gearbox_br = -0.15594999492168427;
const Scalar tz_fr_motor_gearbox_br = 0.07599999755620956;
const Scalar tx_fr_motor_gearbox_fl = 0.3215000033378601;
const Scalar ty_fr_motor_gearbox_fl = 0.15594999492168427;
const Scalar tz_fr_motor_gearbox_fl = 0.07599999755620956;
const Scalar tx_fr_motor_gearbox_fr = 0.3215000033378601;
const Scalar ty_fr_motor_gearbox_fr = -0.15594999492168427;
const Scalar tz_fr_motor_gearbox_fr = 0.07599999755620956;
const Scalar tx_fr_mount_plate_link = 0.22499999403953552;
const Scalar tz_fr_mount_plate_link = 0.12449999898672104;
const Scalar tx_fr_xarmlink_base = 0.17499999701976776;
const Scalar tz_fr_xarmlink_base = 0.13349999487400055;
const Scalar tx_fr_xarmlink1_COM = -0.0020000000949949026;
const Scalar ty_fr_xarmlink1_COM = 0.026920000091195107;
const Scalar tz_fr_xarmlink1_COM = -0.013319999910891056;
const Scalar tx_fr_xarmlink2_COM = 0.03531000018119812;
const Scalar ty_fr_xarmlink2_COM = -0.21398000419139862;
const Scalar tz_fr_xarmlink2_COM = 0.03386000171303749;
const Scalar tx_fr_xarmlink3_COM = 0.06780999898910522;
const Scalar ty_fr_xarmlink3_COM = 0.10749000310897827;
const Scalar tz_fr_xarmlink3_COM = 0.014569999650120735;
const Scalar tx_fr_xarmlink4_COM = -2.099999983329326E-4;
const Scalar ty_fr_xarmlink4_COM = 0.025779975578188896;
const Scalar tz_fr_xarmlink4_COM = -0.025379976257681847;
const Scalar tx_fr_xarmlink5_COM = 0.05428000167012215;
const Scalar ty_fr_xarmlink5_COM = 0.017810000106692314;
const Scalar tz_fr_xarmlink5_COM = 0.005429999902844429;
const Scalar tz_fr_extruder_ee = 0.1699998378753662;
const Scalar tz_fr_extruder_link = 0.0849999189376831;
const Scalar ty_fr_xarmlink6_COM = 6.0489768657134846E-5;
const Scalar tz_fr_xarmlink6_COM = 0.07606633752584457;
const Scalar m_base_link_footprint = 19.353559494018555;
const Scalar comx_base_link_footprint = -0.05270084738731384;
const Scalar comz_base_link_footprint = 0.16858650743961334;
const Scalar ix_base_link_footprint = 0.788155198097229;
const Scalar ixy_base_link_footprint = 3.5000000480067683E-6;
const Scalar ixz_base_link_footprint = -0.19653096795082092;
const Scalar iy_base_link_footprint = 1.8098585605621338;
const Scalar iyz_base_link_footprint = -1.6699999605407356E-6;
const Scalar iz_base_link_footprint = 1.3811004161834717;
const Scalar m_omniwheel_fl = 0.6000000238418579;
const Scalar ix_omniwheel_fl = 3.465599729679525E-4;
const Scalar ixz_omniwheel_fl = 8.673617379884035E-19;
const Scalar iy_omniwheel_fl = 3.465599729679525E-4;
const Scalar iyz_omniwheel_fl = -1.734723475976807E-18;
const Scalar iz_omniwheel_fl = 3.4656000207178295E-4;
const Scalar m_omniwheel_fr = 0.6000000238418579;
const Scalar ix_omniwheel_fr = 3.465599729679525E-4;
const Scalar ixz_omniwheel_fr = 8.673617379884035E-19;
const Scalar iy_omniwheel_fr = 3.465599729679525E-4;
const Scalar iyz_omniwheel_fr = 1.734723475976807E-18;
const Scalar iz_omniwheel_fr = 3.4656000207178295E-4;
const Scalar m_omniwheel_bl = 0.6000000238418579;
const Scalar ix_omniwheel_bl = 3.465599729679525E-4;
const Scalar ixz_omniwheel_bl = -8.673617379884035E-19;
const Scalar iy_omniwheel_bl = 3.465599729679525E-4;
const Scalar iyz_omniwheel_bl = -1.734723475976807E-18;
const Scalar iz_omniwheel_bl = 3.4656000207178295E-4;
const Scalar m_omniwheel_br = 0.6000000238418579;
const Scalar ix_omniwheel_br = 3.465599729679525E-4;
const Scalar ixz_omniwheel_br = -8.673617379884035E-19;
const Scalar iy_omniwheel_br = 3.465599729679525E-4;
const Scalar iyz_omniwheel_br = 1.734723475976807E-18;
const Scalar iz_omniwheel_br = 3.4656000207178295E-4;
const Scalar m_xarmlink1 = 2.1600000858306885;
const Scalar comx_xarmlink1 = -0.0020000000949949026;
const Scalar comy_xarmlink1 = 0.026920000091195107;
const Scalar comz_xarmlink1 = -0.013319999910891056;
const Scalar ix_xarmlink1 = 0.007342825643718243;
const Scalar ixy_xarmlink1 = -1.2724440603051335E-4;
const Scalar ixz_xarmlink1 = 5.590740329353139E-5;
const Scalar iy_xarmlink1 = 0.005289772525429726;
const Scalar iyz_xarmlink1 = -0.0015675206668674946;
const Scalar iz_xarmlink1 = 0.00468969251960516;
const Scalar m_xarmlink2 = 1.7100000381469727;
const Scalar comx_xarmlink2 = 0.03531000018119812;
const Scalar comy_xarmlink2 = -0.21398000419139862;
const Scalar comz_xarmlink2 = 0.03386000171303749;
const Scalar ix_xarmlink2 = 0.10512444376945496;
const Scalar ixy_xarmlink2 = -0.008613623678684235;
const Scalar ixz_xarmlink2 = 0.0027224402874708176;
const Scalar iy_xarmlink2 = 0.008948016911745071;
const Scalar iyz_xarmlink2 = -0.016962021589279175;
const Scalar iz_xarmlink2 = 0.10430681705474854;
const Scalar m_xarmlink3 = 1.3839999437332153;
const Scalar comx_xarmlink3 = 0.06780999898910522;
const Scalar comy_xarmlink3 = 0.10749000310897827;
const Scalar comz_xarmlink3 = 0.014569999650120735;
const Scalar ix_xarmlink3 = 0.021654076874256134;
const Scalar ixy_xarmlink3 = 0.008669332601130009;
const Scalar ixz_xarmlink3 = 0.002288320567458868;
const Scalar iy_xarmlink3 = 0.009900005534291267;
const Scalar iyz_xarmlink3 = 0.0038593029603362083;
const Scalar iz_xarmlink3 = 0.0273720882833004;
const Scalar m_xarmlink4 = 1.1150000095367432;
const Scalar comx_xarmlink4 = -2.099999983329326E-4;
const Scalar comy_xarmlink4 = 0.025779975578188896;
const Scalar comz_xarmlink4 = -0.025379976257681847;
const Scalar ix_xarmlink4 = 0.005851886700838804;
const Scalar ixy_xarmlink4 = -5.631638123304583E-5;
const Scalar ixz_xarmlink4 = -7.797278158250265E-6;
const Scalar iy_xarmlink4 = 0.004725968930870295;
const Scalar iyz_xarmlink4 = -0.0011829191353172064;
const Scalar iz_xarmlink4 = 0.0018442962318658829;
const Scalar m_xarmlink5 = 1.274999976158142;
const Scalar comx_xarmlink5 = 0.05428000167012215;
const Scalar comy_xarmlink5 = 0.017810000106692314;
const Scalar comz_xarmlink5 = 0.005429999902844429;
const Scalar ix_xarmlink5 = 0.001644776319153607;
const Scalar ixy_xarmlink5 = 7.401486509479582E-4;
const Scalar ixz_xarmlink5 = 7.67264049500227E-4;
const Scalar iy_xarmlink5 = 0.006081749685108662;
const Scalar iyz_xarmlink5 = 2.4680307251401246E-4;
const Scalar iz_xarmlink5 = 0.006847580894827843;
const Scalar m_xarmlink6 = 1.159600019454956;
const Scalar comy_xarmlink6 = 6.0489768657134846E-5;
const Scalar comz_xarmlink6 = 0.07606633752584457;
const Scalar ix_xarmlink6 = 0.01082650851458311;
const Scalar iy_xarmlink6 = 0.010829281993210316;
const Scalar iyz_xarmlink6 = -6.677805686194915E-7;
const Scalar iz_xarmlink6 = 0.0013922598445788026;

}
}
#endif
