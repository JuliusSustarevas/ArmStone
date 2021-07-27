#include <iit/commons/SL/eigen_conversion.h>
#include <iit/commons/SL/rbd_conversion.h>
#include <iit/commons/SL/joint_status_conversion.h>

#include <iit/robots/asarm/declarations.h>
#include <iit/robots/asarm/transforms.h>
#include <iit/robots/asarm/jacobians.h>
#include <iit/robots/asarm/traits.h>
#include <iit/robots/asarm/inertia_properties.h>

#include <iit/rbd/utils.h>

#include "robcogen_globals.h"

#include <iostream>

// the system headers
#include "SL_system_headers.h"
#include "SL.h"
#include "SL_user.h"
#include "SL_common.h"
#include "mdefs.h"
#include "SL_kinematics.h"
#include "utility_macros.h"

using namespace iit;
using namespace iit::asArm;


static JointState q;
static dyn::InertiaProperties inertias;

typedef typename commons::SL::SLtoRobogen<Traits> SLtoRGen;

/*!
 * Original documentation:
 *
 *        computes the m*cog, rotation axis, and local coord.sys. orgin for
 *        every link. This information can be used to compute the COG and
 *        COG jacobians, assuming the mass and center of mass parameters are
 *        properly identified.
 *
 * \param[in]     state   : the state containing th, thd, thdd
 * \param[in]     basec   : the position state of the base
 * \param[in]     baseo   : the orientational state of the base
 * \param[in]     endeff  : the endeffector parameters
 * \param[out]    Xmcog   : array of mass*cog vectors
 * \param[out]    Xaxis   : array of rotation axes
 * \param[out]    Xorigin : array of coord.sys. origin vectors
 * \param[out]    Xlink   : array of link position
 * \param[out]    Ahmat   : homogeneous transformation matrices of each link
 *
 *
 * Marco's notes:
 * I am not really sure what is the difference between Xorigin and Xlink
 */
void linkInformation(
        SL_Jstate *state,
        SL_Cstate *basec,
        SL_quat *baseo,
        SL_endeff *eff,
        double **Xmcog, double **Xaxis, double **Xorigin, double **Xlink,
        double ***Ahmat, double ***Ahmatdof)
{
    // Convenient alias of the global variable
    iit::asArm::HomogeneousTransforms& HT = * iit::asArm::SL::homogeneousTransforms;
    // Copy the joint status
    SLtoRGen::pos(state, q);
    // Support vector
    Eigen::Matrix<double,4,1> tmp_vec;

    // The transform from xarmlink1 to world
    SL::world_X_base = SL::world_X_base * HT.fr_base_link_X_fr_xarmlink1(q);
    
    commons::SL::copy(Xaxis  [::XARMJOINT1], iit::rbd::Utils::zAxis(SL::world_X_base) );
    commons::SL::copy(Xorigin[::XARMJOINT1], iit::rbd::Utils::positionVector(SL::world_X_base) );
    
    commons::SL::copy(Xlink[::XARMLINK1], iit::rbd::Utils::positionVector(SL::world_X_base) );
    commons::SL::copy(Ahmat[::XARMLINK1], SL::world_X_base);
    
    tmp_vec.block<3,1>(0,0) = inertias.getCOM_xarmlink1() * inertias.getMass_xarmlink1();
    tmp_vec(3) = inertias.getMass_xarmlink1();
    commons::SL::copy(Xmcog[::XARMJOINT1], SL::world_X_base * tmp_vec);
    
    // The transform from xarmlink2 to world
    SL::world_X_base = SL::world_X_base * HT.fr_xarmlink1_X_fr_xarmlink2(q);
    
    commons::SL::copy(Xaxis  [::XARMJOINT2], iit::rbd::Utils::zAxis(SL::world_X_base) );
    commons::SL::copy(Xorigin[::XARMJOINT2], iit::rbd::Utils::positionVector(SL::world_X_base) );
    
    commons::SL::copy(Xlink[::XARMLINK2], iit::rbd::Utils::positionVector(SL::world_X_base) );
    commons::SL::copy(Ahmat[::XARMLINK2], SL::world_X_base);
    
    tmp_vec.block<3,1>(0,0) = inertias.getCOM_xarmlink2() * inertias.getMass_xarmlink2();
    tmp_vec(3) = inertias.getMass_xarmlink2();
    commons::SL::copy(Xmcog[::XARMJOINT2], SL::world_X_base * tmp_vec);
    
    // The transform from xarmlink3 to world
    SL::world_X_base = SL::world_X_base * HT.fr_xarmlink2_X_fr_xarmlink3(q);
    
    commons::SL::copy(Xaxis  [::XARMJOINT3], iit::rbd::Utils::zAxis(SL::world_X_base) );
    commons::SL::copy(Xorigin[::XARMJOINT3], iit::rbd::Utils::positionVector(SL::world_X_base) );
    
    commons::SL::copy(Xlink[::XARMLINK3], iit::rbd::Utils::positionVector(SL::world_X_base) );
    commons::SL::copy(Ahmat[::XARMLINK3], SL::world_X_base);
    
    tmp_vec.block<3,1>(0,0) = inertias.getCOM_xarmlink3() * inertias.getMass_xarmlink3();
    tmp_vec(3) = inertias.getMass_xarmlink3();
    commons::SL::copy(Xmcog[::XARMJOINT3], SL::world_X_base * tmp_vec);
    
    // The transform from xarmlink4 to world
    SL::world_X_base = SL::world_X_base * HT.fr_xarmlink3_X_fr_xarmlink4(q);
    
    commons::SL::copy(Xaxis  [::XARMJOINT4], iit::rbd::Utils::zAxis(SL::world_X_base) );
    commons::SL::copy(Xorigin[::XARMJOINT4], iit::rbd::Utils::positionVector(SL::world_X_base) );
    
    commons::SL::copy(Xlink[::XARMLINK4], iit::rbd::Utils::positionVector(SL::world_X_base) );
    commons::SL::copy(Ahmat[::XARMLINK4], SL::world_X_base);
    
    tmp_vec.block<3,1>(0,0) = inertias.getCOM_xarmlink4() * inertias.getMass_xarmlink4();
    tmp_vec(3) = inertias.getMass_xarmlink4();
    commons::SL::copy(Xmcog[::XARMJOINT4], SL::world_X_base * tmp_vec);
    
    // The transform from xarmlink5 to world
    SL::world_X_base = SL::world_X_base * HT.fr_xarmlink4_X_fr_xarmlink5(q);
    
    commons::SL::copy(Xaxis  [::XARMJOINT5], iit::rbd::Utils::zAxis(SL::world_X_base) );
    commons::SL::copy(Xorigin[::XARMJOINT5], iit::rbd::Utils::positionVector(SL::world_X_base) );
    
    commons::SL::copy(Xlink[::XARMLINK5], iit::rbd::Utils::positionVector(SL::world_X_base) );
    commons::SL::copy(Ahmat[::XARMLINK5], SL::world_X_base);
    
    tmp_vec.block<3,1>(0,0) = inertias.getCOM_xarmlink5() * inertias.getMass_xarmlink5();
    tmp_vec(3) = inertias.getMass_xarmlink5();
    commons::SL::copy(Xmcog[::XARMJOINT5], SL::world_X_base * tmp_vec);
    
    // The transform from xarmlink6 to world
    SL::world_X_base = SL::world_X_base * HT.fr_xarmlink5_X_fr_xarmlink6(q);
    
    commons::SL::copy(Xaxis  [::XARMJOINT6], iit::rbd::Utils::zAxis(SL::world_X_base) );
    commons::SL::copy(Xorigin[::XARMJOINT6], iit::rbd::Utils::positionVector(SL::world_X_base) );
    
    commons::SL::copy(Xlink[::XARMLINK6], iit::rbd::Utils::positionVector(SL::world_X_base) );
    commons::SL::copy(Ahmat[::XARMLINK6], SL::world_X_base);
    
    tmp_vec.block<3,1>(0,0) = inertias.getCOM_xarmlink6() * inertias.getMass_xarmlink6();
    tmp_vec(3) = inertias.getMass_xarmlink6();
    commons::SL::copy(Xmcog[::XARMJOINT6], SL::world_X_base * tmp_vec);
    
    //TODO  add the code for the endeffector links!!
}
