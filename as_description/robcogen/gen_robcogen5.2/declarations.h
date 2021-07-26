#ifndef RCG_ARMSTONE_DECLARATIONS_H_
#define RCG_ARMSTONE_DECLARATIONS_H_

#include "rbd_types.h"

namespace armstone {
namespace rcg {

static constexpr int JointSpaceDimension = 10;
static constexpr int jointsCount = 10;
/** The total number of rigid bodies of this robot, including the base */
static constexpr int linksCount  = 11;

typedef Matrix<10, 1> Column10d;
typedef Column10d JointState;

enum JointIdentifiers {
    MOTOR_WHEEL_JOINT_FL = 0
    , MOTOR_WHEEL_JOINT_FR
    , MOTOR_WHEEL_JOINT_BL
    , MOTOR_WHEEL_JOINT_BR
    , XARMJOINT1
    , XARMJOINT2
    , XARMJOINT3
    , XARMJOINT4
    , XARMJOINT5
    , XARMJOINT6
};

enum LinkIdentifiers {
    BASE_LINK_FOOTPRINT = 0
    , OMNIWHEEL_FL
    , OMNIWHEEL_FR
    , OMNIWHEEL_BL
    , OMNIWHEEL_BR
    , XARMLINK1
    , XARMLINK2
    , XARMLINK3
    , XARMLINK4
    , XARMLINK5
    , XARMLINK6
};

static const JointIdentifiers orderedJointIDs[jointsCount] =
    {MOTOR_WHEEL_JOINT_FL,MOTOR_WHEEL_JOINT_FR,MOTOR_WHEEL_JOINT_BL,MOTOR_WHEEL_JOINT_BR,XARMJOINT1,XARMJOINT2,XARMJOINT3,XARMJOINT4,XARMJOINT5,XARMJOINT6};

static const LinkIdentifiers orderedLinkIDs[linksCount] =
    {BASE_LINK_FOOTPRINT,OMNIWHEEL_FL,OMNIWHEEL_FR,OMNIWHEEL_BL,OMNIWHEEL_BR,XARMLINK1,XARMLINK2,XARMLINK3,XARMLINK4,XARMLINK5,XARMLINK6};

}
}
#endif
