#ifndef IIT_ROBOT_ASARM_DECLARATIONS_H_
#define IIT_ROBOT_ASARM_DECLARATIONS_H_

#include <iit/rbd/rbd.h>

namespace iit {
namespace asArm {

static const int JointSpaceDimension = 6;
static const int jointsCount = 6;
/** The total number of rigid bodies of this robot, including the base */
static const int linksCount  = 7;

namespace tpl {
template <typename SCALAR>
using Column6d = iit::rbd::PlainMatrix<SCALAR, 6, 1>;

template <typename SCALAR>
using JointState = Column6d<SCALAR>;
}

using Column6d = tpl::Column6d<double>;
typedef Column6d JointState;

enum JointIdentifiers {
    XARMJOINT1 = 0
    , XARMJOINT2
    , XARMJOINT3
    , XARMJOINT4
    , XARMJOINT5
    , XARMJOINT6
};

enum LinkIdentifiers {
    BASE_LINK = 0
    , XARMLINK1
    , XARMLINK2
    , XARMLINK3
    , XARMLINK4
    , XARMLINK5
    , XARMLINK6
};

static const JointIdentifiers orderedJointIDs[jointsCount] =
    {XARMJOINT1,XARMJOINT2,XARMJOINT3,XARMJOINT4,XARMJOINT5,XARMJOINT6};

static const LinkIdentifiers orderedLinkIDs[linksCount] =
    {BASE_LINK,XARMLINK1,XARMLINK2,XARMLINK3,XARMLINK4,XARMLINK5,XARMLINK6};

}
}
#endif
