#ifndef ARMSTONE_JACOBIANS_H_
#define ARMSTONE_JACOBIANS_H_

#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include "kinematics_parameters.h"
#include "transforms.h" // to use the same 'Parameters' struct defined there
#include "model_constants.h"

namespace armstone {
namespace rcg {

template<int COLS, class M>
class JacobianT : public iit::rbd::JacobianBase<JointState, COLS, M>
{};

/**
 *
 */
class Jacobians
{
    public:
    public:
        Jacobians();
        void updateParameters(const Params_lengths& _lengths, const Params_angles& _angles);
    public:

    protected:
        Parameters params;

};


}
}

#endif
