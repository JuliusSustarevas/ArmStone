#include "jacobians.h"

armstone::rcg::Jacobians::Jacobians()
{}

void armstone::rcg::Jacobians::updateParameters(const Params_lengths& _lengths, const Params_angles& _angles)
{
    params.lengths = _lengths;
    params.angles = _angles;
    params.trig.update();
}

