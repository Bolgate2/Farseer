#ifndef STATE_ARRAY_H_
#define STATE_ARRAY_H_

#include <Eigen/Dense>

namespace Sim{
    enum StateMappings {
        Xp, Xv, Yp, Yv, Zp, Zv, Phi, dPhi, Theta, dTheta, Psi, dPsi, LAST
    };

    using StateArray = Eigen::Array<double, StateMappings::LAST, 1>;

    StateArray defaultStateVector();

    StateArray defaultDeriv(StateArray state);
}

#endif