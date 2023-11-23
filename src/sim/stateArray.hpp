#ifndef STATE_ARRAY_H_
#define STATE_ARRAY_H_

#include <Eigen/Dense>
#include <vector>
#include <utility>
#include <string>
#include <unordered_map>

namespace Sim{
    enum StateMappings {
        Xp, Xv, Yp, Yv, Zp, Zv, Phi, dPhi, Theta, dTheta, Psi, dPsi, LAST
    };

    using StateArray = Eigen::Array<double, StateMappings::LAST, 1>;

    StateArray defaultStateVector();

    StateArray defaultDeriv(StateArray state);

    const Eigen::Vector3d stateArrayVelocity(StateArray state);
    const Eigen::Vector3d stateArrayPosition(StateArray state);
    const Eigen::Vector3d stateArrayAngVelocity(StateArray state);
    const Eigen::Vector3d stateArrayOrientation(StateArray state);
    
    using StepData = std::unordered_map<std::string, double>;
}

#endif