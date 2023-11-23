#include "stateArray.hpp"

namespace Sim
{
    StateArray defaultStateVector() {
        return StateArray::Zero();
    } // this is a function so that it returns a new instance each time it is called

    // automatically shifts derivative quantities to the left, and sets their cells to 0;
    StateArray defaultDeriv(StateArray state){
        return StateArray{state[Xv], 0, state[Yv], 0, state[Zv], 0, state[dPhi], 0, state[dTheta], 0, state[dPsi], 0};
    }

    const Eigen::Vector3d stateArrayVelocity(StateArray state){
        return {state[Xv], state[Yv], state[Zv]};
    }
    const Eigen::Vector3d stateArrayPosition(StateArray state){
        return {state[Xp], state[Yp], state[Zp]};
    }
    const Eigen::Vector3d stateArrayAngVelocity(StateArray state){
        return {state[dPhi], state[dTheta], state[dPsi]};
    }
    const Eigen::Vector3d stateArrayOrientation(StateArray state){
        return {state[Phi], state[Theta], state[Psi]};
    }
} // namespace Sim
