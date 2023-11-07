#include "simulation.hpp"
#include <iostream>
#include <fmt/core.h>
#include <chrono>

namespace Sim{

    static std::string toString(const Eigen::MatrixXd& mat){
        std::stringstream ss;
        ss << mat;
        return ss.str();
    }

    StateArray defaultStateVector() {
        return StateArray::Zero();
    } // this is a function so that it returns a new instance each time it is called

    StateArray defaultDeriv(StateArray state){
        // automatically shifts derivative quantities to the left, and sets their cells to 0;
        return {state[Xv], 0, state[Yv], 0, state[Zv], 0, state[dPhi], 0, state[dTheta], 0, state[dPsi], 0};
    }

    Sim::Sim(RocketInterface* rocket){
        _rocket = rocket;
    }

    std::shared_ptr<Sim> Sim::create(RocketInterface* rocket){
        auto obj = std::shared_ptr<Sim>(
            new Sim(rocket)
        );
        return obj;
    }

    StateArray Sim::solve( StateArray initialConditions ){
        _takeoff = false;
        _onRod = true;
        std::vector<StateArray> states = { initialConditions };
        std::vector<double> times = { 0 };
        std::vector<int> compTimes = { };

        std::chrono::high_resolution_clock clock;
        StateArray lastState = initialConditions;
        StateArray state;
        const int maxSteps = 1e6;
        int counter = 0;
        double step = 0.002;
        double time = 0;
        bool term = false;
        // start timer
        fmt::print("starting sim\n");
        // loop will not terminate until a termination event is reached
        auto lastCalc = clock.now();
        while(!term){
            // doing calc
            //auto timeAndState = eulerIntegrate(time, step, lastState);
            auto timeAndState = adaptiveRKIntegrate(time, step, lastState);
            state = std::get<1>(timeAndState);
            auto thisStep = std::get<0>(timeAndState) - time;
            step = thisStep;
            state = (std::numeric_limits<double>::epsilon() < state.abs()).select(state, 0);
            // adjusting for takeoff
            if(!_takeoff){
                if(state[Zv] > 0){
                    _takeoff = true;
                } else {
                    state[Zv] = 0.0;
                }
            }
            //fmt::print("{}\n", toString(state.transpose()));

            // checking termination events
            // terminating on landing
            if( state[Zp] < 0 && lastState[Zp] >= 0 && _takeoff){
                // rocket has passed 0 negatively
                term = true;
            }
            // terminating on max steps
            if( counter >= maxSteps ){
                term = true;
            }
            // incrementing time
            counter++;
            time += thisStep;
            // storing data
            times.push_back(time);
            states.push_back(state);
            // reallocating array
            lastState = state;
            // store timer val
            auto thisCalc = clock.now();
            std::chrono::duration<int64_t, std::nano> calcTimeDur {thisCalc - lastCalc};
            int cTime = std::chrono::nanoseconds(calcTimeDur).count()/1e3;
            compTimes.push_back(cTime);
            lastCalc = thisCalc;
        }
        // getting apogee to print
        double apogee = 0;
        double apogeeTime = 0;
        for(int i = 0; i < states.size(); i++){
            StateArray st = states[i];
            double t = times[i];
            if( st[Zp] > apogee ){
                apogee = st[Zp];
                apogeeTime = t;
            }
        }
        fmt::print("{:.10f} m apogee at t = {:.10f}\n", apogee, apogeeTime);
        // summing comp times
        int totalTime = 0;
        for(int i = 0; i < compTimes.size(); i++) totalTime += compTimes[i];
        fmt::print("comp time {} s, final step {} s num steps {}\n", totalTime/1e6, step, counter);
        // just returning the final state
        return *(states.rbegin());
    }

    std::tuple<double, StateArray> Sim::eulerIntegrate( const double time, const double step, const StateArray state){
        StateArray k1 = calculate(time, state);
        StateArray newState = state + k1*step;
        return { time+step, newState };
    }

    std::tuple<double, StateArray> Sim::adaptiveRKIntegrate( const double time, const double step, const StateArray state, const double rtol, const double atol){
        bool errPass = false;
        
        double newStep = step;
        StateArray newState;

        while(!errPass){
            // initializing new state and k vector
            newState = state;
            std::vector<StateArray> ks = {};
            for(int i = 0; i < RK_A.size(); i++){
                // calculating x val
                double xVal = time + newStep*RK_A[i];
                // adding weighted sum of previous ks to y value
                StateArray yVal = state;
                for(int j = 0; j < ks.size(); j++){
                    yVal += RK_B(i,j)*ks[j];
                }
                // calculating this k value
                StateArray thisK = calculate(xVal, yVal);
                thisK *= newStep;
                ks.push_back(thisK);
            }
            // calculating the final output and output error
            StateArray err = StateArray::Zero();
            for(int i = 0; i < ks.size(); i++){
                newState += RK_CH[i]*ks[i];
                err += RK_CT[i]*ks[i];
            }
            double eps = ((newState.abs()*rtol) + atol).minCoeff();
            // use the smallest eps/err for the step value
            double errVal = Eigen::VectorXd(err).norm();

            newStep = 0.9 * newStep * std::pow(eps/errVal, 0.2);
            if( eps > errVal ){
                errPass = true;
            } else {
                errPass = false;
            }
        }
        //fmt::print("t = {:<8.3}, step {:<8.3} new state [{}]\n", time, newStep, toString(newState.transpose()));
        return {time+newStep, newState};
    }

    StateArray Sim::calculate( double time, StateArray state ){
        StateArray res = defaultDeriv(state);
        auto th = _rocket->thrust(time);
        auto m = _rocket->mass(time);
        //fmt::print("time {:<6.4f} thrust {:<6.4f} mass {:<6.4f}\n", time, th.x(), m);
        res[Zv] = -9.802 - th.x()/m;
        return res;
    }

}