#include "simulation.hpp"
#include "RealAtmos.hpp"
#include <iostream>
#include <fstream>
#include <fmt/core.h>
#include <chrono>
#include <cassert>

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
        _atmos = RealAtmos::RealAtmos::GetInstance();
        // https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
        auto thisUp = Eigen::Vector3d{0,0,1};
        auto rocketUp = rocket->thisWayUp();
        // setting rotation matrix
        // the maths in the else block doesn't apply if the vectors are either the same or opposite
        if(thisUp == -rocketUp){
            // cannot use cross product with rocket vec as it returns [0,0,0]
            //_rotmat = Eigen::AngleAxisd(M_PI, thisUp.cross(rocketUp));
            // use cross product with this vector and some other arbitraty vector
            // need 2 candicates just in case the arbitrary addition is parallel
            if(thisUp.normalized() != Eigen::Vector3d{1,1,1}.normalized()){
                _rotmat = Eigen::AngleAxisd(M_PI, thisUp.cross(thisUp + Eigen::Vector3d{1,1,1}));
            } else {
                // something parallel with [1,1,1] cant be parallel with [1,2,3]
                _rotmat = Eigen::AngleAxisd(M_PI, thisUp.cross(thisUp + Eigen::Vector3d{1,2,3}));
            }
        } else if(thisUp == rocketUp){
            _rotmat = Eigen::Matrix3d::Identity();
        } else {
            auto ang = std::acos( thisUp.dot(rocketUp)/(thisUp.norm()*rocketUp.norm()) );
            auto v = rocketUp.cross(thisUp);
            auto s = v.norm()*std::sin(ang);
            auto c = rocketUp.dot(thisUp)*std::cos(ang);
            Eigen::Matrix3d vx = v.asSkewSymmetric();
            _rotmat = Eigen::Matrix3d::Identity() + vx + (vx*vx)*(1-c)/std::pow(s,2);
        }
        assert( _rotmat*rocketUp == thisUp );
    }

    std::shared_ptr<Sim> Sim::create(RocketInterface* rocket){
        auto obj = std::shared_ptr<Sim>(
            new Sim(rocket)
        );
        return obj;
    }

    std::filesystem::path Sim::outFile() const {
        const auto initPth = std::filesystem::current_path().append("..").append("results");
        unsigned int fnum = 0;
        std::filesystem::path nuPth;
        while(true){
            std::filesystem::path basePath = initPth;
            nuPth = basePath.append( std::to_string(fnum));
            if(!std::filesystem::exists(nuPth)) break;
            fnum++;
        }
        return nuPth;
    }

    StateArray Sim::solve( StateArray initialConditions ){
        _takeoff = false;
        _onRod = true;
        std::vector<StateArray> states = { initialConditions };
        std::vector<double> times = { 0 };
        std::vector<int> compTimes = { 0 };

        std::chrono::high_resolution_clock clock;
        StateArray lastState = initialConditions;
        StateArray state;
        const int maxSteps = 1e6;
        int counter = 0;
        double step = 0.01;
        double time = 0;
        bool term = false;
        // start timer
        fmt::print("starting sim\n");
        // loop will not terminate until a termination event is reached
        auto lastCalc = clock.now();
        while(!term){
            // doing calc
            //auto timeAndState = eulerIntegrate(time, step, lastState);
            //auto timeAndState = adaptiveRKIntegrate(time, step, lastState);
            auto timeAndState = RK4Integrate(time, step, lastState);
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
        for(long long unsigned int i = 0; i < states.size(); i++){
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
        for(long long unsigned int i = 0; i < compTimes.size(); i++) totalTime += compTimes[i];
        fmt::print("comp time {} s, final step {} s num steps {}\n", totalTime/1e6, step, counter);
        // just returning the final state
        // writing to file
        auto fname = outFile();
        const Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");
        std::ofstream resFile;
        resFile.open(fname, std::ios::out | std::ios::trunc);
        resFile << fmt::format("t, ctime, Xp, Xv, Yp, Yv, Zp, Zv, Phi, dPhi, Theta, dTheta, Psi, dPsi\n"); //dont need to include LAST
        fmt::print("writing results to file \"{}\"\n", fname.string());
        for(int i = 0; i < times.size(); i++){
            resFile << fmt::format("{}, ", times[i]);
            resFile << fmt::format("{}, ", compTimes[i]);
            resFile << states[i].transpose().format(CSVFormat);
            resFile << "\n";
        }

        resFile.close();
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
                for(long long unsigned int j = 0; j < ks.size(); j++){
                    yVal += RK_B(i,j)*ks[j];
                }
                // calculating this k value
                StateArray thisK = calculate(xVal, yVal);
                thisK *= newStep;
                ks.push_back(thisK);
            }
            // calculating the final output and output error
            StateArray err = StateArray::Zero();
            for(long long unsigned int i = 0; i < ks.size(); i++){
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

    std::tuple<double, StateArray> Sim::RK4Integrate( const double time, const double step, const StateArray state){
        StateArray k1 = calculate(time, state);
        StateArray k2 = calculate(time + step/2, state + k1*step/2);
        StateArray k3 = calculate(time + step/2, state + k2*step/2);
        StateArray k4 = calculate(time + step, state + k3*step);

        StateArray newState = state + step/6*(k1 + 2*k2 + 2*k3 + k4);
        return {time+step, newState};
    }

    StateArray Sim::calculate( double time, StateArray state ){
        StateArray res = defaultDeriv(state);
        // getting position vectors
        const Eigen::Vector3d position{ state[Xp], state[Yp], state[Zp] };
        const Eigen::Vector3d orientation{ state[Phi], state[Theta], state[Psi] };
        // getting velocity vectors
        const Eigen::Vector3d velocity{ state[Xv], state[Yv], state[Zv] };
        const Eigen::Vector3d angVelocity{ state[dPhi], state[dTheta], state[dPsi] };
        // initializing acceleration vectors
        Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
        Eigen::Vector3d angAcceleration = Eigen::Vector3d::Zero();

        // getting atmospheric properties
        auto g = _atmos->g(position.z());

        // getting rocket properties
        Eigen::Vector3d th = _rotmat*(_rocket->thrust(time));
        auto m = _rocket->mass(time);
        auto aRef = _rocket->referenceArea();
        auto lRef = _rocket->referenceLength();

        // adding thrust
        acceleration += th/m;
        // adding gravity
        auto gravVec = centerOfEarthVector(position)*g;
        acceleration += gravVec;

        // adding final acceleration vector
        res[Xv] = acceleration.x();
        res[Yv] = acceleration.y();
        res[Zv] = acceleration.z();
        // adding final angular acceleration vector
        res[Phi] = angAcceleration.x();
        res[Theta] = angAcceleration.y();
        res[Psi] = angAcceleration.z();

        return res;
    }

    double Sim::altitude(Eigen::Vector3d position) const {
        // TODO: modify this based on latitude and longitude
        return position.z();
    }

    Eigen::Vector3d Sim::originToCenterOfEarth() const {
        // TODO: modify this based on lat and long
        return Eigen::Vector3d{0,0,-RealAtmos::R_0};
    }

    Eigen::Vector3d Sim::centerOfEarthVector(Eigen::Vector3d position) const {
        Eigen::Vector3d combinedVec = originToCenterOfEarth() - position; // -position because its from the rocket to the origin
        Eigen::Vector3d normVec = combinedVec.normalized();
        return normVec;
    }

}