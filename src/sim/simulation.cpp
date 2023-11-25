#include "simulation.hpp"
#include "RealAtmos.hpp"
#include "maths.hpp"
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <fmt/core.h>
#include <chrono>
#include <cassert>
#include <cmath>

namespace Sim{

    static std::string toString(const Eigen::MatrixXd& mat){
        std::stringstream ss;
        ss << mat;
        return ss.str();
    }

    const std::tuple<StateArray, StepData> Sim::defK1arg = {defaultStateVector()*NAN_D, {}};

    Sim::Sim(RocketInterface* rocket, double timeStep, std::filesystem::path destination){
        saveFile = destination;
        _userStep = timeStep;
        _rocket = rocket;
        _rodLen = 0.1;
        _aRef = _rocket->referenceArea();
        _lRef = _rocket->referenceLength();
        _atmos = RealAtmos::RealAtmos::GetInstance();
        // https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
        auto thisUp = thisWayUp();
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

    std::shared_ptr<Sim> Sim::create(RocketInterface* rocket, double timeStep, std::filesystem::path destination){
        auto obj = std::shared_ptr<Sim>(
            new Sim(rocket, timeStep, destination)
        );
        return obj;
    }

    std::filesystem::path Sim::outFile() const {
        return saveFile;
    }

    StateArray Sim::solve( StateArray initialConditions ){
        _takeoff = false;
        _onRod = true;
        std::vector<StateArray> states = { initialConditions };
        std::vector<StateArray> diffs = {};
        std::vector<double> steps = {};
        std::vector<StepData> stepData = { std::get<1>(calculate(0, initialConditions)) };
        stepData[0]["t"] = 0; stepData[0]["ctime"] = 0;

        std::chrono::high_resolution_clock clock;
        std::srand( 69 );
        StateArray lastState = initialConditions;
        StateArray state = initialConditions;
        StateArray newState;

        setRodVec(Utils::eulerToRotmat(initialConditions[Phi], initialConditions[Theta], initialConditions[Psi])*thisWayUp());

        const int maxSteps = 1e5;
        int counter = 0;
        double step = userStep();
        double time = 0;
        bool term = false;
        // start timer
        fmt::print("starting sim\n");
        // loop will not terminate until a termination event is reached
        auto lastCalc = clock.now();
        while(!term){
            // doing calc
            std::tuple<double, StateArray, StepData> timeAndState;
            //timeAndState = eulerIntegrate(time, step, &state, &lastState);
            //timeAndState = ABRKIntegrate(time, step, &state, &lastState, &diffs, &stepData, &steps);
            timeAndState = ORKIntegrate(time, step, &state, &lastState);
            //timeAndState = adaptiveRKIntegrate(time, step, state);

            //timeAndState = RK4Integrate(time, step, state, lastState);

            newState = std::get<1>(timeAndState);
            auto thisStep = std::get<0>(timeAndState) - time;

            StateArray diff = (newState-state)/thisStep;
            step = thisStep;
            newState = (std::numeric_limits<double>::epsilon() < newState.abs()).select(newState, 0);

            // adjusting for takeoff
            if(!takeoff()){
                if(newState[Zv] > 0){
                    setTakeoff(true);
                }
            }
            // adjusting for rod
            if(onRod()){
                if( stateArrayPosition(newState).norm() > rodLen()){
                    setOnRod(false);
                    fmt::print("off rod at step {}\n", counter);
                }
            }

            // checking termination events
            // terminating on landing
            if( newState[Zp] < 0 && state[Zp] >= 0 && _takeoff){
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

            // reallocating arrays
            lastState = state;
            state = newState;
            // store timer val
            auto thisCalc = clock.now();
            std::chrono::duration<int64_t, std::nano> calcTimeDur {thisCalc - lastCalc};
            int cTime = std::chrono::nanoseconds(calcTimeDur).count()/1e3;
            lastCalc = thisCalc;

            // storing data
            auto stepDat = std::get<2>(timeAndState);
            stepDat["t"] = time;
            stepDat["ctime"] = cTime;

            steps.push_back(thisStep);
            states.push_back(newState);
            diffs.push_back(diff);
            stepData.push_back(stepDat);
        }
        
        // getting apogee to print
        double apogee = 0;
        double apogeeTime = 0;
        for(long long unsigned int i = 0; i < states.size(); i++){
            StateArray st = states[i];
            StepData dat = stepData[i];
            if( st[Zp] > apogee ){
                apogee = st[Zp];
                apogeeTime = dat["t"];
            }
        }
        fmt::print("{:.10f} m apogee at t = {:.10f}\n", apogee, apogeeTime);

        // summing comp times
        int totalTime = 0;
        for(auto elem = stepData.begin(); elem != stepData.end(); elem++){
            totalTime += (*elem)["ctime"];
        }
        fmt::print("comp time {} s, final step {} s num steps {}\n", totalTime/1e6, step, counter);

        // writing to file
        auto fname = outFile();
        const Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");
        std::ofstream resFile;
        resFile.open(fname, std::ios::out | std::ios::trunc);
        resFile << fmt::format("Xp, Xv, Yp, Yv, Zp, Zv, Phi, dPhi, Theta, dTheta, Psi, dPsi"); //dont need to include LAST
        // writing custom data headers
        std::vector<std::string> keys = {}; //getting keys in a specific order
        for(auto el = stepData[0].begin(); el != stepData[0].end(); el++){
            resFile << ", " << el->first;
            keys.push_back(el->first);
        }
        resFile << "\n";
        // writing results
        fmt::print("writing results to file \"{}\"\n", fname.string());
        auto defaultPrecision = resFile.precision();
        // writing 
        resFile << std::setprecision(std::numeric_limits<double>::digits10 + 1); // 17
        for(int i = 0; i < states.size(); i++){
            resFile << states[i].transpose().format(CSVFormat);
            for(auto k = keys.begin(); k != keys.end(); k++){
                resFile << ", " << stepData[i].at(*k);
            }
            resFile << "\n";
        }

        resFile << std::setprecision(defaultPrecision);

        resFile.close();

        return *(states.rbegin());
    }
    
    std::tuple<double, StateArray, StepData> Sim::eulerIntegrate( const double time, const double step, const StateArray* state, const StateArray* lastState){
        std::tuple<StateArray, StepData> k1Dat = calculate(time, *state);
        StateArray k1 = std::get<0>(k1Dat);
        auto newStep = selectTimeStep(state, &k1, step);
        StateArray newState = (*state) + k1 * newStep;
        double newTime = time + newStep;
        return { newTime, newState, std::get<1>(k1Dat)};
    }
   
    std::tuple<double, StateArray, StepData> Sim::adaptiveRKIntegrate( const double time, const double step, const StateArray state, const double rtol, const double atol){
        bool errPass = false;
        
        double newStep = step;
        double usedStep = step;
        StateArray newState;
        StepData stateData;

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
                auto thisKdat = calculate(xVal, yVal);
                auto thisK = std::get<0>(thisKdat);
                stateData = std::get<1>(thisKdat);
                thisK *= newStep;
                ks.push_back(thisK);
            }
            usedStep = newStep;
            // calculating the final output and output error
            StateArray err = StateArray::Zero();
            //std::cout << ks.size() << "\n";
            for(int i = 0; i < ks.size(); i++){
                newState += RK_CH[i]*ks[i];
                err += RK_CT[i]*ks[i];
            }
            double eps = ((newState.abs()*rtol) + atol).sum();

            //std::cout << "new step " << newStep << "\neps " << eps << "\nerr " << err << "\n";
            newStep = (0.9 * newStep * std::pow(eps/err.sum(),0.2));
            if( err.sum() <= eps ){
                errPass = true;
            } else {
                errPass = false;
            }
        }
        //fmt::print("t = {:<8.3}, step {:<8.3} new state [{}]\n", time, newStep, toString(newState.transpose()));
        return {time+usedStep, newState, stateData};
    }
    

    std::tuple<double, StateArray, StepData> Sim::ABRKIntegrate(
        const double time, const double step, const StateArray* state, const StateArray* lastState,
        const std::vector<StateArray>* diffs, const std::vector<StepData>* stepData, const std::vector<double>* steps
        ){
        std::tuple<StateArray, StepData> k1Dat = calculate(time, *state);
        StateArray k1 = std::get<0>(k1Dat);
        auto newStep = selectTimeStep(state, &k1, step);

        if(diffs->size() < 8) // not enough steps to do AB4 integration
        {
            return RK4Integrate(time, newStep, state, &k1Dat);
        }
        // check if the last 3 steps are the same as the selected one
        bool stepIsSame = true;
        auto stepRevIter = steps->crbegin();

        for(int i = 0; i < 8; i++){
            if( std::abs((*stepRevIter) - newStep) > std::numeric_limits<double>::epsilon() ) // checking if steps are equal accounting for floating point badness
            {
                stepIsSame = false;
                break;
            }
            stepRevIter++;
        }

        std::tuple<double, StateArray, StepData> newdata;
        if(stepIsSame){
            newdata = AB44Integrate(time, newStep, state, diffs, stepData, &k1Dat);
        } else {
            newdata = RK4Integrate(time, newStep, state, &k1Dat);
        }
        return newdata;
    }

    std::tuple<double, StateArray, StepData> Sim::AB4Integrate(
        const double time, const double step, const StateArray* state, const std::vector<StateArray>* diffs,
        const std::vector<StepData>* stepData, const std::tuple<StateArray, StepData>* inK1Dat
        ){
        auto diffSiz = diffs->size();
        
        if(diffSiz < 5) // not enough steps to do integration
        {
            if(std::get<0>(*inK1Dat).hasNaN()){
                return RK4Integrate(time, step, state);
            } else {
                return RK4Integrate(time, step, state, inK1Dat);
            }
            //return eulerIntegrate(time, step, state, lastState);
        }

        std::tuple<StateArray, StepData> k1Dat;
        if(std::get<0>(*inK1Dat).hasNaN()){
            k1Dat = calculate(time, *state);
        } else {
            k1Dat = *inK1Dat;
        }

        StateArray k1 = std::get<0>(k1Dat);
        StateArray k2 = diffs->at(diffSiz-1);
        StateArray k3 = diffs->at(diffSiz-2);
        StateArray k4 = diffs->at(diffSiz-3);
        StateArray k5 = diffs->at(diffSiz-4);

        //StateArray newState = (*state) + step*(55.0*k1 - 59.0*k2 + 37.0*k3 - 9.0*k4)/24.0;
        StateArray newState = (*state) + step*((1901.0*k1 - 2774.0*k2 + 2616.0*k3 - 1274.0*k4 + 251.0*k5)/720);

        return { time+step, newState, std::get<1>(k1Dat)};
    }

    std::tuple<double, StateArray, StepData> Sim::AB22Integrate(
        const double time, const double step, const StateArray* state, const std::vector<StateArray>* diffs,
        const std::vector<StepData>* stepData, const std::tuple<StateArray, StepData>* inK1Dat
        ){
        auto diffSiz = diffs->size();
        
        if(diffSiz < 4) // not enough steps to do integration
        {
            if(std::get<0>(*inK1Dat).hasNaN()){
                return RK4Integrate(time, step, state);
            } else {
                return RK4Integrate(time, step, state, inK1Dat);
            }
            //return eulerIntegrate(time, step, state, lastState);
        }

        std::tuple<StateArray, StepData> k1Dat;
        if(std::get<0>(*inK1Dat).hasNaN()){
            k1Dat = calculate(time, *state);
        } else {
            k1Dat = *inK1Dat;
        }


        // getting derivatives
        StateArray zn3o = std::get<0>(k1Dat); // this will end up replaced, and is used to get the derivative
        StateArray zn2o = diffs->at(diffSiz-1);
        StateArray zn1 = diffs->at(diffSiz-2);
        StateArray zn = diffs->at(diffSiz-3);
        // getting second derivatives
        StateArray znp = (zn1-zn)/step;
        StateArray zn1p = (zn2o-zn1)/step;
        StateArray zn2p = (zn3o-zn2o)/step;

        // forming new derivatives
        StateArray zn2 = zn1 + step/2*(3*zn1p - znp);
        StateArray zn3 = zn2 + step/2*(3*zn2p - zn1p);

        // getting new difference
        StateArray diff = step/2*(3*zn3 - zn2);

        StateArray newState = (*state) + diff;// + dvdt*step;

        return { time+step, newState, std::get<1>(k1Dat)};
    }

    std::tuple<double, StateArray, StepData> Sim::AB44Integrate(
        const double time, const double step, const StateArray* state, const std::vector<StateArray>* diffs,
        const std::vector<StepData>* stepData, const std::tuple<StateArray, StepData>* inK1Dat
        ){
        auto diffSiz = diffs->size();
        
        if(diffSiz < 8) // not enough steps to do integration
        {
            if(std::get<0>(*inK1Dat).hasNaN()){
                return RK4Integrate(time, step, state);
            } else {
                return RK4Integrate(time, step, state, inK1Dat);
            }
            //return eulerIntegrate(time, step, state, lastState);
        }

        std::tuple<StateArray, StepData> k1Dat;
        if(std::get<0>(*inK1Dat).hasNaN()){
            k1Dat = calculate(time, *state);
        } else {
            k1Dat = *inK1Dat;
        }


        // getting derivatives
        StateArray zn7o = std::get<0>(k1Dat); // this will end up replaced, and is used to get the derivative
        StateArray zn6o = diffs->at(diffSiz-1);
        StateArray zn5o = diffs->at(diffSiz-2);
        StateArray zn4o = diffs->at(diffSiz-3);
        StateArray zn3 = diffs->at(diffSiz-4);
        StateArray zn2 = diffs->at(diffSiz-5);
        StateArray zn1 = diffs->at(diffSiz-6);
        StateArray zn = diffs->at(diffSiz-7);
        // getting second derivatives
        StateArray znp = (zn1-zn)/(step);
        StateArray zn1p = (zn2-zn1)/(step);
        StateArray zn2p = (zn3-zn2)/(step);
        StateArray zn3p = (zn4o-zn3)/(step);
        StateArray zn4p = (zn5o-zn4o)/(step);
        StateArray zn5p = (zn6o-zn5o)/(step);
        StateArray zn6p = (zn7o-zn6o)/(step);
        // forming new derivatives

        StateArray zn4 = zn3 + step/24*(55*zn3p - 59*zn2p + 37*zn1p - 9*znp);
        StateArray zn5 = zn4 + step/24*(55*zn4p - 59*zn3p + 37*zn2p - 9*zn1p);
        StateArray zn6 = zn5 + step/24*(55*zn5p - 59*zn4p + 37*zn3p - 9*zn2p);
        StateArray zn7 = zn6 + step/24*(55*zn6p - 59*zn5p + 37*zn4p - 9*zn3p);
        // getting new difference
        StateArray diff = step/24*(55*zn7 - 59*zn6 + 37*zn5 - 9*zn4);

        StateArray newState = (*state) + diff;// + dvdt*step;

        StepData avgDat = {};
        
        auto zn7dat = std::get<1>(k1Dat);
        /*
        auto zn6dat = stepData->at(diffSiz-1);
        auto zn5dat = stepData->at(diffSiz-2);
        auto zn4dat = stepData->at(diffSiz-3);
        for(auto i = zn7dat.begin(); i != zn7dat.end(); i++){
            auto k = i->first;
            double avg = 55.0/24*zn7dat[k] - 59.0/24*zn6dat[k] + 37.0/24*zn5dat[k] - 9.0/24 * zn4dat[k];
            avgDat[k] = avg;
        }
        */

        auto correctorData = calculate(time+step, newState);

        StateArray corrctorState = std::get<0>(correctorData);

        StateArray finalState = (*state) + step/24*(9*corrctorState + 19*zn7o - 5*zn6o + zn5o);

        return { time+step, finalState, zn7dat};
    }

    std::tuple<double, StateArray, StepData> Sim::ORKIntegrate( const double time, const double step, const StateArray* state, const StateArray* lastState){
        std::tuple<StateArray, StepData> k1Dat = calculate(time, *state);
        StateArray k1 = std::get<0>(k1Dat);
        // determine step size
        auto newStep = selectTimeStep(state, &k1, step);
        return RK4Integrate( time, newStep, state, &k1Dat);
    }
    
    std::tuple<double, StateArray, StepData> Sim::RK4Integrate( const double time, const double step, const StateArray* state, const std::tuple<StateArray, StepData>* inK1Dat){
        std::tuple<StateArray, StepData> k1Dat;
        if( std::get<0>(*inK1Dat).hasNaN() ){
            k1Dat = calculate(time, *state);
        } else {
            k1Dat = *inK1Dat;
        }

        StateArray k1 = std::get<0>(k1Dat);

        std::tuple<StateArray, StepData> k2Dat = calculate(time + step/2, (*state) + k1*step/2);
        StateArray k2 = std::get<0>(k2Dat);
        
        std::tuple<StateArray, StepData> k3Dat = calculate(time + step/2, (*state) + k2*step/2);
        StateArray k3 = std::get<0>(k3Dat);

        std::tuple<StateArray, StepData> k4Dat = calculate(time + step, (*state) + k3*step);
        StateArray k4 = std::get<0>(k4Dat);

        StateArray newState = (*state) + step/6*(k1 + 2*k2 + 2*k3 + k4);

        StepData stepDatAvg = {}; // weighted avg of all step data
        StepData k1St = std::get<1>(k1Dat);
        StepData k2St = std::get<1>(k2Dat);
        StepData k3St = std::get<1>(k3Dat);
        StepData k4St = std::get<1>(k4Dat);

        for(auto el = k1St.begin(); el != k1St.end(); el++){
            auto key = el->first;
            auto avg = (k1St[key] + 2*k2St[key] + 2*k3St[key] + k4St[key])/6;
            stepDatAvg[key] = avg;
        }

        std::tuple<double, StateArray, StepData> res = {time+step, newState, stepDatAvg};
        return res;
    }
    

    double Sim::selectTimeStep(const StateArray* state, const StateArray* k1, const double currStep) const{

        static const double maxAngleStep = 3 * M_PI / 180; // 3 degrees
        static const double maxRollStepAng = 2 * 28.32 * M_PI;
        static const double maxRollRateChange = 2 * M_PI / 180;
        static const double maxPitchStepChange = 4 * M_PI / 180; // 4 degrees
        static const double minTimeStep = 0.001;

        Eigen::Array<double, 8, 1> stepCandidates = Eigen::Array<double, 8, 1>::Ones() * std::numeric_limits<double>::max();
        stepCandidates[0] = std::max(userStep(), minTimeStep); // the current time step
        //stepCandidates[1] = ; // the maximum allowed time step
        stepCandidates[2] = std::abs(maxAngleStep/ std::sqrt(std::pow((*k1)[Theta],2) + std::pow((*k1)[Psi],2) )); // the maximum pitch rate per second
        // the max roll rate
        // the max roll rate change
        stepCandidates[5] = std::abs(maxPitchStepChange/ std::sqrt(std::pow((*k1)[dTheta],2) + std::pow((*k1)[dPsi],2)) );
        if(onRod()){
            stepCandidates[0] /= 5;
            stepCandidates[6] = (rodLen()/stateArrayPosition(*k1).norm())/10;
        }
        stepCandidates[7] = 1.5*currStep;
        assert(!stepCandidates.hasNaN());
        auto chosenStep = stepCandidates.minCoeff();
        //fmt::print("chosen step {}\n", chosenStep);

        return chosenStep;
    }


    std::tuple<StateArray, StepData> Sim::calculate( const double time, const StateArray state ){
        //fmt::print("TIME {}, IN [{}]\n", time, toString(state.transpose()));
        StateArray res = defaultDeriv(state);
        //fmt::print("TIME {}, INITRES [{}]\n", time, toString(res.transpose()));
        // getting position vectors
        const Eigen::Vector3d position = stateArrayPosition(state);
        const Eigen::Vector3d orientation = stateArrayOrientation(state); //yaw, pitch, roll
        // getting velocity vectors
        const Eigen::Vector3d velocity = stateArrayVelocity(state);
        const Eigen::Vector3d angVelocity = stateArrayAngVelocity(state); //yaw, pitch, roll
        // initializing acceleration vectors
        Eigen::Vector3d forces = Eigen::Vector3d::Zero();
        Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();

        Eigen::Vector3d moments = Eigen::Vector3d::Zero();
        Eigen::Vector3d angAcceleration = Eigen::Vector3d::Zero();

        /*
        --------------------------
        --- THRUST AND GRAVITY ---
        --------------------------
        */
        auto m = _rocket->mass(time);
        Eigen::Matrix3d rocketRotationMat = Utils::eulerToRotmat(orientation.x(), orientation.y(), orientation.z());
        Eigen::Vector3d rocketOrientationVec = rocketRotationMat*thisWayUp(); // the rockets current "up" vector in global coords
        const Eigen::Matrix3d inertia = (_rotmat*_rocket->inertia(time)*(_rotmat.transpose()));
        const double Ixx = inertia(0,0);
        const double Iyy = inertia(1,1);
        const double Izz = inertia(2,2);


        // adding thrust
        Eigen::Vector3d th = rocketRotationMat*_rotmat*(_rocket->thrust(time));
        forces += th;

        // adding gravity
        // getting atmospheric properties
        const double alt = altitude(position);
        const auto g = _atmos->g(alt);
        Eigen::Vector3d gravVec = centerOfEarthVector(position)*g;
        acceleration += gravVec;


        /*
        ---------------------------
        ------- AERO FORCES -------
        ---------------------------
        */
        /*
        ---------------------------
        ------ NORMAL FORCES ------
        ---------------------------
        */
        const auto atmDens = _atmos->density(alt);
        const auto atmTemp = _atmos->temperature(alt);
        const auto cSound = _atmos->sound(alt);
        const auto pres = _atmos->pressure(alt);
        //fmt::print("TIME: {}, STATE [{}]\n", time, toString(state.transpose()));

        //fmt::print("ATM CONDS: pos = [{}] alt = {}, g = {}, cSound = {}, atmDens = {}, pres = {}\n", toString(position.transpose()), alt, g, atmDens, cSound, pres);

        // getting wind velocity
        const Eigen::Vector3d windVel = wind(position);
        const Eigen::Vector3d relativeVelocity = velocity - windVel; // velocity of the rocket relative to the wind, this is opposite freestream velocity (-v_0)
        const double relativeSpeed = relativeVelocity.norm();

        // getting atmospheric probs dependent on velocity
        const double mach = velocity.norm()/cSound;
        if(std::isnan(mach)){
            fmt::print("TIME: {}, STATE AT FAILURE [{}]\n", time, toString(state.transpose()));
            fmt::print("MACH IS NAN vel.norm = [{}], csound = {}\n", velocity.norm(), cSound);
            assert(!std::isnan(mach));
        }
        const double dynamicPressure = atmDens*std::pow(relativeSpeed,2)/2;

        // getting rocket properties
        double angleOfAttack;
        if(onRod()){
            angleOfAttack = 0;
        }
        else if(relativeSpeed == 0){
            angleOfAttack = 0;
        } else {
            Eigen::Vector3d normRelVelVec = relativeVelocity.normalized();
            // floating point errs occur here without clamping
            double cosAoA = std::clamp( normRelVelVec.dot(rocketOrientationVec)/( normRelVelVec.norm()*rocketOrientationVec.norm() ), -1.0, 1.0);
            angleOfAttack = std::acos(cosAoA);
        }
        if(std::isnan(angleOfAttack)){
            fmt::print("AoA IS NAN relvel = [{}], ori = [{}], prod= {}\n", toString(relativeVelocity.normalized().transpose()), toString(rocketOrientationVec.transpose()),
            relativeVelocity.dot(rocketOrientationVec)/( relativeVelocity.norm()*rocketOrientationVec.norm()));
            assert(!std::isnan(angleOfAttack));
        }

        auto cn = _rocket->c_n(mach, angleOfAttack);
        if(std::isnan(cn)){
            fmt::print("TIME {}, STATE AT FAILURE [{}]\n", time, toString(state.transpose()));
            fmt::print("CN IS NAN M={:.4f} AoA={:.4f}\n", mach, angleOfAttack);
            assert(!std::isnan(cn));
        }
        //fmt::print("time {:.4f}, cn: {} aoa: {}\n", time, cn, angleOfAttack/M_PI*180);

        // getting direction of normal force
        Eigen::Vector3d normForceDirection;
        if(angleOfAttack <= std::numeric_limits<double>::epsilon() ){
            normForceDirection = Eigen::Vector3d::Zero();
        } else {
            Eigen::Vector3d vdiff = relativeVelocity-rocketOrientationVec;
            normForceDirection = (rocketOrientationVec.cross(rocketOrientationVec.cross(vdiff))).normalized();
        }

        Eigen::Vector3d normForce = cn*referenceArea()*dynamicPressure*normForceDirection;
        forces += normForce;
        //fmt::print("TIME {}, NORM [{}]\n", time, toString(normForce.transpose()));
        assert(!normForce.hasNaN());

        //fmt::print("TIME {}, STATE AT FAILURE [{}]}\n", time, toString(state.transpose()));
        //fmt::print("t={:<8.4f} aoa {}\n", time, angleOfAttack);
        //fmt::print("t={:<8.4f} cn {}\n", time, cn);
        
        //fmt::print("t={:<8.4f} norm force     [{}]\n", time, toString(normForce.transpose()));

        Eigen::Vector3d rockCP = _rotmat*_rocket->cp(mach, angleOfAttack);
        Eigen::Vector3d rockCM = _rotmat*_rocket->cm(time);
        Eigen::Vector3d globCP = rocketRotationMat*rockCP;
        Eigen::Vector3d globCM = rocketRotationMat*rockCM;

        Eigen::Vector3d normMoments = (rocketRotationMat.transpose()*normForce).cross(rockCM - rockCP);
        moments += normMoments;
        assert(!normMoments.hasNaN());

        // adding damping
        auto yawDampingCoeff = _rocket->c_m_damp((_rotmat.transpose()*rockCM).x(), angVelocity.x(), relativeSpeed);
        auto pitchDampingCoeff = _rocket->c_m_damp((_rotmat.transpose()*rockCM).x(), angVelocity.y(), relativeSpeed);

        Eigen::Vector3d yawDampingMoment = { yawDampingCoeff*referenceArea()*referenceLength()*dynamicPressure, 0, 0 };
        Eigen::Vector3d pitchDampingMoment = { 0, pitchDampingCoeff*referenceArea()*referenceLength()*dynamicPressure, 0 };
        assert(!yawDampingMoment.hasNaN());
        assert(!pitchDampingMoment.hasNaN());
        
        // applying yaw damping in opposite direction of yaw velocity
        if(angVelocity.x() < 0){
            moments += yawDampingMoment;
        } else {
            moments -= yawDampingMoment;
        }
        // applying pitch damping in opposite direction of pitch
        if(angVelocity.z() < 0){
            moments += pitchDampingMoment;
        } else {
            moments -= pitchDampingMoment;
        }

        /*
        ---------------------------
        ------- DRAG FORCES -------
        ---------------------------
        */
        // calculate reynolds number of air
        double kinVisc = _atmos->kinematic_viscosity(alt);
        double reynL = relativeVelocity.norm()/kinVisc;

        Eigen::Vector3d dragDir = -relativeVelocity.normalized(); // drag occurs in opposite direction to relative velocity

        double cdf = _rocket->Cdf(mach, reynL, angleOfAttack);
        double cdp = _rocket->Cdp(mach, angleOfAttack);
        double cdb = _rocket->Cdb(mach, time, angleOfAttack);
        double cd = 0.0;// cdf + cdp + cdb;
        double dragMag = cd*referenceArea()*dynamicPressure;

        assert(!std::isnan(cdf));
        assert(!std::isnan(cdp));
        assert(!std::isnan(cdb));
        if(cdf < -0.001 || cdp < -0.001 || cdb < -0.001){
            fmt::println("Drag is less than zero Cdf = {:<.8f}, Cdp = {:<.8f}, Cdb = {:<.8f}", cdf, cdp, cdf);
            fmt::println("TIME {}, STATE AT FAILURE [{}]\nm = {}, AoA= {}", time, toString(state.transpose()), mach, angleOfAttack);
            assert(cdp >= 0);
            assert(cdb >= 0);
            assert(cdf >= 0);
        }

        Eigen::Vector3d dragForce = dragMag*dragDir;
        assert(!dragForce.hasNaN());
        //forces += dragForce;
        //fmt::print("TIME {}, DRAG [{}]\n", time, toString(dragForce.transpose()));

        Eigen::Vector3d dragMoments = (rocketRotationMat.transpose()*dragForce).cross(rockCP - rockCM);
        //moments += dragMoments;

        /*
        --------------------------------
        CONSOLIDATING FORCES AND MOMENTS
        --------------------------------
        */

        // adding forces to acceleration
        acceleration += forces/m;
        //fmt::print("TIME {:<8.4f} ACCELERATION WITH F [{}]\n", time, toString(acceleration.transpose()));
        //fmt::print("TIME {:<8.4f} ACCELERATION WITH GRAV [{}]\n", time, toString(acceleration.transpose()));


        // adding random pitch and yaw to flight
        double randPitchCoeff = (((double) std::rand())/RAND_MAX - 0.5)*2*0.0005;
        double randYawCoeff = (((double) std::rand())/RAND_MAX - 0.5)*2*0.0005;

        moments += Eigen::Vector3d{ randYawCoeff, randPitchCoeff, 0 }*_aRef*_lRef*dynamicPressure;
        assert(!moments.hasNaN());
        // adding moments to angular acceleration
        angAcceleration += inertia.inverse()*moments;
        

        assert(!acceleration.hasNaN());
        assert(!angAcceleration.hasNaN());
        
        // adjusting for takeoff
        if(!takeoff()){
            if(acceleration.z() < 0){
                acceleration.z() = 0;
            }
        }

        // adjusting for onRod
        if(onRod()){
            auto velNorm = velocity.norm();
            angAcceleration = Eigen::Vector3d::Zero();
            acceleration = acceleration.norm()*rodVec();
        }

        /*
        if((angAcceleration.cwiseGreater(100)).any()){
            fmt::println("Abnormally large ang acceleration [{}]",toString(angAcceleration.transpose()));
            fmt::println("TIME {}, STATE AT FAILURE [{}]\nm = {}, AoA = {} cn = {} Cdf = {:<.8f}, Cdp = {:<.8f}, Cdb = {:<.8f}", time, toString(state.transpose()), mach, angleOfAttack, cn, cdf, cdp, cdf);
            fmt::println("rho = {}, dragDir = [{}], normdir = [{}], pitchDampingC = {}, yawDampingC = {}", atmDens, toString(dragDir.transpose()), toString(normForceDirection.transpose()), pitchDampingCoeff, yawDampingCoeff);
            fmt::println("norm moments = [{}], norm accs = [{}], dynPres = {}", toString(normMoments.transpose()), toString((inertia.inverse()*normMoments).transpose() ), dynamicPressure);
            fmt::println("velocity = [{}], relative_velocity = [{}]", toString(velocity.transpose()), toString(relativeVelocity.transpose()));
            assert(false);
        }
        */

        // adding final acceleration vector
        res[Xv] = acceleration.x();
        res[Yv] = acceleration.y();
        res[Zv] = acceleration.z();
        // adding final angular acceleration vector
        res[dPhi] = angAcceleration.x();
        res[dTheta] = angAcceleration.y();
        res[dPsi] = angAcceleration.z();

        //fmt::println("INERTIA\n{}\n", toString(inertia));

        StepData data = {
            std::pair<std::string, double>{"Altitude", alt},
            std::pair<std::string, double>{"Pressure", pres},
            std::pair<std::string, double>{"Density", atmDens},
            std::pair<std::string, double>{"Mass", m},
            std::pair<std::string, double>{"g", g},
            std::pair<std::string, double>{"CGx", (_rotmat.transpose()*rockCM).x()},
            std::pair<std::string, double>{"Thrust", th.norm()},
            std::pair<std::string, double>{"CN", cn},
            std::pair<std::string, double>{"AoA", angleOfAttack/M_PI*180},
            std::pair<std::string, double>{"M", mach},
            std::pair<std::string, double>{"CPx", (_rotmat.transpose()*rockCP).x()},
            std::pair<std::string, double>{"Yaw Damping", yawDampingCoeff},
            std::pair<std::string, double>{"Pitch Damping", pitchDampingCoeff},
            std::pair<std::string, double>{"Ixx", Ixx},
            std::pair<std::string, double>{"Iyy", Iyy},
            std::pair<std::string, double>{"Izz", Izz},
            std::pair<std::string, double>{"ReL", reynL},
            std::pair<std::string, double>{"Cdf", cdf},
            std::pair<std::string, double>{"Cdp", cdp},
            std::pair<std::string, double>{"Cdb", cdb},
            std::pair<std::string, double>{"Cd", cd}
        };

        //fmt::print("TIME {}, OUT [{}]\n\n", time, toString(res.transpose()));
        
        return {res, data};
    }

    double Sim::altitude(Eigen::Vector3d position) const {
        // TODO: modify this based on latitude and longitude
        Eigen::Vector3d centVec = originToCenterOfEarth();
        Eigen::Vector3d combinedVec = centVec - position; // -position because its from the rocket to the origin
        double combDist = combinedVec.norm();
        double centDist = centVec.norm();
        double distToEarthSurf = combDist - centDist; // assuming a spherical earth
        return distToEarthSurf;
    }

    Eigen::Vector3d Sim::wind(Eigen::Vector3d position) const {
        return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d Sim::originToCenterOfEarth() const {
        // TODO: modify this based on lat and long
        Eigen::Vector3d originVec =  Eigen::Vector3d{0,0,-RealAtmos::R_0};
        return originVec;
    }

    Eigen::Vector3d Sim::centerOfEarthVector(Eigen::Vector3d position) const {
        Eigen::Vector3d centVec = originToCenterOfEarth();
        Eigen::Vector3d combinedVec = centVec - position; // -position because its from the rocket to the origin
        Eigen::Vector3d normVec = combinedVec.normalized();
        return normVec;
    }

}