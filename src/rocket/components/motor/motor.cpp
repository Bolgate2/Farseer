#include "motor.hpp"
#include <fstream>
#include <filesystem>
#include <sstream>
#include <map>
#include "maths.hpp"

namespace Rocket{
    //static variables and functions
    const std::string Motor::_defaultName = "Motor";
    std::shared_ptr<Motor> Motor::fromFile( std::filesystem::path fileName, Component* parent, Eigen::Vector3d position, double ignitionTime, double dataReferencePressure, double nozzleExitArea){
        std::cout << fileName << std::endl;
        std::string line;
        std::ifstream motorFile(fileName);
        // things to be read
        std::map<double, double> thrustData = {}; // key:time, value:thrust
        std::string name = "";
        double diameter = 0;
        double length = 0;
        double propMass = 0;
        double totalMass = 0;

        if(motorFile.is_open()){
            // file has successfully opened
            bool headerLineRead = false;
            while(std::getline(motorFile, line)){
                if(line.rfind(";",0) == 0) continue; // comment line
                    
                if(!headerLineRead){
                    std::vector<std::string> parts = {};
                    // splits the string by spaces
                    std::string part;
                    std::stringstream lineStream(line);
                    while(lineStream >> part){
                        parts.push_back(part);
                    }
                    
                    if(parts.size() != 7){
                        // throw error
                        std::cerr << "invalid motor header file" << std::endl;
                        motorFile.close();
                        return nullptr; // return null
                    }
                    
                    headerLineRead = true;
                    
                    // delays and manufacturer ignored
                    name = parts[0];
                    diameter = std::stod(parts[1])/1000;
                    length = std::stod(parts[2])/1000;
                    propMass = std::stod(parts[4]);
                    totalMass = std::stod(parts[5]);
                } else {
                    std::vector<std::string> parts = {};
                    // splits the string by spaces
                    std::string part;
                    std::stringstream lineStream(line);
                    while(lineStream >> part){
                        parts.push_back(part);
                    }
                    // throw error if more than 2 elems
                    double time = std::stod(parts[0]);
                    double thrust = std::stod(parts[1]);
                    thrustData[time] = thrust;
                    if(thrustData.rbegin()->second == 0) break; // break if last thrust is 0, not sure if is neccecary
                }
            }
            motorFile.close();
        } else {
            // throw error/return null, file failed to open
            return nullptr;
        }
        auto radius = diameter/2;

        auto motorPtr = create(radius, length, thrustData, propMass, totalMass, parent, name, position, ignitionTime, dataReferencePressure, nozzleExitArea);
        return motorPtr;
    }

    std::shared_ptr<Motor> Motor::create(double radius, double length, std::map<double,double> thrustData, double propMass, double totalMass, Rocket::Component *parent, std::string name, Eigen::Vector3d position, double ignitionTime, double referencePressure, double nozzleExitArea){
        auto obj = std::shared_ptr<Motor>(
            new Motor(radius, length, thrustData, propMass, totalMass, name, position, ignitionTime, referencePressure, nozzleExitArea)
            );
        if(parent != nullptr){
            parent->addComponent(obj.get());
        }
        return obj;
    }

    // constructor
    Motor::Motor(double radius, double length, std::map<double, double> thrustData, double propMass, double totalMass, std::string name, Eigen::Vector3d position, double ignitionTime, double referencePressure, double nozzleExitArea):
    InternalComponent(name, position)
    {
        auto shp = std::make_unique<Shapes::Cylinder>(radius, length);
        _shape = std::move(shp);
        _thrustData = thrustData;
        _propMass = propMass;
        _totalMass = totalMass;
        _ignitionTime = ignitionTime;
        _referencePressure = referencePressure;
        _nozzleExitArea = nozzleExitArea;
        _massData = calculateMassData(thrustData, propMass, totalMass);
    }

    // calculating mass data
    std::map<double,double> Motor::calculateMassData(std::map<double,double> thrustData, double propMass, double totalMass){
        // collating data into vectors
        std::vector<double> timeV = {0};
        std::vector<double> thrustV = {0};
        for(auto iter = thrustData.begin(); iter != thrustData.end(); iter++){
            timeV.push_back(iter->first);
            thrustV.push_back(iter->second);
        }
        // moving vector data into eigen arrays
        Eigen::ArrayXd timeD;
        timeD.resize(timeV.size());
        for(auto i = 0; i < timeV.size(); i++) timeD[i] = timeV[i];

        Eigen::ArrayXd thrustD;
        thrustD.resize(thrustV.size());
        for(auto i = 0; i < thrustV.size(); i++) thrustD[i] = thrustV[i];
        // calculating total impulse
        auto timeSteps = timeD(Eigen::seqN(1, timeD.size()-1)) - timeD(Eigen::seqN(0, timeD.size()-1));
        auto thrustAvg = (thrustD(Eigen::seqN(0, thrustD.size()-1)) + thrustD(Eigen::seqN(1, thrustD.size()-1)))/2;

        auto totalImpulse = (timeSteps*thrustAvg).sum();
        _totalImpulse = totalImpulse;
        // calculating proportion of impulse that has been spent at each time point
        std::vector<double> impulsePropsV = {};

        for(int i = 0; i < timeSteps.size(); i++){
            auto spentImpulse = (timeSteps(Eigen::seq(0,i))*thrustAvg(Eigen::seq(0,i))).sum();
            auto impulseProp = spentImpulse/totalImpulse;
            impulsePropsV.push_back(impulseProp);
        }

        std::map<double, double> massD = { {0,totalMass} };

        for(int i = 0; i < impulsePropsV.size(); i++){
            auto prop = impulsePropsV[i];
            double m = totalMass - propMass*prop;
            auto t = timeD[i+1];
            massD[t] = m;
        }

        return massD;
    }

    double Motor::calculateBurnoutTime() {
        auto btime = ignitionTime() + thrustData().rbegin()->first;
        _burnoutTime = btime;
        return btime;
    }

    // calculating properties
    double Motor::calculateMass(double time) const {
        auto adjTime = time-ignitionTime();
        auto mData = massData();
        auto maxTime = massData().rbegin()->first;
        if(adjTime <= 0) return totalMass();
        if(adjTime >= maxTime) return totalMass()-propMass();
        auto upper = mData.upper_bound(adjTime);
        auto lower = upper--;
        auto lowerVal = lower->second;
        auto lowerTime = lower->first;
        auto timeStep = adjTime-lowerTime;

        auto lowerThrust = thrustData()[lowerTime];
        auto tThrust = calculateThrust(adjTime);
        auto avgThrust = (-tThrust.x() + lowerThrust)/2;

        auto imp = timeStep*avgThrust;
        auto dMass = propMass() * imp/_totalImpulse;
        auto m = lowerVal - dMass;

        return m;
    }

    Eigen::Vector3d Motor::calculateThrust(double time) const {
        auto adjTime = time-ignitionTime();
        auto tData = thrustData();
        auto maxTime = thrustData().rbegin()->first;
        if(adjTime <= 0) return Eigen::Vector3d::Zero();
        if(adjTime >= maxTime) return Eigen::Vector3d::Zero();
        auto upper = tData.upper_bound(adjTime);
        auto lower = upper--;
        auto grad = (upper->second - lower->second)/(upper->first - lower->first);
        auto interpVal = lower->second + grad*(adjTime - lower->first);
        return {-interpVal, 0, 0};
    }

    Eigen::Vector3d Motor::calculateThrustPosition(double time) const {
        return Eigen::Vector3d{shape()->length(), 0, 0} + position();
    }

    Eigen::Vector3d Motor::calculateCm(double time) const {
        return Eigen::Vector3d{shape()->length()/2,0,0} + position();
    }

    Eigen::Matrix3d Motor::calculateInertia(double time) const {
        auto m = calculateMass(time);
        auto density = m/shape()->volume();
        auto thisInertia = shape()->inertia()*density;
        auto originInertia = Utils::parallel_axis_transform(thisInertia, calculateCm(time), m);
        //std::cout << "MOTOR INERTIA\n" << thisInertia << std::endl;
        return originInertia;
    }

    // getters, no setters
    Shapes::Cylinder* Motor::shape() const {
        return _shape.get();
    }

    std::map<double,double> Motor::thrustData() const {
        return _thrustData;
    }

    std::map<double,double> Motor::massData() const {
        return _massData;
    }

    double Motor::propMass() const {
        return _propMass;
    }

    double Motor::totalMass() const {
        return _totalMass;
    }

    double Motor::ignitionTime() const {
        return _ignitionTime;
    }

    double Motor::referencePressure() const {
        return _referencePressure;
    }
    
    double Motor::nozzleExitArea() const {
        return _nozzleExitArea;
    }
}