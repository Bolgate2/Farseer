#ifndef MOTOR_H_
#define MOTOR_H_

#include <string>
#include <memory>
#include <filesystem>
#include <map>
#include <Eigen/Dense>
#include "../internalComponent.hpp"
#include "../../shapes/primitives/cylinder.hpp"

namespace Rocket{

    class Motor: public InternalComponent{
        private:
            static const std::string _defaultName;
            std::unique_ptr<Shapes::Cylinder> _shape;

            std::map<double,double> _thrustData;
            std::map<double,double> _massData;
            std::map<double,double> calculateMassData(std::map<double,double> thrustData, double propMass, double totalMass);

            double _propMass;
            double _totalMass;
            double _totalImpulse;

            double _ignitionTime;
            double _referencePressure;
            double _nozzleExitArea;

        protected:

            virtual double calculateBurnoutTime() override;
            virtual std::vector< std::shared_ptr<const Motor> > motors() const override;

            virtual double calculateMass(double time) const override;
            virtual Eigen::Vector3d calculateCm(double time) const override;
            virtual Eigen::Matrix3d calculateInertia(double time) const override;
            virtual Eigen::Vector3d calculateThrust(double time) const override;
            virtual Eigen::Vector3d calculateThrustPosition(double time) const override;

            Motor(
                double radius, double length, std::map<double,double> thrustData, double propMass, double totalMass,
                std::string name = _defaultName, Eigen::Vector3d position = Eigen::Vector3d::Zero(), double ignitionTime = 0, double referencePressure = 0, double nozzleExitArea = 0
                );
        public:
            // only void for now
            static std::shared_ptr<Motor> create(
                double radius, double length, std::map<double,double> thrustData, double propMass, double totalMass, Component* parent = nullptr,
                std::string name = _defaultName, Eigen::Vector3d position = Eigen::Vector3d::Zero(), double ignitionTime = 0, double referencePressure = 0, double nozzleExitArea = 0
                );

            static std::shared_ptr<Motor> fromFile(
                std::filesystem::path fileName, Component* parent = nullptr, Eigen::Vector3d position = Eigen::Vector3d::Zero(),
                double ignitionTime = 0, double dataReferencePressure = 0, double nozzleExitArea = 0
                );

            // motors cannot have components
            virtual std::vector< std::shared_ptr<Component> > components() const override { return {}; }
            virtual std::shared_ptr<Component> findComponent(std::string id) const override { return nullptr; }
            virtual void addComponent(Component* component) {/*do nothing*/}
            virtual void removeComponent(Component* component) {/*do nothing*/}

            // getters, no setters
            virtual Shapes::Cylinder* shape() const;
            virtual std::map<double,double> thrustData() const;
            virtual std::map<double,double> massData() const;

            virtual double propMass() const;
            virtual double totalMass() const;

            virtual double ignitionTime() const;
            virtual double referencePressure() const;
            virtual double nozzleExitArea() const;
    };
}

#endif