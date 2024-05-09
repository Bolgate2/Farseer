#pragma once
#include "component.hpp"
#include "material.hpp"
#include "finish.hpp"

namespace Rocket{

class BodyTube : public Component{
    private:
        double _height = 0;
        double _diameter = 0;
        double _thickness = 0;
        bool _filled = false;
        std::unique_ptr<Material> _material;
        std::unique_ptr<Finish> _finish;
    protected:
        virtual json propertiesToJson() override;
        // this will also go about creating sub-components
        virtual void jsonToProperties(json j) override;
    
    public:
        BodyTube(
            std::string name = "", Eigen::Vector3d position = Eigen::Vector3d::Zero(),
            std::unique_ptr<Material> material = std::move(std::make_unique<Material>("Default", 0.0)),
            std::unique_ptr<Finish> finish = std::move(std::make_unique<Finish>("Default", 0.0))
        );
        BodyTube(
            double height, double diameter, double thickness, bool filled = false,
            std::string name = "", Eigen::Vector3d position = Eigen::Vector3d::Zero(),
            std::unique_ptr<Material> material = std::move(std::make_unique<Material>("Default", 0.0)),
            std::unique_ptr<Finish> finish = std::move(std::make_unique<Finish>("Default", 0.0))
        );

        double getHeight(){ return _height; }
        void setHeight(double height) { _height = std::max(0.0,height); }
        
        double getDiameter(){ return _diameter; }
        void setDiameter(double diameter) { _diameter = std::max(0.0,diameter); }

        double getThickness(){ return _thickness; }
        void setThickness(double thickness) { _thickness = std::max(0.0,thickness); }

        bool getFilled(){ return _filled; }
        void setFilled(bool filled) { _filled = filled; }

        Material* getMaterial() { return _material.get(); }
        void setMaterial(std::unique_ptr<Material> material) { _material = std::move(material); }

        Finish* getFinish() { return _finish.get(); }
        void setFinish(std::unique_ptr<Finish> finish) { _finish = std::move(finish); }

        virtual std::string type() override { return COMPONENT_NAMES::BODY_TUBE; };
        virtual std::vector<std::string> allowedComponents() override {
            return std::vector<std::string>{
                COMPONENT_NAMES::MOTOR,
                COMPONENT_NAMES::FIN_SET
            };
        };

};

}