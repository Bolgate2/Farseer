#pragma once
#include <memory>
#include <vector>
#include <algorithm>

#include <Eigen/Dense>
#include <uuid_v4/uuid_v4.h>
#include <nlohmann/json.hpp>
using json = nlohmann::json;


namespace Rocket {

enum COMPONENT_TYPE{
    INVALID = -1,
    ROCKET,
    STAGE,
    BODY_TUBE,
    NOSE_CONE,
    FIN_SET,
    MOTOR
};


class Component : std::enable_shared_from_this<Component>{
    private:
        static UUIDv4::UUIDGenerator<std::mt19937_64> _uuidGenerator;
        std::string _id;

        std::vector<std::shared_ptr<Component>> _components = std::vector<std::shared_ptr<Component>>{};
        std::weak_ptr<Component> _parent = std::weak_ptr<Component>{};

        Eigen::Vector3d _position;
    protected:
        virtual json propertiesToJson() = 0;
        // this will also go about creating sub-components
        virtual void jsonToProperties(json j) = 0;

    public:
        // constructors
        // NO JSON CONSTRUCTOR AS THE JSON PROPERTIES METHOD IS VIRTUAL
        // NO CONSTRUCTOR WITH PARENT AS THE ADD CHILD METHOD RELIES ON VIRTUAL FUNCTIONS
        // to construct using virtual methods, create an empty object, then apply stuff to it
        Component(std::string name = "", Eigen::Vector3d position = Eigen::Vector3d::Zero());

        std::string name;

        Eigen::Vector3d getPosition(){ return _position; };
        Eigen::Vector3d setPosition(Eigen::Vector3d position){ _position = position; };

        std::string id(){ return _id; };

        // Component Typing
        // this method says what type of component this is in the central register and must be implemented for any non-virtual components
        virtual COMPONENT_TYPE type() = 0;
        // this method dictates what types of component can be added as children, used in addComponent
        virtual std::vector<COMPONENT_TYPE> allowedComponents() = 0;

        // sub component methods
        std::vector<std::shared_ptr<Component>> components() { return _components; };
        Component* parent() { return _parent.lock().get(); }
        // sets parent directly, DO NOT DO THIS USE THE ADD AND REMOVE CHILD FUNCTIONS
        bool setParent( Component* parent );

        bool addComponent( Component* comp );
        std::shared_ptr<Component> findComponent( std::string uuid );
        bool removeComponent( Component* comp );
        bool removeComponent( std::string uuid );

        // JSON methods
        // applies the properties in a JSON to this component
        void fromJson(json j);
        json toJson();
};

class BodyTube : public Component{
    private:
        double _height = 0;
        double _diameter = 0;
        double _thickness = 0;
        bool _filled = false;
    
    protected:
        virtual json propertiesToJson() override;
        // this will also go about creating sub-components
        virtual void jsonToProperties(json j) override;
    
    public:
        BodyTube(std::string name = "", Eigen::Vector3d position = Eigen::Vector3d::Zero());
        BodyTube(double height, double diameter, double thickness, bool filled = false, std::string name = "", Eigen::Vector3d position = Eigen::Vector3d::Zero());

        double getHeight(){ return _height; };
        void setheight(double height) { _height = std::max(0.0,height); };
        double getDiameter(){ return _diameter; };
        void setDiameter(double diameter) { _diameter = std::max(0.0,diameter); };
        double getThickness(){ return _thickness; };
        void setThickness(double thickness) { _thickness = std::max(0.0,thickness); };
        bool getFilled(){ return _filled; };
        void setFilled(bool filled) { filled = _filled; };

        virtual COMPONENT_TYPE type() override { return COMPONENT_TYPE::BODY_TUBE; };
        virtual std::vector<COMPONENT_TYPE> allowedComponents() override {
            return std::vector<COMPONENT_TYPE>{
                COMPONENT_TYPE::MOTOR,
                COMPONENT_TYPE::FIN_SET
            };
        };
};

template<typename T, typename... args>
std::shared_ptr<Component> create(args...){
    std::shared_ptr<Component> comp = std::make_shared<T>(args...);
    return comp;
};

// if a component is supplied as the first argument, it will be considered the intended parent
template<typename T, typename... args>
std::shared_ptr<Component> create(std::shared_ptr<Component> parent, args...){
    std::shared_ptr<Component> comp = std::make_shared<T>(args...);
    comp->setParent(parent);
    return comp;
};

// creating from a json where a type is specified, may not be neccecary tbh
template<typename T>
std::shared_ptr<Component> create(json j){
    std::shared_ptr<Component> comp = std::make_shared<T>();
    comp->fromJson(j);
    return comp;
};

// creating from a json, type will be inferred from json
std::shared_ptr<Component> create(json j){
    COMPONENT_TYPE type = j.at("component_type");
    std::shared_ptr<Component> comp = nullptr;

    switch (type){
        case BODY_TUBE:
            comp = std::make_shared<BodyTube>();
            break;
        default:
            comp = nullptr;
            break;
    }

    if(comp.get() == nullptr){
        // TODO: ERROR LOGGING
        return nullptr;
    }
    comp->fromJson(j);

    return comp;
};

}