#pragma once
#include <memory>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>

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

// THESE NAMES MUST BE UNIQUE
// std::unordered_set<std::string> COMPONENT_TYPE_NAMES;

// use hash define to create macro for registering components by name in a set
// this define will check if the name is unique


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
        void setPosition(Eigen::Vector3d position){ _position = position; };

        std::string id(){ return _id; };

        // Component Typing
        // this method says what type of component this is in the central register and must be implemented for any non-virtual components
        virtual COMPONENT_TYPE type() = 0;
        virtual std::string typeName() = 0;
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

template<class T>
class DerivedComponent : public Component{
    protected:
        DerivedComponent(std::string name = "", Eigen::Vector3d position = Eigen::Vector3d::Zero()):
        Component(name, position) {};
    public:

        template<typename... args>
        static std::shared_ptr<T> create(args... a){
            return std::make_shared<T>(a...);
        }

        static std::shared_ptr<T> create(json a){
            std::shared_ptr<T> comp = std::make_shared<T>();
            comp->fromJson(a);
            return comp;
        }
};

#define REGISTER_COMPONENT(CLASS_NAME, NAME) \
    static_assert( true ); \
    class CLASS_NAME; \
    class __ ## CLASS_NAME ## _INTERIM : public DerivedComponent<CLASS_NAME>{ \
        public: \
            virtual std::string typeName() override { return #NAME; }; \
    }; \
    class CLASS_NAME : public __ ## CLASS_NAME ## _INTERIM

/*
template<typename T, typename... args>
std::shared_ptr<Component> create(args... a){
    std::shared_ptr<Component> comp = std::make_shared<T>(a...);
    return comp;
};

// if a component is supplied as the first argument, it will be considered the intended parent
template<typename T, typename... args>
std::shared_ptr<Component> create(Component* parent, args... a){
    std::shared_ptr<Component> comp = std::make_shared<T>(a...);
    parent->addComponent(comp.get());
    return comp;
};

// creating from a json, type will be inferred from json
template<typename T, typename... args>
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
*/
}