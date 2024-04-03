#pragma once
#include <memory>
#include <vector>

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
    
    public:
        // constructors
        // NO JSON CONSTRUCTOR AS THE JSON PROPERTIES METHOD IS VIRTUAL
        // NO CONSTRUCTOR WITH PARENT AS THE ADD CHILD METHOD RELIES ON VIRTUAL FUNCTIONS
        // to construct using virtual methods, create an empty object, then apply stuff to it
        Component(std::string name = "", Eigen::Vector3d position = Eigen::Vector3d::Zero());

        std::string name;

        std::string id(){ return _id; };

        // Component Typing
        // this method says what type of component this is in the central register and must be implemented for any non-virtual components
        virtual COMPONENT_TYPE type() = 0;
        // this method dictates what types of component can be added as children, used in addChild
        virtual std::vector<COMPONENT_TYPE> allowedChildren() = 0;

        // sub component methods
        std::vector<std::shared_ptr<Component>> components() { return _components; };
        std::weak_ptr<Component> parent() { return _parent; }

        bool addChild( std::shared_ptr<Component> comp );
        std::shared_ptr<Component> findChild( std::string uuid );

        bool removeChild( std::shared_ptr<Component> comp );
        bool removeChild( std::string uuid );

        bool setParent( std::shared_ptr<Component> parent );

        // JSON methods
        // applies the properties in a JSON to this component
        void fromJson(json j);
        json toJson();
        virtual json propertiesToJson() = 0;
        // this will also go about creating components
        virtual void jsonToProperties(json j) = 0;
};

}