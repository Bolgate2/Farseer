#ifndef COMPONENT_H
#define COMPONENT_H
#include <string>
#include <vector>
#include <Eigen/Dense>
#include "uuid_v4.h"

namespace Rocket{

    // this is an empty component with no real qualities, kind of like an empty gameobject in unreal or unity (ew)
    class Component{
        private:
            // unique identifier
            UUIDv4::UUIDGenerator<std::mt19937_64>* _uuidGenerator;
            uint32_t _id;
            // tree stuff
            Component* _parent;
            std::vector<Component*> _components;
            Eigen::Vector3d _position;
        public:
            // public variables
            std::string name;
            // constructors
            Component();
            // destructor
            ~Component();

            // getters and setters
            // parent
            Component* parent(){ return _parent;}
            void setParent( Component* parent ){ parent->addComponent(this); }

            // components
            std::vector<Component*> components(){ return _components; }
            Component* findComponent(std::string id);

            virtual Component* addComponent(Component* component);
            virtual void removeComponent(Component* component);
            void removeComponent(std::string id);

            // position
            Eigen::Vector3d position();
            void setPosition(Eigen::Vector3d pos);
            void setPosition(double pos[3]);
            void setPosition(double x, double y, double z);
            
    };

}

#endif