#ifndef COMPONENT_H
#define COMPONENT_H
#include <string>

namespace Component{
    void print_name();

    class Component{
        private:
            uint32_t id;
            Component* parent;

        public:
            std::string name;

    };
}

#endif