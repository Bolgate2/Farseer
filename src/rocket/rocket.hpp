#ifndef ROCKET_H_
#define ROCKET_H_

#include <vector>
#include <string>
#include "uuid_v4.h"
#include "stage.hpp"

namespace Rocket{
    class Rocket{
        private:
            UUIDv4::UUIDGenerator<std::mt19937_64> _uuidGenerator;
            std::string _id;
            std::vector<Stage::Stage*> _stages; //hold pointers rather than the objects for speed (passing pointers around is faster than passing objects)
        public:
            std::string _name;
            // constructors
            Rocket();
            // destructors
            ~Rocket();
            // getters and setters
            UUIDv4::UUIDGenerator<std::mt19937_64> uuidGenerator(){ return _uuidGenerator; } //no setter, read only
            std::vector<Stage::Stage*> stages(){return _stages;}
    };
}

#endif