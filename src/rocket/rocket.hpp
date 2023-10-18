#ifndef ROCKET_H_
#define ROCKET_H_

#include <vector>
#include <string>
#include "uuid_v4.h"
#include "stage.hpp"
#include "launchConditions.hpp"

namespace Rocket{
    class Rocket{
        private:
            std::string _id;
            //hold pointers rather than the objects for speed (passing pointers around is faster than passing objects)
            UUIDv4::UUIDGenerator<std::mt19937_64>* _uuidGenerator;
            std::vector<Stage::Stage*> _stages;
            LaunchConditions::LaunchConditions* _launchConditions;
        public:
            std::string _name;
            // constructors
            Rocket();
            Rocket(UUIDv4::UUIDGenerator<std::mt19937_64>* uuidGenerator);
            // destructors
            ~Rocket();

            //functions
            std::string generateId();

            // getters and setters
            UUIDv4::UUIDGenerator<std::mt19937_64>* uuidGenerator(){ return _uuidGenerator; } //no setter, read only
            std::vector<Stage::Stage*> stages(){ return _stages; }
            LaunchConditions::LaunchConditions* launchConditions(){ return _launchConditions; }
    };
}

#endif