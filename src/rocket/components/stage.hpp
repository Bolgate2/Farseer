#ifndef STAGE_H_
#define STAGE_H_

#include <vector>
#include "uuid_v4.h"

namespace Stage{
    class Stage{
        private:

        public:
            // constructors
            Stage();
            Stage(UUIDv4::UUIDGenerator<std::mt19937_64>* uuidGenerator);
            // destructor
            ~Stage();
    };
}

#endif