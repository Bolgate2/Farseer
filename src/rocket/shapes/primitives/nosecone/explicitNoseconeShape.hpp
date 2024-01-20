#ifndef EXPLICIT_NOSECONE_SHAPE_H
#define EXPLICIT_NOSECONE_SHAPE_H

#include "noseconeShape.hpp"
#include <map>

namespace Shapes{
    // this is a class for which explicit solutions exist for the nosecones area, volume, etc e.g. conical
    // this will remain abstract
    class ExplicitNosecone : public NoseconeShape{
        
    };
}

#endif
