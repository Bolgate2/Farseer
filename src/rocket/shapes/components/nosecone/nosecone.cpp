#include "nosecone.hpp"
#include "../../primitives/nosecone/noseconeShapeTypes.hpp"
#include <iostream>

namespace Shapes{
    NoseconeComponentShape::NoseconeComponentShape( NoseconeShapeTypes type, double radius, double length, double thickness, double shapeParam ){
        _shape = NoseconeShapeFactory::create( type, radius, length, thickness, shapeParam);
    }
    
    NoseconeShape* NoseconeComponentShape::shape(){
        return _shape.get();
    }
    
    void NoseconeComponentShape::setShape( std::unique_ptr<Shape> shape ){
        NoseconeShape* noseconeCast = dynamic_cast<NoseconeShape*>(shape.get());
        if(noseconeCast != nullptr){
            auto newNoseconePtr = std::unique_ptr<NoseconeShape>(noseconeCast);
            shape.release();
            _shape = std::move(newNoseconePtr);
        } else {
            std::cerr << "Invalid shape type for BodyTubeComponentShape" << std::endl;
        }
    }
    
}