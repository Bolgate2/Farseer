#include "nosecone.hpp"
#include "../../shapes/components/nosecone/nosecone.hpp"
#include "maths.hpp"
#include <iostream>
#include <memory>
#include <cmath>

namespace Rocket{

    std::string Nosecone::defaultName = "Nosecone";
    // constructors
    Nosecone::Nosecone(Shapes::NoseconeShapeTypes type, double radius, double length, double thickness, double shapeParam, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish, std::string name, Eigen::Vector3d position):
    BodyComponent(nullptr, std::move(material), std::move(finish), name, position )
    {
        auto shape = std::make_unique<Shapes::NoseconeComponentShape>(type, radius, length, thickness, shapeParam);
        setShape(std::move(shape));
    }
    // constructor without thickness
    Nosecone::Nosecone(Shapes::NoseconeShapeTypes type, double radius, double length, double shapeParam, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish, std::string name, Eigen::Vector3d position):
    BodyComponent(nullptr, std::move(material), std::move(finish), name, position )
    {
        std::cerr << "NOSECONE WITHOUT THICKNESS HAS NOT YET BEEN IMPLEMENTED. Using thickness 0.001" << std::endl;
        auto shape = std::make_unique<Shapes::NoseconeComponentShape>(type, radius, length, 0.001, shapeParam);
        setShape(std::move(shape));
    }
    // constructor with existing shape
    Nosecone::Nosecone(std::unique_ptr<Shapes::NoseconeComponentShape> shape, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish, std::string name, Eigen::Vector3d position):
    BodyComponent(nullptr, std::move(material), std::move(finish), name, position )
    {
        setShape(std::move(shape));
    }

    // creator
    std::shared_ptr<Nosecone> Nosecone::create(Shapes::NoseconeShapeTypes type, double radius, double length, double thickness, double shapeParam, std::unique_ptr<Material> material, std::unique_ptr<Finish> finish, Stage *parent, std::string name, Eigen::Vector3d position)
    {
        auto obj = std::shared_ptr<Nosecone>(
            new Nosecone(type, radius, length, thickness, shapeParam, std::move(material), std::move(finish), name, position)
        );
        if(parent != nullptr){
            parent->addComponent(obj.get());
        }
        return obj;
    }

    // GETTERS AND SETTERS FOR SHAPE
    Shapes::NoseconeComponentShape* Nosecone::shape() const {
        return _shape.get();
    }

    void Nosecone::setShape( std::unique_ptr<Shapes::AeroComponentShape> shape ){
        // try to cast the underlying pointer to a pointer of the inherited class
        auto castedShapePointer = dynamic_cast<Shapes::NoseconeComponentShape*>(shape.get());

        if(castedShapePointer != nullptr){
            // if casting is successful the a unique pointer of the correct type is crated and the old one is released
            auto newShapeUniquePtr = std::unique_ptr<Shapes::NoseconeComponentShape>(castedShapePointer);
            shape.release();
            setShape(std::move(newShapeUniquePtr));
        }
        // if the function has reached here it has failed
        std::cerr << "Invalid shape for nosecone\n"; // TODO: make this more descriptive
    }

    
    void Nosecone::setShape( std::unique_ptr<Shapes::BodyComponentShape> shape ){
        // try to cast the underlying pointer to a pointer of the inherited class
        auto castedShapePointer = dynamic_cast<Shapes::NoseconeComponentShape*>(shape.get());
        if(castedShapePointer != nullptr){
            // if casting is successful the a unique pointer of the correct type is crated and the old one is released
            auto newShapeUniquePtr = std::unique_ptr<Shapes::NoseconeComponentShape>(castedShapePointer);
            shape.release();
            setShape(std::move(newShapeUniquePtr));
        }
        // if the function has reached here it has failed
        std::cerr << "Invalid shape for nosecone\n"; // TODO: make this more descriptive
    }

    void Nosecone::setShape(std::unique_ptr<Shapes::NoseconeComponentShape> shape){
        // try to cast the underlying pointer to a pointer of the inherited class
        clearCaches();
        _shape = std::move(shape);
    }

    // GETTER AND SETTER FOR SHAPEPARAM
    double Nosecone::shapeParam() const {
        return shape()->shapeParam();
    }

    void Nosecone::setShapeParam(double val){
        clearCaches();
        shape()->setShapeParam(val);
    }

    Eigen::Matrix3d Nosecone::calculateInertia(double time) const {
        // inertia about cm
        auto thisInertia = shape()->inertia(); // inertia/density of the shape about cm
        auto disp = position(); // THIS LINE HAS CHANGED, the nosecones inertia is calculated about its tip
        auto thisVolume = shape()->volume();
        auto zeroInertia = Utils::parallel_axis_transform(thisInertia, -disp, thisVolume);
        zeroInertia *= material()->density; // multiplying by density so it's the actual inertia
        return zeroInertia;
    }

    double Nosecone::maxSurfaceDistanceTravelled() const {
        return shape()->maxSurfaceDistanceTravelled();
    }

    double Nosecone::calculateSurfaceDistanceTravelled(double x) const {
        return shape()->calculateSurfaceDistanceTravelled(x);
    }

    double Nosecone::zeta() const {
        double z;
        switch(type()){
            case Shapes::NoseconeShapeTypes::OGIVE:
                z = std::atan(1.0/(2*finenessRatio())); // may not be correct
                break;
            case Shapes::NoseconeShapeTypes::CONICAL:
                z = std::atan(1.0/(2*finenessRatio()));
                break;
            case Shapes::NoseconeShapeTypes::ELLIPSOID:
                z = 0;
                break;
            case Shapes::NoseconeShapeTypes::POWER:
                z = std::atan( radius()*shapeParam()*std::pow(1.0/length(),shapeParam()) );
                break;
            case Shapes::NoseconeShapeTypes::PARABOLIC:
                
                break;
            case Shapes::NoseconeShapeTypes::HAACK:
                z = 0;
                break;
            default:
                z = 0;
                break;
        }
        return z;
    }

    double Nosecone::cdm0() const {
        return 0.8*std::pow(std::sin(zeta()),2);
    }

    double Nosecone::cDotStag(const double mach) const {
        double qstagonq; 
        if(mach <= 1){
            qstagonq = 1 + std::pow(mach,2)/4 + std::pow(mach,4)/40;
        } else {
            qstagonq = 1.84 - 0.76/std::pow(mach,2) + 0.166/std::pow(mach,4) + 0.035/std::pow(mach,6);
        }
        return 0.85*qstagonq; // should never reach here, just here so that compiler stops complaining
    }

    double Nosecone::conicalSubsonicCdpdot(const double mach) const {
        const double z = zeta();
        const double sigma = std::sin(z);
        const double exponent = ((25.0/6.0)*sigma-(25.0/3.0))/(sigma*(4*sigma-5));
        double cdpdot = 4.0*std::pow(sigma,2)/5.0 + std::pow(mach,exponent)*(sigma-0.8*std::pow(sigma,2));
        return cdpdot;
    }

    // can precalc alot of this
    double Nosecone::conicalTransonicCdpdot(const double mach) const {
        static const Eigen::Matrix<double, 4, 3> zetaCoeffs {
            {-155.5556,    7.6267,   18.5185},
            {536.6667,  -26.8133,  -66.6667},
            {-606.6667,   29.9132,   79.4444},
            {225.5556,   -9.7266,  -31.2963}
        };
        const double z = zeta();
        const double sigma = std::sin(z);
        Eigen::Vector3d sigVec = (sigma*Eigen::Array3d::Ones()).pow(Eigen::Array3d{2,1,0});
        Eigen::Vector<double,4> machVec = (mach*Eigen::Array<double,4,1>::Ones()).pow(Eigen::Array<double,4,1>{3,2,1,0});
        double cdpdot = (zetaCoeffs*sigVec).dot(machVec);
        return cdpdot;
    }

    double Nosecone::conicalSupersonicCdpdot(const double mach) const {
        const auto z = zeta();
        return 2.1*std::pow(std::sin(z),2) + std::sin(z)/(2*Utils::beta(mach));
    }

    double Nosecone::conicalCdpdot(const double mach) const {
        double cdpdot = 0;
        if(mach <= 1){
            cdpdot = conicalSubsonicCdpdot(mach);
        } else if(mach <= 1.3){
            cdpdot = conicalTransonicCdpdot(mach);
        } else {
            cdpdot = conicalSupersonicCdpdot(mach);
        }
        return cdpdot;
    }

    double Nosecone::finenessCorrection(const double mach, const double c3 ) const {
        const auto fn = finenessRatio();
        const auto exponent = std::log(fn+1)/std::log(4);
        const auto c0 = cDotStag(mach);
        return c0*std::pow(c3/c0,exponent);
    }
    
    double Nosecone::ellipsoidCdpdot(const double mach) const {
        auto data = noseconeDataMap.at(NoseconeDataSets::ELLIPSOID);

        auto c3 = interpNoseconeData(mach, 0, data);

        return finenessCorrection(mach, c3);
    }

    double Nosecone::powerCdpdot(const double mach) const {
        const auto data14 = noseconeDataMap.at(NoseconeDataSets::POWER14);
        const auto data24 = noseconeDataMap.at(NoseconeDataSets::POWER24);
        const auto data34 = noseconeDataMap.at(NoseconeDataSets::POWER34);
        const auto k = shapeParam();
        auto interpParam = k;
        double c0 = cdm0();
        double d1, d2;

        if(k <= 0.25)
        {
            d1 = cDotStag(mach);
            d2 = interpNoseconeData(mach, c0, data14);
            interpParam = k*4;
        }
        else if(k <= 0.5)
        {
            d1 = interpNoseconeData(mach, c0, data14);
            d2 = interpNoseconeData(mach, c0, data24);
            interpParam = (k-0.25)*4;
        }
        else if(k <= 0.75)
        {
            d1 = interpNoseconeData(mach, c0, data24);
            d2 = interpNoseconeData(mach, c0, data34);
            interpParam = (k-0.5)*4;
        } else {
            d1 = interpNoseconeData(mach, c0, data34);
            d2 = conicalCdpdot(mach);
            interpParam = (k-0.75)*4;
        }

        auto c3 = (1-interpParam)*d1 + interpParam*d2;

        return finenessCorrection(mach, c3);
    }

    double Nosecone::parabolicCdpdot(const double mach) const {
        auto data1 = noseconeDataMap.at(NoseconeDataSets::PARABOLIC1);
        auto data12 = noseconeDataMap.at(NoseconeDataSets::PARABOLIC12);
        auto data34 = noseconeDataMap.at(NoseconeDataSets::PARABOLIC34);
        auto k = shapeParam();
        auto interpParam = k;
        double c0 = cdm0();
        double d1, d2;

        if(k <= 0.5)
        {
            d1 = cDotStag(mach);
            d2 = interpNoseconeData(mach, c0, data12);
            interpParam = k*2;
        }
        else if(k <= 0.75)
        {
            d1 = interpNoseconeData(mach, c0, data12);
            d2 = interpNoseconeData(mach, c0, data34);
            interpParam = (k-0.5)*4;
        }
        else {
            d1 = interpNoseconeData(mach, c0, data34);
            d2 = interpNoseconeData(mach, c0, data1);
            interpParam = (k-0.75)*4;
        }

        auto c3 = (1-interpParam)*d1 + interpParam*d2;

        return finenessCorrection(mach, c3);
    }

    double Nosecone::haackCdpdot(const double mach) const {
        auto dataVk = noseconeDataMap.at(NoseconeDataSets::VKHAACK);
        auto dataLV = noseconeDataMap.at(NoseconeDataSets::LVHAACK);
        auto k = shapeParam();
        auto interpParam = k*3; //haack shape param is between 0 and 1/3
        auto d1 = interpNoseconeData(mach, 0, dataVk);
        auto d2 = interpNoseconeData(mach, 0, dataLV);

        auto c3 = (1-interpParam)*d1 + interpParam*d2;

        auto findrag = finenessCorrection(mach, c3);

        return findrag;
    }


    double Nosecone::calculateCdpA(const double mach) const {
        double cdpdot = 0;
        double kappa = shapeParam();
        double conicalOgiveAdj = 1;

        switch(type()){
            case Shapes::NoseconeShapeTypes::OGIVE:
                conicalOgiveAdj = (0.72*std::pow(kappa-0.5,2)+0.82);
                [[fallthrough]]
            case Shapes::NoseconeShapeTypes::CONICAL:
                cdpdot = conicalOgiveAdj * conicalCdpdot(mach);
                break;
            case Shapes::NoseconeShapeTypes::ELLIPSOID:
                cdpdot = ellipsoidCdpdot(mach);
                break;
            case Shapes::NoseconeShapeTypes::POWER:
                cdpdot = powerCdpdot(mach);
                break;
            case Shapes::NoseconeShapeTypes::PARABOLIC:
                cdpdot = parabolicCdpdot(mach);
                break;
            case Shapes::NoseconeShapeTypes::HAACK:
                cdpdot = haackCdpdot(mach);
                break;
            default:
                cdpdot = 0;
                break;
        }
        return cdpdot;
    }
}