#pragma once
#include <memory>
#include <string>
#include <map>
#include <vector>
#include <tuple>

#include "uuid_v4"

#include "aerodynamicCalculator.hpp"
#include "kinematicCalculator.hpp"

namespace Rocket{

// purely virtual class
class AbstractComponent : public std::enable_shared_from_this<AbstractComponent>{
    private:
        //static UUIDv4::UUIDGenerator<std::mt19937_64> uuidGenerator;

    protected:
        // getters and setters
        virtual AerodynamicCalculator* getAeroCalc() = 0;

        virtual void setAeroCalc( std::unique_ptr<AerodynamicCalculator> aerodynamicCalculator ) = 0;

        virtual KinematicCalculator* getKinCalc() = 0;

        virtual void setKinCalc( std::unique_ptr<KinematicCalculator> kinematicCalculator ) = 0;

        // protected constructor so it isn't usable, create function must be used
        virtual void init() = 0;
    public:
        std::string name = "jeff";

        // tree functions
        /*
        virtual std::shared_ptr<AbstractComponent> parent() = 0;
        virtual std::vector<std::shared_ptr<AbstractComponent>> children() = 0;

        virtual void setParent(std::shared_ptr<AbstractComponent> comp) = 0;
        virtual void addChild(std::shared_ptr<AbstractComponent> comp) = 0;

        virtual void findChild(std::shared_ptr<AbstractComponent> comp) = 0;
        virtual void findChild(std::string id) = 0;
        */
        // aero functions
        virtual double CnAlpha(double mach, double alpha) = 0;

        // kinematic functions
        virtual double mass(double time) = 0;
};

// checks that the class is a subclass of AbstractComponent
template<class T> concept IsComponent = std::is_base_of<AbstractComponent, T>::value && std::is_class<T>::value;

// generic template setup for components
// this allows for partial specializations with organized args
// this allows further extensibility and for multiple parameter packs
template<typename ... >
class Component : public AbstractComponent{};

template<typename... Children, class AC, class KC>
class Component<std::tuple<Children...>, AC, KC> : public AbstractComponent {
    private:
        std::unique_ptr<AC> aeroCalc = nullptr;
        std::unique_ptr<KC> kinCalc = nullptr;

    protected:
        virtual AerodynamicCalculator* getAeroCalc() override {return aeroCalc.get();}
        virtual void setAeroCalc( std::unique_ptr<AerodynamicCalculator> aerodynamicCalculator ) override {
            // getting the pointer to the object
            auto aeroCalcPtr = aerodynamicCalculator.get();
            // checking the type of the object
            auto castedPtr = dynamic_cast<AC*>(aeroCalcPtr);
            if(castedPtr != nullptr){
                // type is correct
                aeroCalc = std::unique_ptr<AC>(
                    dynamic_cast<AC*>(aerodynamicCalculator.release())
                );
            } else {
                // type is incorrect do some error logging
            }
            // returning the original unique ptr object, if the set was successful it will contain a null pointer, otherwise it will remain unchanged
            // return std::move(aerodynamicCalculator);
        }


        virtual KinematicCalculator* getKinCalc() override{ return kinCalc.get(); }
        virtual void setKinCalc( std::unique_ptr<KinematicCalculator> kinematicCalculator ) override {
            // getting the pointer to the object
            auto kinCalcPtr = kinematicCalculator.get();
            // checking the type of the object
            auto castedPtr = dynamic_cast<KC*>(kinCalcPtr);
            if(castedPtr != nullptr){
                // type is correct
                kinCalc = std::unique_ptr<KC>(
                    dynamic_cast<KC*>(kinematicCalculator.release())
                );
            } else {
                // type is incorrect do some error logging
            }
            // returning the original unique ptr object, if the set was successful it will contain a null pointer
            // return std::move(kinematicCalculator);
        }

        Component(){};

        virtual void init() override {
            // creating the aero calc
            setAeroCalc(std::move( std::make_unique<AC>(this) ));
            // creating the kinematic calculator
            setKinCalc(std::move( std::make_unique<KC>(this) ));
        }
    public:
        // tree functions

        // aero functions
        virtual double CnAlpha(double mach, double alpha) override {
            return getAeroCalc()->CnAlpha(mach, alpha);
        }

        // kinematic functions
        virtual double mass(double time) override {
            return getKinCalc()->mass(time);
        }

        friend class ComponentConstructor;
};

// checks that args are the same as the constructor args of class T
// this is buggy with unique pointers and probs will be with other data structures
/*
template <class T, class ... args>
concept constructargs = requires (T a, args... b){a(b...);};
*/

class ComponentConstructor{
    public:
        template<IsComponent T, typename... Args>
        T* create(Args... args){
            T* comp = new T(
                    std::move(args)...
                );
            comp->init();
            return comp;
        }
};

template<IsComponent T, typename... Args>
T* create(Args... args){
    auto constructor = ComponentConstructor();
    return constructor.create<T>(
            std::move(args)...
        );
}

using DefaultComponent = Component<std::tuple<>,AerodynamicCalculator,KinematicCalculator>;


// not sure why this doesnt work like above
/*
template <class T, class C>
concept aeroCalcCompType = requires (T a, C b){a(b);};
*/

}
