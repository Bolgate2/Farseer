#pragma once
#include <memory>
#include <string>
#include <map>
#include <vector>
#include <tuple>

#include "uuid_v4/uuid_v4.h"

#include "aerodynamicCalculator.hpp"
#include "kinematicCalculator.hpp"

namespace Rocket{

// thread safe method of generating a UUID
// UUID generation needs to be made thread safe so that the ID's are guaranteed to be unique
static std::mutex compUUIDLock = std::mutex();
static UUIDv4::UUIDGenerator<std::mt19937_64> compUUIDGenerator;

// purely virtual class
class AbstractComponent : public std::enable_shared_from_this<AbstractComponent>{
    private:
        std::string _id;
        std::string generateId(){
            compUUIDLock.lock();
            auto uuid = compUUIDGenerator.getUUID();
            compUUIDLock.unlock();
            return uuid.str();
        }

    protected:
        // getters and setters
        virtual AbstractAerodynamicCalculator* getAeroCalc() = 0;

        virtual void setAeroCalc( std::unique_ptr<AbstractAerodynamicCalculator> aerodynamicCalculator ) = 0;

        virtual AbstractKinematicCalculator* getKinCalc() = 0;

        virtual void setKinCalc( std::unique_ptr<AbstractKinematicCalculator> kinematicCalculator ) = 0;

        // protected constructor so it isn't usable, create function must be used
        //virtual void init(AbstractComponent* comp) = 0;

        AbstractComponent(){ _id = generateId(); }

    public:
        std::string name = "jeff";

        std::string id(){ return _id; }
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
        virtual AbstractAerodynamicCalculator* getAeroCalc() override {return aeroCalc.get();}
        virtual void setAeroCalc( std::unique_ptr<AbstractAerodynamicCalculator> aerodynamicCalculator ) override {
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


        virtual AbstractKinematicCalculator* getKinCalc() override{ return kinCalc.get(); }
        virtual void setKinCalc( std::unique_ptr<AbstractKinematicCalculator> kinematicCalculator ) override {
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

        template<class T>
        void init(T* comp) {
            // creating the aero calc
            setAeroCalc(std::move( std::make_unique<AC>(comp) ));
            // creating the kinematic calculator
            setKinCalc(std::move( std::make_unique<KC>(comp) ));
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
            comp->init(comp);
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

using DefaultComponent = Component<std::tuple<>,AerodynamicCalculator<AbstractComponent>,KinematicCalculator<AbstractComponent>>;


// not sure why this doesnt work like above
/*
template <class T, class C>
concept aeroCalcCompType = requires (T a, C b){a(b);};
*/

}
