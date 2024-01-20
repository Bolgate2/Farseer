# ifndef COMPONENT_PROPERTY_CALCULATOR_H_
# define COMPONENT_PROPERTY_CALCULATOR_H_

#include <memory>
#include <iostream>
#include <map>
#include <unordered_map>
#include <tuple>
#include <functional>
#include <type_traits>


#include <tuple>
// function has to live in the std namespace 
// so that it is picked up by argument-dependent name lookup (ADL).
namespace std{
    namespace
    {
        // Code from boost
        // Reciprocal of the golden ratio helps spread entropy
        //     and handles duplicates.
        // See Mike Seymour in magic-numbers-in-boosthash-combine:
        //     https://stackoverflow.com/questions/4948780

        template <class T>
        inline void hash_combine(std::size_t& seed, T const& v)
        {
            seed ^= hash<T>()(v) + 0x9e3779b9 + (seed<<6) + (seed>>2);
        }

        // Recursive template code derived from Matthieu M.
        template <class Tuple, size_t Index = std::tuple_size<Tuple>::value - 1>
        struct HashValueImpl
        {
          static void apply(size_t& seed, Tuple const& tuple)
          {
            HashValueImpl<Tuple, Index-1>::apply(seed, tuple);
            hash_combine(seed, get<Index>(tuple));
          }
        };

        template <class Tuple>
        struct HashValueImpl<Tuple,0>
        {
          static void apply(size_t& seed, Tuple const& tuple)
          {
            hash_combine(seed, get<0>(tuple));
          }
        };
    }

    template <typename ... TT>
    struct hash<std::tuple<TT...>> 
    {
        size_t
        operator()(std::tuple<TT...> const& tt) const
        {                                              
            size_t seed = 0;                             
            HashValueImpl<std::tuple<TT...> >::apply(seed, tt);    
            return seed;                                 
        }                                              

    };
}

namespace Rocket{
    class Component; // forward declaration
    
    template<class T, class... Ps>
    class PropertyCalculator{
        private:
            std::weak_ptr<Component> _component;
            T (Component::*func)(Ps... args);
            std::unordered_map< std::tuple<Ps...>, T> cache;

        public:
            PropertyCalculator(T (Component::*function)(Ps...) ){
                func = function;
            }

            void clearCache(){
                cache.clear();
            }

            T calculate(Ps... args){
                auto key = std::make_tuple(args...);
                auto fnd = cache.find(key);
                if( fnd != cache.end() ){
                    std::cout << "I FOUND SOMETHING" << std::endl;
                } else {
                    cache[key] = 0;
                }
                return func(args...);
            }
    };

}

#endif