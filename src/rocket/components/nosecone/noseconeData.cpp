#include "noseconeData.hpp"
#include <cmath>

namespace Rocket {
    std::tuple<double,double> powerInterpCoeffs(const double cdm0, const std::map<double, double> map) {
        auto firstElem = map.begin();
        auto secondElem = firstElem++;

        auto dcdm1 = (secondElem->second - firstElem->second)/(secondElem->first - firstElem->first); // gradient between 1st and 2nd pts
        auto b = firstElem->first * dcdm1/( firstElem->second - cdm0 );
        auto a = (firstElem->second- cdm0)/std::pow( firstElem->first,b );
        return {a,b};
    }

    const double interpNoseconeData(const double mach, const double cdm0, const std::map<double, double> map){
        if(mach == 0) return cdm0;
        auto geqBound = map.lower_bound(mach);
        auto firstElem = map.begin();
        double retVal;
        if(geqBound == map.end()){ // mach is higher than the highest data point, just use existing highest value
            retVal = map.rbegin()->second;
        }
        else if(geqBound->first == mach)// mach is exactly equal to a data point yay
        {
            retVal = geqBound->second;
        }
        else if( geqBound->first == firstElem->first)// mach is lower than the lowest data point
        {
            if(firstElem->second == 0){
                // if the Cp at the lowest data pt is 0, return 0
                retVal = 0;
            } else {
                // get values for power curve coeffs
                double mMin = firstElem->first;
                double cdpMin = firstElem->second;
                auto secondElem = firstElem++;
                double minDeriv = (secondElem->second-firstElem->second)/(secondElem->first-firstElem->first);
                // create power curve coeffs
                auto coeffs = powerInterpCoeffs(cdm0, map);
                retVal = cdm0 + std::get<0>(coeffs)*std::pow(mach, std::get<1>(coeffs));
            }
        } 
        else // mach is in range
        {
            auto lowerBound = geqBound--;
            double grad = (geqBound->second-lowerBound->second)/(geqBound->first-lowerBound->first);
            retVal = lowerBound->second + (geqBound->first - mach)*grad;
        }
        return retVal;
    }
}