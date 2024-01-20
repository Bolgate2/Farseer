#ifndef NOSECONE_DATA_H_
#define NOSECONE_DATA_H_

#include <map>
#include <unordered_map>
#include <tuple>

namespace Rocket{

    const std::map<double, double> vkHaackNoseconeData {
        {0.9, 0},
        {0.95, 0.01},
        {1.0, 0.027},
        {1.05, 0.055},
        {1.1, 0.070},
        {1.2, 0.081},
        {1.4, 0.095},
        {1.6, 0.097},
        {2.0, 0.091},
        {3.0, 0.083}
    };

    const std::map<double, double> lvHaackNoseconeData {
        {0.9, 0},
        {0.95, 0.01},
        {1.0, 0.024},
        {1.05, 0.066},
        {1.1, 0.084},
        {1.2, 0.1},
        {1.4, 0.114},
        {1.6, 0.117},
        {2.0, 0.113}
    };

    const std::map<double, double> ellipsoidNoseconeData {
        {1.2, 0.11},
        {1.25, 0.128},
        {1.3, 0.140},
        {1.4, 0.148},
        {1.6, 0.152},
        {2.0, 0.159},
        {2.4, 0.162}
    };

    const std::map<double, double> power14NoseconeData {
        {1.2, 0.140},
        {1.3, 0.156},
        {1.4, 0.169},
        {1.6, 0.192},
        {1.8, 0.206},
        {2.2, 0.227},
        {2.6, 0.241},
        {3.0, 0.249},
        {3.6, 0.252}
    };

    const std::map<double, double> power24NoseconeData {
        {0.925, 0},
        {0.95, 0.014},
        {1.0, 0.050},
        {1.05, 0.060},
        {1.1, 0.059},
        {1.2, 0.081},
        {1.3, 0.084},
        {1.7, 0.085},
        {2.0, 0.078}
    };

    const std::map<double, double> power34NoseconeData {
        {0.8, 0},
        {0.9, 0.015},
        {1.0, 0.078},
        {1.06, 0.121},
        {1.2, 0.110},
        {1.4, 0.098},
        {1.6, 0.090},
        {2.0, 0.084},
        {2.8, 0.078},
        {3.4, 0.074}
    };

    const std::map<double, double> parabolic1NoseconeData {
        {0.95, 0},
        {0.975, 0.016},
        {1.0, 0.041},
        {1.05, 0.092},
        {1.1, 0.109},
        {1.2, 0.119},
        {1.4, 0.113},
        {1.7, 0.108}
    };

    const std::map<double, double> parabolic12NoseconeData {
        {0.8, 0},
        {0.9, 0.016},
        {0.95, 0.042},
        {1.0, 0.100},
        {1.05, 0.126},
        {1.1, 0.125},
        {1.3, 0.100},
        {1.5, 0.090},
        {1.8, 0.088}
    };

    const std::map<double, double> parabolic34NoseconeData {
        {0.8, 0},
        {0.9, 0.016},
        {0.95, 0.042},
        {1.0, 0.100},
        {1.05, 0.126},
        {1.1, 0.125},
        {1.3, 0.100},
        {1.5, 0.090},
        {1.8, 0.088}
    };

    std::tuple<double,double> powerInterpCoeffs(const double cdm0, const std::map<double, double> map);

    const double interpNoseconeData(const double mach, const double cdm0, const std::map<double, double> map);

    enum NoseconeDataSets {
        VKHAACK,
        LVHAACK,
        ELLIPSOID,
        POWER14,
        POWER24,
        POWER34,
        PARABOLIC1,
        PARABOLIC12,
        PARABOLIC34
    };

    const std::unordered_map<NoseconeDataSets,std::map<double, double>> noseconeDataMap {
        {VKHAACK, vkHaackNoseconeData},
        {LVHAACK, lvHaackNoseconeData},
        {ELLIPSOID, ellipsoidNoseconeData},
        {POWER14, power14NoseconeData},
        {POWER24, power24NoseconeData},
        {POWER34, power34NoseconeData},
        {PARABOLIC1, parabolic1NoseconeData},
        {PARABOLIC12, parabolic12NoseconeData},
        {PARABOLIC34, parabolic34NoseconeData}
    };
}

#endif