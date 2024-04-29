#include "RealAtmos.hpp"
#include <algorithm>
#include <cmath>
#include <map>
#include <limits>
#include <string>
#include <Eigen/Dense>

#include <iostream>

namespace RealAtmos
{
    
    // Effective radius of earth in meters. From Table 2 of US Standard Atmosphere 1976.
    const double R_0 = 6356.766e3;

    // Accelration due to gravity at sea-level in m/s^2. From Table 2 of US Standard Atmosphere 1976.
    const double G_0 = 9.80665;

    struct GEOP_CONSTS
    {
        double T_MB;
        double L_MB;
        double P_B;
    };

    // Geopotential height reference levels in meters. From Teable 4 of US Standard Atmosphere 1976.
    // Height-dependent, molecular-scale temperature at specific geopotential heights (see GEOP_HEIGHTS) in K. From Part 4, Table 1 of US Standard Atmosphere 1976.
    // Molecular-scale temperature gradient in K/km. From Table 4 of US Standard Atmosphere 1976.
    // Height-dependent, total atmospheric pressure at specific geopotential heights (see GEOP_HEIGHTS) in Pa. From Part 4, Table 1 of US Standard Atmosphere 1976.
    const std::map<double, GEOP_CONSTS> GEOPS{
        {00000.0, {288.15, -6.5, 101325.00000}},
        {11000.0, {216.65, -6.5,  22632.00000}},
        {20000.0, {216.65,  0.0,   5474.80000}},
        {32000.0, {228.65,  1.0,    868.01000}},
        {47000.0, {270.65,  2.8,    110.90000}},
        {51000.0, {270.65,  0.0,    669.38000}},
        {71000.0, {214.65, -2.8,      3.95640}},
        {84852.0, {186.95, -2.0,      0.37338}}
    };


    // Geometric height in meteres (first list) and corresponding molecular-weight ratio (second list). From Table 8 of US Standard Atmosphere 1976.
    const std::map<double, double> M_M0{
        {00000, 1.000000},
        {80000, 1.000000},
        {80500, 0.999996},
        {81000, 0.999989},
        {81500, 0.999971},
        {82000, 0.999941},
        {82500, 0.999909},
        {83000, 0.999870},
        {83500, 0.999829},
        {84000, 0.999786},
        {84500, 0.999741},
        {85000, 0.999694},
        {85500, 0.999641},
        {86000, 0.999579}
    };

    // Molecular-weight of air at sea-level in kg/kmol. From Table 10 of US Standard Atmosphere 1976.
    const double M_0 = 2.89644e+1;

    // Ideal Gas Constant in N*m/(kmol*K). From Table 2 of US Standard Atmosphere 1976.
    const double R_STAR = 8.31432e+3;

    // Avagadro's Number in kmol^-1. From Table 2 of US Standard Atmosphere 1976.
    const double N_A = 6.022169e+26;

    // Ratio of specific heat capacity of air. From Table 2 of US Standard Atmosphere 1976.
    const double GAMMA = 1.40;

    // ???
    const double BETA = 1.458e-6;

    // Sutherland's constant in K.
    const double S = 110.4;

    struct Species
    {
        double M;
        double a;
        double b;
        double alpha;
        double n;
        double Q;
        double q;
        double U;
        double u;
        double W;
        double w;
    };

    const std::map<std::string, Species> SPECIES{
        {"N2", {28.0134, -0, -0, 0, 1.129794e+20, -0, -0, -0, -0, -0, -0}},
        {"O",  {31.9988/2, 6.986e+20, 0.750, 0.00, 8.6e+16, -5.809644e-13, -3.416248e-12, 56.90311e+3, 97.0e+3, 2.706240e-14, 5.008765e-13}},
        {"O2", {31.9988, 4.863e+20, 0.750, 0.00, 3.030898426e+19, 1.366212e-13, 0, 86.000e+3, -0, 8.333333e-14, -0}},
        {"Ar", {39.948, 4.487e+20, 0.870, 0.00, 1.35140022e+18, 9.434079e-14, 0, 86.000e+3, -0, 8.333333e-14, -0}},
        {"He", {4.0026, 1.700e+21, 0.691, -0.40, 7.58173e+14, -2.457369e-13, 0, 86.000e+3, -0, 6.666667e-13, -0}},
        {"H", {2.01594/2, 3.305e+21, 0.500, -0.25, 8e+10, -0, -0, -0, -0, -0, -0}}
    };


    double interp(double x, double x0, double x1, double y0, double y1)
    {
        return y0 + (y1 - y0) * (x - x0)/(x1 - x0);
    }

    double interp(double val, std::map<double, double> map) 
    {
        auto upper = map.upper_bound(val);
        auto lower = upper--;
        return interp(val, lower->first, upper->first, lower->second, upper->second);
    }

    GEOP_CONSTS interp(double val, std::map<double, GEOP_CONSTS> map) 
    {
        auto upper = map.upper_bound(val);
        auto lower = upper;
        --lower;
        auto T_MB = interp(val, lower->first, upper->first, lower->second.T_MB, upper->second.T_MB);
        auto L_MB = interp(val, lower->first, upper->first, lower->second.L_MB, upper->second.L_MB);
        auto P_B = interp(val, lower->first, upper->first, lower->second.P_B, upper->second.P_B);
        return {T_MB, L_MB, P_B};
    }
    
    MOLE interp(double val, std::map<double, MOLE> map) 
    {
        auto upper = map.upper_bound(val);
        auto lower = upper;
        --lower;
        auto n = interp(val, lower->first, upper->first, lower->second.n, upper->second.n);
        auto M = interp(val, lower->first, upper->first, lower->second.M, upper->second.M);
        return {n, M};
    }
    
    Eigen::ArrayXd cumulative_trapezoid(Eigen::ArrayXd x, Eigen::ArrayXd y)
    {
        Eigen::ArrayXd result = Eigen::ArrayXd::Zero(x.size());

        for (auto i = 1; i < x.size(); i++) {
            result(i) = result(i - 1) + (y(i) + y(i-1))/2 * (x(i) - x(i-1));
        }

        return result;
    }

    RealAtmos* RealAtmos::pinstance_{nullptr};
    std::mutex RealAtmos::mutex_;

    RealAtmos *RealAtmos::GetInstance()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (pinstance_ == nullptr)
        {
            pinstance_ = new RealAtmos();
        }
        return pinstance_;
    }

    RealAtmos::RealAtmos()
    {
        double start = 86e3;
        double end = 1000e3;
        double stepSize = 100;
        int N = (end - start)/stepSize + 1;

        Eigen::ArrayXd altitudes = Eigen::ArrayXd::LinSpaced(N, start, end);

        Eigen::ArrayXd gravity = altitudes.unaryExpr([this](double z) { return g(z); });
        Eigen::ArrayXd T = altitudes.unaryExpr([this](double z) { return temperature(z); });
        Eigen::ArrayXd K = altitudes.unaryExpr([this](double z) { return K_(z); });
        Eigen::ArrayXd dTdZ = altitudes.unaryExpr([this](double z) { return dTdZ_(z); });

        auto M_z = [this](double z, double M) { return z < 100e3 ? M_0 : M; };

        Eigen::ArrayXd M_tot = altitudes.unaryExpr([this, M_z](double z) { return M_z(z, SPECIES.at("N2").M); });

        Eigen::ArrayXd n_N2 = SPECIES.at("N2").n * (186.8673/T) * Eigen::exp(-1 * cumulative_trapezoid(altitudes, M_tot * gravity/(R_STAR * T)));

        auto D_n = [](double temp, double a, double b, double n) { return a/n * std::pow((temp/273.15), b); };

        auto n_i = [this, altitudes, D_n, gravity, T, K, dTdZ](Species species, Eigen::ArrayXd n, Eigen::ArrayXd M) {
            Eigen::ArrayXd D = Eigen::ArrayXd::Zero(n.size());
            for (auto i = 0; i < n.size(); i++) {
                D(i) = D_n(T(i), species.a, species.b, n(i));
            }
                
            Eigen::ArrayXd f = (gravity/(R_STAR * T)) * (D/(D+K)) * (species.M + (M*K/D) + (species.alpha*R_STAR/gravity) * dTdZ);

            Eigen::ArrayXd flux = altitudes.unaryExpr([species](double z) {
                if (z > 150e3) {
                    return 0.0;
                }

                auto flux = species.Q * std::pow((z-species.U), 2) * std::exp(-1 * species.W * std::pow(z-species.U, 3));

                if (z <= 97e3 && species.q != 0) {
                    flux += species.q * std::pow(species.u-z, 2) * std::exp(-1 * species.w * std::pow(species.u-z, 3));
                }

                return flux;
            });

            Eigen::ArrayXd result = species.n * (186.8673/T) * Eigen::exp(-1* cumulative_trapezoid(altitudes, f + flux));
            return result;
        };
    
        Eigen::ArrayXd n_tot = n_N2;

        Eigen::ArrayXd n_O = n_i(SPECIES.at("O"), n_tot, M_tot);
        Eigen::ArrayXd n_O2 = n_i(SPECIES.at("O2"), n_tot, M_tot);

        n_tot += n_O + n_O2;

        M_tot = (n_N2 * SPECIES.at("N2").M + n_O * SPECIES.at("O").M + n_O2 * SPECIES.at("O2").M)/n_tot;
        for (int i = 0; i < M_tot.size(); i++) {
            M_tot(i) = M_z(altitudes(i), M_tot(i));
        }

        Eigen::ArrayXd n_Ar = n_i(SPECIES.at("Ar"), n_tot, M_tot);
        Eigen::ArrayXd n_He = n_i(SPECIES.at("He"), n_tot, M_tot);

        n_tot += n_Ar + n_He;

        auto split = std::upper_bound(altitudes.begin(), altitudes.end(), 500e3);
        split--;
        auto index = split - altitudes.begin();

        Eigen::ArrayXd tau_integrand = gravity*SPECIES.at("H").M/(R_STAR * T);

        Eigen::ArrayXd Z_1 = altitudes(Eigen::seq(index, 0, -1));
        Eigen::ArrayXd tau_1 = cumulative_trapezoid(Z_1, tau_integrand(Eigen::seq(index, 0, -1)));
        Eigen::ArrayXd D_H = Eigen::ArrayXd::Zero(index + 1);
        for (auto i = 0; i <= index; i++) {
            D_H(i) = D_n(T(index - i), SPECIES.at("H").a, SPECIES.at("H").b, n_tot(index - i));
        }
        Eigen::ArrayXd T_H_1 = T(Eigen::seq(index, 0 , -1));

        Eigen::ArrayXd integrand = 7.2e11/D_H * Eigen::pow(T_H_1/999.2356, 1+SPECIES.at("H").alpha) * Eigen::exp(tau_1);

        Eigen::ArrayXd n_H_1 = (SPECIES.at("H").n - cumulative_trapezoid(Z_1, integrand)) * Eigen::pow(999.2356/T_H_1, 1+SPECIES.at("H").alpha) * Eigen::exp(-1 * tau_1);

        Eigen::ArrayXd Z_2 = altitudes(Eigen::seq(index+1, Eigen::indexing::last, 1));
        Eigen::ArrayXd tau_2 = cumulative_trapezoid(Z_2, tau_integrand(Eigen::seq(index+1, Eigen::indexing::last, 1)));
        Eigen::ArrayXd T_H_2 = T(Eigen::seq(index+1, Eigen::indexing::last , 1));
        Eigen::ArrayXd n_H_2 = SPECIES.at("H").n * Eigen::pow(999.2356/T_H_2, 1+SPECIES.at("H").alpha) * Eigen::exp(-1 * tau_2);

        Eigen::ArrayXd n_H(n_tot.size());
        n_H << n_H_1(Eigen::seq(Eigen::indexing::last, 0, -1)), n_H_2;

        split = std::upper_bound(altitudes.begin(), altitudes.end(), 150e3);
        split--;
        index = split - altitudes.begin();

        for (int i = 0; i < index; i++) {
            n_H(i) = 0;
        }

        n_tot += n_H;

        M_tot = (n_N2 * SPECIES.at("N2").M + n_O * SPECIES.at("O").M + n_O2 * SPECIES.at("O2").M + n_Ar * SPECIES.at("Ar").M + n_He * SPECIES.at("He").M + n_H * SPECIES.at("H").M)/n_tot;
        for (int i = 0; i < M_tot.size(); i++) {
            M_tot(i) = M_z(altitudes(i), M_tot(i));
        }

        for (auto i = 0; i < altitudes.size(); i++) {
            double alt = altitudes(i);
            double n = n_tot(i);
            double M = M_tot(i);
            MOLE mole = {n, M};
            nMap.emplace(std::make_pair(alt, mole));
        }
    } 

    RealAtmos::~RealAtmos()
    {

    }


    double RealAtmos::temperature(double z)
    {
        z = std::clamp(z, -5e3, 1000e3);

        if (z <= 0) {
            auto h = H_(z) / 1000.0;
            auto geop = GEOPS.at(0.0);
            return geop.T_MB - geop.L_MB*h;
        } else if (z <= 91e3) {
            z = std::clamp(z, 0e3, 86e3);

            auto T_M = Tm_(z);
            auto M_ratio = interp(z, M_M0);

            return T_M * M_ratio;
        } else if (z <= 110e3) {
            // Constants defined by equation 27 from US Standard Atmosphere 1976
            double Tc = 263.1905;
            double A = -76.3232;
            double a = 19.9429;

            z /= 1000;

            return Tc + A* std::sqrt(1 - std::pow((z-91)/a, 2)); // Equation 27 from US Standard Atmosphere 1976
        } else if (z <= 120e3) {
            // Convert z from meters to kilometers
            z /= 1000;

            return 240 + 12 * (z - 110); //Equation 29 and 30 from US Stanbdard Atmosphere 1976 
        } else if (z <= 1000e3) {
            // Convert z from meters to kilometers
            z /= 1000;
            double r_0 = R_0 / 1000;

            // Constants defined by equation 29 from US Standard Atmosphere 1976.
            double lam = 0.01875;
            double xi = (z-120) * (r_0+120) / (r_0+z);

            return 1000 - (1000-360) * std::exp(-lam*xi);
        }
        return 1000; 
    }

    double RealAtmos::pressure(double z)
    {
        if (z > 1000e3) {
            return 0.0;
        }

        z = std::clamp(z, -5e3, 1000e3);

        if (z < 86e3) {
            auto H = H_(z);
            auto geop_map = GEOPS.upper_bound(H);
            auto H_lim = geop_map->first;
            auto geop = geop_map->second;
            if (geop.L_MB == 0) {
                return geop.P_B * std::exp((-G_0 * M_0 * (H-H_lim))/(R_STAR * geop.T_MB));
            } else {
                return geop.P_B * std::pow(geop.T_MB / (geop.T_MB + geop.L_MB * (H-H_lim)/1000), (G_0 * M_0)/(R_STAR * geop.L_MB/1000));
            }
        }

        return n_(z) * R_STAR * temperature(z) / N_A;
    }


    double RealAtmos::density(double z)
    {
        return pressure(z) * M_(z)/(R_STAR * temperature(z));
    }

    double RealAtmos::g(double z)
    {
        return G_0 * std::pow( R_0/(R_0 + z),2);
    }

    double RealAtmos::sound(double z)
    {
        z = std::clamp(z, -5e3, 86e3);
        return std::sqrt(GAMMA * R_STAR * Tm_(z)/M_0);
    }

    double RealAtmos::dynamic_viscosity(double z)
    {
        z = std::clamp(z, -5e3, 86e3);
        auto T = temperature(z);
        return BETA * std::pow(T, 1.5)/(T + S);
    }

    double RealAtmos::kinematic_viscosity(double z)
    {
        return dynamic_viscosity(z)/density(z);
    }
    
    /**
     * Calculates geopotential height(s) from given geometric height(s) using equation 19 from US Standard Atmosphere 1976.
     * 
     * @param z the geometric height in meters
     * @return The geopotential height in meters
     */
    double RealAtmos::H_(double z)
    {
        return (R_0 * z)/(R_0 + z);
    }

    /**
     * Calculates the height-dependent, molecular-scale temperature at given geometric height by interpolating values of T_M,b given by US Standard Atmosphere 1976.
     *
     * @param z the geometric height in meters
     * @return The height-dependent, molecular-scale temperature at the given geometric height in Kelvin.
     */
    double RealAtmos::Tm_(double z)
    {
        return interp(H_(z), GEOPS).T_MB;
    }

    /**
     * Calculates the height-dependent, molecular number density at the given geometric height by interpolating pre-computed values.
     *
     * @param z the geometric height in meters
     * @return The height-dependent, molecular number density at the given geometric height in m ^-3.
     */
    double RealAtmos::n_(double z)
    {
        auto mole = interp(z, nMap);
        return mole.n;
    }

    /**
     * Calculates the height-dependent, molar mass at the given geometric height by interpolating pre-computed values.
     * 
     * @param z the geometric height in meters.
     * @return The height-dependent, molar mass at the given geometric height in kg/kmol.
     */
    double RealAtmos::M_(double z)
    {
        if(z <= 86e3) return M_0;
        auto mole = interp(z, nMap);
        return mole.M;
    }

    double RealAtmos::K_(double z)
    {
        if (z < 86e3){
            return std::numeric_limits<double>::quiet_NaN();
        } else if (z < 95e3) {
            return 1.2e2;
        } else if (z < 115e3) {
            return 1.2e2 * std::exp(1- 400e6/(400e6 - std::pow(z-95e3, 2)));
        }

        return 0;
    }

    double RealAtmos::dTdZ_(double z)
    {
        if (z < 86e3) {
            return std::numeric_limits<double>::quiet_NaN();
        } else if (z < 91e3) {
            return 0;
        } else if (z < 110e3) {
            auto A = -76.3232;
            auto a = 19.9429e3;

            return -A/a * ((z-91e3)/a) / std::sqrt(1-std::pow((z-91e3)/a, 2));
        } else if (z < 120e3) {
            return 12e-3;
        } else if (z < 1000e3) {
            auto lam = 0.01875e-3;
            auto xi = (z-120e3) * (R_0+120e3) / (R_0+z);

            return lam * (1000-360) * std::pow((R_0 + 120e3)/(R_0 + z), 2) * std::exp(-lam*xi);
        }
        return 0;
    }
}
