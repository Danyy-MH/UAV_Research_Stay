#include <eigen3/Eigen/Dense>

#ifndef UAV_H
#define UAV_H

using namespace Eigen;

// Class for the UAV
class UAV {

    public:

        // UAV constants
        float m;
        float l;
        float g;
        float Jxx;
        float Jyy;
        float Jzz;
        float kT;
        float kQ;
        float pi;

        float integral_step = 0.01;

        UAV () {

            m = 2.0;
            l = 0.25; // length m
            g = 9.81; // gravity m/s²
            Jxx = 24778420.82/(1000*1000000); // kg m²
            Jyy = 42217933.96/(1000*1000000); // kg m²
            Jzz = 23948304.24/(1000*1000000); // kg m²
            kT = 0.000006975938113;
            kQ = 0.000118290418538;
            pi = 3.141592;
            }
};
#endif 