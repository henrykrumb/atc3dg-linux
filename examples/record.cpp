#include <iostream>

#include "atc3dg.hpp"


int main(int argc, char* argv[])
{
    ATC3DGTracker tracker;

    double x, y, z;
    double ax, ay, az;
    double matrix[9];
    double q0, qi, qj, qk;
    double quality;
    bool button;

    tracker.connect();
    for (int i = 0; i < 500; ++i)
    {
        tracker.update(0, x, y, z, ax, ay, az, matrix, q0, qi, qj, qk, quality, button);
        std::cout << x << " " << y << " " << z << " " << button << std::endl;
    }
    std::cout << tracker.get_number_sensors() << " sensors attached." << std::endl;
    tracker.disconnect();

    return 0;
}