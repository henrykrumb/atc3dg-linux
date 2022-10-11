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

    int sensors = tracker.get_number_sensors();

    tracker.connect();

    for (int s = 0; s < sensors; s++)
    {
        for (int i = 0; i < 10; ++i)
        {
            tracker.update(s, x, y, z, ax, ay, az, matrix, q0, qi, qj, qk, quality, button);
            std::cout << x << " " << y << " " << z << " " << ax << " " << ay << " " << az << " " << button << std::endl;
        }
    }
    tracker.disconnect();

    return 0;
}