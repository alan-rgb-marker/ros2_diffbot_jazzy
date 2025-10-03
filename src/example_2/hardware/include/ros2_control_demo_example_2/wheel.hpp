#ifndef WHEEL_HPP
#define WHEEL_HPP

#include <string>
#include <cmath>

class wheel
{
private:
    /* data */
public:
    std::string name = "";
    int enc = 0;
    double cmd = 0.0;
    double pos = 0.0;
    double vel = 0.0;
    double rads_per_count = 0.0;

    wheel(/* args */) = default;

    wheel(std::string &wheel_name, int counts_per_rev)
    {
        setup(wheel_name, counts_per_rev);
    }

    void setup(const std::string &wheel_name, int counts_per_rev)
    {
        name = wheel_name;
        rads_per_count = 2.0 * M_PI / counts_per_rev;
    }

    double calcEncAngle()
    {
        return enc * rads_per_count;
    }
};

#endif // WHEEL_HPP