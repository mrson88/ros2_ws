#ifndef DIFFDRIVE_ARDUINO_ARM_H
#define DIFFDRIVE_ARDUINO_ARM_H

#include <string>



class Arm
{
    public:

    std::string name = "";
    double pos = 0;

    Arm() = default;
    

    Arm(const std::string &arm_name, double position);


};


#endif // DIFFDRIVE_ARDUINO_WHEEL_H