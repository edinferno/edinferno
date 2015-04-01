//  motion_wrapper.h
#include <vector>

//stiffness control
void wakeUp();
void rest();
void setStiffnesses(const std::vector<std::string> names, const std::vector<float> stiffnesses);
std::vector<float> getStiffnesses(const std::vector<std::string> names);

