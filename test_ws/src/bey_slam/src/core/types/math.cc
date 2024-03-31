#include"math.h"
#include<cmath>

bool equal(double a, double b, double epsilon)
{
    return std::abs(a - b) < epsilon;
}
