#include "radian.h"
#include<cmath>

Radian::Radian(double t) : m_theta(t) {Normalize(m_theta);}

double Radian::value() const {return m_theta;}

void Radian::Normalize(double &t)
{
    while(t > M_PI) t -= 2 * M_PI;
    while(t < -M_PI) t += 2 * M_PI;
}
