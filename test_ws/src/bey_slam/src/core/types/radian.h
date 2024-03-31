#ifndef __TYPES_RADIAN_H__
#define __TYPES_RADIAN_H__

#include<cmath>
struct Radian
{
    double m_theta;

    Radian(double t = 0.f);

    double value() const;
private:
    void Normalize(double &t);
};

#endif // __TYPES_RADIAN_H__