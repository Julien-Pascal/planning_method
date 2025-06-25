#ifndef COMPARISON_HPP
#define COMPARISON_HPP

#include "../Point.hpp"

struct Compare
{
    bool operator()(const Point* p1, const Point* p2) const
    {
        return *p1 > *p2; 
    }
};

#endif // COMPARISON_HPP