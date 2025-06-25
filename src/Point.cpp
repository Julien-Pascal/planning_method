#include "Point.hpp"
#include <stdexcept>

Point::Point() : _dim(0), coords(), _state(FAR), U_value(INFINITY), parent(nullptr), obs(false) {}

Point::Point(int const d) : _dim(d), coords(d, 0), _state(FAR), U_value(INFINITY), parent(nullptr), obs(false) {}

Point::Point(int const d, const std::vector<float>& ds) : _dim(d), coords(ds), _state(FAR), U_value(INFINITY), parent(nullptr), obs(false)
{
    if (ds.size() < static_cast<size_t>(d))
    {
        throw std::invalid_argument("The size of the input vector is less than the number of dimension d");
    }
}

Point::Point(const Point& pt) : _dim(pt.get_dim()), coords(pt.get_coords()), _state(pt.get_state()), U_value(pt.get_value()), parent(pt.get_parent()), obs(pt.get_obs()) {}

Point& Point::operator=(const Point& pt) {
    if (this != &pt) {
        _dim = pt.get_dim();
        coords = pt.get_coords();
        _state = pt.get_state();
        U_value = pt.get_value();
        parent = pt.get_parent();
        obs = pt.get_obs();
    }
    return *this;
}