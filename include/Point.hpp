#ifndef POINT_HPP
#define POINT_HPP

#include <vector>
#include <limits>
#include <cmath>

enum State
{
    FROZEN,
    FRONT,
    FAR,
};

class Point
{
private:
    int _dim;
    std::vector<float> coords;
    enum State _state;
    float U_value;
    Point* parent;
    bool obs;

public:
    Point();
    Point(int const d);
    Point(int const d, const std::vector<float>& ds);
    Point(const Point& pt);
    
    Point& operator=(const Point& pt);
    ~Point() = default;
    
    // Accesseurs (getters)
    int get_dim() const { return _dim; }
    std::vector<float> get_coords() const { return coords; }
    State get_state() const { return _state; }
    float get_value() const { return U_value; }
    Point* get_parent() const { return parent; }
    bool get_obs() const { return obs; }

    // Mutateurs (setters)
    void set_dim(const int& d) { _dim = d; }
    void set_coords(const std::vector<float>& ds) { coords = ds; }
    void set_state(const State s) { _state = s; }
    void set_value(const float& v) { U_value = v; }
    void set_parent(Point* pt) { parent = pt; }
    void set_obs(const bool o) { obs = o; }

    // Opérateurs de comparaison
    bool operator<(const Point& pt) const { return U_value < pt.get_value(); }
    bool operator>(const Point& pt) const { return U_value > pt.get_value(); }
    bool operator<=(const Point& pt) const { return U_value <= pt.get_value(); }
    bool operator>=(const Point& pt) const { return U_value >= pt.get_value(); }
};

#endif // POINT_HPP