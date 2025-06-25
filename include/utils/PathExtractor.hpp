#ifndef PATHEXTRACTOR_HPP
#define PATHEXTRACTOR_HPP

#include "../Point.hpp"
#include <vector>

class PathExtractor
{
public:
    // Futur: Extraction de chemins depuis les parents
    static std::vector<Point*> extractPath(Point* goal);
    
    // Futur: Validation de chemin
    static bool isValidPath(const std::vector<Point*>& path);
    
    // Futur: Smoothing de chemin
    static std::vector<Point*> smoothPath(const std::vector<Point*>& path);
};

#endif // PATHEXTRACTOR_HPP