#ifndef ENVIRONNEMENT_HPP
#define ENVIRONNEMENT_HPP

#include "Point.hpp"
#include <map>
#include <vector>
#include <string>

class Environnement
{
private:
    std::map<std::vector<float>, Point> pointMap;
    std::vector<int> dims;

public:
    Environnement();
    Environnement(const Environnement& env);
    ~Environnement() = default;

    // Setter pour les dimensions
    void set_dims(const std::vector<int>& dimensions);

    // Gestion des points
    void addPoint(const Point& point);
    Point& getPoint(const std::vector<float>& coords);
    const Point& getPoint(const std::vector<float>& coords) const;
    bool hasPoint(const std::vector<float>& coords) const;

    // Voisinage
    std::vector<Point*> get_neigh(const Point& pt);

    // Accesseurs
    std::map<std::vector<float>, Point> get_map() const { return pointMap; }
    std::map<std::vector<float>, Point>& get_map_ref() { return pointMap; }
    std::vector<int> get_dims() const { return dims; }

    // Fonctions de chargement/sauvegarde d'images
    static Environnement fromPNG(const std::string& filename, int obstacle_threshold = 128);
    void toPNG(const std::string& filename) const;

    // Fonctions utilitaires statiques pour les coordonnées n-dimensionnelles
    static std::vector<int> indexToCoordinates(long long index, const std::vector<int>& dimensions);
    static long long coordinatesToIndex(const std::vector<int>& coords, const std::vector<int>& dimensions);
    static long long calculateTotalPoints(const std::vector<int>& dimensions);

    // Création d'environnements
    static Environnement createRandomEnvironment(const std::vector<int>& dimensions, 
                                               double obstacle_probability = 0.3, 
                                               unsigned int seed = 0);
    
    static Environnement createMazeEnvironment(const std::vector<int>& dimensions, 
                                             unsigned int seed = 0);

private:
    // Fonctions utilitaires privées
    static bool isAtBorder(const std::vector<int>& coords, const std::vector<int>& dimensions);
    static bool isValidCoordinate(const std::vector<int>& coords, const std::vector<int>& dimensions);
    static std::vector<std::vector<int>> getPossibleDirections(int num_dimensions);
};

#endif // ENVIRONNEMENT_HPP