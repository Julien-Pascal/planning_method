#include "Environnement.hpp"
#include <stdexcept>
#include <iostream>
#include <algorithm>
#include <random>
#include <chrono>

// Pour STB
#define STB_IMAGE_IMPLEMENTATION
#include <stb/stb_image.h>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb/stb_image_write.h>

Environnement::Environnement() : pointMap(), dims() {}

Environnement::Environnement(const Environnement& env) : pointMap(env.get_map()), dims(env.get_dims()) {}

void Environnement::set_dims(const std::vector<int>& dimensions) {
    dims = dimensions;
}

void Environnement::addPoint(const Point& point) 
{
    auto coords = point.get_coords();
    if (coords.size() == dims.size())
    {
        for (size_t i = 0; i < coords.size(); i++)
        {
            if(coords[i] >= dims[i] || coords[i] < 0)
            {
                throw std::invalid_argument("Coordinates alongside axis " + std::to_string(i) + " are out of bounds");
            }
        }
        pointMap[point.get_coords()] = point;
    }
    else
    {
        throw std::invalid_argument("The dimension of the point is different from the dimension of the environement");
    }
}

Point& Environnement::getPoint(const std::vector<float>& coords)
{
    if (hasPoint(coords)) return pointMap.at(coords);
    else throw std::out_of_range("Point with given coordinates does not exist.");
}

const Point& Environnement::getPoint(const std::vector<float>& coords) const
{
    if (hasPoint(coords)) return pointMap.at(coords);
    else throw std::out_of_range("Point with given coordinates does not exist.");
}

bool Environnement::hasPoint(const std::vector<float>& coords) const {
    return pointMap.count(coords) > 0;
}

std::vector<Point*> Environnement::get_neigh(const Point& pt)
{
    std::vector<Point*> neigh;
    auto coords = pt.get_coords();

    for (size_t i = 0; i < coords.size(); i++)
    {
        // Créer des coordonnées pour les voisins potentiels
        std::vector<float> neighbor_coords_lower = coords;
        std::vector<float> neighbor_coords_upper = coords;

        // Décrémenter la coordonnée pour obtenir le voisin "inférieur"
        neighbor_coords_lower[i] -= 1.0f;

        // Incrémenter la coordonnée pour obtenir le voisin "supérieur"
        neighbor_coords_upper[i] += 1.0f;

        auto it_lower = pointMap.find(neighbor_coords_lower);
        if (it_lower != pointMap.end())
        {
            neigh.push_back(&(it_lower->second));
        }
        else neigh.push_back(&(pointMap.find(neighbor_coords_upper)->second));

        
        auto it_upper = pointMap.find(neighbor_coords_upper);
        if (it_upper != pointMap.end())
        {
            neigh.push_back(&(it_upper->second));
        }
        else neigh.push_back(&(pointMap.find(neighbor_coords_lower)->second));
    }
    return neigh;
}

Environnement Environnement::fromPNG(const std::string& filename, int obstacle_threshold)
{
    Environnement env;
    
    // Charger l'image
    int width, height, channels;
    unsigned char* image = stbi_load(filename.c_str(), &width, &height, &channels, 0);
    
    if (!image) {
        throw std::runtime_error("Impossible de charger l'image: " + filename);
    }
    
    std::cout << "Image chargée: " << width << "x" << height << " avec " << channels << " canaux" << std::endl;
    
    // Définir les dimensions de l'environnement
    env.set_dims({width, height});
    
    // Parcourir chaque pixel de l'image
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            // Calculer l'index du pixel
            int pixel_index = (y * width + x) * channels;
            
            // Obtenir la valeur du pixel (pour image en niveaux de gris ou RGB)
            unsigned char pixel_value;
            if (channels == 1) {
                // Image en niveaux de gris
                pixel_value = image[pixel_index];
            } else {
                // Image RGB/RGBA - prendre la moyenne des canaux RGB
                int sum = image[pixel_index] + image[pixel_index + 1] + image[pixel_index + 2];
                pixel_value = sum / 3;
            }
            
            // Créer un point pour cette position
            std::vector<float> coords = {static_cast<float>(x), static_cast<float>(y)};
            Point point(2, coords);
            
            // Déterminer si c'est un obstacle
            // Convention: pixel sombre = obstacle, pixel clair = libre
            bool is_obstacle = pixel_value < obstacle_threshold;
            point.set_obs(is_obstacle);
            
            // Ajouter le point à l'environnement
            env.addPoint(point);
        }
    }
    
    // Libérer la mémoire de l'image
    stbi_image_free(image);
    
    std::cout << "Environnement créé avec " << env.pointMap.size() << " points" << std::endl;
    
    return env;
}

void Environnement::toPNG(const std::string& filename) const
{
    if (dims.size() != 2) {
        throw std::invalid_argument("Cette fonction ne fonctionne que pour des environnements 2D");
    }
    
    int width = dims[0];
    int height = dims[1];
    
    // Créer un buffer pour l'image
    std::vector<unsigned char> image(width * height * 3); // RGB
    
    // Initialiser en blanc
    std::fill(image.begin(), image.end(), 255);
    
    // Parcourir tous les points
    for (const auto& pair : pointMap) {
        const auto& coords = pair.first;
        const Point& point = pair.second;
        
        int x = static_cast<int>(coords[0]);
        int y = static_cast<int>(coords[1]);
        
        if (x >= 0 && x < width && y >= 0 && y < height) {
            int pixel_index = (y * width + x) * 3;
            
            if (point.get_obs()) {
                // Obstacle = noir
                image[pixel_index] = 0;     // R
                image[pixel_index + 1] = 0; // G
                image[pixel_index + 2] = 0; // B
            } else {
                // Libre = blanc (déjà initialisé)
                // On peut aussi marquer différemment selon l'état
                if (point.get_state() == FROZEN) {
                    image[pixel_index] = 100;     // R
                    image[pixel_index + 1] = 100; // G
                    image[pixel_index + 2] = 255; // B (bleuté)
                } else if (point.get_state() == FRONT) {
                    image[pixel_index] = 255;     // R
                    image[pixel_index + 1] = 100; // G
                    image[pixel_index + 2] = 100; // B (rougeâtre)
                }
            }
        }
    }
    
    // Sauvegarder l'image
    int result = stbi_write_png(filename.c_str(), width, height, 3, image.data(), width * 3);
    
    if (result) {
        std::cout << "Image sauvegardée avec succès: " << filename << std::endl;
    } else {
        std::cerr << "Erreur lors de la sauvegarde de l'image: " << filename << std::endl;
    }
}

// Fonctions utilitaires statiques
std::vector<int> Environnement::indexToCoordinates(long long index, const std::vector<int>& dimensions) {
    std::vector<int> coords(dimensions.size());
    
    for (int i = dimensions.size() - 1; i >= 0; i--) {
        coords[i] = index % dimensions[i];
        index /= dimensions[i];
    }
    
    return coords;
}

long long Environnement::coordinatesToIndex(const std::vector<int>& coords, const std::vector<int>& dimensions) {
    long long index = 0;
    long long multiplier = 1;
    
    for (int i = dimensions.size() - 1; i >= 0; i--) {
        index += coords[i] * multiplier;
        multiplier *= dimensions[i];
    }
    
    return index;
}

long long Environnement::calculateTotalPoints(const std::vector<int>& dimensions) {
    long long total = 1;
    for (int dim : dimensions) {
        total *= dim;
    }
    return total;
}

Environnement Environnement::createRandomEnvironment(const std::vector<int>& dimensions, 
                                                   double obstacle_probability, 
                                                   unsigned int seed)
{
    Environnement env;
    
    // Vérifier que les dimensions sont valides
    if (dimensions.empty()) {
        throw std::invalid_argument("Le vecteur de dimensions ne peut pas être vide");
    }
    
    for (int dim : dimensions) {
        if (dim <= 0) {
            throw std::invalid_argument("Toutes les dimensions doivent être positives");
        }
    }
    
    // Définir les dimensions
    env.set_dims(dimensions);
    
    // Initialiser le générateur de nombres aléatoires
    std::mt19937 generator;
    if (seed == 0) {
        auto now = std::chrono::high_resolution_clock::now();
        auto time_seed = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        generator.seed(static_cast<unsigned int>(time_seed));
    } else {
        generator.seed(seed);
    }
    
    // Distribution uniforme entre 0 et 1
    std::uniform_real_distribution<double> distribution(0.0, 1.0);
    
    // Calculer le nombre total de points
    long long total_points = calculateTotalPoints(dimensions);
    
    // Créer tous les points de l'environnement
    for (long long i = 0; i < total_points; i++) {
        // Convertir l'index en coordonnées
        std::vector<int> coords = indexToCoordinates(i, dimensions);
        
        // Convertir les coordonnées entières en float
        std::vector<float> float_coords(coords.size());
        for (size_t j = 0; j < coords.size(); j++) {
            float_coords[j] = static_cast<float>(coords[j]);
        }
        
        Point point(static_cast<int>(coords.size()), float_coords);
        
        // Déterminer aléatoirement si c'est un obstacle
        bool is_obstacle = distribution(generator) < obstacle_probability;
        point.set_obs(is_obstacle);
        
        // Ajouter le point à l'environnement
        env.addPoint(point);
    }
    
    // Afficher les informations
    std::cout << "Environnement aléatoire créé ";
    std::cout << "(";
    for (size_t i = 0; i < dimensions.size(); i++) {
        std::cout << dimensions[i];
        if (i < dimensions.size() - 1) std::cout << "x";
    }
    std::cout << ") avec " << static_cast<int>(obstacle_probability * 100) << "% d'obstacles" << std::endl;
    std::cout << "Nombre total de points: " << total_points << std::endl;
    
    return env;
}

// Fonctions utilitaires privées
bool Environnement::isAtBorder(const std::vector<int>& coords, const std::vector<int>& dimensions) {
    for (size_t i = 0; i < coords.size(); i++) {
        if (coords[i] == 0 || coords[i] == dimensions[i] - 1) {
            return true;
        }
    }
    return false;
}

bool Environnement::isValidCoordinate(const std::vector<int>& coords, const std::vector<int>& dimensions) {
    if (coords.size() != dimensions.size()) return false;
    
    for (size_t i = 0; i < coords.size(); i++) {
        if (coords[i] < 0 || coords[i] >= dimensions[i]) {
            return false;
        }
    }
    return true;
}

std::vector<std::vector<int>> Environnement::getPossibleDirections(int num_dimensions) {
    std::vector<std::vector<int>> directions;
    
    // Pour chaque dimension, on peut aller dans le sens positif ou négatif
    for (int dim = 0; dim < num_dimensions; dim++) {
        // Direction positive (+1 dans cette dimension)
        std::vector<int> pos_dir(num_dimensions, 0);
        pos_dir[dim] = 1;
        directions.push_back(pos_dir);
        
        // Direction négative (-1 dans cette dimension)
        std::vector<int> neg_dir(num_dimensions, 0);
        neg_dir[dim] = -1;
        directions.push_back(neg_dir);
    }
    
    return directions;
}

Environnement Environnement::createMazeEnvironment(const std::vector<int>& dimensions, 
                                                 unsigned int seed)
{
    Environnement env;
    
    // Vérifier que les dimensions sont valides
    if (dimensions.empty()) {
        throw std::invalid_argument("Le vecteur de dimensions ne peut pas être vide");
    }
    
    for (int dim : dimensions) {
        if (dim <= 0) {
            throw std::invalid_argument("Toutes les dimensions doivent être positives");
        }
    }
    
    env.set_dims(dimensions);
    
    // Initialiser le générateur
    std::mt19937 generator;
    if (seed == 0) {
        auto now = std::chrono::high_resolution_clock::now();
        auto time_seed = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        generator.seed(static_cast<unsigned int>(time_seed));
    } else {
        generator.seed(seed);
    }
    
    // Créer une structure pour stocker l'état des obstacles
    // Utiliser un vector pour l'efficacité (indexation directe)
    long long total_points = calculateTotalPoints(dimensions);
    std::vector<bool> obstacle_grid(total_points, true);
    
    // Rendre les bordures libres
    for (long long i = 0; i < total_points; i++) {
        std::vector<int> coords = indexToCoordinates(i, dimensions);
        if (isAtBorder(coords, dimensions)) {
            obstacle_grid[i] = false;
        }
    }
    
    // Obtenir les directions possibles
    std::vector<std::vector<int>> directions = getPossibleDirections(dimensions.size());
    
    // Créer des distributions pour les coordonnées de départ
    std::vector<std::uniform_int_distribution<int>> coord_distributions;
    for (int dim : dimensions) {
        coord_distributions.emplace_back(1, dim - 2);
    }
    
    std::uniform_int_distribution<int> dir_dist(0, directions.size() - 1);
    
    // Calculer le nombre de chemins basé sur le volume total
    long long total_volume = 1;
    for (int dim : dimensions) {
        total_volume *= dim;
    }
    int num_paths = static_cast<int>(total_volume / 10); // Ajustable selon les besoins
    
    // Créer des couloirs aléatoires
    for (int i = 0; i < num_paths; i++) {
        // Générer une coordonnée de départ aléatoire (pas sur les bordures)
        std::vector<int> start_coords(dimensions.size());
        for (size_t j = 0; j < dimensions.size(); j++) {
            start_coords[j] = coord_distributions[j](generator);
        }
        
        int length = 5 + (generator() % 10); // Longueur du chemin
        std::vector<int> current_coords = start_coords;
        int direction_index = dir_dist(generator);
        
        for (int j = 0; j < length; j++) {
            // Marquer le point actuel comme libre s'il est valide
            if (isValidCoordinate(current_coords, dimensions)) {
                long long index = coordinatesToIndex(current_coords, dimensions);
                obstacle_grid[index] = false;
            }
            
            // Avancer dans la direction actuelle
            for (size_t k = 0; k < current_coords.size(); k++) {
                current_coords[k] += directions[direction_index][k];
            }
            
            // Parfois changer de direction
            if (generator() % 5 == 0) {
                direction_index = dir_dist(generator);
            }
        }
    }
    
    // Convertir la grille en points
    for (long long i = 0; i < total_points; i++) {
        std::vector<int> coords = indexToCoordinates(i, dimensions);
        std::vector<float> float_coords(coords.size());
        for (size_t j = 0; j < coords.size(); j++) {
            float_coords[j] = static_cast<float>(coords[j]);
        }
        
        Point point(static_cast<int>(coords.size()), float_coords);
        point.set_obs(obstacle_grid[i]); // true = obstacle
        env.addPoint(point);
    }
    
    // Afficher les informations
    std::cout << "Environnement en forme de labyrinthe créé ";
    std::cout << "(";
    for (size_t i = 0; i < dimensions.size(); i++) {
        std::cout << dimensions[i];
        if (i < dimensions.size() - 1) std::cout << "x";
    }
    std::cout << ")" << std::endl;
    
    return env;
}