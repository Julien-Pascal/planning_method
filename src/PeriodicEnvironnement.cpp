#include "PeriodicEnvironnement.hpp"
#include <stdexcept>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <random>
#include <chrono>

PeriodicEnvironnement::PeriodicEnvironnement() : Environnement(), periodic_dims() {}

PeriodicEnvironnement::PeriodicEnvironnement(const std::vector<int>& dimensions, const std::vector<bool>& periodic) 
    : Environnement(), periodic_dims(periodic) 
{
    set_dims(dimensions);
    validate_periodic_dims();
}

PeriodicEnvironnement::PeriodicEnvironnement(const Environnement& env, const std::vector<bool>& periodic) 
    : Environnement(env), periodic_dims(periodic) 
{
    validate_periodic_dims();
}

void PeriodicEnvironnement::set_periodic_dims(const std::vector<bool>& periodic) {
    periodic_dims = periodic;
    validate_periodic_dims();
}

bool PeriodicEnvironnement::is_periodic(int dim_index) const {
    if (dim_index < 0 || dim_index >= static_cast<int>(periodic_dims.size())) {
        return false;
    }
    return periodic_dims[dim_index];
}

std::vector<bool> PeriodicEnvironnement::get_periodic_dims() const {
    return periodic_dims;
}

bool PeriodicEnvironnement::is_in_bounds(const std::vector<float>& coords) const {
    if (coords.size() != dims.size()) {
        return false;
    }
    
    for (size_t i = 0; i < coords.size(); ++i) {
        if (!is_periodic(i)) {
            // Pour les dimensions non-périodiques, vérifier les bornes classiques
            if (coords[i] < 0 || coords[i] >= dims[i]) {
                return false;
            }
        }
        // Pour les dimensions périodiques, toujours dans les bornes (on normalise)
    }
    return true;
}

std::vector<Point*> PeriodicEnvironnement::get_neigh(const Point& pt) {
    std::vector<Point*> neigh;
    auto coords = pt.get_coords();

    for (size_t i = 0; i < coords.size(); ++i) {
        // Voisin "inférieur" (coords[i] - 1)
        std::vector<float> neighbor_coords_lower = coords;
        neighbor_coords_lower[i] -= 1.0f;
        
        // Voisin "supérieur" (coords[i] + 1)  
        std::vector<float> neighbor_coords_upper = coords;
        neighbor_coords_upper[i] += 1.0f;
        
        // Normaliser les coordonnées selon la périodicité
        neighbor_coords_lower = normalize_coords(neighbor_coords_lower);
        neighbor_coords_upper = normalize_coords(neighbor_coords_upper);
        
        // Ajouter les voisins s'ils existent et sont dans les bornes
        if (is_in_bounds(neighbor_coords_lower) && hasPoint(neighbor_coords_lower)) {
            neigh.push_back(&getPoint(neighbor_coords_lower));
        }
        
        if (is_in_bounds(neighbor_coords_upper) && hasPoint(neighbor_coords_upper)) {
            neigh.push_back(&getPoint(neighbor_coords_upper));
        }
    }
    
    return neigh;
}

std::vector<std::pair<Point*, float>> PeriodicEnvironnement::get_hypercube_corners_with_weights(
    const std::vector<float>& query_coords) const {
    
    std::vector<std::pair<Point*, float>> corners_with_points_and_weights;

    if (!is_in_bounds(query_coords)) {
        return corners_with_points_and_weights;
    }

    const size_t num_dimensions = dims.size();
    std::vector<int> base_coords_int(num_dimensions);
    std::vector<float> fractional_parts(num_dimensions);

    // Normaliser les coordonnées d'abord
    std::vector<float> normalized_coords = normalize_coords(query_coords);

    for (size_t i = 0; i < num_dimensions; ++i) {
        base_coords_int[i] = static_cast<int>(std::floor(normalized_coords[i]));
        fractional_parts[i] = normalized_coords[i] - base_coords_int[i];

        // Gérer les cas limites
        if (base_coords_int[i] >= dims[i] - 1) {
            if (is_periodic(i)) {
                // En périodique, on peut avoir des hypercubes qui traversent les bords
                base_coords_int[i] = dims[i] - 1;
                // Garder la partie fractionnaire pour l'interpolation
            } else {
                base_coords_int[i] = dims[i] - 2;
                fractional_parts[i] = 1.0f;
            }
        }
        if (base_coords_int[i] < 0) {
            base_coords_int[i] = 0;
            fractional_parts[i] = 0.0f;
        }
    }

    int num_corners = 1 << num_dimensions;

    for (int i = 0; i < num_corners; ++i) {
        std::vector<float> corner_coords(num_dimensions);
        float current_weight = 1.0f;

        for (size_t dim_idx = 0; dim_idx < num_dimensions; ++dim_idx) {
            if ((i >> dim_idx) & 1) {
                corner_coords[dim_idx] = static_cast<float>(base_coords_int[dim_idx] + 1);
                current_weight *= fractional_parts[dim_idx];
            } else {
                corner_coords[dim_idx] = static_cast<float>(base_coords_int[dim_idx]);
                current_weight *= (1.0f - fractional_parts[dim_idx]);
            }
        }

        // Normaliser les coordonnées du coin (pour gérer le wraparound périodique)
        corner_coords = normalize_coords(corner_coords);

        if (hasPoint(corner_coords)) {
            corners_with_points_and_weights.emplace_back(
                const_cast<Point*>(&getPoint(corner_coords)), current_weight);
        }
    }

    return corners_with_points_and_weights;
}

float PeriodicEnvironnement::calculate_distance(const std::vector<float>& a, const std::vector<float>& b, int norm_type) const {
    if (a.size() != b.size() || a.size() != dims.size()) {
        throw std::invalid_argument("Les vecteurs doivent avoir la même taille que le nombre de dimensions");
    }

    float distance = 0.0f;
    
    for (size_t i = 0; i < a.size(); ++i) {
        float dim_distance = calculate_dimension_distance(a[i], b[i], dims[i], is_periodic(i));
        
        if (norm_type == 1) {
            // Norme Manhattan (L1)
            distance += dim_distance;
        } else if (norm_type == 2) {
            // Norme Euclidienne (L2)
            distance += dim_distance * dim_distance;
        } else {
            // Norme infinie (Linfini) ou autres
            distance = std::max(distance, dim_distance);
        }
    }
    
    if (norm_type == 2) {
        distance = std::sqrt(distance);
    }
    
    return distance;
}

// Factory methods
std::unique_ptr<PeriodicEnvironnement> PeriodicEnvironnement::createPeriodicRandomEnvironment(
    const std::vector<int>& dimensions,
    const std::vector<bool>& periodic,
    double obstacle_probability,
    unsigned int seed) {
    
    auto env = std::make_unique<PeriodicEnvironnement>(dimensions, periodic);
    
    // Initialiser le générateur de nombres aléatoires
    std::mt19937 generator;
    if (seed == 0) {
        auto now = std::chrono::high_resolution_clock::now();
        auto time_seed = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        generator.seed(static_cast<unsigned int>(time_seed));
    } else {
        generator.seed(seed);
    }
    
    std::uniform_real_distribution<double> distribution(0.0, 1.0);
    
    // Calculer le nombre total de points
    long long total_points = calculateTotalPoints(dimensions);
    
    // Créer tous les points
    for (long long i = 0; i < total_points; ++i) {
        std::vector<int> coords = indexToCoordinates(i, dimensions);
        std::vector<float> float_coords(coords.size());
        for (size_t j = 0; j < coords.size(); ++j) {
            float_coords[j] = static_cast<float>(coords[j]);
        }
        
        Point point(static_cast<int>(coords.size()), float_coords);
        bool is_obstacle = distribution(generator) < obstacle_probability;
        point.set_obs(is_obstacle);
        
        env->addPoint(point);
    }
    
    std::cout << "Environnement périodique aléatoire créé ";
    std::cout << "(";
    for (size_t i = 0; i < dimensions.size(); ++i) {
        std::cout << dimensions[i];
        if (periodic[i]) std::cout << "p";  // 'p' pour périodique
        if (i < dimensions.size() - 1) std::cout << "x";
    }
    std::cout << ") avec " << static_cast<int>(obstacle_probability * 100) << "% d'obstacles" << std::endl;
    
    return env;
}

std::unique_ptr<PeriodicEnvironnement> PeriodicEnvironnement::createPeriodicMazeEnvironment(
    const std::vector<int>& dimensions,
    const std::vector<bool>& periodic,
    unsigned int seed) {
    
    auto env = std::make_unique<PeriodicEnvironnement>(dimensions, periodic);
    
    // Pour l'instant, utiliser la même logique que createMazeEnvironment
    // mais il faudrait adapter l'algorithme pour tenir compte de la périodicité
    
    std::mt19937 generator;
    if (seed == 0) {
        auto now = std::chrono::high_resolution_clock::now();
        auto time_seed = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        generator.seed(static_cast<unsigned int>(time_seed));
    } else {
        generator.seed(seed);
    }
    
    long long total_points = calculateTotalPoints(dimensions);
    std::vector<bool> obstacle_grid(total_points, true);
    
    // Rendre les bordures libres (sauf si périodiques)
    for (long long i = 0; i < total_points; ++i) {
        std::vector<int> coords = indexToCoordinates(i, dimensions);
        bool is_at_border = false;
        
        for (size_t dim = 0; dim < coords.size(); ++dim) {
            if (!periodic[dim] && (coords[dim] == 0 || coords[dim] == dimensions[dim] - 1)) {
                is_at_border = true;
                break;
            }
        }
        
        if (is_at_border) {
            obstacle_grid[i] = false;
        }
    }
    
    // Créer des couloirs (logique simplifiée pour l'instant)
    std::vector<std::vector<int>> directions = getPossibleDirections(dimensions.size());
    std::uniform_int_distribution<int> dir_dist(0, directions.size() - 1);
    
    int num_paths = static_cast<int>(total_points / 10);
    
    for (int i = 0; i < num_paths; ++i) {
        std::vector<int> start_coords(dimensions.size());
        for (size_t j = 0; j < dimensions.size(); ++j) {
            start_coords[j] = generator() % dimensions[j];
        }
        
        int length = 5 + (generator() % 10);
        std::vector<int> current_coords = start_coords;
        int direction_index = dir_dist(generator);
        
        for (int j = 0; j < length; ++j) {
            if (isValidCoordinate(current_coords, dimensions)) {
                long long index = coordinatesToIndex(current_coords, dimensions);
                obstacle_grid[index] = false;
            }
            
            for (size_t k = 0; k < current_coords.size(); ++k) {
                current_coords[k] += directions[direction_index][k];
                
                // Appliquer la périodicité
                if (periodic[k]) {
                    if (current_coords[k] < 0) current_coords[k] += dimensions[k];
                    if (current_coords[k] >= dimensions[k]) current_coords[k] -= dimensions[k];
                }
            }
            
            if (generator() % 5 == 0) {
                direction_index = dir_dist(generator);
            }
        }
    }
    
    // Convertir en points
    for (long long i = 0; i < total_points; ++i) {
        std::vector<int> coords = indexToCoordinates(i, dimensions);
        std::vector<float> float_coords(coords.size());
        for (size_t j = 0; j < coords.size(); ++j) {
            float_coords[j] = static_cast<float>(coords[j]);
        }
        
        Point point(static_cast<int>(coords.size()), float_coords);
        point.set_obs(obstacle_grid[i]);
        env->addPoint(point);
    }
    
    std::cout << "Labyrinthe périodique créé ";
    std::cout << "(";
    for (size_t i = 0; i < dimensions.size(); ++i) {
        std::cout << dimensions[i];
        if (periodic[i]) std::cout << "p";
        if (i < dimensions.size() - 1) std::cout << "x";
    }
    std::cout << ")" << std::endl;
    
    return env;
}

// Méthodes utilitaires privées
float PeriodicEnvironnement::normalize_coordinate(float coord, int dim_index) const {
    if (!is_periodic(dim_index)) {
        return coord;  // Pas de normalisation pour les dimensions non-périodiques
    }
    
    int dim_size = dims[dim_index];
    
    // Utiliser fmod pour gérer les coordonnées négatives correctement
    float normalized = std::fmod(coord, static_cast<float>(dim_size));
    if (normalized < 0) {
        normalized += dim_size;
    }
    
    return normalized;
}

std::vector<float> PeriodicEnvironnement::normalize_coords(const std::vector<float>& coords) const {
    std::vector<float> normalized(coords.size());
    
    for (size_t i = 0; i < coords.size(); ++i) {
        normalized[i] = normalize_coordinate(coords[i], i);
    }
    
    return normalized;
}

float PeriodicEnvironnement::calculate_dimension_distance(float a, float b, int dim_size, bool is_periodic) const {
    if (!is_periodic) {
        return std::abs(a - b);
    }
    
    // Pour les dimensions périodiques, calculer la distance minimale
    float direct_distance = std::abs(a - b);
    float wrap_distance = dim_size - direct_distance;
    
    return std::min(direct_distance, wrap_distance);
}

void PeriodicEnvironnement::validate_periodic_dims() const {
    if (periodic_dims.size() != dims.size()) {
        throw std::invalid_argument("Le vecteur de périodicité doit avoir la même taille que le nombre de dimensions");
    }
}