#include "graph_search.hpp"
#include <iostream>
#include <iomanip>
#include <algorithm>


#include <stb/stb_image_write.h>

graph_search::graph_search(Environnement* environment) : env(environment) {}

void graph_search::add_start(const std::vector<float>& coords) {
    starts.push_back(coords);
}

void graph_search::add_end(const std::vector<float>& coords) {
    ends.push_back(coords);
}

void graph_search::clear_starts() { 
    starts.clear(); 
}

void graph_search::clear_ends() { 
    ends.clear(); 
}

void graph_search::reset_environment() {
    auto& point_map = env->get_map_ref();
    for (auto& pair : point_map) {
        Point& point = pair.second;
        if (!point.get_obs()) { // Ne pas toucher aux obstacles
            point.set_state(FAR);
            point.set_value(INFINITY);
            point.set_parent(nullptr);
        }
    }
}

void graph_search::execute()
{
    if (starts.empty()) {
        std::cerr << "Aucun point de départ défini!" << std::endl;
        return;
    }
    
    reset_environment();
    
    // Vider la queue
    while (!front.empty()) front.pop();
    
    // Initialiser les points de départ
    for (const auto& start_coords : starts) {
        if (env->hasPoint(start_coords)) {
            Point& start_point = env->getPoint(start_coords);
            if (!start_point.get_obs()) {
                start_point.set_value(0.0f);
                start_point.set_state(FRONT);
                front.push(&start_point);
            }
        }
    }
    
    int processed_count = 0;
    
    while (!front.empty()) {
        Point* current = front.top();
        front.pop();
        
        // Passer ce point à l'état FROZEN
        current->set_state(FROZEN);
        processed_count++;
        
        // Examiner tous les voisins
        std::vector<Point*> neighbors = env->get_neigh(*current);
        
        for (Point* neighbor : neighbors) {
            // Ignorer les obstacles et les points déjà traités
            if (neighbor->get_obs() || neighbor->get_state() == FROZEN) {
                continue;
            }
            
            // Calculer la nouvelle valeur (distance depuis le départ)
            float new_value = current->get_value() + 1.0f; // Distance euclidienne = 1 pour voisins adjacents
            
            // Si c'est un nouveau point ou si on a trouvé un chemin plus court
            if (neighbor->get_state() == FAR || new_value < neighbor->get_value()) {
                neighbor->set_value(new_value);
                neighbor->set_parent(current);
                neighbor->set_state(FRONT);
                front.push(neighbor);
            }
        }
    }
    
    std::cout << "Algorithme terminé. Points traités: " << processed_count << std::endl;
}

void graph_search::display_U_values_grid() const {
    auto dims = env->get_dims();
    if (dims.size() != 2) {
        std::cerr << "Affichage en grille disponible seulement pour les environnements 2D" << std::endl;
        return;
    }
    
    int width = dims[0];
    int height = dims[1];
    
    std::cout << "\n=== Grille des U_values ===" << std::endl;
    std::cout << "Légende: '###' = obstacle, valeurs = distance depuis le départ" << std::endl;
    
    // En-tête avec numéros de colonnes
    std::cout << "     ";
    for (int x = 0; x < width; x++) {
        if (x % 10 == 0) std::cout << std::setw(3) << x;
        else std::cout << "   ";
    }
    std::cout << std::endl;
    
    for (int y = 0; y < height; y++) {
        // Numéro de ligne
        std::cout << std::setw(3) << y << ": ";
        
        for (int x = 0; x < width; x++) {
            std::vector<float> coords = {static_cast<float>(x), static_cast<float>(y)};
            
            if (env->hasPoint(coords)) {
                const Point& point = env->getPoint(coords);
                
                if (point.get_obs()) {
                    std::cout << "###";
                } else if (point.get_value() == INFINITY) {
                    std::cout << " ∞ ";
                } else {
                    std::cout << std::setw(3) << static_cast<int>(point.get_value());
                }
            } else {
                std::cout << " ? ";
            }
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

void graph_search::save_U_values_image(const std::string& filename) const {
    auto dims = env->get_dims();
    if (dims.size() != 2) {
        std::cerr << "Sauvegarde d'image disponible seulement pour les environnements 2D" << std::endl;
        return;
    }
    
    int width = dims[0];
    int height = dims[1];
    
    // Trouver la valeur maximale pour la normalisation
    float max_value = 0.0f;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            std::vector<float> coords = {static_cast<float>(x), static_cast<float>(y)};
            if (env->hasPoint(coords)) {
                const Point& point = env->getPoint(coords);
                if (!point.get_obs() && point.get_value() != INFINITY) {
                    max_value = std::max(max_value, point.get_value());
                }
            }
        }
    }
    
    // Créer l'image
    std::vector<unsigned char> image(width * height * 3);
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int pixel_index = (y * width + x) * 3;
            std::vector<float> coords = {static_cast<float>(x), static_cast<float>(y)};
            
            if (env->hasPoint(coords)) {
                const Point& point = env->getPoint(coords);
                
                if (point.get_obs()) {
                    // Obstacle = noir
                    image[pixel_index] = 0;
                    image[pixel_index + 1] = 0;
                    image[pixel_index + 2] = 0;
                } else if (point.get_value() == INFINITY) {
                    // Non atteint = gris foncé
                    image[pixel_index] = 50;
                    image[pixel_index + 1] = 50;
                    image[pixel_index + 2] = 50;
                } else {
                    // Gradient de couleur basé sur la distance
                    float normalized = (max_value > 0) ? point.get_value() / max_value : 0.0f;
                    
                    // Palette bleu -> vert -> jaune -> rouge
                    if (normalized < 0.33f) {
                        float t = normalized / 0.33f;
                        image[pixel_index] = static_cast<unsigned char>(0 * (1-t) + 0 * t);     // R
                        image[pixel_index + 1] = static_cast<unsigned char>(0 * (1-t) + 255 * t); // G
                        image[pixel_index + 2] = static_cast<unsigned char>(255 * (1-t) + 255 * t); // B
                    } else if (normalized < 0.66f) {
                        float t = (normalized - 0.33f) / 0.33f;
                        image[pixel_index] = static_cast<unsigned char>(0 * (1-t) + 255 * t);     // R
                        image[pixel_index + 1] = static_cast<unsigned char>(255 * (1-t) + 255 * t); // G
                        image[pixel_index + 2] = static_cast<unsigned char>(255 * (1-t) + 0 * t);   // B
                    } else {
                        float t = (normalized - 0.66f) / 0.34f;
                        image[pixel_index] = static_cast<unsigned char>(255 * (1-t) + 255 * t);     // R
                        image[pixel_index + 1] = static_cast<unsigned char>(255 * (1-t) + 0 * t);   // G
                        image[pixel_index + 2] = static_cast<unsigned char>(0 * (1-t) + 0 * t);     // B
                    }
                }
            }
        }
    }
    
    // Sauvegarder
    int result = stbi_write_png(filename.c_str(), width, height, 3, image.data(), width * 3);
    if (result) {
        std::cout << "Image des U_values sauvegardée: " << filename << std::endl;
        std::cout << "Palette: Bleu (proche) -> Vert -> Jaune -> Rouge (loin)" << std::endl;
    } else {
        std::cerr << "Erreur lors de la sauvegarde: " << filename << std::endl;
    }
}