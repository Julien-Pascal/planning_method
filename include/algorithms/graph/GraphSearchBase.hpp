#ifndef GRAPHSEARCHBASE_HPP
#define GRAPHSEARCHBASE_HPP

#include "../../Point.hpp"
#include "../../Environnement.hpp"
#include "../../utils/Comparison.hpp"
#include <queue>
#include <vector>
#include <string>

/**
 * @brief Classe de base abstraite pour tous les algorithmes de recherche de graphes
 * 
 * Cette classe fournit l'infrastructure commune pour les algorithmes comme Dijkstra, A*, FMM, etc.
 * Elle gère la priority queue, les points de départ/arrivée, et les fonctions de visualisation.
 */
class GraphSearchBase
{
protected:
    Environnement* env; 
    std::priority_queue<Point*, std::vector<Point*>, Compare> front;
    std::vector<std::vector<float>> starts;
    std::vector<std::vector<float>> ends;

public:
    /**
     * @brief Constructeur
     * @param environment Pointeur vers l'environnement
     */
    explicit GraphSearchBase(Environnement* environment);
    
    /**
     * @brief Destructeur virtuel
     */
    virtual ~GraphSearchBase() = default;
    
    // Gestion des points de départ et d'arrivée
    void add_start(const std::vector<float>& coords);
    void add_end(const std::vector<float>& coords);
    void clear_starts();
    void clear_ends();
    
    // Réinitialisation
    void reset_environment();
    
    /**
     * @brief Méthode principale d'exécution de l'algorithme
     * Template method pattern - appelle les méthodes virtuelles spécialisées
     */
    void execute();
    
    // Affichage et sauvegarde des résultats (communs à tous les algorithmes)
    void display_U_values_grid() const;
    void save_U_values_image(const std::string& filename) const;

protected:
    /**
     * @brief Initialise les points de départ
     * Peut être overridée par les classes dérivées si nécessaire
     */
    virtual void initialize_starts();
    
    /**
     * @brief Calcule la nouvelle valeur U pour un voisin
     * Méthode virtuelle pure - doit être implémentée par chaque algorithme
     * @param current Point actuel
     * @param neighbor Point voisin
     * @return Nouvelle valeur U pour le voisin
     */
    virtual float calculate_new_value(const Point* current, const Point* neighbor) = 0;
    
    /**
     * @brief Détermine si l'algorithme doit continuer
     * Par défaut continue tant que la front n'est pas vide
     * Peut être overridée (ex: A* s'arrête quand il atteint le goal)
     * @return true si l'algorithme doit continuer
     */
    virtual bool should_continue() const;
    
    /**
     * @brief Traite un point de la frontière
     * Par défaut traite tous les voisins
     * Peut être overridée pour des comportements spécifiques
     * @param current Point à traiter
     */
    virtual void process_point(Point* current);
    
    /**
     * @brief Détermine si un voisin doit être mis à jour
     * Par défaut: ignore obstacles et points FROZEN
     * @param neighbor Point voisin à tester
     * @return true si le voisin peut être mis à jour
     */
    virtual bool should_update_neighbor(const Point* neighbor) const;
    
    /**
     * @brief Met à jour un voisin avec une nouvelle valeur
     * @param neighbor Point voisin à mettre à jour
     * @param new_value Nouvelle valeur U
     * @param parent Nouveau parent
     */
    virtual void update_neighbor(Point* neighbor, float new_value, Point* parent);

private:
    int processed_count; ///< Compteur de points traités
};

#endif // GRAPHSEARCHBASE_HPP