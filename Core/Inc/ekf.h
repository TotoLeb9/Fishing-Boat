/*
 * ekf.h
 *
 *  Created on: Jan 20, 2026
 *      Author: totoleb
 */

#ifndef INC_EKF_H_
#define INC_EKF_H_

#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "gps.h"
#include "gy271.h"

typedef struct {

    // ========================================================================
    // ÉTAT ESTIMÉ (Ce qu'on cherche à connaître avec précision)
    // ========================================================================

    float cap;                    // État 1 : Cap du bateau en degrés [0-360°]
                                  // 0° = Nord, 90° = Est, 180° = Sud, 270° = Ouest
                                  // C'est le RÉSULTAT PRINCIPAL du filtre
                                  // Valeur lissée et précise fusionnant boussole + GPS

    float vitesse_angulaire;      // État 2 : Vitesse de rotation en °/s
                                  // Positif = tourne à droite (tribord)
                                  // Négatif = tourne à gauche (bâbord)
                                  // Zéro = va tout droit
                                  // Permet de PRÉDIRE le prochain cap
                                  // Exemple : si ω = 5°/s, dans 1s le cap aura tourné de 5°


    // ========================================================================
    // MATRICE DE COVARIANCE P (2x2) - L'INCERTITUDE
    // ========================================================================
    // Représente "à quel point je suis sûr de mon estimation"
    // Plus P est petit, plus on est sûr
    // P diminue quand on mesure (on gagne en certitude)
    // P augmente quand on prédit (on perd en certitude avec le temps)

    float P[2][2];                // Matrice 2x2 de covariance de l'état

    // P[0][0] : VARIANCE DU CAP (en degrés²)
    //           Mesure l'incertitude sur le cap
    //           Écart-type = √P[0][0]
    //           Exemples :
    //             P[0][0] = 100  → écart-type = 10°  → Très incertain
    //             P[0][0] = 25   → écart-type = 5°   → Moyennement sûr
    //             P[0][0] = 1    → écart-type = 1°   → Très précis
    //           Interprétation : "Mon cap est X° ± √P[0][0]"

    // P[1][1] : VARIANCE DE LA VITESSE ANGULAIRE (en (°/s)²)
    //           Mesure l'incertitude sur la vitesse de rotation
    //           Exemples :
    //             P[1][1] = 16   → écart-type = 4°/s   → Très incertain
    //             P[1][1] = 4    → écart-type = 2°/s   → Moyennement sûr
    //             P[1][1] = 0.25 → écart-type = 0.5°/s → Très précis

    // P[0][1] et P[1][0] : COVARIANCE CAP-VITESSE (toujours égaux)
    //           Mesure la CORRÉLATION entre cap et vitesse
    //           C'est LA variable magique du filtre de Kalman !
    //
    //           Si P[0][1] > 0 (positif) :
    //             Quand le cap augmente, la vitesse augmente aussi
    //             "Si je suis plus à droite que prévu, c'est que je tournais plus vite"
    //
    //           Si P[0][1] < 0 (négatif) :
    //             Quand le cap augmente, la vitesse diminue
    //             (Rare dans notre cas)
    //
    //           Si P[0][1] = 0 :
    //             Cap et vitesse sont indépendants
    //             (État initial ou après beaucoup de mesures)
    //
    //           RÔLE CRUCIAL : Permet de corriger la vitesse même si
    //           le capteur ne la mesure pas directement !
    //           Exemple : si la boussole dit "cap +2° plus grand que prévu",
    //           le filtre déduit automatiquement "donc je tournais plus vite"


    // ========================================================================
    // MATRICE DE BRUIT DU PROCESSUS Q (2x2)
    // ========================================================================
    // Représente "à quel point mon modèle physique est imparfait"
    // Modélise toutes les perturbations imprévisibles :
    //   - Vent qui pousse le bateau
    //   - Vagues qui font tourner
    //   - Courants marins
    //   - Manœuvres du pilote

    float Q[2][2];                // Matrice 2x2 du bruit du processus

    // Q[0][0] : BRUIT SUR LE CAP (en degrés²)
    //           À quel point le cap peut changer de manière imprévisible
    //
    //           Valeurs typiques :
    //             Q[0][0] = 0.001  → Mer calme, pilote auto, vent nul
    //             Q[0][0] = 0.01   → Conditions normales
    //             Q[0][0] = 0.1    → Mer agitée, vent fort
    //             Q[0][0] = 1.0    → Tempête, manœuvres brusques
    //
    //           EFFET sur le filtre :
    //             Q grand → Le filtre RÉAGIT VITE aux mesures
    //                       (fait moins confiance à la prédiction)
    //             Q petit → Le filtre LISSE BEAUCOUP
    //                       (fait confiance à la prédiction)

    // Q[1][1] : BRUIT SUR LA VITESSE ANGULAIRE (en (°/s)²)
    //           À quel point la vitesse de rotation peut changer
    //
    //           Valeurs typiques :
    //             Q[1][1] = 0.1   → Vitesse stable (pilote auto)
    //             Q[1][1] = 0.5   → Conditions normales
    //             Q[1][1] = 2.0   → Manœuvres fréquentes, mer agitée
    //
    //           EFFET :
    //             Détermine la réactivité du filtre aux changements de cap

    // Q[0][1] et Q[1][0] : COVARIANCE DU BRUIT
    //           Généralement mis à ZÉRO
    //           On suppose que les perturbations sur cap et vitesse
    //           sont indépendantes
    //           (Une rafale peut changer le cap OU la vitesse, mais pas de corrélation)


    // ========================================================================
    // BRUIT DE MESURE R - PRÉCISION DES CAPTEURS
    // ========================================================================
    // Représente "à quel point mes capteurs sont bruités"
    // Se mesure expérimentalement en collectant des données

    float R_boussole;             // VARIANCE DE LA BOUSSOLE (en degrés²)
                                  // Écart-type de la mesure de cap par la boussole
                                  //
                                  // Comment le mesurer :
                                  //   1. Bateau immobile
                                  //   2. Collecter 100+ mesures de cap
                                  //   3. Calculer l'écart-type σ
                                  //   4. R_boussole = σ²
                                  //
                                  // Valeurs typiques :
                                  //   R = 1    → Boussole excellente (±1°)
                                  //   R = 9    → Boussole bonne (±3°)
                                  //   R = 25   → Boussole moyenne (±5°)
                                  //   R = 100  → Boussole médiocre (±10°)
                                  //
                                  // Facteurs qui augmentent R :
                                  //   - Interférences magnétiques (moteur, métal)
                                  //   - Inclinaison du bateau (vagues)
                                  //   - Mauvaise calibration
                                  //   - Rotation rapide (mouvement)

    float R_gps;                  // VARIANCE DU CAP GPS (en degrés²)
                                  // Écart-type de la mesure de cap par GPS
                                  //
                                  // ATTENTION : Le cap GPS n'est fiable QUE si le bateau bouge !
                                  //
                                  // R_gps dépend FORTEMENT de la vitesse :
                                  //   Vitesse < 0.5 m/s  → GPS inutilisable (R → ∞)
                                  //   Vitesse = 1 m/s    → R ≈ 100 (±10°, très bruité)
                                  //   Vitesse = 3 m/s    → R ≈ 25  (±5°, acceptable)
                                  //   Vitesse = 5 m/s    → R ≈ 9   (±3°, bon)
                                  //   Vitesse > 10 m/s   → R ≈ 4   (±2°, excellent)
                                  //
                                  // Pourquoi cette dépendance ?
                                  //   Le GPS calcule le cap à partir de la trajectoire :
                                  //     cap = atan2(Δlon, Δlat)
                                  //   À faible vitesse, Δlat et Δlon sont minuscules
                                  //   → Le bruit GPS crée de grandes erreurs d'angle
                                  //
                                  // Autres facteurs :
                                  //   - Précision GPS (HDOP, nombre de satellites)
                                  //   - Environnement (canyon urbain, forêt)


    float alpha_adaptation;       // Coefficient pour adaptation dynamique de R
                                  // Entre 0 et 1, typiquement 0.9-0.99
                                  // Permet de lisser les changements de R au fil du temps
                                  // Exemple : R_adapté = alpha × R_ancien + (1-alpha) × R_nouveau
                                  // Évite des sauts brusques de confiance dans les capteurs

    uint32_t compteur_gps;        // Nombre d'itérations depuis la dernière mise à jour GPS
                                  // Permet de détecter si le GPS est déconnecté ou stale
                                  // Si compteur > seuil (ex: 50 itérations à 50Hz = 1s)
                                  // → Augmenter Q ou P pour refléter l'incertitude croissante
                                  // → Réduire la confiance en la prédiction


    // ========================================================================
    // VARIABLES DE DIAGNOSTIC (optionnelles mais utiles)
    // ========================================================================

    float derniere_innovation;    // Dernière valeur de y (mesure - prédiction)
                                  // Utile pour détecter des anomalies :
                                  //   - Si |y| très grand → capteur défaillant ou saut brusque
                                  //   - Si y ≈ 0 toujours → capteur bloqué
                                  // Permet aussi de visualiser la performance du filtre

    float dernier_gain_cap;       // Dernière valeur de K[0]
                                  // Indique qui on croit :
                                  //   K → 0 : on croit la prédiction
                                  //   K → 1 : on croit la mesure
                                  // Permet de monitorer le comportement adaptatif

    uint32_t nb_updates_boussole; // Compteur de mises à jour boussole
    uint32_t nb_updates_gps;      // Compteur de mises à jour GPS
                                  // Statistiques pour diagnostiquer le système

    bool gps_valide;              // Flag : GPS utilisable (vitesse suffisante)
                                  // true si vitesse > seuil, false sinon

    float vitesse_gps_courante;   // Dernière vitesse GPS connue
                                  // Stockée pour adapter R_gps dynamiquement
    TickType_t timestamp;
    Compass compass;
    GPS_Struct Gps;

} KalmanCap_t;


typedef struct {
    Compass compass;            // Cap de démarrage (lu de la boussole)
    GPS_Struct gps;
    float variance_cap_init;      // P[0][0] initial, typiquement grand (50-100)
    float variance_vitesse_init;  // P[1][1] initial, typiquement moyen (5-20)
    float bruit_cap;              // Q[0][0], à ajuster selon conditions
    float bruit_vitesse;          // Q[1][1], à ajuster selon conditions
    float variance_boussole;      // R_boussole, à calibrer expérimentalement
    float variance_gps;           // R_gps de base, sera adapté selon vitesse
    float alpha_adaptation;       // Coefficient de lissage (0.9-0.99)

} KalmanConfig_t;




float NormalizeAngle180(float angle);
float NormalizeAngle360(float angle);

static inline float AngularDiff(float calculatedCap, float estimatedCap){
	return NormalizeAngle180(estimatedCap - calculatedCap);
}

static inline float KnotToMs(float knot){
	return (knot*1.852)/3.6;
}

void KalmanConfigInit(KalmanConfig_t *config, float cap_initial, float variance_cap_init, float variance_vitesse_init,
		float bruit_cap, float bruit_vitesse, float variance_boussole, float variance_gps, float alpha_adaptation);
void KalmanInitWithConfig(KalmanCap_t *kalman, const KalmanConfig_t *config);
void KalmanPredict(KalmanCap_t *kf, float dt_s);
void KalmanUpdate(KalmanCap_t *kf);
void KalmanUpdateGps(KalmanCap_t *kf);
uint8_t DirectionToGo(float actual_cap, float cap_to_go);
float NormalizeAngle(float angle);


#endif /* INC_EKF_H_ */
