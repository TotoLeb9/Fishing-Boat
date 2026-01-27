/*
 * ekf.c
 *
 *  Created on: Jan 20, 2026
 *      Author: totoleb
 */
#include "ekf.h"

float NormalizeAngle180(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

float NormalizeAngle360(float angle) {
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f) angle += 360.0f;
    return angle;
}



void KalmanConfigInit(KalmanConfig_t *config, float cap_initial, float variance_cap_init, float variance_vitesse_init,
		float bruit_cap, float bruit_vitesse, float variance_boussole, float variance_gps, float alpha_adaptation)
{
	config->compass.cap_boussole = cap_initial;
	config->variance_cap_init = variance_cap_init;
	config->variance_vitesse_init = variance_vitesse_init;
	config->bruit_cap = bruit_cap;
	config->bruit_vitesse = bruit_vitesse;
	config->variance_boussole = variance_boussole;
	config->variance_gps=variance_gps;
	config->alpha_adaptation = alpha_adaptation;
}

void KalmanInitWithConfig(KalmanCap_t *kalman, const KalmanConfig_t *config){
	kalman->compass.cap_boussole = config->compass.cap_boussole;
	kalman->vitesse_angulaire = 0.0;

	kalman->P[0][0] = config->variance_cap_init;
	kalman->P[0][1] = 0.0f;
	kalman->P[1][0] = 0.0f;
	kalman->P[1][1] = config->variance_vitesse_init;

	kalman->Q[0][0] = config->bruit_cap;
	kalman->Q[0][1] = 0.0f;
	kalman->Q[1][0] = 0.0f;
	kalman->Q[1][1] = config->bruit_vitesse;

	kalman->R_boussole = config->variance_boussole;
	kalman->R_gps = config->variance_gps;

	kalman->alpha_adaptation = config->alpha_adaptation;
	kalman->compteur_gps = 0;
	kalman->derniere_innovation = 0.0f;
	kalman->dernier_gain_cap = 0.0f;
	kalman->nb_updates_boussole = 0;
	kalman->nb_updates_gps = 0;
	kalman->gps_valide = false;
	kalman->vitesse_gps_courante = 0.0f;
}

/*
 * F = [1 dt]
 *	   [0  1]
 */
void KalmanPredict(KalmanCap_t *kf, float dt_s) {
    kf->cap = NormalizeAngle360(kf->cap + (kf->vitesse_angulaire * dt_s));
    float P01_new = kf->P[0][1] + dt_s * kf->P[1][1];
    kf->P[0][0] += dt_s * (kf->P[1][0] + P01_new) + kf->Q[0][0];
    kf->P[0][1] = P01_new;
    kf->P[1][0] = P01_new;
    kf->P[1][1] += kf->Q[1][1];
}

void KalmanUpdate(KalmanCap_t *kf) {
    float innovation = NormalizeAngle180(kf->compass.cap_boussole - kf->cap);
    float S = kf->P[0][0] + kf->R_boussole;
    float k_cap = kf->P[0][0] / S;
    float k_speed = kf->P[1][0] / S;
    kf->cap = NormalizeAngle360(kf->cap + (k_cap * innovation));
    kf->vitesse_angulaire = kf->vitesse_angulaire + (k_speed * innovation);
    float P00_old = kf->P[0][0];
    float P01_old = kf->P[0][1];
    kf->P[0][0] -= k_cap * P00_old;
    kf->P[0][1] -= k_cap * P01_old;
    kf->P[1][0] = kf->P[0][1];
    kf->P[1][1] -= k_speed * P01_old;
}

void KalmanUpdateGps(KalmanCap_t *kf){
	if(KnotToMs(kf->Gps.groundSpeed)<1.5)return;
	float innovation = NormalizeAngle180(kf->Gps.azimuth - kf->cap);
	float S = kf->P[0][0] + kf->R_gps;
	float k_cap = kf->P[0][0] / S;
	float k_speed = kf->P[1][0] / S;
	kf->cap = NormalizeAngle360(kf->cap + (k_cap * innovation));
	kf->vitesse_angulaire = kf->vitesse_angulaire + (k_speed * innovation);
	float P00_temp = kf->P[0][0];
	float P01_temp = kf->P[0][1];
	kf->P[0][0] -= k_cap * P00_temp;
	kf->P[0][1] -= k_cap * P01_temp;
	kf->P[1][0] = kf->P[0][1];
	kf->P[1][1] -= k_speed * P01_temp;
	kf->nb_updates_gps++;
}
