/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// Set the number of particles
	num_particles = 100;
	particles.resize(num_particles);
	weights.resize(num_particles);

	// Create normal distributions based on GPS estimates and random Gaussian noise
	default_random_engine gen;
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	for (int i = 0; i < num_particles; ++i) {
		// Initialize all particles to first position
		particles[i].id = i;		
		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);
		// Initialize all weights to 1
		particles[i].weight = 1.0;
	}
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	default_random_engine gen;
	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);
	
	for (int i = 0; i < num_particles; ++i) {
		// Add measurements to each particle
		if ((abs(yaw_rate) > 0.0001)) {
			particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
			particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
			particles[i].theta += yaw_rate * delta_t;
		} else {
			particles[i].x += velocity * delta_t * cos(particles[i].theta);
			particles[i].y += velocity * delta_t * sin(particles[i].theta);
		}
		// Add random Gaussian noise
		particles[i].x += dist_x(gen);
		particles[i].y += dist_y(gen);
		particles[i].theta += dist_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	for (unsigned int i = 0; i < observations.size(); ++i) { // for every observed landmark
		double nearestNeighbourDist = 1000;
		int nearestNeighbourId = 0;
		for (int j = 0; j < predicted.size(); ++j) { // find closest predicted landmark
			double distance = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
			if (distance < nearestNeighbourDist) {
				nearestNeighbourDist = distance;
				nearestNeighbourId = predicted[j].id;
			}
		}
		observations[i].id = nearestNeighbourId;
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// Update the weights of each particle using a mult-variate Gaussian distribution
	for (int i = 0; i < num_particles; ++i) {
		// Extract info of particle
		double p_x = particles[i].x;
		double p_y = particles[i].y;
		double p_theta = particles[i].theta;

		// Collect observations in range
		vector<LandmarkObs> lmInRange;
		for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); ++j) {
			double lm_x = map_landmarks.landmark_list[j].x_f;
			double lm_y = map_landmarks.landmark_list[j].y_f;
			int lm_id = map_landmarks.landmark_list[j].id_i;
			if (dist(p_x, p_y, lm_x, lm_y) <= sensor_range) // select landmark if in range
				lmInRange.push_back(LandmarkObs{lm_id, lm_x, lm_y});
		}
		// Transform observations from Vehicle coordinates to Map coordinates
		vector<LandmarkObs> obsOnMap;
		for (unsigned int k = 0; k < observations.size(); ++k) {
			int obs_id = observations[k].id;
			double obs_xv = observations[k].x;
			double obs_yv = observations[k].y;
			double obs_xm = p_x + obs_xv * cos(p_theta) - obs_yv * sin(p_theta);
			double obs_ym = p_y + obs_xv * sin(p_theta) + obs_yv * cos(p_theta);
			obsOnMap.push_back(LandmarkObs{obs_id, obs_xm, obs_ym});
		}
		
		// Associate landmarks with nearest observations
		dataAssociation(lmInRange, obsOnMap);
		
		// Resetting & calculating weights
		particles[i].weight = 1.0;
		for (unsigned int k = 0; k < obsOnMap.size(); ++k) { //for each observation
			double lm_x, lm_y;
			double obs_x = obsOnMap[k].x;
			double obs_y = obsOnMap[k].y;
			for (unsigned int j = 0; j < lmInRange.size(); ++j) { //finding associated landmark
				if (lmInRange[j].id == obsOnMap[k].id) {
					lm_x = lmInRange[j].x;
					lm_y = lmInRange[j].y;
					break;
				}
			}
			// weight is the product sequence based on Bivariate Gaussian distribution
			double w = exp(-0.5*(pow((obs_x-lm_x)/std_landmark[0],2) + pow((obs_y-lm_y)/std_landmark[1],2))) /
				(2*M_PI*std_landmark[0]*std_landmark[1]);
			particles[i].weight *= w;
		}
		weights[i] = particles[i].weight;
	}
}

void ParticleFilter::resample() {
	// Resample particles with replacement with probability proportional to their weight. 
	default_random_engine generator;
	discrete_distribution<int> distribution(weights.begin(), weights.end());
	vector<Particle> resampled;
	for (int i; i < num_particles; ++i) {
		int index = distribution(generator);
		resampled.push_back(particles[index]);
	}
	particles = resampled;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
