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
	num_particles = 1000;
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
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);
	
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
	for (int i = 0; i < observations.size(); ++i) { // for every observed landmark
		double nearestNeighbourDist = 1000;
		int nearestNeighbourId = 0;
		for (int j = 0; j < predicted.size(); ++j) { // find closest predicted landmark 
			double dist = sqrt(pow((observations[i].x - prediction[j].x),2) + pow((observations[i].y - prediction[j].y),2));
			if (dist < nearestNeighbourDist) {
				nearestNeighbourDist = dist;
				nearestNeighbourId = predicted[j].id;
			}
		}
		observations[i].id = nearestNeighbourId
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
	
	for (int i; i < num_particles; ++i) {
		// Extract info of particle
		int p_id = particles[i].id;
		double p_x = particles[i].x;
		double p_y = particles[i].y;
		double p_theta = particles[i].theta;

		// Select observations in range
		vector<LandmarkObs> lmInRange;
		for (int j; j < map_landmarks.landmark_list.size(); ++j) {
			double lm_x = map_landmarks.landmark_list[j].x_f;
			double lm_y = map_landmarks.landmark_list[j].y_f;
			if (dist(p_x, p_y, lm_x, lm_y) <= sensor_range)
				lmInRange.push_back(map_landmarks.landmark_list[j]);
		}
		// Transform observations from Vehicle coordinates to Map coordinates
		vector<LandmarkObs> obsOnMap;
		for (int k; k < observations.size; ++k) {
			int obs_id = observations[k].id;
			double obs_xv = observations[k].x;
			double obs_yv = observations[k].y;
			double obs_xm = obs_xv * cos(p_theta) - obs_yv * sin(p_theta) + p_x;
			double obs_ym = obs_xv * sin(p_theta) + obs_yv * cos(p_theta) + p_y;
			obsOnMap.push_back(LandmarkObs{obs_id, obs_xm, obs_ym});
		}
		
		// Associate landmarks with nearest observations
		dataAssociation(lmInRange, obsOnMap);
		
		// Resetting weight. Calculate weights.
		
		// Transform observations from MCS into VCS
		
		
		// Calculate Multivariate Gaussian distribution
		// For one particle, the product sequence of each measurement of landmark_i: exp(-0.5*(x_i - mu_i)^T * Summa^-1 * (x_i - mu_i)) / sqrt(abs(2*PI*Summa))
		
		// Summa << sigma_x^2, 0, 0, sigma_y^2;
		
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

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
