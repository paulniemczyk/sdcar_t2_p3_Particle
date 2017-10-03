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
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	/*** My code below ***/

	// std is std deviation array of uncertanities for gps
	// x,y,theta are x,y,bearing per GPS
	// sample from Gaussian distribution centered on these measurements to init all particles

	if (is_initialized) {		// Skip this method if init already
		return;
	}
	
	num_particles = 100;		// Try this. Initially set to 0 in constructor
	
	
	// Add some Gaussian noise to x, y, and theta so that initial particle setup is a little random
	// from the initial GPS coordinate

	double std_x, std_y, std_theta;
	std_x = std[0];											// std[] = {x,y,theta}
	std_y = std[1];
	std_theta = std[2];

	default_random_engine gen;								// Set random generator gen
	normal_distribution<double> dist_x(x, std_x);			// Create normal dist 
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);
	
	/* double sample_x, sample_y, sample_theta; */

	// Set up each particle
	for (int i=0; i<num_particles; i++) {					
		
		Particle particle;
		particle.id = i;
		particle.x = dist_x(gen);
		particle.y = dist_y(gen);
		particle.theta = dist_theta(gen);
		particle.weight = 1;						// Initially set all weights to 1

		particles.push_back(particle);
		weights.push_back(1);						// Initially set all weights to 1
		
	}

	is_initialized = true;	

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/


	/*** My code below ***/
	
	// Calculate predicted position using prior velocity and yaw rate, with time step delta_t
	// for each particle (velocity & yaw_rate are prior vel & yaw)
	// Note: add some noise (jitter) to the control so that when the partice is resampled, you don't end up
	// with all the particles doing the exact same thing, b/c they they're all doing the same thing slightly wrong.



	double std_x, std_y, std_theta;
	std_x = std_pos[0];				// std[] = {x,y,theta}
	std_y = std_pos[1];
	std_theta = std_pos[2];

	default_random_engine gen;	// Set random generator gen

	for (int i=0; i<num_particles; i++) {

		double new_x, new_y, new_theta;			// Temp hold of new state
		double x, y, theta;						// Hold last (current) state state, prior to applying these controls

		x = particles[i].x;						
		y = particles[i].y;
		theta = particles[i].theta;

		if (abs(yaw_rate) < 0.001) {			// Equations for when yaw_rate is approx 0
			
			new_x = x + velocity * delta_t * cos(theta);
			new_y = y + velocity * delta_t * sin(theta);
			new_theta = theta; 					// theta doesn't change when yaw rate ~ 0


		} else {						// Motion equations with constant yaw (when yaw_rate > 0)

			// Update x, y, theta to new state predictions
			new_x = x + (velocity/yaw_rate) * (sin(theta + yaw_rate*delta_t) - sin(theta));
			new_y = y + (velocity/yaw_rate) * (cos(theta) - cos(theta + yaw_rate*delta_t));
			new_theta = theta + yaw_rate*delta_t;
		}

		
		// Add some jitter to newly calculated x, y, theta
		normal_distribution<double> dist_x(new_x, std_x);			// Create normal dist 
		normal_distribution<double> dist_y(new_y, std_y);
		normal_distribution<double> dist_theta(new_theta, std_theta);

		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);

	}	


}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular lanedmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.


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


	/* Note from Lesson 14 Sec 14 
	"The objective is to use the particle coordinates and heading to transform the car's frame of reference 
	to the map's frame of reference, associate the observations, then use the associated observations in 
	the map domain to update the particle weight."
	i.e., transform particle (which represents car) observations to map coordinates
	Remember that we don't know where the car is, just the particle that approximates the car.
	Particle x & y are maintained in map coords
	*/

	/*** My code below ***/

	// Objective is to determine difference between measurement (observation) and actual landmark it's supposed to measure
	
	// std_landmark is the noise in the car's sensor measurement to an observed landmark
	// observations are the car's measurements of the detected landmarks
	// map_landmarks are the ground truth map of all landmarks

	// Transform observed measurements from car/particle frame to map frame
	// 		x_m = x_p + (cos(theta) * x_c) - (sin(theta) * y_c)
	// 		y_m = y_p + (sin(theta) * x_c) + (cos(theta) * y_c)
	// 		where:
	// 		x_p, y_p = position of particle in map frame
	// 		x_c, y_c = observations (in particle/car frame)	
	// 		theta = particle theta
	// 		where particle x is direction of travel; and map x is to the right


	
	double sig_x = std_landmark[0];
	double sig_y = std_landmark[1];

	// Loop through each particle, transform observations, associate to landmark, and set particle weight

	for (int p=0; p<particles.size(); p++) {
			
		std::vector<int> associations;						// Going to set these for each particle		
		std::vector<double> sense_x;
		std::vector<double> sense_y;
		
		particles[p].weight = 1.0;							// Init particle weight
		
		/****************************** 
		* Translate observations into map frame
		*******************************/
		// Translate each observation into the map frame, from the perspective of the particle

		std::vector<LandmarkObs> trans_observations;		// Contains all the translated particle observations
		LandmarkObs temp_obs;								// Temp container

		double x_p, y_p, theta_p, x_c, y_c, x_m, y_m;		// Coords used in calculation
		x_p = particles[p].x;
		y_p = particles[p].y;
		theta_p = particles[p].theta;

		// Translate each observation
		for (int i=0; i<observations.size(); i++) {

			x_c = observations[i].x;
			y_c = observations[i].y;
			x_m = x_p + (cos(theta_p) * x_c) - (sin(theta_p) * y_c);
			y_m = y_p + (sin(theta_p) * x_c) + (cos(theta_p) * y_c);
			temp_obs.x = x_m;
			temp_obs.y = y_m;

			trans_observations.push_back(temp_obs);
		}

		
		// Loop through each translated observation to associate & calc weight of particle
		for (int i=0; i<trans_observations.size(); i++) {	 

			/****************************** 
			* Data association using nearest neighbor
			*******************************/
			// For each transformed observation, find the closest landmark and set
			// the index of the landmark object to association for observation

			double closest_dist = sensor_range;					// Inititialize 
			int association = 0;

			// Loop thru each landmark to find closest landmark to observation
			for (int j=0; j<map_landmarks.landmark_list.size(); j++) {

				
				double landmark_x = map_landmarks.landmark_list[j].x_f;
				double landmark_y = map_landmarks.landmark_list[j].y_f;

				double calc_dist = sqrt(pow(trans_observations[i].x-landmark_x,2.0)+pow(trans_observations[i].y-landmark_y,2.0));
				
				if (calc_dist < closest_dist) {
					
					closest_dist = calc_dist;
					association = j;						// The association is the index of the landmark
				}

			} // end j loop

			/****************************** 
			* Set weights
			*******************************/

			if (association != 0) {								// Check that there was an association

				double x_obs_m = trans_observations[i].x;		// my x_obs_m (observation translated into map frame)
				double y_obs_m = trans_observations[i].y;		// my y_obs_m
				double mu_x = map_landmarks.landmark_list[association].x_f;
				double mu_y = map_landmarks.landmark_list[association].y_f;
				double gauss_norm, exponent;

				gauss_norm = 1 / ( 2 * M_PI * sig_x * sig_y);
				exponent = ( (x_obs_m - mu_x)*(x_obs_m - mu_x) / (2 * sig_x * sig_x) ) + ( (y_obs_m-mu_y)*(y_obs_m-mu_y)/(2*sig_y*sig_y) );
				double multiplier = gauss_norm * exp(-1 * exponent);

				if (multiplier > 0) {
					particles[p].weight *= multiplier;
				}
			}
			associations.push_back(association + 1);			// Not sure about this... worked only with +1!
			sense_x.push_back(trans_observations[i].x);
			sense_y.push_back(trans_observations[i].y);

		} // end i loop


		particles[p] = SetAssociations(particles[p], associations, sense_x, sense_y);
		weights[p] = particles[p].weight;

	} // End p loop
}




void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution


	/*** My code below ***/
	// Note: I borrowed the core concept from the youtube help video for this project
	// Link:  https://www.youtube.com/watch?v=-3HI3Iw3Z9g&index=3&list=PLAwxTw4SYaPnfR7TzRZN-uxlxGbqxhtm2&pbjreload=10
	// This method doesn't appear to reflect Prof Thrun's approach to resmapling, which involves
	// going around the wheel. (See lessons for his implementation.)

	default_random_engine gen;
	discrete_distribution<int> distribution(weights.begin(), weights.end());

	vector<Particle> resample_particles;

	for (int i=0; i < num_particles; i++) {

		resample_particles.push_back(particles[distribution(gen)]);

	}

	particles = resample_particles;


}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
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
