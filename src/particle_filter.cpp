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
#include <limits>

#include "particle_filter.h"
#include "transforms.hpp"

const double sigma_pos [3] = {0.3, 0.3, 0.01}; // GPS measurement uncertainty [x [m], y [m], theta [rad]]
const double sigma_landmark [2] = {0.3, 0.3}; // Landmark measurement uncertainty [x [m], y [m]]

using namespace std;

static default_random_engine gen;



void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

 const int std_x = std[0];
 const int std_y = std[1];
 const int std_theta = std[2];

 // This line creates a normal (Gaussian) distribution for x.
	normal_distribution<double> dist_x(0, std_x);
	// TODO: Create normal distributions for y and theta.
	normal_distribution<double> dist_y(0, std_y);
	normal_distribution<double> dist_theta(0, std_theta);

 cout << "Initializing PF with " << num_particles << " particles" << endl;
	for (int i= 0; i < num_particles; i++)
	{
		Particle p;
		particleCount++;
		p.x = x + dist_x(gen);
	  p.y = y +dist_y(gen);
		p.theta = theta + dist_theta(gen);
		p.weight = 1.0;
		particles.push_back(p);
	}

	is_initialized = true;


}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

  cout << "prediction" << endl;
	// [C++ syntax] auto & to be able to modify the particles
	const double v = velocity;
	const double dt = yaw_rate;

	// set up random distrib
  const int std_x = std_pos[0];
  const int std_y = std_pos[1];
  const int std_theta = std_pos[2];

   // sensor noise normal distributions
	 normal_distribution<double> dist_x(0, std_x);
	 normal_distribution<double> dist_y(0, std_y);
	 normal_distribution<double> dist_theta(0, std_theta);

	for (auto &p : particles)
	{
		if (abs(dt) < 0.0001)
		{
			// not turning case
			p.x = p.x + velocity * delta_t * cos(p.theta);
			p.y = p.y + velocity * delta_t * sin(p.theta);
		}
		else
		{
			p.x = p.x + v / dt * (sin(p.theta + dt * delta_t) - sin(p.theta));
			p.y = p.y + v / dt * (cos(p.theta) - cos(p.theta + dt * delta_t));
			p.theta = p.theta + dt * delta_t;
		}

		// add sensor noise
		p.x += dist_x(gen);
		p.y += dist_y(gen);
		p.theta += dist_theta(gen);
	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.
	// output vector saves index of observation to corresponding predicted obs
	// naive nearest-neighbor implementation

	for (int i = 0; i < observations.size(); i++)
	{
		// set dist to max
		double min_dist = std::numeric_limits<double>::max();
		int map_id = -99;

		LandmarkObs obs = observations[i];

		for (int k = 0; k < predicted.size(); k++)
		{
			LandmarkObs pred = predicted[k];

			// Calculate Euclidean distance
			double delta = EuclideanDist(pred, obs);
			if (delta < min_dist)
			{
				min_dist = delta;
				map_id = pred.id;
			}
		}

		observations[i].id = map_id;
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


	// NOTE TO UDACITY TEAM: please add this definition of "predictions" !!1!
	// Predictions: defined as landmark positions that are within sensor range !!

	// transform observations to map coords
  cout << "updateWeights" << endl;
  for (auto & p : particles)
	{
		// generate predictions based on sensor range
		std::vector<LandmarkObs> predictions;
		for (auto lm : map_landmarks.landmark_list)
		{
			if (EuclideanDist(p, lm.x_f, lm.y_f) <= sensor_range)
			{
				predictions.push_back(LandmarkObs{lm.id_i, lm.x_f, lm.y_f});
			}
		}

		// 1 transform observation to map coords based on current particle
		std::vector<LandmarkObs> observations_trans;

		for (auto o : observations)
		{
			LandmarkObs transformed = VehicleToMapWithParticle(o, p);
			//cout << "trans" << o.x << " " << o.y << " " << transformed.x << " " << transformed.y << endl;
			observations_trans.push_back(transformed);
		}

		// of no fitting observations, then continue
		if (observations_trans.size() == 0)
		{
			//continue;
		}

		// data association step
		dataAssociation(predictions, observations_trans);

		// re-set weight
		p.weight = 1.0;

		// calculate measurement probability from associations
		for (auto obs : observations_trans)
		{
			const double x_obs = obs.x;
			const double y_obs = obs.y;
			LandmarkObs pred;

			// get prediction associated with observation
			for (auto pr : predictions)
			{
				if (pr.id == obs.id)
				{
					pred.x = pr.x;
					pred.y = pr.y;
					LandmarkObs associatedPrediction = predictions[obs.id];
				}
			}

			// weight for this observation with multivariate gaussian
			const double sig_x = std_landmark[0];
			const double sig_y = std_landmark[1];

			const double g_norm = 1.0 / (2.0 * M_PI * sig_x * sig_y);
			const double expo =  pow(x_obs - pred.x, 2) / (2 * sig_x*sig_x) + pow(y_obs - pred.y, 2) / (2 * sig_y*sig_y);
			const double weight = g_norm * exp(-expo);
			p.weight *= weight;

			//cout << abs(pred.x - x_obs) << " " << abs(pred.y - y_obs) << endl;

		}

		//cout << "Final particle weight " << p.weight << endl;
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

 cout << "Resample" << endl;
	std::vector<Particle> resampled;


	std::vector<double> weights;

	double w_max = 0.0;

	// build particles array and get maximum weight
	for (auto p : particles)
	{
		const double w = p.weight;
		weights.push_back(w);
		w_max = max(w,w_max);
	}

	// random starting index
	uniform_int_distribution<int> dist(0, num_particles-1);
	int idx = dist(gen);

	// uniform real distribution with wmax
	uniform_real_distribution<double> rdist(0, w_max);

	double beta = 0.0;

	// sampling wheel
	for (int i = 0; i < num_particles; i++)
	{
		beta += rdist(gen) * 2.0;
		while (weights[idx] < beta)
		{
				beta -= weights[idx];
				idx = (idx + 1) % num_particles;
		}
		resampled.push_back(particles[idx]);
	}

	// update particles list
	particles = resampled;
}

void ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
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
