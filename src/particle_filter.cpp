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

  num_particles = 100;
  particles.reserve(num_particles);

  // Noise generator
  default_random_engine gen;
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  for (int i = 0; i< num_particles; i++){
    particles[i].id = i + 1;
    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);

    particles[i].weight = 1;
  }

  is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

  Particle particle;
  default_random_engine gen;

  for (int i = 0; i < num_particles; i++){
    particle = particles[i];

    double angle1 = (particle.theta + yaw_rate * delta_t) * 180 / M_PI;
    double angle2 = particle.theta * 180 / M_PI;

    double xf = particle.x + velocity * (sin(angle1 - angle2) - sin(angle1))/yaw_rate;
    double yf = particle.y + velocity * (cos(angle1) - cos(angle1 + angle2))/yaw_rate;
    double thetaf = particle.theta + yaw_rate * delta_t;

    // Noise generator
    normal_distribution<double> dist_x(xf, std_pos[0]);
    normal_distribution<double> dist_y(yf, std_pos[1]);
    normal_distribution<double> dist_theta(thetaf, std_pos[2]);

    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
  }

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.



}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
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

  // (1) Loop through each particle
  // (2) For each particle convert each observation coordinate from Car coordinates to map coordinates
  // (3) Based on sensor range, predict the closest landmarks for that particle
  // (4) Call dataAssociation for each observation and associate to one nearest predicted Landmark
  // (5) Take the associated landmark and calculate weight for that particle using the map observations x & y

  Particle particle;

  // (1)
  for (int i = 0; i < num_particles; i++){
    std::vector<LandmarkObs> observations_map;
    LandmarkObs observation_map;

    particle = particles[i];

    // (2)
    for (int j = 0; j< observations.size(); j++){
      LandmarkObs observation = observations[j];
      observation_map.x = particle.x + (cos(particle.theta * 180 / M_PI) * observation.x) - (sin(particle.theta * 180 / M_PI) * observation.y);
      observation_map.y = particle.y + (sin(particle.theta * 180 / M_PI) * observation.x) + (cos(particle.theta * 180 / M_PI) * observation.y);

      observations_map.push_back(observation_map);
    }

    // (3)
    std:vector<LandmarkObs> predicted;
    LandmarkObs predicted_landmark;
    default_random_engine gen;
    for (int k = 0; k < map_landmarks.landmark_list.size(); k++){
      normal_distribution<double> dist_x(map_landmarks.landmark_list[k].x_f, std_landmark[0]);
      normal_distribution<double> dist_y(map_landmarks.landmark_list[k].y_f, std_landmark[1]);

      predicted_landmark.x = dist_x(gen);
      predicted_landmark.y = dist_y(gen);

      double distance = sqrt((predicted_landmark.x - particle.x)*(predicted_landmark.x - particle.x) + (predicted_landmark.y - particle.y)*(predicted_landmark.y - particle.y));
      if (distance <= sensor_range){
        predicted.push_back(predicted_landmark);
      }
    }

    // (4)
    dataAssociation(predicted, observations_map);
  }

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

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
