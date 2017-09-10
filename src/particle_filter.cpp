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
#include <array>

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

  //std::cout << "prediction()" << std::endl;

  Particle particle;
  default_random_engine gen;

  for (int i = 0; i < num_particles; i++){
    particle = particles[i];

    // check if yaw_rate is zero or close to zero

    double xf, yf, thetaf;

    if (yaw_rate <= 0.0001 && yaw_rate >= -0.0001 ){
      xf = particle.x + velocity * cos(particle.theta) * delta_t;
      yf = particle.y + velocity * sin(particle.theta) * delta_t;
      thetaf = particle.theta;
    } else {
      double angle1 = particle.theta + yaw_rate * delta_t;
      double angle2 = particle.theta;

      xf = particle.x + velocity * (sin(angle1) - sin(angle2))/yaw_rate;
      yf = particle.y + velocity * (cos(angle2) - cos(angle1))/yaw_rate;
      thetaf = particle.theta + yaw_rate * delta_t;
    }

    // Noise generator
    normal_distribution<double> dist_x(xf, std_pos[0]);
    normal_distribution<double> dist_y(yf, std_pos[1]);
    normal_distribution<double> dist_theta(thetaf, std_pos[2]);

    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);

    //std::cout << "weight" << i << " " << particles[i].weight << std::endl;

  }

  //std::cout << "FInished prediction" << std::endl;
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

  for (int i = 0; i < observations.size(); i++){
    //std::cout << "Inside dataAssociation()" << std::endl;

    double closest_distance = 999999999999;
    for (int j = 0; j < predicted.size(); j++){
      double distance = sqrt((predicted[j].x - observations[i].x)*(predicted[j].x - observations[i].x) + (predicted[j].y - observations[i].y)*(predicted[j].y - observations[i].y));

      //std::cout << "Distance " << distance << std::endl;

      if (distance <= closest_distance){
        observations[i].id = predicted[j].id;
        closest_distance = distance;
      }
    }
    //std::cout << "Closest Distance " << closest_distance << std::endl;
  }

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
    std::vector<LandmarkObs> observations_map(observations.size());

    particle = particles[i];

    // (2)
    for (int j = 0; j< observations.size(); j++){
      LandmarkObs observation_map;
      LandmarkObs observation = observations[j];
      observation_map.x = particle.x + (cos(particle.theta) * observation.x) - (sin(particle.theta) * observation.y);
      observation_map.y = particle.y + (sin(particle.theta) * observation.x) + (cos(particle.theta) * observation.y);

      observations_map[j] = observation_map;
    }

    // (3)
    std:vector<LandmarkObs> predicted;
    default_random_engine gen;
    int landmark_index = 0;
    for (int k = 0; k < map_landmarks.landmark_list.size(); k++){
      LandmarkObs predicted_landmark;
      normal_distribution<double> dist_x(map_landmarks.landmark_list[k].x_f, std_landmark[0]);
      normal_distribution<double> dist_y(map_landmarks.landmark_list[k].y_f, std_landmark[1]);

      predicted_landmark.x = dist_x(gen);
      predicted_landmark.y = dist_y(gen);

      double distance = sqrt((predicted_landmark.x - particle.x)*(predicted_landmark.x - particle.x) + (predicted_landmark.y - particle.y)*(predicted_landmark.y - particle.y));
      if (distance <= sensor_range){
        predicted_landmark.id = landmark_index;
        landmark_index++;
        predicted.push_back(predicted_landmark);
        //std::cout << "Predicted landmark (x,y) " << predicted_landmark.x << " " << predicted_landmark.y << std::endl;
      }
    }

    // (4)
    dataAssociation(predicted, observations_map);

    // (5)
    double particle_weight = 1;
    for (int l = 0; l < observations_map.size(); l++) {
      LandmarkObs observation = observations_map[l];

      //std::cout << "Observation Predicted association id" << observation.id << std::endl;
      //std::cout << "Observation x,y " << observation.x << "," << observation.y << std::endl;
      //std::cout << "Predicted x,y " << predicted[observation.id].x << "," << predicted[observation.id].y << std::endl;

      // Calculate normalization term
      double guass_norm = (1/(2 * M_PI * std_landmark[0] * std_landmark[1]));

      //std::cout << "guass_norm:" << guass_norm;

      // Calculate exponent
      double exponent = ((observation.x - predicted[observation.id].x)*(observation.x - predicted[observation.id].x)/(2 * std_landmark[0] * std_landmark[0])) + ((observation.y - predicted[observation.id].y)*(observation.y - predicted[observation.id].y)/(2 * std_landmark[1] * std_landmark[1]));

      //std::cout << " exponent:" << exponent;

      // Calculate weight using normalization terms and exponent
      double weight = guass_norm * exp(-exponent);

      //std::cout << " weight:" << weight << std::endl;

      // If the weight is zero then don't multiply
      //if (weight != 0){
        //std::cout << "multiplying" << std::endl;
        particle_weight *= weight;
      //}
    }

    particles[i].weight = particle_weight;

    //std::cout << "Particle " << i << " weight " << particle_weight << std::endl;

  }

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  //std::cout << "resample()" << std::endl;

  // Normalize weights

  std::vector<double> jweights(num_particles);

  /*
  double total_weight = 0;
  for (int i =0; i < num_particles; i++){
    total_weight += particles[i].weight;
  }
  //std::cout << "Total Weight " << total_weight << std::endl;

  for (int n=0; n < num_particles; n++){
    weights[n] = particles[n].weight / total_weight;
  }
   */

  //std::cout << "weight " << std::endl;
  for (int n=0; n < num_particles; n++){
    jweights[n] = particles[n].weight;
    //std::cout << particles[n].weight << " ";
  }
  //std::cout << std::endl;

  default_random_engine gen;

  // Create the distribution with those weights
  std::discrete_distribution<> d (jweights.begin(), jweights.end());

  //std::cout << "distribution: " << d << std::endl;

  std::vector<Particle> particles_resampled(num_particles);
  //particles_resampled.reserve(num_particles);

  //std::cout << "numbers picked: ";
  for (int i =0; i< num_particles; ++i){
    int number = d(gen);
    //std::cout << number << " w:" << particles[number].weight << " ";
    particles_resampled[i] = particles[number];
  }
  //std::cout << std::endl;

  /*
  std::cout << "weights resampled " << std::endl;
  for (int n=0; n < num_particles; n++){
    std::cout << particles_resampled[n].weight << " ";
  }
  std::cout << std::endl;
  */

  particles = particles_resampled;

  /*
  std::cout << "weights final " << std::endl;
  for (int n=0; n < num_particles; n++){
    std::cout << particles[n].weight << " ";
  }
  std::cout << std::endl;
  */

  //std::cout << "FInished resample()" << std::endl;

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
