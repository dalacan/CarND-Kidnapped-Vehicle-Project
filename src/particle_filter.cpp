/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1.
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method
   *   (and others in this file).
   */
  num_particles = 500;  // TODO: Set the number of particles

  // Initialize variables required to add gaussian noise to each particle
  std::default_random_engine gen;

  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  // Initialize the required number of particles to the particles variable
  for(int i=0; i<num_particles; i++) {
    Particle p;
    p.id = i;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    p.weight = 1.0;
    particles.push_back(p);
  }

  // Initialize size of weights to number of particles
  weights.resize(num_particles);

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  double theta;

  // Initialize variables required to add gaussian noise using on the measurement's standard deviation
  // (for the x, y and theta) to each particle during prediction
  std::default_random_engine gen;
  normal_distribution<double> dist_x_noise(0, std_pos[0]);
  normal_distribution<double> dist_y_noise(0, std_pos[1]);
  normal_distribution<double> dist_theta_noise(0, std_pos[2]);


  for(unsigned int i=0; i<particles.size(); i++) {
      theta = particles[i].theta;

      // If the yaw rate is very small (<0.00001) calculate the new x and y coordinates assuming yaw rate is 0
      if (fabs(yaw_rate) < 0.00001) {
          particles[i].x += velocity * delta_t * cos(theta);
          particles[i].y += velocity * delta_t * sin(theta);
      } else {
          // Calculate the new x and y coordinates taking the yaw rate into consideration
          particles[i].x += velocity / yaw_rate * (sin((theta + yaw_rate * delta_t)) - sin(theta));
          particles[i].y += velocity / yaw_rate * (cos(theta) - cos((theta + yaw_rate * delta_t)) );
          particles[i].theta += yaw_rate * delta_t;
     }

      // Add gaussian noise
      particles[i].x += dist_x_noise(gen);
      particles[i].y += dist_y_noise(gen);
      particles[i].theta += dist_theta_noise(gen);
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each
   *   observed measurement and assign the observed measurement to this
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will
   *   probably find it useful to implement this method and use it as a helper
   *   during the updateWeights phase.
   */

   double closest_landmark_distance = std::numeric_limits<double>::max();;

   for(unsigned int i=0; i<observations.size(); i++) {
     int closest_landmark_id = -1;

     for(unsigned int j=0; j<predicted.size(); j++) {
       double distance = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
       if(distance < closest_landmark_distance) {
           closest_landmark_distance = distance;
           closest_landmark_id = predicted[j].id;
       }
     }

     observations[i].id =closest_landmark_id;
   }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian
   *   distribution. You can read more about this distribution here:
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system.
   *   Your particles are located according to the MAP'S coordinate system.
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  for(unsigned int i=0; i<particles.size(); i++) {
    vector<LandmarkObs> observations_map;
    vector<LandmarkObs> predicted;
    double particle_weight= 1.0;
    std::vector<int> particle_associations;
    std::vector<double> particle_assoc_sense_x;
    std::vector<double> particle_assoc_sense_y;

    // Transform observation to map coordinates
    for(unsigned int j = 0; j < observations.size(); j++) {
      LandmarkObs observation_map;
      observation_map.x = particles[i].x + observations[j].x * cos(particles[i].theta) - observations[j].y * sin(particles[i].theta);
      observation_map.y = particles[i].y + observations[j].x * sin(particles[i].theta) + observations[j].y * cos(particles[i].theta);

      observation_map.id = observations[j].id;
      observations_map.push_back(observation_map);
    }

    // Predict landmarks by getting landmarks within sensor range of particle
    for(unsigned int k=0; k<map_landmarks.landmark_list.size(); k++) {
        if (fabs(map_landmarks.landmark_list[k].x_f - particles[i].x)<=sensor_range && fabs(map_landmarks.landmark_list[k].y_f - particles[i].y)<=sensor_range) {
        LandmarkObs predicted_landmark;
        predicted_landmark.x = map_landmarks.landmark_list[k].x_f;
        predicted_landmark.y = map_landmarks.landmark_list[k].y_f;
        predicted_landmark.id = map_landmarks.landmark_list[k].id_i;
        predicted.push_back(predicted_landmark);
      }
    }

    // Find closest landmarks for each observation
    dataAssociation(predicted, observations_map);

    // Calculate weights
    for(unsigned int m=0;m<observations_map.size(); m++) {
      double landmark_x;
      double landmark_y;
      double weight;
      bool found = false;
      // Get associating landmark
      for(unsigned int n=0; n<map_landmarks.landmark_list.size(); n++) {
        if(observations_map[m].id == map_landmarks.landmark_list[n].id_i) {
          landmark_x = map_landmarks.landmark_list[n].x_f;
          landmark_y = map_landmarks.landmark_list[n].y_f;
          found = true;
          break;
        }
      }

      // If landmark is found, set weight, else set weight to a very low probability (i.e. 0.00001)
      if(found) {
          weight = multiv_prob(std_landmark[0], std_landmark[1], observations_map[m].x, observations_map[m].y, landmark_x, landmark_y);
          particle_associations.push_back(observations_map[m].id);
          particle_assoc_sense_x.push_back(landmark_x);
          particle_assoc_sense_y.push_back(landmark_y);
      } else {
          weight = 0.00001;
      }

      // Multiple the weights
      particle_weight *= weight;
    }

    // Update weight, associated landmark and respective x & y map coordinates for each particle
    particles[i].weight = particle_weight;
    particles[i].associations = particle_associations;
    particles[i].sense_x = particle_assoc_sense_x;
    particles[i].sense_y = particle_assoc_sense_y;

    // Populate list of particles weights to be used with the discrete distribution in the re-sampling function
    weights[i] = particle_weight;
  }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional
   *   to their weight.
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  std::random_device rd;  //Will be used to obtain a seed for the random number engine
  std::mt19937 gen(rd());
  std::uniform_int_distribution<int> dist_particle_id {0,num_particles-1};

  std::vector<Particle> resampled_particles;

  // Use the discrete distribution to generate a random particle index base on the weights
  std::discrete_distribution<> dist_particle(weights.begin(),weights.end());
  for (unsigned int i = 0; i < particles.size() ; ++i) {
    int particle_index = dist_particle(gen);
    resampled_particles.push_back(particles[particle_index]);
  }

  // Set the resampled particles as the new particles
  particles = resampled_particles;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

double ParticleFilter::multiv_prob(double sig_x, double sig_y, double x_obs, double y_obs,
                   double mu_x, double mu_y) {
    // calculate normalization term
    double gauss_norm;
    gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);

    // calculate exponent
    double exponent;
    exponent = (pow(x_obs - mu_x, 2) / (2 * pow(sig_x, 2)))
               + (pow(y_obs - mu_y, 2) / (2 * pow(sig_y, 2)));

    // calculate weight using normalization terms and exponent
    double weight;
    weight = gauss_norm * exp(-exponent);

    return weight;
}