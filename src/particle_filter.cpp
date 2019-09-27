/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
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
std::default_random_engine gen;

// static default_random_engine rand_gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */

   if(is_initialized) {
		return;
	}
  num_particles = 100;  // TODO: Set the number of particles

  // This line creates a normal (Gaussian) distribution for x , y , and thetha
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  //Initialize 100 particles

  for (int i=0; i<num_particles;i++){

    Particle particle;
    particle.id = i;
    particle.x = dist_x(gen);;
    particle.y = dist_y(gen);;
    particle.theta = dist_theta(gen);
    particle.weight = 1.0;



    particles.push_back(particle);

  }
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

   // This line creates a normal (Gaussian) distribution for x , y , and thetha
   normal_distribution<double> dist_x(0, std_pos[0]);
   normal_distribution<double> dist_y(0, std_pos[1]);
   normal_distribution<double> dist_theta(0, std_pos[2]);

   for (int i=0; i<num_particles;i++){

    //check it yaw_rate it greater than 0
    if (fabs(yaw_rate) < 0.00001) {
      //predict the new postion of the particles
      particles[i].x += velocity * delta_t * cos(particles[i].theta);
      particles[i].y += velocity * delta_t * sin(particles[i].theta);

    }
    else {
      //yaw_rate is grater than 0
      particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
      particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
      particles[i].theta += yaw_rate * delta_t;

    }

    // Adding noise
    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);

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

   for ( unsigned int i = 0; i < observations.size(); i++) {

     //current observation
     LandmarkObs obs = observations[i];

     // min distance possible
    double min_distance = std::numeric_limits<double>::max();

    int map_id = -1;

    for ( unsigned int k = 0; k < predicted.size(); k++) {
            // Get current prediction
            LandmarkObs pred = predicted[k];

            // Get distance between current and predicted landmarks
            double cur_dist = dist(obs.x, obs.y, pred.x, pred.y);

            // Find predicted landmark nearest to current observed landmark
            if (cur_dist < min_distance) {
                map_id = pred.id;
                min_distance = cur_dist;
                
            }
        }

    // Set the observation id to nearest predicted landmark id
    observations[i].id = map_id;

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
  for (int i =0; i < num_particles; i++) {
    //get particle information
    double particle_x = particles[i].x;
    double particle_y = particles[i].y;
    double particle_theta = particles[i].theta;

    //Vector for the precited landmark locations
    vector<LandmarkObs> predictions;

    for ( unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {
      // Get id and x,y coordinates
      float landmmark_x = map_landmarks.landmark_list[j].x_f;
      float landmark_y = map_landmarks.landmark_list[j].y_f;
      int landmark_id = map_landmarks.landmark_list[j].id_i;

      // Only consider landmarks within sensor range of particle
      if (fabs(landmmark_x - particle_x) <= sensor_range && fabs(landmark_y - particle_y) <= sensor_range) {
          predictions.push_back(LandmarkObs{ landmark_id, landmmark_x, landmark_y });
            }
    }

    //Transform from vehicle coords to map coords
    vector<LandmarkObs> transformed_cordinate;
    for (unsigned int j = 0; j < observations.size(); j++) {
        double transformed_x = cos(particle_theta)*observations[j].x - sin(particle_theta)*observations[j].y + particle_x;
        double transformed_y = sin(particle_theta)*observations[j].x + cos(particle_theta)*observations[j].y + particle_y;
        transformed_cordinate.push_back(LandmarkObs{ observations[j].id, transformed_x, transformed_y });
    }

    // for data association
    dataAssociation(predictions, transformed_cordinate);

    //set particle wait to 1
    particles[i].weight = 1.0;

    for (unsigned int j = 0; j < transformed_cordinate.size(); j++){
      double observed_x, observed_y, predicted_x, predicted_y;
      int prediction_id;
      observed_x = transformed_cordinate[j].x;
      observed_y = transformed_cordinate[j].y;

      prediction_id = transformed_cordinate[j].id;

      // Get the x and y coords of prediction for current observation
      for ( unsigned int k = 0; k < predictions.size(); k++) {
          if (predictions[k].id == prediction_id) {
              predicted_x = predictions[k].x;
              predicted_y = predictions[k].y;
          }
      }

      //Use multivariate Guassian to calculate weight of the observation
      double std_x = std_landmark[0];
      double std_y = std_landmark[1];
      double obs_w = ( 1/(2*M_PI*std_x*std_y)) * exp( -( pow(predicted_x-observed_x,2)/(2*pow(std_x, 2)) + (pow(predicted_y-observed_y,2)/(2*pow(std_y, 2))) ) );

      
      particles[i].weight *= obs_w;

    }

  }


}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

   vector<double> particle_weight;

   // get the max weight among the particles
   double max_weight = numeric_limits<double>::min();

   double beta = 0.0;

   //get max weight from the particles
   for(int i = 0; i < num_particles; i++) {
      particle_weight.push_back(particles[i].weight);
      if(particles[i].weight > max_weight) {
        max_weight = particles[i].weight;
      }
    }

    uniform_real_distribution<double> dist1(0.0, max_weight);
    uniform_int_distribution<int> dist2(0, num_particles - 1);
    int index = dist2(gen); //random index


   vector<Particle> resampled_particles;


   for (int i = 0; i < num_particles; i++) {
        beta += dist1(gen) * 2.0;
        while (beta > particle_weight[index]) {
            beta -= particle_weight[index];
            index = (index + 1) % num_particles;
        }
        resampled_particles.push_back(particles[index]);
    }

   

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