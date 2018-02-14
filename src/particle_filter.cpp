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

    // Setting Number of Particles
    num_particles = 100;
   
    // Normal Distribution for Initializing X, Y, Theta of particles
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);
    
    // Initialzing Particles
    for(int i = 0; i<num_particles ; i++)
    {
        Particle particle;
        particle.id = i;
        particle.x = dist_x(gen);
        particle.y = dist_y(gen);
        particle.theta = dist_theta(gen);
        
        // Assigning same weight to all the particles initially
        particle.weight = 1.0;
        particles.push_back(particle);
    }
    
    // Particles Initialized, Updating Initialization Flag
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

    // Checking whether delta t > 0, skipping prediction if delta t == 0
    if(delta_t > 0)
    {
        // Prediction Based Bicycle Motion Model
        for (int i = 0; i < num_particles; i++)
        {
            // If yaw rate is 0
            if (fabs(yaw_rate) < 0.001)
            {
                particles[i].x += velocity*delta_t * cos(particles[i].theta);
                particles[i].y += velocity*delta_t * sin(particles[i].theta);
            }
            // Else
            else
            {
                // Calculating Change in Theta
                double delta_theta = yaw_rate*delta_t;
                
                particles[i].x += (velocity/yaw_rate)* ( sin(particles[i].theta + delta_theta) - sin(particles[i].theta));
                
                particles[i].y += (velocity/yaw_rate)* ( cos(particles[i].theta) - cos(particles[i].theta + delta_theta));
                
                particles[i].theta += delta_theta;
            }
            
            // Adding Noise
            
            // Normal Distribution for X, Y, Theta
            normal_distribution<double> dist_x(0, std_pos[0]);
            normal_distribution<double> dist_y(0, std_pos[1]);
            normal_distribution<double> dist_theta(0, std_pos[2]);
            
            particles[i].x += dist_x(gen);
            particles[i].y += dist_y(gen);
            particles[i].theta += dist_theta(gen);
            
        }
    }

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
    
    // Associating predicted measurement and observed measurement using Nearest Neighbour Approach
    for  (int i = 0; i < observations.size(); i++)
    {
        int prediction_id;
        double min_distance;
        
        for(int j = 0; j < predicted.size(); j++)
        {
            double distance = dist(predicted[j].x, predicted[j].y, observations[i].x, observations[i].y);
            
            if (j == 0)
            {
                prediction_id = j;
                min_distance = distance;
            }
            else
            {
                if (distance < min_distance)
                {
                    prediction_id = j;
                    min_distance = distance;
                }

            }
        }
        
        observations[i].id = prediction_id;
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
    
    for ( int i = 0; i < num_particles; i++)
    {
        // Fetching particles paramter
        double p_x = particles[i].x;
        double p_y = particles[i].y;
        double p_theta = particles[i].theta;
        
        
        vector<LandmarkObs> predictions;
       
        // Checking Landmark Location w.r.t. Sensor range
        for (int j = 0; j < map_landmarks.landmark_list.size(); j++)
        {
            // Fetching Landmark Parameters
            double l_x = map_landmarks.landmark_list[j].x_f;
            double l_y = map_landmarks.landmark_list[j].y_f;
            int l_id = map_landmarks.landmark_list[j].id_i;
            
            // In order to optimize the Association process between landmarks and observations,
            // considering landmarks within the range of the sensor
            if( dist(p_x, p_y, l_x, l_y) <= sensor_range)
            {
                // Landmark within the range of sensor, add it to the prediction vector
                predictions.push_back(LandmarkObs{l_id, l_x, l_y});
            }
            
        }
        
        // Transforming observations to Map Co-ordinate System
        vector<LandmarkObs> transformed_obs;
        
        for (int j = 0; j < observations.size(); j++)
        {
            double x_m = observations[j].x * cos(p_theta) - observations[j].y * sin(p_theta) + p_x;
            double y_m = observations[j].x * sin(p_theta) + observations[j].y * cos(p_theta) + p_y;
            
            transformed_obs.push_back(LandmarkObs{observations[j].id, x_m, y_m});
        }
       
        // Association of observations and landmarks
        dataAssociation(predictions, transformed_obs);
        
        // Calculating weight for the particle under consideration
        double std_x = std_landmark[0];
        double std_y = std_landmark[1];
        
        // Calculating Normalizer for the weights
        double k = 1/(2* M_PI * std_x * std_y);
        
        double final_weight = 1.0;
        
        for (int j = 0; j < transformed_obs.size(); j++)
        {
            double x = transformed_obs[j].x;
            double y = transformed_obs[j].y;
            
            // Nearest Landmark Id
            int landmark_id = transformed_obs[j].id;
            
            // Fetching parameters of nearest landmark
            double l_x = predictions[landmark_id].x;
            double l_y = predictions[landmark_id].y;
            
            // Calculating Weight using Multivariate-Gaussian Probability Density Function
            double temp_a = pow(x - l_x, 2)/(2 * std_x * std_x);
            double temp_b = pow(y - l_y, 2)/(2 * std_y * std_y);
            double obs_weight = k * exp(-(temp_a+temp_b));
            
            // Updating Final Weight for the particle
            final_weight *= obs_weight;
        }
        
        // Assigning Final Weight to the particle
        particles[i].weight = final_weight;
    }
    
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    
    // Resampling using Discrete Distribution where probability of of each individual integer i is defined as
    // w_i/S, that is the weight of the ith integer divided by the sum of all n weights
    vector<Particle> resampled_particles;
    vector<double> weights;
    
    for (int i = 0 ; i < particles.size(); i++)
    {
        weights.push_back(particles[i].weight);
    }
    
    discrete_distribution<int> dist(weights.begin(), weights.end());
    
    // Selecting a particle as per the weight
    for (int i = 0; i < particles.size(); i++)
    {
        int index = dist(gen);
        
        resampled_particles.push_back(particles[index]);
    }
    
    // Assigning new particles 
    particles = resampled_particles;
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
