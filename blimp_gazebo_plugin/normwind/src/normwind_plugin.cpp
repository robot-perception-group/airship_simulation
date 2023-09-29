/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "normwind_plugin.hpp"
#include "std_msgs/String.h"
#include "blimp_description/WindGust.h"
#include <ros/ros.h>

#include <fstream>
#include <math.h>

#include "ConnectGazeboToRosTopic.pb.h"

namespace gazebo {

GazeboWindPlugin::~GazeboWindPlugin() {
  
}

// PG: This gets executed when the plugin is loaded
void GazeboWindPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {


  if (kPrintOnPluginLoad) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  // Store the pointer to the model.
  model_ = _model;
  world_ = model_->GetWorld();

  //double wind_gust_start = kDefaultWindGustStart;
  //double wind_gust_duration = kDefaultWindGustDuration;

  //==============================================//
  //========== READ IN PARAMS FROM SDF ===========//
  //==============================================//

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_wind_plugin] Please specify a robotNamespace.\n";

  // Create Gazebo Node.
  node_handle_ = gazebo::transport::NodePtr(new transport::Node());

  // Initialise with default namespace (typically /gazebo/default/).
  node_handle_->Init();

  //if (_sdf->HasElement("xyzOffset"))
  //  xyz_offset_ = _sdf->GetElement("xyzOffset")->Get<ignition::math::Vector3d >();
  //else
  //  gzerr << "[gazebo_wind_plugin] Please specify a xyzOffset.\n";

  getSdfParam<std::string>(_sdf, "windForcePubTopic", wind_force_pub_topic_,
                           wind_force_pub_topic_);
  getSdfParam<std::string>(_sdf, "windSpeedPubTopic", wind_speed_pub_topic_,
                           wind_speed_pub_topic_);
  getSdfParam<std::string>(_sdf, "frameId", frame_id_, frame_id_);
  getSdfParam<std::string>(_sdf, "linkName", link_name_, link_name_);
  // Get the wind speed params from SDF.
  getSdfParam<double>(_sdf, "windSpeedMean", wind_speed_mean_,
                      wind_speed_mean_);
  getSdfParam<int>(_sdf, "windTurbulenceLevel", wind_turbulence_level,
                      wind_turbulence_level);
  getSdfParam<ignition::math::Vector3d>(_sdf, "windDirectionMean", wind_direction_mean_, wind_direction_mean_);

  // Get the wind gust params from sdf
  getSdfParam<std::string>(_sdf,"gustGenerationMode",gust_generation_mode_,gust_generation_mode_);
  getSdfParam<double>(_sdf, "maxGustVelocity", max_gust_velocity_,
                      max_gust_velocity_);
  getSdfParam<double>(_sdf, "gustDuration", gust_duration_,
                      gust_duration_);        
  getSdfParam<ignition::math::Vector3d>(_sdf, "gustDirectionMean", gust_direction_mean_,
                      gust_direction_mean_);  
  getSdfParam<ignition::math::Vector3d>(_sdf, "gustDirectionStd", gust_direction_std_,
                      gust_direction_std_);                             
  getSdfParam<double>(_sdf,"gustOccurenceIntervalMean",gust_occurence_interval_mean_,
                      gust_occurence_interval_mean_);
  getSdfParam<double>(_sdf,"gustOccurenceIntervalStd",gust_occurence_interval_std_,
                      gust_occurence_interval_std_);                                                     





  //getSdfParam<double>(_sdf, "windDirectionVariance", wind_direction_variance_, wind_direction_variance_);

  // Wind gust parameters for 1-cos wind gust

  // Check if a custom static wind field should be used.
  getSdfParam<bool>(_sdf, "useCustomStaticWindField", use_custom_static_wind_field_,
                      use_custom_static_wind_field_);
  if (!use_custom_static_wind_field_) {
    gzdbg << "[gazebo_wind_plugin] Using user-defined random wind field and gusts.\n";

    // Get the wind params from SDF.
    //getSdfParam<double>(_sdf, "windForceMean", wind_force_mean_,
    //                    wind_force_mean_);
    //getSdfParam<double>(_sdf, "windForceVariance", wind_force_variance_,
    //                    wind_force_variance_);
    // Get the wind gust params from SDF.
    //getSdfParam<double>(_sdf, "windGustStart", wind_gust_start, wind_gust_start);
    //getSdfParam<double>(_sdf, "windGustDuration", wind_gust_duration,
    //                    wind_gust_duration);
    //getSdfParam<double>(_sdf, "windGustForceMean", wind_gust_force_mean_,
    //                    wind_gust_force_mean_);
    //getSdfParam<double>(_sdf, "windGustForceVariance", wind_gust_force_variance_,
    //                    wind_gust_force_variance_);
    //getSdfParam<ignition::math::Vector3d>(_sdf, "windGustDirectionMean", wind_gust_direction_mean_, wind_gust_direction_mean_);
    //getSdfParam<double>(_sdf, "windGustDirectionVariance", wind_gust_direction_variance_, wind_gust_direction_variance_);


    wind_direction_mean_.Normalize();
    //wind_gust_direction_mean_.Normalize();
    //wind_gust_start_ = common::Time(wind_gust_start);
    //wind_gust_end_ = common::Time(wind_gust_start + wind_gust_duration);

    // Set random wind direction mean and standard deviation
    //wind_direction_distribution_X_.param(std::normal_distribution<double>::param_type(wind_direction_mean_.X(), sqrt(wind_direction_variance_)));
    //wind_direction_distribution_Y_.param(std::normal_distribution<double>::param_type(wind_direction_mean_.Y(), sqrt(wind_direction_variance_)));
    //wind_direction_distribution_Z_.param(std::normal_distribution<double>::param_type(wind_direction_mean_.Z(), sqrt(wind_direction_variance_)));

    // Set random wind gust direction mean and standard deviation
    //wind_gust_direction_distribution_X_.param(std::normal_distribution<double>::param_type(wind_gust_direction_mean_.X(), sqrt(wind_gust_direction_variance_)));
    //wind_gust_direction_distribution_Y_.param(std::normal_distribution<double>::param_type(wind_gust_direction_mean_.Y(), sqrt(wind_gust_direction_variance_)));
    //wind_gust_direction_distribution_Z_.param(std::normal_distribution<double>::param_type(wind_gust_direction_mean_.Z(), sqrt(wind_gust_direction_variance_)));

  } else {
    gzdbg << "[gazebo_wind_plugin] Using custom wind field from text file.\n";
    // Get the wind field text file path, read it and save data.
    std::string custom_wind_field_path;
    getSdfParam<std::string>(_sdf, "customWindFieldPath", custom_wind_field_path,
                        custom_wind_field_path);
    ReadCustomWindField(custom_wind_field_path);
  }

  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_wind_plugin] Couldn't find specified link \"" << link_name_
                                                                   << "\".");
  previousRun = world_->SimTime();
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboWindPlugin::OnUpdate, this, _1));


// Initialize ROS
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_plugin_wind_gust_publisher_node");
    ros::NodeHandle nh;
  // Create the publisher for wind gust information
   pub = nh.advertise<blimp_description::WindGust>("/gazebo/WindGust", 1);
   // Create the subscriber to set wind velocity
   set_wind_gust_speed_subscriber = nh.subscribe("/gazebo/SetWindGustSpeed",3,&GazeboWindPlugin::read_set_wind_gust_speed,this);
}

double GazeboWindPlugin::POEValue(int s, double h) {
	for (int i=12;i>0;i--) {
		if (h>POE[0][i]) {
			return POE[s][i];
		}
	}
	return POE[s][0];
}

// This gets called by the world update start event.
void GazeboWindPlugin::OnUpdate(const common::UpdateInfo& _info) {
  if (kPrintOnUpdates) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  if (!pubs_and_subs_created_) {
    CreatePubsAndSubs();
    pubs_and_subs_created_ = true;
  }

  // Get the current simulation time.
  common::Time now = world_->SimTime();
  double deltaT = now.Double() - previousRun.Double();
  previousRun = now;



  
  ignition::math::Vector3d wind_velocity(0.0, 0.0, 0.0);
  

  // Get the current position of the aircraft in world coordinates.
  ignition::math::Vector3d link_position = link_->WorldPose().Pos();

  // Choose user-specified method for calculating wind velocity.
  if (use_custom_static_wind_field_) {

    // Calculate the x, y index of the grid points with x, y-coordinate 
    // just smaller than or equal to aircraft x, y position.
    std::size_t x_inf = floor((link_position.X() - min_x_) / res_x_);
    std::size_t y_inf = floor((link_position.Y() - min_y_) / res_y_);

    // In case aircraft is on one of the boundary surfaces at max_x or max_y,
    // decrease x_inf, y_inf by one to have x_sup, y_sup on max_x, max_y.
    if (x_inf == n_x_ - 1u) {
      x_inf = n_x_ - 2u;
    }
    if (y_inf == n_y_ - 1u) {
      y_inf = n_y_ - 2u;
    }

    // Calculate the x, y index of the grid points with x, y-coordinate just
    // greater than the aircraft x, y position. 
    std::size_t x_sup = x_inf + 1u;
    std::size_t y_sup = y_inf + 1u;

    // Save in an array the x, y index of each of the eight grid points 
    // enclosing the aircraft.
    constexpr unsigned int n_vertices = 8;
    std::size_t idx_x[n_vertices] = {x_inf, x_inf, x_sup, x_sup, x_inf, x_inf, x_sup, x_sup};
    std::size_t idx_y[n_vertices] = {y_inf, y_inf, y_inf, y_inf, y_sup, y_sup, y_sup, y_sup};

    // Find the vertical factor of the aircraft in each of the four surrounding 
    // grid columns, and their minimal/maximal value.
    constexpr unsigned int n_columns = 4;
    float vertical_factors_columns[n_columns];
    for (std::size_t i = 0u; i < n_columns; ++i) {
      vertical_factors_columns[i] = (
        link_position.Z() - bottom_z_[idx_x[2u * i] + idx_y[2u * i] * n_x_]) /
        (top_z_[idx_x[2u * i] + idx_y[2u * i] * n_x_] - bottom_z_[idx_x[2u * i] + idx_y[2u * i] * n_x_]);
    }
    
    // Find maximal and minimal value amongst vertical factors.
    float vertical_factors_min = std::min(std::min(std::min(
      vertical_factors_columns[0], vertical_factors_columns[1]),
      vertical_factors_columns[2]), vertical_factors_columns[3]);
    float vertical_factors_max = std::max(std::max(std::max(
      vertical_factors_columns[0], vertical_factors_columns[1]),
      vertical_factors_columns[2]), vertical_factors_columns[3]);

    // Check if aircraft is out of wind field or not, and act accordingly.
    if (x_inf >= 0u && y_inf >= 0u && vertical_factors_max >= 0u && 
        x_sup <= (n_x_ - 1u) && y_sup <= (n_y_ - 1u) && vertical_factors_min <= 1u) {
      // Find indices in z-direction for each of the vertices. If link is not 
      // within the range of one of the columns, set to lowest or highest two.
      std::size_t idx_z[n_vertices] = {0u, static_cast<int>(vertical_spacing_factors_.size()) - 1u,
                              0u, static_cast<int>(vertical_spacing_factors_.size()) - 1u,
                              0u, static_cast<int>(vertical_spacing_factors_.size()) - 1u,
                              0u, static_cast<int>(vertical_spacing_factors_.size()) - 1u};
      for (std::size_t i = 0u; i < n_columns; ++i) {
        if (vertical_factors_columns[i] < 0u) {
          // Link z-position below lowest grid point of that column.
          idx_z[2u * i + 1u] = 1u;
        } else if (vertical_factors_columns[i] >= 1u) {
          // Link z-position above highest grid point of that column.
          idx_z[2u * i] = vertical_spacing_factors_.size() - 2u;
        } else {
          // Link z-position between two grid points in that column.
          for (std::size_t j = 0u; j < vertical_spacing_factors_.size() - 1u; ++j) {
            if (vertical_spacing_factors_[j] <= vertical_factors_columns[i] && 
                vertical_spacing_factors_[j + 1u] > vertical_factors_columns[i]) {
              idx_z[2u * i] = j;
              idx_z[2u * i + 1u] = j + 1u;
              break;
            }
          }
        }
      }

      // Extract the wind velocities corresponding to each vertex.
      ignition::math::Vector3d wind_at_vertices[n_vertices];
      for (std::size_t i = 0u; i < n_vertices; ++i) {
        wind_at_vertices[i].X() = u_[idx_x[i] + idx_y[i] * n_x_ + idx_z[i] * n_x_ * n_y_];
        wind_at_vertices[i].Y() = v_[idx_x[i] + idx_y[i] * n_x_ + idx_z[i] * n_x_ * n_y_];
        wind_at_vertices[i].Z() = w_[idx_x[i] + idx_y[i] * n_x_ + idx_z[i] * n_x_ * n_y_];
      }

      // Extract the relevant coordinate of every point needed for trilinear 
      // interpolation (first z-direction, then x-direction, then y-direction).
      constexpr unsigned int n_points_interp_z = 8;
      constexpr unsigned int n_points_interp_x = 4;
      constexpr unsigned int n_points_interp_y = 2;
      double interpolation_points[n_points_interp_x + n_points_interp_y + n_points_interp_z];
      for (std::size_t i = 0u; i < n_points_interp_x + n_points_interp_y + n_points_interp_z; ++i) {
        if (i < n_points_interp_z) {
          interpolation_points[i] = (
            top_z_[idx_x[i] + idx_y[i] * n_x_] - bottom_z_[idx_x[i] + idx_y[i] * n_x_])
            * vertical_spacing_factors_[idx_z[i]] + bottom_z_[idx_x[i] + idx_y[i] * n_x_];
        } else if (i >= n_points_interp_z && i < n_points_interp_x + n_points_interp_z) {
          interpolation_points[i] = min_x_ + res_x_ * idx_x[2u * (i - n_points_interp_z)];
        } else {
          interpolation_points[i] = min_y_ + res_y_ * idx_y[4u * (i - n_points_interp_z - n_points_interp_x)];
        }
      }

      // Interpolate wind velocity at aircraft position.
      wind_velocity = TrilinearInterpolation(
        link_position, wind_at_vertices, interpolation_points);
    } 
  } else if (wind_speed_mean_>0.0 and wind_turbulence_level>0) {
    /* Code based on Flightgear implementation "https://github.com/FlightGear/flightgear/blob/22de9d30b518646894ac190cc6b04371daa6d5c2/src/FDM/JSBSim/models/atmosphere/FGWinds.cpp"
       MILSPEC Dryden spectrum
       @see Yeager, Jessie C.: "Implementation and Testing of Turbulence Models for
         the F18-HARV" (<a href="http://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19980028448_1998081596.pdf">pdf</a>), NASA CR-1998-206937, 1998
        @see MIL-F-8785C: Military Specification: Flying Qualities of Piloted Aircraft
    */
    ignition::math::Vector3d turbulence(0.0, 0.0, 0.0);
    double psiw = atan2(wind_direction_mean_.Y(),wind_direction_mean_.X());
    double b_w = 30., L_u, L_w, sig_u, sig_w;

    double h = link_position.Z() * 3.28; // convert hight into feet - because this stupid model is imperial
    double windspeed_at_20ft = wind_speed_mean_ * 3.28;
    int probability_of_exceedence_index = (wind_turbulence_level<=7)?wind_turbulence_level:7;

    // clip height functions at 10 ft
    if (h <= 10.) h = 10;

    // Scale lengths L and amplitudes sigma as function of height
    if (h <= 1000) {
      L_u = h/pow(0.177 + 0.000823*h, 1.2); // MIL-F-8785c, Fig. 10, p. 55
      L_w = h;
      sig_w = 0.1*windspeed_at_20ft;
      sig_u = sig_w/pow(0.177 + 0.000823*h, 0.4); // MIL-F-8785c, Fig. 11, p. 56
    } else if (h <= 2000) {
      // linear interpolation between low altitude and high altitude models
      L_u = L_w = 1000 + (h-1000.)/1000.*750.;
      sig_u = sig_w = 0.1*windspeed_at_20ft
                    + (h-1000.)/1000.*(POEValue(probability_of_exceedence_index, h) - 0.1*windspeed_at_20ft);
    } else {
      L_u = L_w = 1750.; //  MIL-F-8785c, Sec. 3.7.2.1, p. 48
      sig_u = sig_w = POEValue(probability_of_exceedence_index, h);
    }

    // keep values from last timesteps
    // TODO maybe use deque?
    static double
      xi_u_km1 = 0, nu_u_km1 = 0,
      xi_v_km1 = 0, xi_v_km2 = 0, nu_v_km1 = 0, nu_v_km2 = 0,
      xi_w_km1 = 0, xi_w_km2 = 0, nu_w_km1 = 0, nu_w_km2 = 0,
      xi_p_km1 = 0, nu_p_km1 = 0,
      xi_q_km1 = 0, xi_r_km1 = 0;


    double
      T_V = deltaT, // for compatibility of nomenclature
      sig_p = 1.9/sqrt(L_w*b_w)*sig_w, // Yeager1998, eq. (8)
      //sig_q = sqrt(M_PI/2/L_w/b_w), // eq. (14)
      //sig_r = sqrt(2*M_PI/3/L_w/b_w), // eq. (17)
      L_p = sqrt(L_w*b_w)/2.6, // eq. (10)
      tau_u = L_u/windspeed_at_20ft, // eq. (6)
      tau_w = L_w/windspeed_at_20ft, // eq. (3)
      tau_p = L_p/windspeed_at_20ft, // eq. (9)
      tau_q = 4*b_w/M_PI/windspeed_at_20ft, // eq. (13)
      tau_r =3*b_w/M_PI/windspeed_at_20ft, // eq. (17)
      nu_u = GaussianRandomNumber(randomGen),
      nu_v = GaussianRandomNumber(randomGen),
      nu_w = GaussianRandomNumber(randomGen),
      nu_p = GaussianRandomNumber(randomGen),
      xi_u=0, xi_v=0, xi_w=0, xi_p=0, xi_q=0, xi_r=0;


      // the following is the MIL-STD-1797A formulation
      // as cited in Yeager's report
      xi_u = (1 - T_V/tau_u)  *xi_u_km1 + sig_u*sqrt(2*T_V/tau_u)*nu_u;  // eq. (30)
      xi_v = (1 - 2*T_V/tau_u)*xi_v_km1 + sig_u*sqrt(4*T_V/tau_u)*nu_v;  // eq. (31)
      xi_w = (1 - 2*T_V/tau_w)*xi_w_km1 + sig_w*sqrt(4*T_V/tau_w)*nu_w;  // eq. (32)
      xi_p = (1 - T_V/tau_p)  *xi_p_km1 + sig_p*sqrt(2*T_V/tau_p)*nu_p;  // eq. (33)
      xi_q = (1 - T_V/tau_q)  *xi_q_km1 + M_PI/4/b_w*(xi_w - xi_w_km1);  // eq. (34)
      xi_r = (1 - T_V/tau_r)  *xi_r_km1 + M_PI/3/b_w*(xi_v - xi_v_km1);  // eq. (35)

    // rotate by wind azimuth and assign the velocities
    double cospsi = cos(psiw), sinpsi = sin(psiw);

    turbulence.X() =  cospsi*xi_u + sinpsi*xi_v;
    turbulence.Y() = -sinpsi*xi_u + cospsi*xi_v;
    turbulence.Z() = xi_w;

    turbulence = turbulence * (1.0/3.28); //convert ft/s back into m/s

    // hand on the values for the next timestep
    xi_u_km1 = xi_u; nu_u_km1 = nu_u;
    xi_v_km2 = xi_v_km1; xi_v_km1 = xi_v; nu_v_km2 = nu_v_km1; nu_v_km1 = nu_v;
    xi_w_km2 = xi_w_km1; xi_w_km1 = xi_w; nu_w_km2 = nu_w_km1; nu_w_km1 = nu_w;
    xi_p_km1 = xi_p; nu_p_km1 = nu_p;
    xi_q_km1 = xi_q;
    xi_r_km1 = xi_r;
    ignition::math::Vector3d wind_gust_velocity(0.0, 0.0,0.0);
    wind_gust_velocity = ComputeWindGust(&max_gust_velocity_,&gust_duration_,&gust_direction_,&gust_direction_mean_,&gust_direction_std_,&gust_occurence_interval_mean_,&gust_occurence_interval_std_, &gust_time_,&time_until_next_gust_,&elapsed_time_between_gusts_,&gust_active_, &waiting_for_next_gust_, &deltaT,&randomGenGust);
    wind_velocity = (wind_speed_mean_ * wind_direction_mean_) + turbulence + wind_gust_velocity;
  } else {
    // if wind speed is received through ros

    // if gazebo_plugin is supposed to be used
    if (gust_generation_mode_== "gazebo_plugin"){
      ignition::math::Vector3d wind_gust_velocity(0.0,0.0,0.0);
      wind_gust_velocity = ComputeWindGust(&max_gust_velocity_,&gust_duration_,&gust_direction_,&gust_direction_mean_,&gust_direction_std_,&gust_occurence_interval_mean_,&gust_occurence_interval_std_, &gust_time_,&time_until_next_gust_,&elapsed_time_between_gusts_,&gust_active_, &waiting_for_next_gust_, &deltaT,&randomGenGust);
      wind_velocity = (wind_speed_mean_ * wind_direction_mean_) + wind_gust_velocity;
    }else{
      wind_velocity.X() = set_wind_gust_speed_value_.point.x;
      wind_velocity.Y() = set_wind_gust_speed_value_.point.y;
      wind_velocity.Z() = set_wind_gust_speed_value_.point.z;
    }
  }

  wind_speed_msg_.mutable_header()->set_frame_id(frame_id_);
  wind_speed_msg_.mutable_header()->mutable_stamp()->set_sec(now.sec);
  wind_speed_msg_.mutable_header()->mutable_stamp()->set_nsec(now.nsec);

  wind_speed_msg_.mutable_velocity()->set_x(wind_velocity.X());
  wind_speed_msg_.mutable_velocity()->set_y(wind_velocity.Y());
  wind_speed_msg_.mutable_velocity()->set_z(wind_velocity.Z());

  wind_speed_pub_->Publish(wind_speed_msg_);
}

void GazeboWindPlugin::CreatePubsAndSubs() {
  // Create temporary "ConnectGazeboToRosTopic" publisher and message.
  gazebo::transport::PublisherPtr connect_gazebo_to_ros_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
          "~/" + kConnectGazeboToRosSubtopic, 1);

  gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;

  // ============================================ //
  // ========= WRENCH STAMPED MSG SETUP ========= //
  // ============================================ //
  wind_force_pub_ = node_handle_->Advertise<gz_geometry_msgs::WrenchStamped>(
      "~/" + namespace_ + "/" + wind_force_pub_topic_, 1);

  // connect_gazebo_to_ros_topic_msg.set_gazebo_namespace(namespace_);
  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   wind_force_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                wind_force_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::WRENCH_STAMPED);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);

  // ============================================ //
  // ========== WIND SPEED MSG SETUP ============ //
  // ============================================ //
  wind_speed_pub_ = node_handle_->Advertise<gz_mav_msgs::WindSpeed>(
      "~/" + namespace_ + "/" + wind_speed_pub_topic_, 1);

  connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   wind_speed_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                wind_speed_pub_topic_);
  connect_gazebo_to_ros_topic_msg.set_msgtype(
      gz_std_msgs::ConnectGazeboToRosTopic::WIND_SPEED);
  connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                           true);





}

void GazeboWindPlugin::read_set_wind_gust_speed(const geometry_msgs::PointStamped::ConstPtr& msg) {
  set_wind_gust_speed_value_ = *msg;
  // // Process the received message
  // ROS_INFO("Received PointStamped message:");
  // ROS_INFO("Header:");
  // ROS_INFO("  Seq: %d", msg->header.seq);
  // ROS_INFO("  Stamp: %d.%09d", msg->header.stamp.sec, msg->header.stamp.nsec);
  // ROS_INFO("  Frame ID: %s", msg->header.frame_id.c_str());
  // ROS_INFO("Point:");
  // ROS_INFO("  X: %f", msg->point.x);
  // ROS_INFO("  Y: %f", msg->point.y);
  // ROS_INFO("  Z: %f", msg->point.z);
}



ignition::math::Vector3d GazeboWindPlugin::ComputeWindGust(double* max_gust_velocity,double* gust_duration,ignition::math::Vector3d* gust_direction,ignition::math::Vector3d* gust_direction_mean,ignition::math::Vector3d* gust_direction_std, double* gust_occurence_interval_mean,double* gust_occurence_interval_std,double* gust_time,double*time_until_next_gust,double* elapsed_time_between_gusts,bool* gust_active, bool* waiting_for_next_gust,double* delta_t,std::default_random_engine* randomGenGust){
  
  double v(0.0);

  if (*gust_time > *gust_duration && *gust_active == true){
    //Reset case
    // reset elapsed time counter
    *elapsed_time_between_gusts = 0;
    // set gust_active to false
    *gust_active = false;
    // determine time until next episode - define distribution first
    std::normal_distribution<double> dist_t(*gust_occurence_interval_mean, *gust_occurence_interval_std);
    std::uniform_real_distribution<double> dist_gust_direction_x(gust_direction_mean->X()-gust_direction_std->X(), gust_direction_mean->X()+gust_direction_std->X());
    std::uniform_real_distribution<double> dist_gust_direction_y(gust_direction_mean->Y()-gust_direction_std->Y(), gust_direction_mean->Y()+gust_direction_std->Y());
    std::uniform_real_distribution<double> dist_gust_direction_z(gust_direction_mean->Z()-gust_direction_std->Z(), gust_direction_mean->Z()+gust_direction_std->Z());
    // draw random number and clip to avoid values lower than zero.
    if (*waiting_for_next_gust == false){
          *time_until_next_gust = std::fmax(dist_t(*randomGenGust),0);
          *waiting_for_next_gust = true;
          (*gust_direction)[0]= dist_gust_direction_x(*randomGenGust);
          (*gust_direction)[1]= dist_gust_direction_y(*randomGenGust);
          (*gust_direction)[2]= dist_gust_direction_z(*randomGenGust);
          gust_direction->Normalize();

    }
  }else if(*gust_active==false && *waiting_for_next_gust==true){
      // if in between gusts
      *elapsed_time_between_gusts += *delta_t;
      msg.time_until_next_gust = *time_until_next_gust-*elapsed_time_between_gusts;


      if (*elapsed_time_between_gusts > *time_until_next_gust){
        *waiting_for_next_gust = false;
        *gust_active = true;
        *gust_time = 0;
  
      }
  }else if(*gust_active==true){
      // if a gust is currently happening 
      // compute the velocity
      v = (*max_gust_velocity/2)*(1-cos(2*3.14159265*(*gust_time)/(*gust_duration)));
      msg.gust_active = *gust_active;
      msg.gust_velocity = v;
      //update the gust duration.
      *gust_time += *delta_t;
    }else{
      v=0;
    };
  // gzdbg << "v="<< v;
  // gzdbg << "gust_time="<< *gust_time<<"\n";
  // gzdbg << "elapsed_time_between_gusts="<< *elapsed_time_between_gusts<<"\n";
  // gzdbg << "time_until_next_gust="<< *time_until_next_gust<<"\n";
  // gzdbg << "waiting for next gust="<<*waiting_for_next_gust<<"\n";
  // gzdbg << "gust_active="<<*gust_active<<"\n";
  // gzdbg << "====================\n";
  // gzdbg << "max_gust_velocity="<<*max_gust_velocity<<"\n";
  // gzdbg << "gust_duration="<<*gust_duration<<"\n";
  // gzdbg << "gust_occurenc_interval_mean="<<*gust_occurence_interval_mean<<"\n";
  // gzdbg << "gust_occurenc_interval_std="<<*gust_occurence_interval_std<<"\n";
  // gzdbg << "====================\n";
  // gzdbg << "gust_direction="<<gust_direction->Normalize()<<"\n";
    // Create ROS message for Wind Gust
  msg.header.stamp = ros::Time::now();
  msg.duration = *gust_duration;
  msg.max_gust_velocity = *max_gust_velocity;
  msg.waiting_for_next_gust = *waiting_for_next_gust;
  msg.direction.x = (*gust_direction)[0];
  msg.direction.y = (*gust_direction)[1];
  msg.direction.z = (*gust_direction)[2];
  msg.gust_active = *gust_active;
  msg.gust_time = *gust_time;
  pub.publish(msg);
  return (v*(gust_direction->Normalize()));
  }



void GazeboWindPlugin::ReadCustomWindField(std::string& custom_wind_field_path) {
  std::ifstream fin;
  fin.open(custom_wind_field_path);
  if (fin.is_open()) {
    std::string data_name;
    float data;
    // Read the line with the variable name.
    while (fin >> data_name) {
      // Save data on following line into the correct variable.
      if (data_name == "min_x:") {
        fin >> min_x_;
      } else if (data_name == "min_y:") {
        fin >> min_y_;
      } else if (data_name == "n_x:") {
        fin >> n_x_;
      } else if (data_name == "n_y:") {
        fin >> n_y_;
      } else if (data_name == "res_x:") {
        fin >> res_x_;
      } else if (data_name == "res_y:") {
        fin >> res_y_;
      } else if (data_name == "vertical_spacing_factors:") {
        while (fin >> data) {
          vertical_spacing_factors_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else if (data_name == "bottom_z:") {
        while (fin >> data) {
          bottom_z_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else if (data_name == "top_z:") {
        while (fin >> data) {
          top_z_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else if (data_name == "u:") {
        while (fin >> data) {
          u_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else if (data_name == "v:") {
        while (fin >> data) {
          v_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else if (data_name == "w:") {
        while (fin >> data) {
          w_.push_back(data);
          if (fin.peek() == '\n') break;
        }
      } else {
        // If invalid data name, read the rest of the invalid line, 
        // publish a message and ignore data on next line. Then resume reading.
        std::string restOfLine;
        getline(fin, restOfLine);
        gzerr << " [gazebo_wind_plugin] Invalid data name '" << data_name << restOfLine <<
              "' in custom wind field text file. Ignoring data on next line.\n";
        fin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      }
    }
    fin.close();
    gzdbg << "[gazebo_wind_plugin] Successfully read custom wind field from text file.\n";
  } else {
    gzerr << "[gazebo_wind_plugin] Could not open custom wind field text file.\n";
  }

}

ignition::math::Vector3d GazeboWindPlugin::LinearInterpolation(
  double position, ignition::math::Vector3d * values, double* points) const {
  ignition::math::Vector3d value = values[0] + (values[1] - values[0]) /
                        (points[1] - points[0]) * (position - points[0]);
  return value;
}

ignition::math::Vector3d GazeboWindPlugin::BilinearInterpolation(
  double* position, ignition::math::Vector3d * values, double* points) const {
  ignition::math::Vector3d intermediate_values[2] = { LinearInterpolation(
                                             position[0], &(values[0]), &(points[0])),
                                           LinearInterpolation(
                                             position[0], &(values[2]), &(points[2])) };
  ignition::math::Vector3d value = LinearInterpolation(
                          position[1], intermediate_values, &(points[4]));
  return value;
}

ignition::math::Vector3d GazeboWindPlugin::TrilinearInterpolation(
  ignition::math::Vector3d link_position, ignition::math::Vector3d * values, double* points) const {
  double position[3] = {link_position.X(),link_position.Y(),link_position.Z()};
  ignition::math::Vector3d intermediate_values[4] = { LinearInterpolation(
                                             position[2], &(values[0]), &(points[0])),
                                           LinearInterpolation(
                                             position[2], &(values[2]), &(points[2])),
                                           LinearInterpolation(
                                             position[2], &(values[4]), &(points[4])),
                                           LinearInterpolation(
                                             position[2], &(values[6]), &(points[6])) };
  ignition::math::Vector3d value = BilinearInterpolation(
    &(position[0]), intermediate_values, &(points[8]));
  return value;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboWindPlugin);

}  // namespace gazebo
