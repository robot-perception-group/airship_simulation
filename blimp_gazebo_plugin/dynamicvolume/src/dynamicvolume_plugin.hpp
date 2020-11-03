#ifndef GAZEBO_PLUGINS_DYNAMICVOLUMEPLUGIN_HH_
#define GAZEBO_PLUGINS_DYNAMICVOLUMEPLUGIN_HH_

#include <chrono>
#include <iostream>
#include <random>
#include <cmath>

#include <glog/logging.h>
#include <ros/ros.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include "gazebo/msgs/msgs.hh"
#include <gazebo/physics/physics.hh>
///#include <sensor_msgs/FluidPressure.h>
#include <mav_msgs/default_topics.h>

#include "FluidPressure.pb.h"
#include "WindSpeed.pb.h"
#include "common.h"

namespace gazebo{

  typedef const boost::shared_ptr<const gz_mav_msgs::WindSpeed>
      GzWindSpeedMsgPtr;

  // Constants
  static constexpr double kGasConstantNmPerKmolKelvin = 8314.32;
  static constexpr double kMeanMolecularAirWeightKgPerKmol = 28.9644;
  static constexpr double kGravityMagnitude = 9.80665;
  static constexpr double kEarthRadiusMeters = 6356766.0;
  static constexpr double kPressureOneAtmospherePascals = 101325.0;
  static constexpr double kPressureAtSeaLevelPascal = 97292.9;
  static constexpr double kSeaLevelTempKelvin = 288.15;
  static constexpr double kTempLapseKelvinPerMeter = 0.0065;
  static constexpr double kAirConstantDimensionless = kGravityMagnitude *
      kMeanMolecularAirWeightKgPerKmol /
          (kGasConstantNmPerKmolKelvin * -kTempLapseKelvinPerMeter);
  static constexpr double kAirdensityConstantKgPerMeter3 = 1.2250;
  static constexpr double kHeliumKmolKr = 3607.95497;
  static constexpr double kHeliumMassKG = 1.723;

  // Default values
  static const std::string kDefaultDynamicVolumePubTopic = "dynamic_volume";
  static const std::string kDefaultHeliumMassSubTopic = "helium_mass";
  static constexpr double kDefaultRefAlt = 341.0; /* m, Tuebingen: h=+341m, WGS84) */
  static constexpr double kDefaultPressureVar = 0.0; /* Pa^2, pressure variance */


  class DynamicVolumePlugin : public ModelPlugin {
    public:
      /// Constructor
      DynamicVolumePlugin();
      /// Destructor
      virtual ~DynamicVolumePlugin();

      typedef std::normal_distribution<> NormalDistribution;

    protected:
      /// load paramaters
      void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

      /// up date start event
      void OnUpdate(const common::UpdateInfo&);

      /// calculate force to be applied to model
      void UpdateForcesAndMoments(double volume, double airdensity, physics::LinkPtr link_);

      void HeliumMassCallback(GzWindSpeedMsgPtr& helium_mass_msg);
    private:
      /// flag that is set true when CreatePubsAndSubs() is called to prevent CreatePubsAndSubs from being called on every OnUpdate().
      bool pubs_and_subs_created_;

      /// create all publishers and subscribers
      void CreatePubsAndSubs();

      /// nodehandle for gazebo
      gazebo::transport::NodePtr node_handle_;

      /// DynamicVolumePlugin messages publisher
      gazebo::transport::PublisherPtr dynamic_volume_pub_;
      gazebo::transport::SubscriberPtr helium_mass_sub_;

      /// Transport namespace
      std::string namespace_;

      /// topic name for publisher messages
      std::string dynamic_volume_topic_;
      std::string helium_mass_topic_;

      /// Frame ID for messages.
      std::string frame_id_;

      /// Pointer to the world.
      physics::WorldPtr world_;

      /// Pointer to the model.
      physics::ModelPtr model_;

      /// Pointer to the link.
      physics::LinkPtr link_;

      /// Pointer to the update event connection.
      event::ConnectionPtr updateConnection_;

      /// Reference altitude (meters).
      double ref_alt_;

      /// Remaining Helium Mass (Kg).
      double heliumMassKG_;

      std::mt19937 random_generator_;

  };


} // namespace gazebo

#endif
