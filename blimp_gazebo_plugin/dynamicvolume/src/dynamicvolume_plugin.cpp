#include "dynamicvolume_plugin.hpp"
#include "ConnectGazeboToRosTopic.pb.h"
#include "ConnectRosToGazeboTopic.pb.h"

namespace gazebo{

  DynamicVolumePlugin::DynamicVolumePlugin()
      : ModelPlugin(),
        node_handle_(0),
        heliumMassKG_(kHeliumMassKG),
        pubs_and_subs_created_(false){}

  DynamicVolumePlugin::~DynamicVolumePlugin(){}

  void DynamicVolumePlugin::Load(physics::ModelPtr _model,
                                    sdf::ElementPtr _sdf){
    if (kPrintOnUpdates) {
      gzdbg << __FUNCTION__ << "() called." << std::endl;
    }

    gzdbg << "_model = " << _model->GetName() << std::endl;

    // Store the pointer to the model
    model_ = _model;
    world_ = model_->GetWorld();

    // Use the robot namespace to create the node handle.
    if (_sdf->HasElement("robotNamespace"))
      namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
      gzerr << "[DynamicVolumePlugin] Please specify a robotNamespace.\n";

    // Create nodehandle
    node_handle_ = transport::NodePtr(new transport::Node());

    // Initialize with default namespace (typically /gazebo/default/)
    node_handle_->Init();

    // Get the link name.
    std::string link_name;
    if (_sdf->HasElement("linkName"))
      link_name = _sdf->GetElement("linkName")->Get<std::string>();
    else
      gzerr << "[dynamicvolume_plugin] Please specify a linkName.\n";
    // Get the pointer to the link.
    link_ = model_->GetLink(link_name);
    if (link_ == NULL) {
      gzthrow("[dynamicvolume_plugin] Couldn't find specified link \""
              << link_name << "\".");
    }

    frame_id_ = link_name;

    // Retrieve the rest of the SDF parameters.
    getSdfParam<std::string>(_sdf, "dynamicvolumetopic", dynamic_volume_topic_, kDefaultDynamicVolumePubTopic);
    getSdfParam<std::string>(_sdf, "heliummasstopic", helium_mass_topic_, kDefaultHeliumMassSubTopic);
    getSdfParam<double>(_sdf, "referenceAltitude", ref_alt_, kDefaultRefAlt);

    if (_sdf->HasElement("heliummass"))
      this->heliumMassKG_ = _sdf->Get<double>("heliummass");

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&DynamicVolumePlugin::OnUpdate, this, _1));

  }

  void DynamicVolumePlugin::OnUpdate(const common::UpdateInfo& _info) {

    if (kPrintOnUpdates) {
      gzdbg << __FUNCTION__ << "() called." << std::endl;
    }

    if (!pubs_and_subs_created_) {
      CreatePubsAndSubs();
      pubs_and_subs_created_ = true;
    }

    common::Time current_time = world_->SimTime();

    // Get the current geometric height.
    double height_geometric_m = ref_alt_ + model_->WorldPose().Pos().Z();

    // Compute the geopotential height.
    double height_geopotential_m = kEarthRadiusMeters * height_geometric_m /
        (kEarthRadiusMeters + height_geometric_m);

    // Compute the temperature at the current altitude.
    double temperature_at_altitude_kelvin =
        kSeaLevelTempKelvin - kTempLapseKelvinPerMeter * height_geopotential_m;

    // Compute the current air pressure.
    double pressure_at_altitude_pascal =
        kPressureOneAtmospherePascals * exp(kAirConstantDimensionless *
            log(kSeaLevelTempKelvin / temperature_at_altitude_kelvin));

    // Compute current volume.
    double volume_at_altitude_meter3 = kHeliumKmolKr * temperature_at_altitude_kelvin / kPressureAtSeaLevelPascal;
    //gzdbg << "volume_at_altitude_meter3 = " << volume_at_altitude_meter3 << std::endl;

    // Compute current airdensity from barometric formula
    double airdensity_ = kAirdensityConstantKgPerMeter3 *
    pow(temperature_at_altitude_kelvin/(temperature_at_altitude_kelvin+kTempLapseKelvinPerMeter*height_geopotential_m),
      1+kGravityMagnitude*kMeanMolecularAirWeightKgPerKmol/kGasConstantNmPerKmolKelvin*kTempLapseKelvinPerMeter);
    //gzdbg << "airdensity_ = " << airdensity_ << std::endl;


    // Compute current Hellium density.
    double heliumdensity_ = heliumMassKG_ / volume_at_altitude_meter3;

    // Compute buoyancy airdensity.
    double density_ = airdensity_ - heliumdensity_;
    //gzdbg << "density = " << density_ << std::endl;

    UpdateForcesAndMoments(volume_at_altitude_meter3, density_, link_);

  }

  void DynamicVolumePlugin::UpdateForcesAndMoments(double volume, double airdensity, physics::LinkPtr link){

    // caculate buoyancy
    const ignition::math::Vector3d buoyancy =
    ignition::math::Vector3d (0, 0, airdensity * volume * kGravityMagnitude);
    //gzdbg << "buoyancy = " << buoyancy << std::endl;
    ignition::math::Pose3d linkFrame = link->WorldPose();

    // rotate buoyancy into the link frame before applying the force.
    ignition::math::Vector3d buoyancyLinkFrame =
         linkFrame.Rot().Inverse().RotateVector(buoyancy);
    link_->AddLinkForce(buoyancyLinkFrame);

  }

  void DynamicVolumePlugin::HeliumMassCallback(
    GzWindSpeedMsgPtr& helium_mass_msg) {

      heliumMassKG_ = helium_mass_msg->velocity().x();
  }

  void DynamicVolumePlugin::CreatePubsAndSubs(){
    // Create temporary "ConnectRosToGazeboTopic" publisher and message
    gazebo::transport::PublisherPtr gz_connect_ros_to_gazebo_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectRosToGazeboTopic>(
          "~/" + kConnectRosToGazeboSubtopic, 1);
    gz_std_msgs::ConnectRosToGazeboTopic connect_ros_to_gazebo_topic_msg;
    // ============================================ //
    // ==== MASS Change MSG SETUP (ROS->GAZEBO) ==== //
    // ============================================ //

    // Wind speed subscriber (Gazebo).
    helium_mass_sub_ =
      node_handle_->Subscribe("~/" + namespace_ + "/" + helium_mass_topic_,
                              &DynamicVolumePlugin::HeliumMassCallback,
                              this);

    connect_ros_to_gazebo_topic_msg.set_ros_topic(namespace_ + "/" +
                                                helium_mass_topic_);
    connect_ros_to_gazebo_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   helium_mass_topic_);
    connect_ros_to_gazebo_topic_msg.set_msgtype(
        gz_std_msgs::ConnectRosToGazeboTopic::WIND_SPEED);
    gz_connect_ros_to_gazebo_topic_pub->Publish(connect_ros_to_gazebo_topic_msg,
                                              true);


    // ============================================ //
    // ========= FLUID PRESSURE MSG SETUP ========= //
    // ============================================ //
    dynamic_volume_pub_ = node_handle_->Advertise<gz_sensor_msgs::FluidPressure>(
        "~/" + namespace_ + "/" + dynamic_volume_topic_, 1);

    gazebo::transport::PublisherPtr connect_gazebo_to_ros_topic_pub =
        node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
            "~/" + kConnectGazeboToRosSubtopic, 1);

    dynamic_volume_pub_ = node_handle_->Advertise<gz_sensor_msgs::FluidPressure>(
        "~/" + namespace_ + "/" + dynamic_volume_topic_, 1);

    gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;
    connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                     dynamic_volume_topic_);
    connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                  dynamic_volume_topic_);
    connect_gazebo_to_ros_topic_msg.set_msgtype(
        gz_std_msgs::ConnectGazeboToRosTopic::FLUID_PRESSURE);
    connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                             true);
  }

  GZ_REGISTER_MODEL_PLUGIN(DynamicVolumePlugin);

}
