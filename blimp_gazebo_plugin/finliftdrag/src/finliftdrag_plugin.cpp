#include "finliftdrag_plugin.hpp"
#include "ConnectRosToGazeboTopic.pb.h"
#include "ConnectGazeboToRosTopic.pb.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(FinLiftDragPlugin)

/////////////////////////////////////////////////
FinLiftDragPlugin::FinLiftDragPlugin()
  : node_handle_(0),
    wind_speed(0, 0, 0),
    cla(1.0),
    cda(0.01),
    cma(0.01),
    rho(1.2041),
    cl0(0),
    cl1(-1),
    cl2(0),
    cw0(0),
    cw1(-1),
    cw2(-1),
    cm0(0),
    cm1(-1),
    cm2(-1),
    pubs_and_subs_created_(false)
{
  this->cp = ignition::math::Vector3d (0, 0, 0);
  this->forward = ignition::math::Vector3d (1, 0, 0);
  this->upward = ignition::math::Vector3d (0, 0, 1);
  this->area = 1.0;
  this->alpha0 = 0.0;
  this->alpha = 0.0;
  this->sweep = 0.0;
  this->velocityStall = 0.0;

  // 90 deg stall
  this->alphaStall = 0.5*M_PI;
  this->claStall = 0.0;

  this->radialSymmetry = false;

  /// \TODO: what's flat plate drag?
  this->cdaStall = 1.0;
  this->cmaStall = 0.0;

  /// how much to change CL per every radian of the control joint value
  this->controlJointRadToCL = 4.0;
}

/////////////////////////////////////////////////
FinLiftDragPlugin::~FinLiftDragPlugin(){}

/////////////////////////////////////////////////
void FinLiftDragPlugin::Load(physics::ModelPtr _model,
                     sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "FinLiftDragPlugin _model pointer is NULL");
  GZ_ASSERT(_sdf, "FinLiftDragPlugin _sdf pointer is NULL");

  this->model = _model;
  this->sdf = _sdf;
  namespace_.clear();

  // Create the node handle.
  node_handle_ = transport::NodePtr(new transport::Node());

  // Initialise with default namespace (typically /gazebo/default/).
  node_handle_->Init();

  this->world = this->model->GetWorld();
  GZ_ASSERT(this->world, "FinLiftDragPlugin world pointer is NULL");

  this->physics = this->world->Physics();
  GZ_ASSERT(this->physics, "FinLiftDragPlugin physics pointer is NULL");

  GZ_ASSERT(_sdf, "FinLiftDragPlugin _sdf pointer is NULL");

  // Get the robot namespace.
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[FinLiftDragPlugin] Please specify a robotNamespace.\n";

  if (_sdf->HasElement("radial_symmetry"))
    this->radialSymmetry = _sdf->Get<bool>("radial_symmetry");

  if (_sdf->HasElement("a0"))
    this->alpha0 = _sdf->Get<double>("a0");

  if (_sdf->HasElement("cla"))
    this->cla = _sdf->Get<double>("cla");

  if (_sdf->HasElement("cda"))
    this->cda = _sdf->Get<double>("cda");

  if (_sdf->HasElement("cma"))
    this->cma = _sdf->Get<double>("cma");

  if (_sdf->HasElement("alpha_stall"))
    this->alphaStall = _sdf->Get<double>("alpha_stall");

  if (_sdf->HasElement("cla_stall"))
    this->claStall = _sdf->Get<double>("cla_stall");

  if (_sdf->HasElement("cda_stall"))
    this->cdaStall = _sdf->Get<double>("cda_stall");

  if (_sdf->HasElement("cma_stall"))
    this->cmaStall = _sdf->Get<double>("cma_stall");
  
  if (_sdf->HasElement("cl0"))
    this->cl0 = _sdf->Get<double>("cl0");
  if (_sdf->HasElement("cl1"))
    this->cl1 = _sdf->Get<double>("cl1");
  if (_sdf->HasElement("cl2"))
    this->cl2 = _sdf->Get<double>("cl2");
  if (_sdf->HasElement("cw0"))
    this->cw0 = _sdf->Get<double>("cw0");
  if (_sdf->HasElement("cw1"))
    this->cw1 = _sdf->Get<double>("cw1");
  if (_sdf->HasElement("cw2"))
    this->cw2 = _sdf->Get<double>("cw2");
  if (_sdf->HasElement("cm0"))
    this->cm0 = _sdf->Get<double>("cm0");
  if (_sdf->HasElement("cm1"))
    this->cm1 = _sdf->Get<double>("cm1");
  if (_sdf->HasElement("cm2"))
    this->cm2 = _sdf->Get<double>("cm2");

  if ((this->cl1 > -1) and (this->cl2  >-1)) {
    this->cla = (this->cl1 - this->cl0) / this->alphaStall;
    if (this->alphaStall<M_PI/2) {
       this->claStall = (this->cl2 - this->cl1) / ((M_PI/2)-this->alphaStall);
    } else {
       this->claStall = 0.0;
    }
    gzdbg << "=============================\n";
    gzdbg << "Auto Calculated cla = " << this->cla;
    gzdbg << "and claStall = " << this->claStall << "\n";
    gzdbg << "=============================\n";
  }

  if ((this->cw1 > -1) and (this->cw2  >-1)) {
    this->cda = (this->cw1 - this->cw0) / this->alphaStall;
    if (this->alphaStall<M_PI/2) {
       this->cdaStall = (this->cw2 - this->cw1) / ((M_PI/2)-this->alphaStall);
    } else {
       this->cdaStall = 0.0;
    }
    gzdbg << "=============================\n";
    gzdbg << "Auto Calculated cda = " << this->cda;
    gzdbg << "and cdaStall = " << this->cdaStall << "\n";
    gzdbg << "=============================\n";
  }

  if ((this->cm1 > -1) and (this->cm2  >-1)) {
    this->cma = (this->cm1 - this->cm0) / this->alphaStall;
    if (this->alphaStall<M_PI/2) {
       this->cmaStall = (this->cm2 - this->cm1) / ((M_PI/2)-this->alphaStall);
    } else {
       this->cmaStall = 0.0;
    }
    gzdbg << "=============================\n";
    gzdbg << "Auto Calculated cma = " << this->cma;
    gzdbg << "and cmaStall = " << this->cmaStall << "\n";
    gzdbg << "=============================\n";
  }

  if (_sdf->HasElement("cp"))
    this->cp = _sdf->Get<ignition::math::Vector3d >("cp");

  // blade forward (-drag) direction in link frame
  if (_sdf->HasElement("forward"))
    this->forward = _sdf->Get<ignition::math::Vector3d >("forward");
  this->forward.Normalize();

  // blade upward (+lift) direction in link frame
  if (_sdf->HasElement("upward"))
    this->upward = _sdf->Get<ignition::math::Vector3d >("upward");
  this->upward.Normalize();

  if (_sdf->HasElement("area"))
    this->area = _sdf->Get<double>("area");

  if (_sdf->HasElement("air_density"))
    this->rho = _sdf->Get<double>("air_density");

  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    // GZ_ASSERT(elem, "Element link_name doesn't exist!");
    std::string linkName = elem->Get<std::string>();
    this->link = this->model->GetLink(linkName);
    // GZ_ASSERT(this->link, "Link was NULL");

    getSdfParam<std::string>(_sdf, "windSpeedSubTopic",
                             wind_speed_sub_topic_,
                             mav_msgs::default_topics::WIND_SPEED);

    publishAirspeed_ = false;
    if (_sdf->HasElement("airspeedPubTopic")) {
        getSdfParam<std::string>(_sdf, "airspeedPubTopic",
                             air_speed_pub_topic_,
                             "indicatedAirspeed");
        publishAirspeed_ = true;
    }

    if (!this->link)
    {
      gzerr << "Link with name[" << linkName << "] not found. "
        << "The FinLiftDragPlugin will not generate forces\n";
    }
    else
    {
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&FinLiftDragPlugin::OnUpdate, this));
    }
  }

  if (_sdf->HasElement("control_joint_name"))
  {
    std::string controlJointName = _sdf->Get<std::string>("control_joint_name");
    this->controlJoint = this->model->GetJoint(controlJointName);
    if (!this->controlJoint)
    {
      gzerr << "Joint with name[" << controlJointName << "] does not exist.\n";
    }
  }

  if (_sdf->HasElement("control_joint_rad_to_cl"))
    this->controlJointRadToCL = _sdf->Get<double>("control_joint_rad_to_cl");
}

/////////////////////////////////////////////////
void FinLiftDragPlugin::OnUpdate()
{
  if (!pubs_and_subs_created_) {
    CreatePubsAndSubs();
    pubs_and_subs_created_ = true;
  }

  GZ_ASSERT(this->link, "Link was NULL");
  // get linear velocity at cp in inertial frame
  ignition::math::Vector3d vel = this->link->WorldLinearVel(this->cp) - wind_speed;
  // gzdbg << "vel: " << vel << "\n";
  // gzdbg << "wind_speed: " << wind_speed << "\n";
  ignition::math::Vector3d velI = vel;
  velI.Normalize();

  // smoothing
  // double e = 0.8;
  // this->velSmooth = e*vel + (1.0 - e)*velSmooth;
  // vel = this->velSmooth;

  if (vel.Length() <= 0.01)
    return;

  // pose of body
  ignition::math::Pose3d pose = this->link->WorldPose();

  // rotate forward and upward vectors into inertial frame
  ignition::math::Vector3d forwardI = pose.Rot().RotateVector(this->forward);

  ignition::math::Vector3d upwardI;
  if (this->radialSymmetry)
  {
    // use inflow velocity to determine upward direction
    // which is the component of inflow perpendicular to forward direction.
    ignition::math::Vector3d tmp = forwardI.Cross(velI);
    upwardI = forwardI.Cross(tmp).Normalize();
  }
  else
  {
    upwardI = pose.Rot().RotateVector(this->upward);
  }

  // spanwiseI: a vector normal to lift-drag-plane described in inertial frame
  ignition::math::Vector3d spanwiseI = forwardI.Cross(upwardI).Normalize();

  const double minRatio = -1.0;
  const double maxRatio = 1.0;
  // check sweep (angle between velI and lift-drag-plane)
  double sinSweepAngle = ignition::math::clamp(
      spanwiseI.Dot(velI), minRatio, maxRatio);

  // get cos from trig identity
  double cosSweepAngle = 1.0 - sinSweepAngle * sinSweepAngle;
  this->sweep = asin(sinSweepAngle);

  // truncate sweep to within +/-90 deg
  while (fabs(this->sweep) > 0.5 * M_PI)
    this->sweep = this->sweep > 0 ? this->sweep - M_PI
                                  : this->sweep + M_PI;

  // angle of attack is the angle between
  // velI projected into lift-drag plane
  //  and
  // forward vector
  //
  // projected = spanwiseI Xcross ( vector Xcross spanwiseI)
  //
  // so,
  // removing spanwise velocity from vel
  ignition::math::Vector3d velInLDPlane = vel - vel.Dot(spanwiseI)*velI;

  // get direction of drag
  ignition::math::Vector3d dragDirection = -velInLDPlane;
  dragDirection.Normalize();

  // get direction of lift
  ignition::math::Vector3d liftI = spanwiseI.Cross(velInLDPlane);
  liftI.Normalize();

  // get direction of moment
  ignition::math::Vector3d momentDirection = spanwiseI;

  // compute angle between upwardI and liftI
  // in general, given vectors a and b:
  //   cos(theta) = a.Dot(b)/(a.Length()*b.Lenghth())
  // given upwardI and liftI are both unit vectors, we can drop the denominator
  //   cos(theta) = a.Dot(b)
  double cosAlpha = ignition::math::clamp(liftI.Dot(upwardI), minRatio, maxRatio);

  // Is alpha positive or negative? Test:
  // forwardI points toward zero alpha
  // if forwardI is in the same direction as lift, alpha is positive.
  // liftI is in the same direction as forwardI?
  if (liftI.Dot(forwardI) >= 0.0)
    this->alpha = this->alpha0 + acos(cosAlpha);
  else
    this->alpha = this->alpha0 - acos(cosAlpha);

  // normalize to within +/-90 deg
  while (fabs(this->alpha) > 0.5 * M_PI)
    this->alpha = this->alpha > 0 ? this->alpha - M_PI
                                  : this->alpha + M_PI;

  // compute dynamic pressure
  double altitude = pose.Pos().Z();
  double thinningFactor = ( 1.0 + ( 0.02 * altitude / 304.8 ));

  double speedInLDPlane = velInLDPlane.Length();
  double rhoAlt = this->rho / thinningFactor; // at high altitude air gets thinner
  double q = 0.5 * rhoAlt * speedInLDPlane * speedInLDPlane; // which reduces dynamic pressure

  // compute airspeed in forward direction
  double trueAirspeed = vel.Dot(forwardI);
  double indicatedAirspeed = trueAirspeed / thinningFactor; // and as such also indicated airspeed

  // compute cl at cp, check for stall, correct for sweep
  double cl;
  if (this->alpha > this->alphaStall)
  {
    cl = (this->cl0 + this->cla * this->alphaStall +
          this->claStall * (this->alpha - this->alphaStall))
         * cosSweepAngle;
    // make sure cl is still great than 0
    cl = std::max(0.0, cl);
  }
  else if (this->alpha < -this->alphaStall)
  {
    cl = (-(this->cl0 + this->cla * this->alphaStall) +
          this->claStall * (this->alpha + this->alphaStall))
         * cosSweepAngle;
    // make sure cl is still less than 0
    cl = std::min(0.0, cl);
  }
  else
    cl = (this->cl0 + this->cla * this->alpha) * cosSweepAngle;

  // modify cl per control joint value
  if (this->controlJoint)
  {
    double controlAngle = this->controlJoint->Position(0);
    cl = cl + this->controlJointRadToCL * controlAngle;
    /// \TODO: also change cm and cd
  }

  // compute lift force at cp
  ignition::math::Vector3d lift = cl * q * this->area * liftI;

  // compute cd at cp, check for stall, correct for sweep
  double cd;
  if (this->alpha > this->alphaStall)
  {
    cd = (this->cw0 + this->cda * this->alphaStall +
          this->cdaStall * (this->alpha - this->alphaStall))
         * cosSweepAngle;
  }
  else if (this->alpha < -this->alphaStall)
  {
    cd = (-(this->cw0 + this->cda * this->alphaStall) +
          this->cdaStall * (this->alpha + this->alphaStall))
         * cosSweepAngle;
  }
  else
    cd = (this->cw0 + this->cda * this->alpha) * cosSweepAngle;

  // make sure drag is positive
  cd = fabs(cd);

  // drag at cp
  ignition::math::Vector3d drag = cd * q * this->area * dragDirection;

  // compute cm at cp, check for stall, correct for sweep
  double cm;
  if (this->alpha > this->alphaStall)
  {
    cm = (this->cm0 + this->cma * this->alphaStall +
          this->cmaStall * (this->alpha - this->alphaStall))
         * cosSweepAngle;
    // make sure cm is still great than 0
    cm = std::max(0.0, cm);
  }
  else if (this->alpha < -this->alphaStall)
  {
    cm = (-(this->cm0 + this->cma * this->alphaStall) +
          this->cmaStall * (this->alpha + this->alphaStall))
         * cosSweepAngle;
    // make sure cm is still less than 0
    cm = std::min(0.0, cm);
  }
  else
    cm = (this->cm0 + this->cma * this->alpha) * cosSweepAngle;

  /// \TODO: implement cm
  /// for now, reset cm to zero, as cm needs testing
  cm = 0.0;

  // compute moment (torque) at cp
  ignition::math::Vector3d moment = cm * q * this->area * momentDirection;

  // moment arm from cg to cp in inertial plane
  ignition::math::Vector3d momentArm = pose.Rot().RotateVector(
    this->cp - this->link->GetInertial()->CoG());
  // gzerr << this->cp << " : " << this->link->GetInertial()->GetCoG() << "\n";

  // force and torque about cg in inertial frame
  ignition::math::Vector3d force = lift + drag;
  // + moment.Cross(momentArm);

  ignition::math::Vector3d torque = moment;
  // - lift.Cross(momentArm) - drag.Cross(momentArm);

  // debug
  //
  // if ((this->link->GetName() == "wing_1" ||
  //      this->link->GetName() == "wing_2") &&
  //     (vel.GetLength() > 50.0 &&
  //      vel.GetLength() < 50.0))
  if (0)
  {
    gzdbg << "=============================\n";
    gzdbg << "sensor: [" << this->GetHandle() << "]\n";
    //gzdbg << "Link: [" << this->link->GetName()
    //      << "] pose: [" << pose
    //       << "] dynamic pressure: [" << q << "]\n";
    // gzdbg << "spd: [" << vel.Length()
    //       << "] vel: [" << vel << "]\n";
    // gzdbg << "LD plane spd: [" << velInLDPlane.Length()
    //       << "] vel : [" << velInLDPlane << "]\n";
    // gzdbg << "forward (inertial): " << forwardI << "\n";
    // gzdbg << "upward (inertial): " << upwardI << "\n";
    // gzdbg << "lift dir (inertial): " << liftI << "\n";
    // gzdbg << "Span direction (normal to LD plane): " << spanwiseI << "\n";
    gzdbg << "sweep: " << (this->sweep*180.0/M_PI) << "\n";
    gzdbg << "alpha: " << (this->alpha*180.0/M_PI) << "\n";
    gzdbg << "lift: " << lift << "\n";
    gzdbg << "drag: " << drag << " cd: "   << cd << " cda: " << this->cda << "\n";
    // gzdbg << "moment: " << moment << "\n";
    // gzdbg << "cp momentArm: " << momentArm << "\n";
    // gzdbg << "vel: " << vel << "\n";
    // gzdbg << "wind_speed: " << wind_speed << "\n";
    gzdbg << "force: " << force << "\n";
    // gzdbg << "torque: " << torque << "\n";
    gzdbg << "=============================\n";
  }

  // Correct for nan or inf
  force.Correct();
  this->cp.Correct();
  torque.Correct();

  // apply forces at cg (with torques for position shift)
  this->link->AddForceAtRelativePosition(force, this->cp);
  this->link->AddTorque(torque);
  
  if (publishAirspeed_) {
      airspeedMsg_.set_data(indicatedAirspeed);
      air_speed_pub_->Publish(airspeedMsg_);
  }
}

/////////////////////////////////////////////////
void FinLiftDragPlugin::WindSpeedCallback(
    GzWindSpeedMsgPtr& wind_speed_msg) {
  if (kPrintOnMsgCallback) {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  wind_speed.X() = wind_speed_msg->velocity().x();
  wind_speed.Y() = wind_speed_msg->velocity().y();
  wind_speed.Z() = wind_speed_msg->velocity().z();
}

/////////////////////////////////////////////////
void FinLiftDragPlugin::CreatePubsAndSubs() {
  gzdbg << __PRETTY_FUNCTION__ << " called." << std::endl;

  // Create temporary "ConnectRosToGazeboTopic" publisher and message
  gazebo::transport::PublisherPtr gz_connect_ros_to_gazebo_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectRosToGazeboTopic>(
          "~/" + kConnectRosToGazeboSubtopic, 1);
  gz_std_msgs::ConnectRosToGazeboTopic connect_ros_to_gazebo_topic_msg;

  // ============================================ //
  // ==== WIND SPEED MSG SETUP (ROS->GAZEBO) ==== //
  // ============================================ //

  // Wind speed subscriber (Gazebo).
  wind_speed_sub_ =
      node_handle_->Subscribe("~/" + namespace_ + "/" + wind_speed_sub_topic_,
                              &FinLiftDragPlugin::WindSpeedCallback,
                              this);

  connect_ros_to_gazebo_topic_msg.set_ros_topic(namespace_ + "/" +
                                                wind_speed_sub_topic_);
  connect_ros_to_gazebo_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                   wind_speed_sub_topic_);
  connect_ros_to_gazebo_topic_msg.set_msgtype(
      gz_std_msgs::ConnectRosToGazeboTopic::WIND_SPEED);
  gz_connect_ros_to_gazebo_topic_pub->Publish(connect_ros_to_gazebo_topic_msg,
                                              true);


  if (publishAirspeed_) {
      gazebo::transport::PublisherPtr connect_gazebo_to_ros_topic_pub =
      node_handle_->Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
          "~/" + kConnectGazeboToRosSubtopic, 1);

  gz_std_msgs::ConnectGazeboToRosTopic connect_gazebo_to_ros_topic_msg;


     air_speed_pub_ = node_handle_->Advertise<gz_std_msgs::Float32>(
         "~/" + namespace_ + "/" + air_speed_pub_topic_, 1);

      connect_gazebo_to_ros_topic_msg.set_gazebo_topic("~/" + namespace_ + "/" +
                                                       air_speed_pub_topic_);
      connect_gazebo_to_ros_topic_msg.set_ros_topic(namespace_ + "/" +
                                                    air_speed_pub_topic_);
      connect_gazebo_to_ros_topic_msg.set_msgtype(
          gz_std_msgs::ConnectGazeboToRosTopic::FLOAT_32);
      connect_gazebo_to_ros_topic_pub->Publish(connect_gazebo_to_ros_topic_msg,
                                               true);
  }

}
