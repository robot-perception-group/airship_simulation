//
// Created by glawless on 23.05.17.
//

#include <mrpt/poses.h>
#include <mrpt/math.h>
#include <mrpt_bridge/mrpt_bridge.h>
#include <target_tracker_distributed_kf/DistributedKF3D.h>
#include <ros/callback_queue.h>
#include <cmath>

namespace target_tracker_distributed_kf {

    static const std::string world_frame{"world"};

    DistributedKF3D::DistributedKF3D() : pnh_("~"),
    I((int) state_size, (int) state_size),
    Hself((int) measurement_state_size, (int) state_size),
    Hother((int) measurement_state_size, (int) state_size),
    R((int) state_size, (int) state_size) {
        // Advertise dynamic reconfigure server
        dynamic_reconfigure::Server<KalmanFilterParamsConfig>::CallbackType cb(
                boost::bind(&DistributedKF3D::dynamicReconfigureCallback, this, _1, _2));
        dyn_rec_server_.setCallback(cb);

        // Some parameters related to initial uncertainty
        pnh_.getParam("initialUncertaintyPosXY", initialUncertaintyPosXY);
        pnh_.getParam("initialUncertaintyPosZ", initialUncertaintyPosZ);
        pnh_.getParam("initialUncertaintyVelXY", initialUncertaintyVelXY);
        pnh_.getParam("initialUncertaintyVelZ", initialUncertaintyVelZ);
        pnh_.getParam("initialUncertaintyOffsetXY", initialUncertaintyOffsetXY);
        pnh_.getParam("initialUncertaintyOffsetZ", initialUncertaintyOffsetZ);

        // false positive detection
        pnh_.getParam("falsePositiveThresholdSigma", falsePositiveThresholdSigma);

        // Advertise publish topic
        string pub_topic{"target_tracker/pose"};
        pnh_.getParam("pub_topic", pub_topic);
        targetPub_ = nh_.advertise<PoseWithCovarianceStamped>(pub_topic, 10);

        string velPub_topic{"target_tracker/twist"};
        pnh_.getParam("velPub_topic", velPub_topic);
        targetVelPub_ = nh_.advertise<TwistWithCovarianceStamped>(velPub_topic, 10);

        // Advertise publish topic
        string offset_topic{"target_tracker/offset"};
        pnh_.getParam("offset_topic", offset_topic);
        offsetPub_ = nh_.advertise<PoseWithCovarianceStamped>(offset_topic, 10);

        ROS_INFO_STREAM("Publishing to " << targetPub_.getTopic());
        ROS_INFO_STREAM("Offset Publishing to " << offsetPub_.getTopic());

        // Time threshold
        pnh_.getParam("reset_time_threshold", time_threshold);

        // Wait for a valid stamp
        ROS_INFO("Waiting for valid stamp");
        ros::Time::waitForValid();
        ROS_INFO("Time received, proceeding");

        // Cache definition
        int cache_size{20};
        pnh_.getParam("cache_size", cache_size);
        ROS_ASSERT(cache_size > 0);
        state_cache_.set_cache_size((std::size_t) cache_size);

        // Initialize the filter
        initializeFilter();
        initializeSubscribers();
    }

    void DistributedKF3D::initializeSubscribers() {
        // Self and other robots info
        int robotID{0};
        int numRobots{0};
        pnh_.getParam("robotID", robotID);
        pnh_.getParam("numRobots", numRobots);

        // Pose subscriber
        std::string pose_topic{"pose"};
        pnh_.getParam("pose_topic", pose_topic);
        pose_sub_ = nh_.subscribe(pose_topic, 300, &DistributedKF3D::predictAndPublish, this);

        // Measurement subscribers
        string measurement_suffix_self{"/nonono"};
        string measurement_suffix{"/nonono"};
        pnh_.getParam("measurement_topic_suffix_self", measurement_suffix_self);
        pnh_.getParam("measurement_topic_suffix", measurement_suffix);

        selfcallbackhandler= unique_ptr<Callbackhandler>(new Callbackhandler(this,true,robotID));
        self_sub_ = nh_.subscribe(measurement_suffix_self, 50, &Callbackhandler::callback, selfcallbackhandler.get());
        ROS_INFO_STREAM("Registered self measurement subscriber for topic " << self_sub_.getTopic());

        for (int robot = 1; robot <= numRobots; robot++) {
            if (robot == robotID)
                continue;

            std::shared_ptr<Callbackhandler> cb(new Callbackhandler(this,false,robot));
            callbackhandlers.emplace_back(cb);
            const auto other_topic = "/machine_" + to_string(robot) + '/' + measurement_suffix;
            other_subs_.emplace_back(unique_ptr<ros::Subscriber>(
                        new ros::Subscriber(nh_.subscribe(
                                other_topic, 50, &Callbackhandler::callback,cb.get()))
                        ));

            ROS_INFO_STREAM(
                    "Registered other robot's measurements subscriber for topic " << other_subs_.back()->getTopic());
        }
    }

    void DistributedKF3D::initializeFilter() {
        // If there is a last element, grab its frame id; if not, use default world_frame
        std::string frame_id{world_frame};
        if (!state_cache_.empty())
            frame_id = state_cache_.back().frame_id;

        // Reset the cache
        state_cache_.clear();

        // Put an initial unknown estimate in the cache
        std_msgs::Header h;
        h.frame_id = frame_id;
        h.stamp = ros::Time::now();
        CacheElement first_element(h, state_size, true, 0);
        setUnknownInitial(first_element);
        first_element.frame_id = frame_id;
        state_cache_.insert_ordered(first_element);

        ROS_INFO("The filter was (re)initialized");
    }

    //void DistributedKF3D::selfMeasurementCallback(const PoseWithCovarianceStamped& msg) {
    //  measurementsCallback(msg, true);
    //      ROS_WARN("shouldhaveupdated...");
    //}

    //void DistributedKF3D::otherMeasurementCallback(const PoseWithCovarianceStamped& msg) {
    //  measurementsCallback(msg, false);
    //}

    void DistributedKF3D::measurementsCallback(const PoseWithCovarianceStamped& msg, const bool isSelf, const int robot) {
        if (detectBackwardsTimeJump()) {
            ROS_WARN("Backwardstimejump in cache - ignoring update");
            return;
        }

        if (state_cache_.empty()) {
            ROS_WARN("Cache is empty - ignoring update");
            return;
        }

        //    ROS_INFO("Measurement callback");
        // Create a new element for the cache
        CacheElement new_element(state_size, msg, isSelf, robot);

        // Insert this element into cache, which returns the iterator at insert position
        auto it = state_cache_.insert_ordered(new_element);

        // Check if failure to insert - this would be due to a very old message
        // Currently we decide to just alert the user, but this would be a good place to detect a jumping backwards and reset the filter
        if (it == state_cache_.end()) {
            ROS_WARN_STREAM(
                    "Trying to insert a measurement that is too old! This is its stamp " << msg.header.stamp << std::endl
                    << "Did you forget to reiniatilize the node after going back to the past e.g. stop and restart playback?");
            return;
        }

        // Rare, but may occur
        if(it == state_cache_.begin())
            ++it;

        // In a loop until we go through the whole cache, keep predicting and updating
        for (; it != state_cache_.end(); ++it) {
            if (!predict(*(it - 1), *it)) {
                ROS_WARN("Prediction step failed!");
                return;
            }
            if (it->measurements.size() > 0) {
                if (!update(*it)) {
                    ROS_WARN("Rewind/Update failed!");
                    return;
                }
            }
        }
        //if (isSelf) {
        //   ROS_INFO_STREAM(state_cache_);
        //}

    }

    void DistributedKF3D::setUnknownInitial(CacheElement &elem) {

        elem.cov << initialUncertaintyPosXY, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, initialUncertaintyPosXY, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, initialUncertaintyPosZ, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, initialUncertaintyVelXY, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, initialUncertaintyVelXY, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, initialUncertaintyVelZ, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, initialUncertaintyOffsetXY, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, initialUncertaintyOffsetXY, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, initialUncertaintyOffsetZ;
    }

    bool DistributedKF3D::predict(const CacheElement &in, CacheElement &out) {

        // Easy access
        const VectorXd &ins = in.state;
        VectorXd &outs = out.state;

        // Time past from one to next
        if (!out.stamp.isValid() || !in.stamp.isValid()) {
            ROS_WARN("One of the stamps is invalid, returning false from predict() without doing anything else");
            return false;
        }

        const double deltaT = out.stamp.toSec() - in.stamp.toSec();

        if (deltaT > time_threshold) {
            ROS_WARN_STREAM("It's been a long time since there was an update (" << deltaT
                    << " seconds). Resetting the filter to be safe... position(0,0,0) and high uncertainty");
            initializeFilter();
            return false;
        }

        const static double velocityDecayTo = 0.1;
        const double velocityDecayAlpha = pow(velocityDecayTo, 1.0 / velocityDecayTime);
        const double velocityDecayFactor = pow(velocityDecayAlpha, deltaT);
        const double velocityIntegralFactor = (velocityDecayFactor - 1) / log(velocityDecayAlpha);

        // Decreasing velocity model
        outs(0) = ins(0) + ins(3) * velocityIntegralFactor;
        outs(1) = ins(1) + ins(4) * velocityIntegralFactor;
        outs(2) = ins(2) + ins(5) * velocityIntegralFactor;

        outs(3) = ins(3) * velocityDecayFactor;
        outs(4) = ins(4) * velocityDecayFactor;
        outs(5) = ins(5) * velocityDecayFactor;

        const static double offsetDecayTo = 0.1;
        const double offsetDecayAlpha = pow(offsetDecayTo, 1.0 / offsetDecayTime);
        const double offsetDecayFactor = pow(offsetDecayAlpha, deltaT);
        //const double offsetIntegralFactor = (offsetDecayFactor - 1)/log(offsetDecayAlpha);

        outs(6) = posGlobalOffsetBiasX + ((ins(6) - posGlobalOffsetBiasX) * offsetDecayFactor);
        outs(7) = posGlobalOffsetBiasY + ((ins(7) - posGlobalOffsetBiasY) * offsetDecayFactor);
        outs(8) = posGlobalOffsetBiasZ + ((ins(8) - posGlobalOffsetBiasZ) * offsetDecayFactor);

        // Construct jacobian G based on deltaT
        MatrixXd G((int) state_size, (int) state_size);
        populateJacobianG(G, deltaT);

        // Update covariance from one to next
        out.cov = MatrixXd((G * in.cov * G.transpose()) + (deltaT / 1.0) * R);

        return true;
    }

    bool DistributedKF3D::update(CacheElement &elem) {

        //    ROS_INFO("Update");

        if (elem.measurements.empty()) {
            ROS_WARN("Tried to perform update step with no measurements in element. Returning without doing anything");
            return false;
        }

        // Find out closest measurement to current state estimate and use that one
        int closest_idx = -1;
        double min_error{DBL_MAX};
        for (size_t i = 0; i < elem.measurements.size(); ++i) {

            const auto measurement = elem.measurements[i];

            double difference[3]{measurement->pose.position.x - elem.state(0),
                measurement->pose.position.y - elem.state(1),
                measurement->pose.position.z - elem.state(2)};

            double sqr_error{
                sqrt(difference[0] * difference[0] + difference[1] * difference[1] + difference[2] * difference[2])};

            if (sqr_error < min_error) {
                min_error = sqr_error;
                closest_idx = i;
            }
        }

        if (closest_idx < 0 || closest_idx > (int) elem.measurements.size()) {
            ROS_ERROR("Something went wrong, couldn't didn't find the closest measurement");
            return false;
        }

        const auto closest_measurement = elem.measurements[closest_idx];
        bool isSelf = elem.isSelfRobot;

        if (isSelf) {
            // check if measurement is a false positive. False positives are allowed to be fused for person estimate, but hazardous for self pose estimate
            // we set isSelf to false if we think we had a false positive

            // the state is the current person estimate
            PoseWithCovariance state;
            state.pose.position.x = elem.state(0);
            state.pose.position.y = elem.state(1);
            state.pose.position.z = elem.state(2);
            state.pose.orientation.w = 1;
            state.pose.orientation.x = 0;
            state.pose.orientation.y = 0;
            state.pose.orientation.z = 0;
            state.covariance = { elem.cov(0*9+0), elem.cov(0*9+1), elem.cov(0*9+2), 0, 0, 0,  elem.cov(1*9+0), elem.cov(1*9+1), elem.cov(1*9+2), 0, 0, 0,  elem.cov(2*9+0), elem.cov(2*9+1), elem.cov(2*9+2), 0, 0, 0 };

            // the measurement distribution is the distribution of the measurement with zero mean
            PoseWithCovariance distribution(*closest_measurement);
            distribution.pose.position.x = 0;
            distribution.pose.position.y = 0;
            distribution.pose.position.z = 0;
            distribution.pose.orientation = state.pose.orientation;

            // state merged with measurement distribution gives the expected mean distribution for new measurements under current state and measurement covariance
            PoseWithCovariance merged;
            pose_cov_ops::compose(state,distribution,merged);

            // calculate normalized expectance density at the mean of the observed measurement
            mrpt::math::CMatrixDouble31 statemean,measurementmean;
            mrpt::math::CMatrixDouble33 expectance;
            statemean << elem.state(0), elem.state(1), elem.state(2);
            measurementmean << closest_measurement->pose.position.x, closest_measurement->pose.position.y, closest_measurement->pose.position.z;
            expectance << merged.covariance[0*6+0], merged.covariance[0*6+1], merged.covariance[0*6+2], merged.covariance[1*6+0], merged.covariance[1*6+1], merged.covariance[1*6+2], merged.covariance[2*6+0], merged.covariance[2*6+1], merged.covariance[2*6+2];
            double density = mrpt::math::normalPDF(measurementmean, statemean, expectance) / mrpt::math::normalPDF(statemean, statemean, expectance);

            // normalizeed density function with sigma=1 and mu=0:  e^(-1/2 * x^2 )
            // then x = sqrt(-2*log(density))
            double x = sqrt(-2*log(density));

            // ignore measurement for self pose estimation if it is less likely to be a true positive than the prior likelihood for false positives.
            if ( x > falsePositiveThresholdSigma ) {
                ROS_INFO_STREAM("Person Measurement likelihood " << x << " * sigma beyond threshold of " << falsePositiveThresholdSigma << " * sigma. Discarding!");
                isSelf = false;
            }
        }

        const auto &H = isSelf ? Hself : Hother;

        MatrixXd Q((int) measurement_state_size, (int) measurement_state_size);
        //populateJacobianQ(Q, closest_measurement);
        Q <<  closest_measurement->covariance[0] , closest_measurement->covariance[1] , closest_measurement->covariance[2], 0.0 , 0.0 , 0.0
            , closest_measurement->covariance[6] , closest_measurement->covariance[7] , closest_measurement->covariance[8], 0.0 , 0.0 , 0.0
            , closest_measurement->covariance[12] , closest_measurement->covariance[13] , closest_measurement->covariance[14], 0.0 , 0.0 , 0.0
            , 0.0 , 0.0 , 0.0, elem.cov(0) + closest_measurement->covariance[0] , elem.cov(1) + closest_measurement->covariance[1] , elem.cov(2) + closest_measurement->covariance[2]
            , 0.0 , 0.0 , 0.0, elem.cov(9) + closest_measurement->covariance[6] , elem.cov(10) + closest_measurement->covariance[7] , elem.cov(11) + closest_measurement->covariance[8]
            , 0.0 , 0.0 , 0.0, elem.cov(18) + closest_measurement->covariance[12] , elem.cov(19) + closest_measurement->covariance[13] , elem.cov(20) + closest_measurement->covariance[14];

        MatrixXd K = elem.cov * H.transpose() * (H * elem.cov * H.transpose() + Q).inverse();

        VectorXd e_measurement((int) measurement_state_size);

        // we aren't really measuring the offset, we can only measure the difference between observed and predicted target, which should be offset corrected already

        double measured_offset_x = elem.state(6) - (closest_measurement->pose.position.x - elem.state(0));
        double measured_offset_y = elem.state(7) - (closest_measurement->pose.position.y - elem.state(1));
        double measured_offset_z = elem.state(8) - (closest_measurement->pose.position.z - elem.state(2));

        e_measurement
            << closest_measurement->pose.position.x, closest_measurement->pose.position.y, closest_measurement->pose.position.z, measured_offset_x, measured_offset_y, measured_offset_z;

        VectorXd e_predicted((int) measurement_state_size);
        e_predicted << elem.state(0), elem.state(1), elem.state(2), elem.state(6), elem.state(7), elem.state(8);

        // Update
        elem.state = elem.state + K * (e_measurement - e_predicted);
        elem.cov = (I - K * H) * elem.cov;

        return true;
    }

    void DistributedKF3D::predictAndPublish(const uav_msgs::uav_poseConstPtr &pose) {

        if (state_cache_.empty())
            return;

        //    ROS_INFO_STREAM(state_cache_);
        // Always self robot because predict is only called for self poses
        CacheElement tmp_element(pose->header, state_size, true,0);

        const auto last = state_cache_.back();
        if (!predict(last, tmp_element))
            return;

        //    ROS_INFO("Predict and Publish");
        publishStateAndCov(tmp_element);
    }

    void DistributedKF3D::initializeStaticMatrices() {
        I << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

        Hself << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

        Hother << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        R << noisePosXVar, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, noisePosYVar, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, noisePosZVar, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, noiseVelXVar, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, noiseVelYVar, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, noiseVelZVar, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, noiseOffXVar, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, noiseOffYVar, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, noiseOffZVar;
    }

    void DistributedKF3D::populateJacobianG(MatrixXd &G, const double deltaT) {
        // offset assumed independent from target detection
        G << 1.0, 0.0, 0.0, deltaT, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 1.0, 0.0, 0.0, deltaT, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 1.0, 0.0, 0.0, deltaT, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0
            , 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    }

    void DistributedKF3D::populateJacobianQ(MatrixXd &Q, const PoseWithCovariance &pcov) {

        Q << pcov.covariance[0], pcov.covariance[1], pcov.covariance[2], 0.0, 0.0, 0.0
            , pcov.covariance[6], pcov.covariance[7], pcov.covariance[8], 0.0, 0.0, 0.0
            , pcov.covariance[12], pcov.covariance[13], pcov.covariance[14], 0.0, 0.0, 0.0
            , 0.0, 0.0, 0.0, pcov.covariance[0], pcov.covariance[1], pcov.covariance[2]
            , 0.0, 0.0, 0.0, pcov.covariance[6], pcov.covariance[7], pcov.covariance[8]
            , 0.0, 0.0, 0.0, pcov.covariance[12], pcov.covariance[13], pcov.covariance[14];
    }

    void DistributedKF3D::publishStateAndCov(const CacheElement &elem) {

        msg_.header.frame_id = elem.frame_id;
        msg_.header.stamp = elem.stamp;

        msg_.pose.pose.position.x = elem.state[0];
        msg_.pose.pose.position.y = elem.state[1];
        msg_.pose.pose.position.z = elem.state[2];
        msg_.pose.covariance[0 * 6 + 0] = elem.cov(0 * 9 + 0);
        msg_.pose.covariance[0 * 6 + 1] = elem.cov(0 * 9 + 1);
        msg_.pose.covariance[0 * 6 + 2] = elem.cov(0 * 9 + 2);
        msg_.pose.covariance[1 * 6 + 0] = elem.cov(1 * 9 + 0);
        msg_.pose.covariance[1 * 6 + 1] = elem.cov(1 * 9 + 1);
        msg_.pose.covariance[1 * 6 + 2] = elem.cov(1 * 9 + 2);
        msg_.pose.covariance[2 * 6 + 0] = elem.cov(2 * 9 + 0);
        msg_.pose.covariance[2 * 6 + 1] = elem.cov(2 * 9 + 1);
        msg_.pose.covariance[2 * 6 + 2] = elem.cov(2 * 9 + 2);

        msg_.pose.pose.orientation.w = 1.0;

        targetPub_.publish(msg_);

        velMsg_.twist.twist.linear.x = elem.state[3];
        velMsg_.twist.twist.linear.y = elem.state[4];
        velMsg_.twist.twist.linear.z = elem.state[5];
        targetVelPub_.publish(velMsg_);

        ///*
        msg_.pose.pose.position.x = elem.state[6];
        msg_.pose.pose.position.y = elem.state[7];
        msg_.pose.pose.position.z = elem.state[8];
        msg_.pose.covariance[0 * 6 + 0] = elem.cov(6 * 9 + 6);
        msg_.pose.covariance[0 * 6 + 1] = elem.cov(6 * 9 + 7);
        msg_.pose.covariance[0 * 6 + 2] = elem.cov(6 * 9 + 8);
        msg_.pose.covariance[1 * 6 + 0] = elem.cov(7 * 9 + 6);
        msg_.pose.covariance[1 * 6 + 1] = elem.cov(7 * 9 + 7);
        msg_.pose.covariance[1 * 6 + 2] = elem.cov(7 * 9 + 8);
        msg_.pose.covariance[2 * 6 + 0] = elem.cov(8 * 9 + 6);
        msg_.pose.covariance[2 * 6 + 1] = elem.cov(8 * 9 + 7);
        msg_.pose.covariance[2 * 6 + 2] = elem.cov(8 * 9 + 8);
        // */
        /*
           // This code would publish a zero offset with static variance regardless of
           // tracked offset state, as such effectively disabling offset correction and
           // reverting to previous behaviour
           msg_.pose.pose.position.x = 0;
           msg_.pose.pose.position.y = 0;
           msg_.pose.pose.position.z = 0;
           msg_.pose.covariance[0 * 6 + 0] = 1;
           msg_.pose.covariance[0 * 6 + 1] = 0;
           msg_.pose.covariance[0 * 6 + 2] = 0;
           msg_.pose.covariance[1 * 6 + 0] = 0;
           msg_.pose.covariance[1 * 6 + 1] = 1;
           msg_.pose.covariance[1 * 6 + 2] = 0;
           msg_.pose.covariance[2 * 6 + 0] = 0;
           msg_.pose.covariance[2 * 6 + 1] = 0;
           msg_.pose.covariance[2 * 6 + 2] = 2;
           */
        /*
           msg_.pose.pose.position.x = 0;
           msg_.pose.pose.position.y = 0;
           msg_.pose.pose.position.z = 0;
           msg_.pose.covariance[0 * 6 + 0] = 1e-6; // groundtruth
           msg_.pose.covariance[0 * 6 + 1] = 0;
           msg_.pose.covariance[0 * 6 + 2] = 0;
           msg_.pose.covariance[1 * 6 + 0] = 0;
           msg_.pose.covariance[1 * 6 + 1] = 1e-6;
           msg_.pose.covariance[1 * 6 + 2] = 0;
           msg_.pose.covariance[2 * 6 + 0] = 0;
           msg_.pose.covariance[2 * 6 + 1] = 0;
           msg_.pose.covariance[2 * 6 + 2] = 1e-4;
           */

        offsetPub_.publish(msg_);

        // Debug - output full state
        //    ROS_INFO_STREAM("Full state at time " << ros::Time::now() << std::endl << elem.state << std::endl << "And covariance " << std::endl << elem.cov);
    }

    void DistributedKF3D::dynamicReconfigureCallback(KalmanFilterParamsConfig &config,
            uint32_t level) {

        ROS_INFO("Received reconfigure request");
        noisePosXVar = config.noisePosXVar;
        noiseVelXVar = config.noiseVelXVar;
        noiseOffXVar = config.noiseOffXVar;

        noisePosYVar = config.noisePosYVar;
        noiseVelYVar = config.noiseVelYVar;
        noiseOffYVar = config.noiseOffYVar;

        noisePosZVar = config.noisePosZVar;
        noiseVelZVar = config.noiseVelZVar;
        noiseOffZVar = config.noiseOffZVar;

        posGlobalOffsetBiasX = config.posGlobalOffsetBiasX;
        posGlobalOffsetBiasY = config.posGlobalOffsetBiasY;
        posGlobalOffsetBiasZ = config.posGlobalOffsetBiasZ;

        velocityDecayTime = config.velocityDecayTime;
        offsetDecayTime = config.offsetDecayTime;

        falsePositiveThresholdSigma = config.falsePositiveThresholdSigma;

        // Reinitialize matrices
        initializeStaticMatrices();
        ROS_INFO_STREAM("Process noise matrix" << std::endl << R);

        // Reinitialize filter
        initializeFilter();
    }

    bool DistributedKF3D::detectBackwardsTimeJump() {
        // Do not detect if not using sim time
        const static bool using_sim_time = ros::Time::isSimTime();
        if (!using_sim_time)
            return false;

        static auto time = ros::Time::now();

        if (ros::Time::now() < time) {
            // Jump backwards detected, reset interface
            ROS_WARN("Backwards jump in time detected, performing reset");
            initializeFilter();
            time = ros::Time::now();
            return true;
        }
        time = ros::Time::now();
        return false;
    }
}
