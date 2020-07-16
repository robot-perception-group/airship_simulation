//
// Created by glawless on 11.05.17.
//

#ifndef POSE_COV_OPS_INTERFACE_POSECOVOPSINTERFACE_H
#define POSE_COV_OPS_INTERFACE_POSECOVOPSINTERFACE_H

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/cache.h>
#include <pose_cov_ops/pose_cov_ops.h>
#include <unordered_map>
#include <vector>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <iostream>

namespace pose_cov_ops {
  namespace interface {

    using geometry_msgs::PoseWithCovariance;
    using geometry_msgs::PoseWithCovarianceStamped;
    using geometry_msgs::Point;

    typedef message_filters::Cache<PoseWithCovarianceStamped> poseCache_t;
    typedef message_filters::Subscriber<PoseWithCovarianceStamped> poseSub_t;

    template<typename K>
    struct topicSubInfo {
      std::string name;
      K key;
      unsigned int cache_size;
      uint32_t queue_size;

      topicSubInfo(std::string _name, K _key, unsigned int _cache_size, uint32_t _queue_size) : name(_name), key(_key), cache_size(_cache_size), queue_size(_queue_size) {};
    };

    template<typename K>
    class Interface {

    private:
      std::vector<std::unique_ptr<poseSub_t>> subs_;
      std::unordered_map<K, std::unique_ptr<poseCache_t>> cache_map_;

    public:
      Interface(const std::vector<topicSubInfo<K>> &pose_topics, ros::NodeHandle &handle);

      ///@brief Returns the oldest pose time available in the cache for a given pose key
      ///@param [in] key pose key
      ///@return ros::Time object
      ///@throw std::out_of_range if a key does not exist in the cache map
      ros::Time oldestTimeInCache(K key) const;

      ///@brief Outputs p1(+)p2, a new pose "as if" p1 was the origin for p2
      ///@param [in] in_pose the pose to be transformed, in the frame corresponding to transformation_key pose
      ///@param [in] transformation_key the key corresponding to the transforming (connecting) pose
      ///@param [in] t the time to use when searching for the poses in the cache
      ///@param [out] out_pose transformed pose in the parent frame
      ///@return true if all poses were found for time t, false otherwise
      ///@throw std::out_of_range if a key does not exist in the cache map
      ///@example in_pose is an object seen from the robot, reference_pose is the robot pose in the world, and you want the object in world
      template<typename P>
      bool compose_up(const P& in_pose, const K &transformation_key, const ros::Time &t, PoseWithCovariance &out_pose);

      ///@brief Overload of compose_up for a geometry_msgs::Point, which transforms it into a Pose, does the composition, then gives back a resulting Point
      bool compose_up(const Point& in_pose, const K &transformation_key, const ros::Time &t, PoseWithCovariance &out_pose);

      ///@brief Outputs (((p1+p2)+p3)+...+plast) , where each succeeding pose is the composed "as if" the previous is the origin for it
      ///@param [in] in_pose the pose to be transformed, in the frame corresponding to composition of the transformation_keys_ordered poses
      ///@param [in] transformation_keys_ordered the ordered keys corresponding to the transforming tree poses in composition order
      ///@param [in] t the time to use when searching for the poses in the cache
      ///@param [out] out_pose transformed pose in the parent frame
      ///@return true if all poses were found for time t, false otherwise or if there are less than 2 keys in the key vector
      ///@throw std::out_of_range if a key does not exist in the cache map
      template<typename P>
      bool compose_up(const P& in_pose, const std::vector<K> &transformation_keys_ordered, const ros::Time &t, PoseWithCovariance &out_pose);

      ///@brief Overload of compose_up for a geometry_msgs::Point, which transforms it into a Pose, does the composition, then gives back a resulting Point
      bool compose_up(const Point& in_pose, const std::vector<K> &transformation_keys_ordered, const ros::Time &t, PoseWithCovariance &out_pose);


      ///@brief Outputs p2(-)p1, a new pose that is p2 "seen from" p1
      ///@param [in] in_pose the pose to be transformed, in the parent frame (p1)
      ///@param [in] transformation_key the key corresponding to the transforming (connecting) pose (p2)
      ///@param [in] t the time to use when searching for the poses in the cache
      ///@param [out] out_pose transformed pose in the transformation_key pose frame
      ///@return true if all poses were found for time t, false otherwise
      ///@throw std::out_of_range if a key does not exist in the cache map
      ///@example in_pose is an object in the world frame, reference_pose is your robot pose, and you want the object seen from the robot
      template<typename P>
      bool compose_down(const P& in_pose, const K &transformation_key, const ros::Time &t, PoseWithCovariance &out_pose);

      ///@brief Overload of compose_down for a geometry_msgs::Point, which transforms it into a Pose, does the composition, then gives back a resulting Point
      bool compose_down(const Point& in_point, const K &transformation_key, const ros::Time &t, PoseWithCovariance &out_pose);

      ///@brief Outputs plast-(((p1+p2)+p3)+...)) , corresponding to plast seen from the upwards composition of all the other poses
      ///@param [in] in_pose the pose to be transformed, in the parent frame (plast)
      ///@param [in] transformation_keys_ordered the ordered keys corresponding to the transforming tree poses in composition order
      ///@param [in] t the time to use when searching for the poses in the cache
      ///@param [out] out_pose transformed pose in the composition of transformation_key poses
      ///@return true if all poses were found for time t, false otherwise or if there are less than 2 keys in the key vector
      ///@throw std::out_of_range if a key does not exist in the cache map
      template<typename P>
      bool compose_down(const P& in_pose, const std::vector<K> &transformation_keys_ordered, const ros::Time &t, PoseWithCovariance &out_pose);

      ///@brief Overload of compose_down for a geometry_msgs::Point, which transforms it into a Pose, does the composition, then gives back a resulting Point
      bool compose_down(const Point& in_point, const std::vector<K> &transformation_keys_ordered, const ros::Time &t, PoseWithCovariance &out_pose);

      ///@brief prints to stream debugging information on the interface
      void print(std::ostream&) const;
    };

    template<typename K>
    inline std::ostream& operator<< (std::ostream& stream, const Interface<K>& interface) {
      interface.print(stream);
      return stream;
    }
  }
}

#endif //POSE_COV_OPS_INTERFACE_POSECOVOPSINTERFACE_H
