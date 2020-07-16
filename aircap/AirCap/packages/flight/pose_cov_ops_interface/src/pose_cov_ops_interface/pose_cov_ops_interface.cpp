//
// Created by glawless on 11.05.17.
//

#include <pose_cov_ops_interface/pose_cov_ops_interface.h>

#define getElementFunction(t) getElemBeforeTime(t)

namespace pose_cov_ops {
  namespace interface {

    template<typename K>
    Interface<K>::Interface(const std::vector<topicSubInfo<K>> &pose_topics, ros::NodeHandle &handle) {

      // For every topic, emplace a subscriber in the vector of subscribers, then emplace the cache in the map of caches
      // Having a map will allow to use the topic key to access it
      for (const topicSubInfo<K> &topic : pose_topics) {
        subs_.emplace_back(std::unique_ptr<poseSub_t>(new poseSub_t(handle, topic.name, topic.queue_size)));
        cache_map_[topic.key] = std::unique_ptr<poseCache_t>(new poseCache_t(*subs_.back(), topic.cache_size));
        cache_map_[topic.key]->setName(topic.name);
      }
    }

    template<typename K>
    ros::Time Interface<K>::oldestTimeInCache(K key) const {
      try {
        return cache_map_.at(key)->getOldestTime();
      }
      catch (std::out_of_range &oor) {
        ROS_ERROR_STREAM("Out of Range error for key " << key << ": " << oor.what());
        ROS_ERROR("Are you sure it exists? Printing interface contents...");
        ROS_ERROR_STREAM(*this);
        throw oor;
      }
    }

    template<typename K>
    template<typename P>
    bool Interface<K>::compose_up(const P &in_pose, const K &transformation_key, const ros::Time &t,
                                  PoseWithCovariance &out_pose) {

      // Get the closest time msg in the cache for transformation_key
      poseCache_t::MConstPtr p_transf;
      try {
        p_transf = cache_map_.at(transformation_key)->getElementFunction(t);
      }
      catch (std::out_of_range &oor) {
        ROS_ERROR_STREAM("Out of Range error for key : " << transformation_key << ": " << oor.what());
        ROS_ERROR("Are you sure it exists? Printing interface contents...");
        ROS_ERROR_STREAM(*this);
        throw oor;
      }

      // Not found at time t
      if (p_transf == nullptr) {
        return false;
      }

      // Compose using pose_cov_ops, place result in out_pose
      pose_cov_ops::compose(p_transf->pose, in_pose, out_pose);

      return true;
    }

    template<typename K>
    bool Interface<K>::compose_up(const Point &in_point, const K &transformation_key, const ros::Time &t,
                                  PoseWithCovariance &out_pose) {

      geometry_msgs::Pose in_pose;
      in_pose.position = in_point;
      in_pose.orientation.w = 1.0;

      return compose_up(in_pose, transformation_key, t, out_pose);
    }

    template<typename K>
    template<typename P>
    bool Interface<K>::compose_up(const P &in_pose, const std::vector<K> &transformation_keys_ordered,
                                  const ros::Time &t, PoseWithCovariance &out_pose) {

      // Need at least 1 key in the vector
      if (transformation_keys_ordered.size() < 1)
        return false;

      // Auxiliary variables
      PoseWithCovariance t_pose, t_outpose;
      t_pose.pose.orientation.w = 1.0;

      // Repeat for all keys in vector
      for(auto it = transformation_keys_ordered.rbegin(); it != transformation_keys_ordered.rend(); ++it) {

        if (!compose_up(t_pose, *it, t, t_outpose))
          return false;

        t_pose = t_outpose;
      }

      // Final composition for the in_pose and the composition of the transformation poses
      pose_cov_ops::compose(t_pose, in_pose, out_pose);

      return true;
    }

    template<typename K>
    bool Interface<K>::compose_up(const Point &in_point, const std::vector<K> &transformation_keys_ordered,
                                  const ros::Time &t, PoseWithCovariance &out_pose) {

      geometry_msgs::Pose in_pose;
      in_pose.position = in_point;
      in_pose.orientation.w = 1.0;

      return compose_up(in_pose, transformation_keys_ordered, t, out_pose);
    }

    template<typename K>
    template<typename P>
    bool Interface<K>::compose_down(const P &in_pose, const K &transformation_key, const ros::Time &t,
                                    PoseWithCovariance &out_pose) {

      // Get the closest time msg in the cache for transformation_key
      poseCache_t::MConstPtr p_transf;
      try {
        p_transf = cache_map_.at(transformation_key)->getElementFunction(t);
      }
      catch (std::out_of_range &oor) {
        ROS_ERROR_STREAM("Out of Range error for key : " << transformation_key << ": " << oor.what());
        ROS_ERROR("Are you sure it exists? Printing interface contents...");
        ROS_ERROR_STREAM(*this);
        throw oor;
      }

      // Not found at time t
      if (p_transf == nullptr) {
        return false;
      }

      // Compose using pose_cov_ops, place result in out_pose
      pose_cov_ops::inverseCompose(in_pose, p_transf->pose, out_pose);

      return true;
    }

    template<typename K>
    bool Interface<K>::compose_down(const Point &in_point, const K &transformation_key, const ros::Time &t,
                                    PoseWithCovariance &out_pose) {

      geometry_msgs::Pose in_pose;
      in_pose.position = in_point;
      in_pose.orientation.w = 1.0;


      return compose_down(in_pose, transformation_key, t, out_pose);
    }

    template<typename K>
    template<typename P>
    bool
    Interface<K>::compose_down(const P &in_pose, const std::vector<K> &transformation_keys_ordered, const ros::Time &t,
                               PoseWithCovariance &out_pose) {

      // Need at least 1 key in the vector
      if (transformation_keys_ordered.size() < 1)
        return false;

      // Auxiliary variables
      PoseWithCovariance t_pose, t_outpose;
      t_pose.pose.orientation.w = 1.0;

      // Repeat for all keys in vector
      for(auto it = transformation_keys_ordered.rbegin(); it != transformation_keys_ordered.rend(); ++it) {
        if (!compose_up(t_pose, *it, t, t_outpose))
          return false;

        t_pose = t_outpose;
      }

      // Final composition for the in_pose and the composition of the transformation poses
      pose_cov_ops::inverseCompose(in_pose, t_pose, out_pose);

      return true;
    }

    template<typename K>
    bool Interface<K>::compose_down(const Point &in_point, const std::vector<K> &transformation_keys_ordered,
                                    const ros::Time &t, PoseWithCovariance &out_pose) {

      geometry_msgs::Pose in_pose;
      in_pose.position = in_point;
      in_pose.orientation.w = 1.0;

      return compose_down(in_pose, transformation_keys_ordered, t, out_pose);
    }

    template<typename K>
    void Interface<K>::print(std::ostream &stream) const {
      stream << "Interface at time " << ros::Time::now() << std::endl;
      if (cache_map_.empty())
        stream << "Empty" << std::endl;

      for (auto it = cache_map_.begin(); it != cache_map_.end(); ++it) {
        poseCache_t &cache = *(it->second);
        stream << "\tcache:\n\t\tkey:\t" << it->first << "\n\t\ttopic:\t" << cache.getName() << "\n\t\tlatest:\t"
               << oldestTimeInCache(it->first) << "" << std::endl;
      }
    }

    //Explicitly instantiate the class to be defined for int and string keys
    template
    class Interface<std::string>;

    template
    class Interface<int>;

    template
    bool
    Interface<std::string>::compose_up(const PoseWithCovariance &, const std::vector<std::string> &, const ros::Time &,
                                       PoseWithCovariance &);

    template
    bool Interface<std::string>::compose_up(const PoseWithCovariance &, const std::string &, const ros::Time &,
                                            PoseWithCovariance &);

    template
    bool Interface<std::string>::compose_down(const PoseWithCovariance &, const std::vector<std::string> &,
                                              const ros::Time &, PoseWithCovariance &);

    template
    bool Interface<std::string>::compose_down(const PoseWithCovariance &, const std::string &, const ros::Time &,
                                              PoseWithCovariance &);

    template
    bool Interface<int>::compose_up(const PoseWithCovariance &, const std::vector<int> &, const ros::Time &,
                                    PoseWithCovariance &);

    template
    bool Interface<int>::compose_up(const PoseWithCovariance &, const int &, const ros::Time &, PoseWithCovariance &);

    template
    bool Interface<int>::compose_down(const PoseWithCovariance &, const std::vector<int> &, const ros::Time &,
                                      PoseWithCovariance &);

    template
    bool Interface<int>::compose_down(const PoseWithCovariance &, const int &, const ros::Time &, PoseWithCovariance &);

  }
}