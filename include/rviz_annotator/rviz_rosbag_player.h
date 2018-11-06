/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
********************************************************************/

#ifndef RVIZ_ROSBAG_PLAYER_H
#define RVIZ_ROSBAG_PLAYER_H

#include <ros/ros.h>

#include <rosbag/bag.h>
#include <rviz_annotator/rviz_time_translator.h>
#include <rosbag/macros.h>
#include <rosbag/message_instance.h>
#include <rosbag/view.h>

#include "rosgraph_msgs/Clock.h"

#include <topic_tools/shape_shifter.h>
#include <std_srvs/SetBool.h>

#include <boost/format.hpp>
#include <mutex>

#define SECURE_OFFSET 10.0

namespace rosbag
{
class MessageInstance;
class TimeTranslator;
class View;
class TopicQuery;
struct ConnectionInfo;
}

namespace rviz_rosbag
{

ros::AdvertiseOptions createAdvertiseOptions(rosbag::MessageInstance const& msg, uint32_t queue_size, const std::string& prefix = "");
ROSBAG_DECL ros::AdvertiseOptions createAdvertiseOptions(const rosbag::ConnectionInfo* c, uint32_t queue_size, const std::string& prefix = "");

struct ROSBAG_DECL PlayerOptions
{
    PlayerOptions();

    void check();

    std::string prefix;
    bool     quiet;
    bool     start_paused;
    bool     at_once;
    bool     bag_time;
    double   bag_time_frequency;
    double   time_scale;
    int      queue_size;
    ros::WallDuration advertise_sleep;
    bool     try_future;
    bool     has_time;
    bool     loop;
    float    time;
    bool     has_duration;
    float    duration;
    bool     keep_alive;
    bool     wait_for_subscribers;
    std::string rate_control_topic;
    float    rate_control_max_delay;
    ros::Duration skip_empty;

    bool sync_topics;

    std::vector<std::string> bags;
    std::vector<std::string> topics;
    std::vector<std::string> pause_topics;
};

class ROSBAG_DECL TimePublisher {
public:
    /*! Create a time publisher
     *  A publish_frequency of < 0 indicates that time shouldn't actually be published
     */
    TimePublisher();

    void insertPassedTime(ros::Time const& t, ros::WallTime const& wt, rosbag::MessageInstance const& m, std::string cid);

    ros::WallTime getPrevWC();
    std::string getPrevCallerId();
    std::vector<rosbag::MessageInstance>& getMsgVec(){ return passed_msg_; };
    bool removeAndCheckEmpty();
    void clearInfo();

    void setPublishFrequency(double publish_frequency);
    
    void setTimeScale(double time_scale);

    /*! Set the horizon that the clock will run to */
    void setHorizon(const ros::Time& horizon);

    /*! Set the horizon that the clock will run to */
    void setWCHorizon(const ros::WallTime& horizon);

    /*! Set the current time */
    void setTime(const ros::Time& time);

    /*! Get the current time */
    ros::Time const& getTime() const;

    /*! Run the clock for AT MOST duration
     *
     * If horizon has been reached this function returns immediately
     */
    void runClock(const ros::WallDuration& duration);

    //! Sleep as necessary, but don't let the click run 
    void runStalledClock(const ros::WallDuration& duration);

    //! Step the clock to the horizon
    void stepClock();

    //! Backstep the clock to the horizon
    void backstepClock();

    bool horizonReached();

private:
    bool do_publish_;
    
    double publish_frequency_;
    double time_scale_;
    
    ros::NodeHandle node_handle_;
    ros::Publisher time_pub_;
    
    ros::WallDuration wall_step_;
    
    ros::WallTime next_pub_;

    ros::WallTime wc_horizon_;
    ros::Time horizon_;
    ros::Time current_;

    //track previous time info
    std::vector<ros::Time> passed_time_;
    std::vector<ros::WallTime> passed_walltime_;
    std::vector<rosbag::MessageInstance> passed_msg_;
    std::vector<std::string> passed_pubs_;
};

class ROSBAG_DECL Player
{
public:
    Player(PlayerOptions const& options);
    ~Player();

    void publish();

    void setChoice(char c){ choice_ = c; };

    void setTerminate(bool t){ terminate_ = t; };

    void changeOptions(PlayerOptions newOptions){ options_ = newOptions; };

    std::mutex lock_choice_;

private:
    void updateRateTopicTime(const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event);

    int doPublish(rosbag::MessageInstance const& m);

    void doKeepAlive();

    void printTime();

    bool pauseCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

    void processPause(const bool paused, ros::WallTime &horizon);

    void waitForSubscribers() const;

    bool isPaused(){ return paused_; };

    //syncing topics to master topic mv
    void syncTopics(rosbag::MessageInstance const& mv);

private:
    typedef std::map<std::string, ros::Publisher> PublisherMap;

    PlayerOptions options_;

    ros::NodeHandle node_handle_;

    ros::ServiceServer pause_service_;

    bool paused_;
    bool delayed_;

    bool pause_for_topics_;

    bool pause_change_requested_;

    bool requested_pause_state_;

    ros::Subscriber rate_control_sub_;
    ros::Time last_rate_control_;

    ros::WallTime paused_time_;

    std::vector<boost::shared_ptr<rosbag::Bag> >  bags_;
    PublisherMap publishers_;

    rviz_rosbag::TimeTranslator time_translator_;
    TimePublisher time_publisher_;

    //runtime choice
    char choice_;

    //terminate loop in pause
    bool terminate_;

    //vector with all messages
    std::vector<rosbag::MessageInstance> all_msgs;

    std::vector< std::vector<rosbag::MessageInstance> > msg_vec_;

    //master topic position
    int max_topic_pos_;

    //psoitions of closest time frame to master topic
    std::vector<int> closest_pos_;

    //initial secure value of min_dist(minimum distance between master topic and candidate topic)
    double max_time_dist_;

    ros::Time start_time_;
    ros::Duration bag_length_;
};

}//end namespace rviz_rosbag

#endif