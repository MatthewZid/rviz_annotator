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

#include <rviz_annotator/rviz_rosbag_player.h>

namespace rviz_rosbag
{

ros::AdvertiseOptions createAdvertiseOptions(const rosbag::ConnectionInfo* c, uint32_t queue_size, const std::string& prefix) {
    ros::AdvertiseOptions opts(prefix + c->topic, queue_size, c->md5sum, c->datatype, c->msg_def);
    ros::M_string::const_iterator header_iter = c->header->find("latching");
    opts.latch = (header_iter != c->header->end() and header_iter->second == "1");
    return opts;
}


ros::AdvertiseOptions createAdvertiseOptions(rosbag::MessageInstance const& m, uint32_t queue_size, const std::string& prefix) {
    return ros::AdvertiseOptions(prefix + m.getTopic(), queue_size, m.getMD5Sum(), m.getDataType(), m.getMessageDefinition());
}

// PlayerOptions

PlayerOptions::PlayerOptions() :
    prefix(""),
    quiet(false),
    start_paused(false),
    at_once(false),
    bag_time(false),
    bag_time_frequency(0.0),
    time_scale(1.0),
    queue_size(0),
    advertise_sleep(0.2),
    try_future(false),
    has_time(false),
    loop(false),
    time(0.0f),
    has_duration(false),
    duration(0.0f),
    keep_alive(false),
    wait_for_subscribers(false),
    rate_control_topic(""),
    rate_control_max_delay(1.0f),
    skip_empty(ros::DURATION_MAX),
    sync_topics(false)
{
}

void PlayerOptions::check() {
    if (bags.size() == 0)
        throw ros::Exception("You must specify at least one bag file to play from");
    if (has_duration and duration <= 0.0)
        throw ros::Exception("Invalid duration, must be > 0.0");
}

// Player

Player::Player(PlayerOptions const& options) :
    options_(options),
    paused_(false),
    // If we were given a list of topics to pause on, then go into that mode
    // by default (it can be toggled later via 't' from the keyboard).
    pause_for_topics_(options_.pause_topics.size() > 0),
    pause_change_requested_(false),
    requested_pause_state_(false),
    choice_('-'),
    terminate_(false),
    max_topic_pos_(-1)
{
  ros::NodeHandle private_node_handle("~");
  pause_service_ = private_node_handle.advertiseService("pause_playback", &Player::pauseCallback, this);
}

Player::~Player() {
    for(boost::shared_ptr<rosbag::Bag> bag : bags_)
        bag->close();
}

void Player::publish() {
    options_.check();

    // Open all the bag files
    for(std::string const& filename : options_.bags) {
        ROS_INFO("Opening %s", filename.c_str());
        bags_.clear();

        try
        {
            boost::shared_ptr<rosbag::Bag> bag(boost::make_shared<rosbag::Bag>());
            bag->open(filename, rosbag::bagmode::Read);
            bags_.push_back(bag);
        }
        catch (rosbag::BagUnindexedException ex) {
            std::cerr << "Bag file " << filename << " is unindexed.  Run rosbag reindex." << std::endl;
            return;
        }
    }

    if (!node_handle_.ok()){
      for(boost::shared_ptr<rosbag::Bag> bag : bags_)
        bag->close();

      return;
    }

    if (!options_.prefix.empty())
    {
      ROS_INFO_STREAM("Using prefix '" << options_.prefix << "'' for topics ");
    }

    if (!options_.quiet)
      puts("");
    
    // Publish all messages in the bags
    rosbag::View full_view;
    for(boost::shared_ptr<rosbag::Bag> bag : bags_)
        full_view.addQuery(*bag);

    ros::Time initial_time = full_view.getBeginTime();

    initial_time += ros::Duration(options_.time);

    ros::Time finish_time = ros::TIME_MAX;
    if (options_.has_duration)
    {
      finish_time = initial_time + ros::Duration(options_.duration);
    }

    rosbag::View view;
    rosbag::TopicQuery topics(options_.topics);

    if (options_.topics.empty())
    {
      for(boost::shared_ptr<rosbag::Bag> bag : bags_)
        view.addQuery(*bag, initial_time, finish_time);
    } else {
      for(boost::shared_ptr<rosbag::Bag> bag : bags_)
        view.addQuery(*bag, topics, initial_time, finish_time);
    }

    if (view.size() == 0)
    {
      std::cerr << "No messages to play on specified topics.  Exiting." << std::endl;
      //ros::shutdown();
      return;
    }

    // Advertise all of our messages
    for(const rosbag::ConnectionInfo* c : view.getConnections())
    {
        ros::M_string::const_iterator header_iter = c->header->find("callerid");
        std::string callerid = (header_iter != c->header->end() ? header_iter->second : std::string(""));

        std::string callerid_topic = callerid + c->topic;

        std::map<std::string, ros::Publisher>::iterator pub_iter = publishers_.find(callerid_topic);
        if (pub_iter == publishers_.end()) {

            ros::AdvertiseOptions opts = createAdvertiseOptions(c, options_.queue_size, options_.prefix);

            ros::Publisher pub = node_handle_.advertise(opts);
            publishers_.insert(publishers_.begin(), std::pair<std::string, ros::Publisher>(callerid_topic, pub));

            pub_iter = publishers_.find(callerid_topic);
        }
    }

    if (options_.rate_control_topic != "")
    {
        std::cout << "Creating rate control topic subscriber..." << std::flush;

        boost::shared_ptr<ros::Subscriber> sub(boost::make_shared<ros::Subscriber>());
        ros::SubscribeOptions ops;
        ops.topic = options_.rate_control_topic;
        ops.queue_size = 10;
        ops.md5sum = ros::message_traits::md5sum<topic_tools::ShapeShifter>();
        ops.datatype = ros::message_traits::datatype<topic_tools::ShapeShifter>();
        ops.helper = boost::make_shared<ros::SubscriptionCallbackHelperT<
            const ros::MessageEvent<topic_tools::ShapeShifter const> &> >(
                boost::bind(&Player::updateRateTopicTime, this, _1));

        rate_control_sub_ = node_handle_.subscribe(ops);

        std::cout << " done." << std::endl;
    }


    std::cout << "Waiting " << options_.advertise_sleep.toSec() << " seconds after advertising topics..." << std::flush;
    options_.advertise_sleep.sleep();
    std::cout << " done." << std::endl;

    std::cout << std::endl << "Hit Pause to toggle paused, >> to step or << to step back." << std::endl;

    paused_ = options_.start_paused;

    if (options_.wait_for_subscribers)
    {
        waitForSubscribers();
    }

    //Categorize messages
    std::set<std::string> msg_set;

    for(rosbag::MessageInstance m : view){
        msg_set.insert(m.getTopic());

        //copy MessageInsances to vector
        all_msgs.push_back(m);
    }

    int msg_type_num = msg_set.size();

    msg_vec_.resize(msg_type_num);

    for(rosbag::MessageInstance m : view){
        auto it = msg_set.find(m.getTopic());

        if(it != msg_set.end()){
            int pos = std::distance(msg_set.begin(), it);

            msg_vec_.at(pos).push_back(m);
        }
        else
            ROS_WARN("%s topic not found!\n", m.getTopic().c_str());
    }

    /*//Find category with max topics; master topic is in position max_topic_pos_
    long max_topics = -1;

    for(auto it = msg_vec_.begin(); it != msg_vec_.end(); it++){
        if((long)(it->size()) > max_topics){
            max_topics = it->size();
            max_topic_pos_ = std::distance(msg_vec_.begin(), it);
        }
    }

    for(int i = 0; i < msg_type_num; i++)
    	closest_pos_.push_back(0);*/

    while (true) {
        // Set up our time_translator and publishers

        time_translator_.setTimeScale(options_.time_scale);

        start_time_ = view.begin()->getTime();
        time_translator_.setRealStartTime(start_time_);
        bag_length_ = view.getEndTime() - view.getBeginTime();

        //secure max value
        max_time_dist_ = (bag_length_ + ros::Duration(SECURE_OFFSET)).toSec();

        // Set the last rate control to now, so the program doesn't start delayed.
        last_rate_control_ = start_time_;

        time_publisher_.setTime(start_time_);

        ros::WallTime now_wt = ros::WallTime::now();
        time_translator_.setTranslatedStartTime(ros::Time(now_wt.sec, now_wt.nsec));

        time_publisher_.setTimeScale(options_.time_scale);

        if (options_.bag_time)
            time_publisher_.setPublishFrequency(options_.bag_time_frequency);
        else
            time_publisher_.setPublishFrequency(-1.0);

        paused_time_ = now_wt;

        //Always start with cleared choice
        choice_ = '-';

        /*// Call do-publish for each message (ORIGINAL)
        for(rosbag::MessageInstance m : view) {
            if (!node_handle_.ok() or terminate_)
                break;

            	doPublish(m);
        }*/

        /*// Call do-publish for each message (with iterator)
        for(auto it = view.begin(); it != view.end(); it++) {
            if (!node_handle_.ok() or terminate_)
                break;

            doPublish(*it);
        }*/

        // Call do-publish for each message (with custom vector)
        for(auto it = all_msgs.begin(); it != all_msgs.end(); it++) {
            if (!node_handle_.ok() or terminate_)
                break;

            if(it < all_msgs.begin())
                it = all_msgs.begin();

            //ROS_WARN("\nTopic: %s\nTime: %lf\n", it->getTopic().c_str(), it->getTime().toSec());
            it -= doPublish(*it);
            //ros::Duration(1).sleep();
        }

        /*//Call doPublish for master topic
        for(auto it = msg_vec_.at(max_topic_pos_).begin(); it != msg_vec_.at(max_topic_pos_).end(); it++){
            if (!node_handle_.ok() or terminate_)
                break;

            doPublish(*it);
        }*/

        //clear info for restart if loop
        time_publisher_.clearInfo();
        time_translator_.clearInfo();

        if (options_.keep_alive)
            while (node_handle_.ok())
                doKeepAlive();

        if (!node_handle_.ok()) {
            std::cout << std::endl;
            break;
        }
        if (!options_.loop) {
        	if(!terminate_)
            	std::cout << std::endl << "Done.\n" << std::endl;
            else
            	std::cout << std::endl << "Terminated.\n" << std::endl;

            //Don't start terminated
            terminate_ = false;

            time_publisher_.clearInfo();
            time_translator_.clearInfo();

            for(auto it = msg_vec_.begin(); it != msg_vec_.end(); it++)
                it->clear();

            msg_vec_.clear();
            all_msgs.clear();

            for(boost::shared_ptr<rosbag::Bag> bag : bags_)
        		bag->close();

            break;
        }
    }

    //ros::shutdown();
}

void Player::updateRateTopicTime(const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event)
{
    boost::shared_ptr<topic_tools::ShapeShifter const> const &ssmsg = msg_event.getConstMessage();
    std::string def = ssmsg->getMessageDefinition();
    size_t length = ros::serialization::serializationLength(*ssmsg);
    
    // Check the message definition.
    std::istringstream f(def);
    std::string s;
    bool flag = false;
    while(std::getline(f, s, '\n')) {
        if (!s.empty() and s.find("#") != 0) {
            // Does not start with #, is not a comment.
            if(s.find("Header ") == 0) {
                flag = true;
            }

            break;
        }
    }
    // If the header is not the first element in the message according to the definition, throw an error.
    if (!flag) {
        std::cout << std::endl << "WARNING: Rate control topic is bad, header is not first. MSG may be malformed." << std::endl;
        return;
    }

    std::vector<uint8_t> buffer(length);
    ros::serialization::OStream ostream(&buffer[0], length);
    ros::serialization::Serializer<topic_tools::ShapeShifter>::write(ostream, *ssmsg);

    // Assuming that the header is the first several bytes of the message.
    //uint32_t header_sequence_id   = buffer[0] | (uint32_t)buffer[1] << 8 | (uint32_t)buffer[2] << 16 | (uint32_t)buffer[3] << 24;
    int32_t header_timestamp_sec  = buffer[4] | (uint32_t)buffer[5] << 8 | (uint32_t)buffer[6] << 16 | (uint32_t)buffer[7] << 24;
    int32_t header_timestamp_nsec = buffer[8] | (uint32_t)buffer[9] << 8 | (uint32_t)buffer[10] << 16 | (uint32_t)buffer[11] << 24;

    last_rate_control_ = ros::Time(header_timestamp_sec, header_timestamp_nsec);
}

void Player::printTime()
{
    if (!options_.quiet) {

        ros::Time current_time = time_publisher_.getTime();
        ros::Duration d = current_time - start_time_;


        if (paused_)
        {
            printf("\r [PAUSED ]  Bag Time: %13.6f   Duration: %.6f / %.6f               \r", time_publisher_.getTime().toSec(), d.toSec(), bag_length_.toSec());
        }
        else if (delayed_)
        {
            ros::Duration time_since_rate = std::max(ros::Time::now() - last_rate_control_, ros::Duration(0));
            printf("\r [DELAYED]  Bag Time: %13.6f   Duration: %.6f / %.6f   Delay: %.2f \r", time_publisher_.getTime().toSec(), d.toSec(), bag_length_.toSec(), time_since_rate.toSec());
        }
        else
        {
            printf("\r [RUNNING]  Bag Time: %13.6f   Duration: %.6f / %.6f               \r", time_publisher_.getTime().toSec(), d.toSec(), bag_length_.toSec());
        }
        fflush(stdout);
    }
}

bool Player::pauseCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    pause_change_requested_ = (req.data != paused_);
  requested_pause_state_ = req.data;

  res.success = pause_change_requested_;

  if (res.success)
  {
    res.message = std::string("Playback is now ") + (requested_pause_state_ ? "paused" : "resumed");
  }
  else
  {
    res.message = std::string("Bag is already ") + (requested_pause_state_ ? "paused." : "running.");
  }

    return true;
}

void Player::processPause(const bool paused, ros::WallTime &horizon)
{
    paused_ = paused;

  if (paused_)
  {
    paused_time_ = ros::WallTime::now();
  }
  else
  {
    // Make sure time doesn't shift after leaving pause.
    ros::WallDuration shift = ros::WallTime::now() - paused_time_;

    paused_time_ = ros::WallTime::now();
    time_translator_.shift(ros::Duration(shift.sec, shift.nsec));

    horizon += shift;
    time_publisher_.setWCHorizon(horizon);
  }
}

void Player::waitForSubscribers() const
{
    bool all_topics_subscribed = false;
    std::cout << "Waiting for subscribers." << std::endl;
    while (!all_topics_subscribed) {
        all_topics_subscribed = true;
        for(const PublisherMap::value_type& pub : publishers_) {
            all_topics_subscribed &= pub.second.getNumSubscribers() > 0;
        }
        ros::Duration(0.1).sleep();
    }
    std::cout << "Finished waiting for subscribers." << std::endl;
}

void Player::syncTopics(rosbag::MessageInstance const& mv)
{
    //publish closest topics to previous master topic
    ros::Time const& previous_topic_time = mv.getTime();
    ROS_WARN("\nPrevious topic: %s\nPrevious time: %lf\n", mv.getTopic().c_str(), previous_topic_time.toSec());

    for(auto it = msg_vec_.begin(); it != msg_vec_.end(); it++){
        if(mv.getTopic() != it->at(0).getTopic()){

            //find closest topic to previous topic
            ros::Duration min_time_dist(max_time_dist_);
            std::vector<rosbag::MessageInstance>::iterator min_msg;
            bool min_found = false;

            for(auto msg_it = it->begin(); msg_it != it->end(); msg_it++){
                ros::Time const& topic_time = msg_it->getTime();
                ros::Duration time_dist = previous_topic_time - topic_time;

                if(time_dist >= ros::Duration(0.0)){
                    if(time_dist < min_time_dist){
                        min_time_dist = time_dist;
                        min_msg = msg_it;
                        min_found = true;
                    }
                }
                else
                    break;
            }

            if(min_found){
                std::string const& min_callerid = min_msg->getCallerId();
                std::string const& min_topic = min_msg->getTopic();
                std::string min_callerid_topic = min_callerid + min_topic;

                std::map<std::string, ros::Publisher>::iterator min_pub_iter = publishers_.find(min_callerid_topic);
                ROS_ASSERT(min_pub_iter != publishers_.end());

                // Update subscribers.
                ros::spinOnce();

                ROS_INFO("\nMin topic: %s\nMin time: %lf\n", min_topic.c_str(), min_msg->getTime().toSec());

                (min_pub_iter->second).publish(*min_msg);
            }
            else
                ROS_WARN("No more past topics!\n");
        }
    }
}

int Player::doPublish(rosbag::MessageInstance const& m)
{
    std::string const& topic   = m.getTopic();
    ros::Time const& time = m.getTime();
    std::string callerid       = m.getCallerId();
    
    ros::Time translated = time_translator_.translate(time);
    ros::WallTime horizon = ros::WallTime(translated.sec, translated.nsec);

    time_publisher_.setHorizon(time);
    time_publisher_.setWCHorizon(horizon);

    std::string callerid_topic = callerid + topic;

    std::map<std::string, ros::Publisher>::iterator pub_iter = publishers_.find(callerid_topic);
    ROS_ASSERT(pub_iter != publishers_.end());

    // Update subscribers.
    ros::spinOnce();

    // If immediate specified, play immediately
    if (options_.at_once) {
        time_publisher_.stepClock();
        pub_iter->second.publish(m);

        //keep time info
        time_publisher_.insertPassedTime(m.getTime(), horizon, m, callerid_topic);
        time_translator_.insertPassedTrStart();

        printTime();
        return 0;
    }

    // If skip_empty is specified, skip this region and shift.
    if (time - time_publisher_.getTime() > options_.skip_empty)
    {
      time_publisher_.stepClock();

      ros::WallDuration shift = ros::WallTime::now() - horizon ;
      time_translator_.shift(ros::Duration(shift.sec, shift.nsec));
      horizon += shift;
      time_publisher_.setWCHorizon(horizon);
      (pub_iter->second).publish(m);

      //keep time info
      time_publisher_.insertPassedTime(m.getTime(), horizon, m, callerid_topic);
      time_translator_.insertPassedTrStart();

      printTime();
      return 0;
    }

    if (pause_for_topics_)
    {
        for (std::vector<std::string>::iterator i = options_.pause_topics.begin();
             i != options_.pause_topics.end();
             ++i)
        {
            if (topic == *i)
            {
                paused_ = true;
                paused_time_ = ros::WallTime::now();
            }
        }
    }

    // Check if the rate control topic has posted recently enough to continue, or if a delay is needed.
    // Delayed is separated from paused to allow more verbose printing.
    if (rate_control_sub_ != NULL) {
        if ((time_publisher_.getTime() - last_rate_control_).toSec() > options_.rate_control_max_delay) {
            delayed_ = true;

            paused_time_ = ros::WallTime::now();
        }
    }

    while ((paused_ or delayed_ or !time_publisher_.horizonReached()) and !terminate_ and node_handle_.ok())
    {
        bool charsleftorpaused = true;
        while (charsleftorpaused and !terminate_ and node_handle_.ok())
        {
            ros::spinOnce();

            if (pause_change_requested_)
            {
              processPause(requested_pause_state_, horizon);
              pause_change_requested_ = false;
            }

//////////// For automated backstep selection after one Pause button click ////////////

            /*if(paused_){
                lock_choice_.lock();
            	choice_ = 'b';
                lock_choice_.unlock();
            }*/

///////////////////////////////////////////////////////////////////////////////////////

            //rviz input while player running
            switch (choice_){
            case ' ':
                processPause(!paused_, horizon);

                lock_choice_.lock();
            	choice_ = '-';
            	lock_choice_.unlock();
                break;
            case 's':
                if (paused_) {
                    time_publisher_.stepClock();

                    ros::WallDuration shift = ros::WallTime::now() - horizon ;
                    paused_time_ = ros::WallTime::now();

                    time_translator_.shift(ros::Duration(shift.sec, shift.nsec));

                    horizon += shift;
                    time_publisher_.setWCHorizon(horizon);

                    (pub_iter->second).publish(m);

                    //keep time info
                    time_publisher_.insertPassedTime(m.getTime(), horizon, m, callerid_topic);
                    time_translator_.insertPassedTrStart();

                    if(options_.sync_topics)
                        syncTopics(m);

                    printTime();

                    lock_choice_.lock();
                    choice_ = '-';
                    lock_choice_.unlock();

                    return 0;
                }

                lock_choice_.lock();
                choice_ = '-';
                lock_choice_.unlock();

                break;
            case 'b':
                if (paused_) {
                    if(time_publisher_.removeAndCheckEmpty() or time_translator_.removeAndCheckEmpty()){
                        ROS_WARN("Back to start!\n");
                        //terminate_ = true;
                        //options_.loop = false;
                        time_publisher_.clearInfo();
                        time_translator_.clearInfo();
                    }
                    else{
                       	time_publisher_.backstepClock();

                        ros::WallDuration shift = ros::WallTime::now() - time_publisher_.getPrevWC() ;
                        paused_time_ = ros::WallTime::now();

                        ros::Time const& prev_tr_start = time_translator_.getPrevTrStart();
                        time_translator_.shiftBack(prev_tr_start, ros::Duration(shift.sec, shift.nsec));

                        //horizon -= shift;
                        horizon = time_publisher_.getPrevWC() + shift;
                        time_publisher_.setWCHorizon(horizon);
                        
                        std::vector<rosbag::MessageInstance> const& mv = time_publisher_.getMsgVec();

                        std::string const& prevcid = time_publisher_.getPrevCallerId();
                        std::map<std::string, ros::Publisher>::iterator prev_pub_iter = publishers_.find(prevcid);
                        ROS_ASSERT(prev_pub_iter != publishers_.end());

                        // Update subscribers.
                        ros::spinOnce();

                        (prev_pub_iter->second).publish(mv.back()); //publish previous message

                        if(options_.sync_topics)
                            syncTopics(mv.back());

                        //mv.pop_back();
                    }

                    printTime();

                    lock_choice_.lock();
                    choice_ = '-';
                    lock_choice_.unlock();

                    return 2;
                }

                lock_choice_.lock();
                choice_ = '-';
                lock_choice_.unlock();

                break;
            case 't':
                pause_for_topics_ = !pause_for_topics_;
                break;
            default:
                if (paused_)
                {
                    printTime();
                    time_publisher_.runStalledClock(ros::WallDuration(.1));
                    ros::spinOnce();
                }
                else if (delayed_)
                {
                    printTime();
                    time_publisher_.runStalledClock(ros::WallDuration(.1));
                    ros::spinOnce();
                    // You need to check the rate here too.
                    if(rate_control_sub_ == NULL || (time_publisher_.getTime() - last_rate_control_).toSec() <= options_.rate_control_max_delay) {
                        delayed_ = false;
                        // Make sure time doesn't shift after leaving delay.
                        ros::WallDuration shift = ros::WallTime::now() - paused_time_;
                        paused_time_ = ros::WallTime::now();
         
                        time_translator_.shift(ros::Duration(shift.sec, shift.nsec));

                        horizon += shift;
                        time_publisher_.setWCHorizon(horizon);
                    }
                }
                else
                    charsleftorpaused = false;
            }
        }

        printTime();
        time_publisher_.runClock(ros::WallDuration(.1));

        ros::spinOnce();
    }

    //keep time info
    //time_publisher_.insertPassedTime(time_publisher_.getTime(), horizon, m, callerid_topic);
    time_publisher_.insertPassedTime(m.getTime(), horizon, m, callerid_topic);
    time_translator_.insertPassedTrStart();

    pub_iter->second.publish(m);

    return 0;
}

void Player::doKeepAlive()
{
    //Keep pushing ourself out in 10-sec increments (avoids fancy math dealing with the end of time)
    ros::Time const& time = time_publisher_.getTime() + ros::Duration(10.0);

    ros::Time translated = time_translator_.translate(time);
    ros::WallTime horizon = ros::WallTime(translated.sec, translated.nsec);

    time_publisher_.setHorizon(time);
    time_publisher_.setWCHorizon(horizon);

    if (options_.at_once) {
        return;
    }

    // If we're done and just staying alive, don't watch the rate control topic. We aren't publishing anyway.
    delayed_ = false;

    while ((paused_ or !time_publisher_.horizonReached()) and !terminate_ and node_handle_.ok())
    {
        bool charsleftorpaused = true;
        while (charsleftorpaused and !terminate_ and node_handle_.ok())
        {
            switch (choice_){
            case ' ':
                paused_ = !paused_;
                if (paused_) {
                    paused_time_ = ros::WallTime::now();
                }
                else
                {
                    // Make sure time doesn't shift after leaving pause.
                    ros::WallDuration shift = ros::WallTime::now() - paused_time_;
                    paused_time_ = ros::WallTime::now();
         
                    time_translator_.shift(ros::Duration(shift.sec, shift.nsec));

                    horizon += shift;
                    time_publisher_.setWCHorizon(horizon);
                }

                lock_choice_.lock();
                choice_ = '-';
                lock_choice_.unlock();
                break;
            default:
                if (paused_)
                {
                    printTime();
                    time_publisher_.runStalledClock(ros::WallDuration(.1));
                    ros::spinOnce();
                }
                else
                    charsleftorpaused = false;
            }
            charsleftorpaused = false;  //remove this line if above section is back
        }

        printTime();
        time_publisher_.runClock(ros::WallDuration(.1));
        ros::spinOnce();
    }
}

TimePublisher::TimePublisher() : time_scale_(1.0)
{
  setPublishFrequency(-1.0);
  time_pub_ = node_handle_.advertise<rosgraph_msgs::Clock>("clock",1);
}

void TimePublisher::insertPassedTime(ros::Time const& t, ros::WallTime const& wt, rosbag::MessageInstance const& m, std::string cid)
{
    passed_time_.push_back(t);
    passed_walltime_.push_back(wt);
    passed_msg_.push_back(m);
    passed_pubs_.push_back(cid);
}

bool TimePublisher::removeAndCheckEmpty()
{
    if(passed_time_.empty() and passed_walltime_.empty() and passed_msg_.empty() and passed_pubs_.empty())
        return true;

    passed_time_.pop_back();
    passed_walltime_.pop_back();
    passed_msg_.pop_back();
    passed_pubs_.pop_back();

    if(passed_time_.empty() and passed_walltime_.empty() and passed_msg_.empty() and passed_pubs_.empty())
        return true;
    else
        return false;
}

void TimePublisher::clearInfo()
{
	passed_time_.clear();
	passed_walltime_.clear();
	passed_msg_.clear();
	passed_pubs_.clear();
}

void TimePublisher::setPublishFrequency(double publish_frequency)
{
    publish_frequency_ = publish_frequency;
  
  do_publish_ = (publish_frequency > 0.0);

  wall_step_.fromSec(1.0 / publish_frequency);
}

void TimePublisher::setTimeScale(double time_scale)
{
    time_scale_ = time_scale;
}

void TimePublisher::setHorizon(const ros::Time& horizon)
{
    horizon_ = horizon;
}

void TimePublisher::setWCHorizon(const ros::WallTime& horizon)
{
    wc_horizon_ = horizon;
}

void TimePublisher::setTime(const ros::Time& time)
{
    current_ = time;
}

ros::Time const& TimePublisher::getTime() const
{
    return current_;
}

ros::WallTime TimePublisher::getPrevWC()
{
    return passed_walltime_.back();
}

std::string TimePublisher::getPrevCallerId()
{
    return passed_pubs_.back();
}

void TimePublisher::runClock(const ros::WallDuration& duration)
{
    if (do_publish_)
    {
        rosgraph_msgs::Clock pub_msg;

        ros::WallTime t = ros::WallTime::now();
        ros::WallTime done = t + duration;

        while (t < done and t < wc_horizon_)
        {
            ros::WallDuration leftHorizonWC = wc_horizon_ - t;

            ros::Duration d(leftHorizonWC.sec, leftHorizonWC.nsec);
            d *= time_scale_;

            current_ = horizon_ - d;

            if (current_ >= horizon_)
              current_ = horizon_;

            if (t >= next_pub_)
            {
                pub_msg.clock = current_;
                time_pub_.publish(pub_msg);
                next_pub_ = t + wall_step_;
            }

            ros::WallTime target = done;
            if (target > wc_horizon_)
              target = wc_horizon_;
            if (target > next_pub_)
              target = next_pub_;

            ros::WallTime::sleepUntil(target);

            t = ros::WallTime::now();
        }
    }
    else {

        ros::WallTime t = ros::WallTime::now();

        ros::WallDuration leftHorizonWC = wc_horizon_ - t;

        ros::Duration d(leftHorizonWC.sec, leftHorizonWC.nsec);
        d *= time_scale_;

        current_ = horizon_ - d;
        
        if (current_ >= horizon_)
            current_ = horizon_;

        ros::WallTime target = ros::WallTime::now() + duration;

        if (target > wc_horizon_)
            target = wc_horizon_;

        ros::WallTime::sleepUntil(target);
    }
}

void TimePublisher::stepClock()
{
    if (do_publish_)
    {
        current_ = horizon_;

        rosgraph_msgs::Clock pub_msg;

        pub_msg.clock = current_;
        time_pub_.publish(pub_msg);

        ros::WallTime t = ros::WallTime::now();
        next_pub_ = t + wall_step_;
    }
    else {
        current_ = horizon_;
    }
}

void TimePublisher::backstepClock()
{
	if (do_publish_)
    {
        current_ = passed_time_.back();

        rosgraph_msgs::Clock pub_msg;

        pub_msg.clock = current_;
        time_pub_.publish(pub_msg);

        ros::WallTime t = ros::WallTime::now();
        next_pub_ = t + wall_step_;
    }
    else {
        current_ = passed_time_.back();
    }
}

void TimePublisher::runStalledClock(const ros::WallDuration& duration)
{
    if (do_publish_)
    {
        rosgraph_msgs::Clock pub_msg;

        ros::WallTime t = ros::WallTime::now();
        ros::WallTime done = t + duration;

        while ( t < done )
        {
            if (t > next_pub_)
            {
                pub_msg.clock = current_;
                time_pub_.publish(pub_msg);
                next_pub_ = t + wall_step_;
            }

            ros::WallTime target = done;

            if (target > next_pub_)
              target = next_pub_;

            ros::WallTime::sleepUntil(target);

            t = ros::WallTime::now();
        }
    }
    else {
        duration.sleep();
    }
}

bool TimePublisher::horizonReached()
{
    return ros::WallTime::now() > wc_horizon_;
}

}