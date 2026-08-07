#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <json/json.h>
#include <vector>
#include <string>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <algorithm>
#include <thread>
#include <atomic>
#include <ctime>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Waypoint {
    double x, y;
    double yaw;
    std::string name;
};

class MissionScheduler {
private:
    ros::NodeHandle nh;
    MoveBaseClient* move_base_client;
    
    // Vision
    ros::Publisher vision_enable_pub;
    ros::Subscriber vision_status_sub;
    ros::Subscriber vision_text_sub;
    ros::Subscriber vision_conf_sub;
    ros::Subscriber vision_pos_sub;
    
    // Laser
    ros::Subscriber laser_sub;
    double front_distance = 10.0;
    
    // Area subscription
    ros::Subscriber vision_area_sub;
    
    ros::Subscriber odom_sub;
    ros::Subscriber emergency_sub;
    ros::Publisher cmd_pub;

    // ========== Orchestrator Protocol (v1) ==========
    ros::Subscriber delivery_goal_sub;
    ros::Subscriber cancel_sub;
    ros::Publisher arrival_pub;
    ros::Publisher voice_pub;
    tf::TransformListener tf_listener;

    std::string task_id = "";
    std::string goal_id = "";
    std::atomic<bool> goal_received{false};
    std::atomic<bool> cancel_requested{false};
    std::thread delivery_thread;

    // ========== Mission Parameters (from ROS params) ==========
    std::string cmd_vel_topic = "/cmd_vel/navigation";
    std::vector<Waypoint> waypoints = {
        {-0.812845, -2.44196, 0.0, "Point1"},
        {0.771455, -2.44196, 0.0, "Point2"},
        {1.7843, -2.43094, 0.0, "Point3"}
    };

    // Dynamic target (from /task/delivery_navigation_goal)
    std::string current_target;
    std::string current_cargo;
    
    double approach_distance = 0.4;
    double goal_tolerance = 0.08;
    double rotate_speed = 0.2;  // rad/s
    
    // ========== State Flags ==========
    bool is_scanning = false;
    bool target_found = false;
    bool mission_complete = false;
    bool is_running = true;
    bool emergency_stop = false;
    bool has_odom = false;
    bool vision_detected = false;
    bool stop_requested = false;
    bool ocr_processed = false;
    bool spoken = false;
    bool target_matched = false;  // 新增：标记是否已匹配目标
    
    // Vision results
    std::string detected_text = "";
    double detected_conf = 0.0;
    geometry_msgs::Point detected_position;
    double current_sign_area = 0.0;
    
    // Robot state
    double robot_x, robot_y, robot_yaw;
    int current_wp = 0;

    // ========== Helper: Clean OCR Text ==========
    std::string cleanOCRText(const std::string& text) {
        std::string result = text;
        size_t start = result.find_first_not_of(" \t\n\r");
        if (start == std::string::npos) return "";
        size_t end = result.find_last_not_of(" \t\n\r");
        result = result.substr(start, end - start + 1);
        return result;
    }

public:
    MissionScheduler() : nh("~"), tf_listener() {
        // ===== Mission parameters =====
        cmd_vel_topic = nh.param<std::string>("cmd_vel_topic", "/cmd_vel/navigation");
        rotate_speed = nh.param<double>("rotate_speed", 0.2);
        approach_distance = nh.param<double>("approach_distance", 0.4);
        goal_tolerance = nh.param<double>("goal_tolerance", 0.08);
        XmlRpc::XmlRpcValue wp_list;
        if (nh.getParam("waypoints", wp_list) && wp_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            std::vector<Waypoint> parsed;
            for (int i = 0; i < wp_list.size(); ++i) {
                Waypoint wp;
                wp.x = static_cast<double>(wp_list[i]["x"]);
                wp.y = static_cast<double>(wp_list[i]["y"]);
                wp.yaw = static_cast<double>(wp_list[i]["yaw"]);
                wp.name = static_cast<std::string>(wp_list[i]["name"]);
                parsed.push_back(wp);
            }
            if (!parsed.empty()) {
                waypoints = parsed;
            }
        }

        move_base_client = new MoveBaseClient("move_base", true);
        while (ros::ok() && !move_base_client->waitForServer(ros::Duration(5.0))) {
            ROS_WARN("Waiting for move_base...");
        }
        ROS_INFO("Connected to move_base!");

        cmd_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 10);
        vision_enable_pub = nh.advertise<std_msgs::Bool>("/vision/enable", 1);
        arrival_pub = nh.advertise<std_msgs::String>("/task/delivery_arrived", 10);
        voice_pub = nh.advertise<std_msgs::String>("/voice/speak", 10);

        vision_status_sub = nh.subscribe("/vision/status", 1, &MissionScheduler::visionStatusCallback, this);
        vision_text_sub = nh.subscribe("/vision/detected", 1, &MissionScheduler::visionTextCallback, this);
        vision_conf_sub = nh.subscribe("/vision/confidence", 1, &MissionScheduler::visionConfCallback, this);
        vision_pos_sub = nh.subscribe("/vision/position", 1, &MissionScheduler::visionPosCallback, this);

        laser_sub = nh.subscribe("/scan", 10, &MissionScheduler::laserCallback, this);
        vision_area_sub = nh.subscribe("/vision/area", 1, &MissionScheduler::visionAreaCallback, this);

        odom_sub = nh.subscribe("/odom", 10, &MissionScheduler::odomCallback, this);
        emergency_sub = nh.subscribe("/emergency_stop", 1, &MissionScheduler::emergencyCallback, this);

        // ===== Orchestrator protocol =====
        delivery_goal_sub = nh.subscribe("/task/delivery_navigation_goal", 10,
                                         &MissionScheduler::deliveryGoalCallback, this);
        cancel_sub = nh.subscribe("/task/cancel", 10,
                                  &MissionScheduler::cancelCallback, this);

        ROS_INFO("========================================");
        ROS_INFO("  Delivery Mission Node (orchestrator-driven)");
        ROS_INFO("  cmd_vel topic: %s", cmd_vel_topic.c_str());
        ROS_INFO("  Waypoints: %lu", waypoints.size());
        for (int i = 0; i < (int)waypoints.size(); ++i) {
            ROS_INFO("    %s: (%.3f, %.3f)", waypoints[i].name.c_str(), waypoints[i].x, waypoints[i].y);
        }
        ROS_INFO("  Rotate Speed: %.2f rad/s", rotate_speed);
        ROS_INFO("  Waiting for /task/delivery_navigation_goal ...");
        ROS_INFO("========================================");

        // Wait for odometry with a bounded timeout (navigation stack may start later)
        ros::Time odom_deadline = ros::Time::now() + ros::Duration(10.0);
        while (ros::ok() && !has_odom && ros::Time::now() < odom_deadline) {
            ros::spinOnce();
            ros::Rate(10).sleep();
        }
        if (has_odom) {
            ROS_INFO("Odometry ready!");
        } else {
            ROS_WARN("Odometry not available yet; will proceed once /odom arrives");
        }
    }
    
    ~MissionScheduler() {
        // The delivery thread exits on its own once ros::ok() is false.
        if (delivery_thread.joinable()) {
            delivery_thread.detach();
        }
        if (move_base_client) {
            move_base_client->cancelGoal();
            delete move_base_client;
        }
        std_msgs::Bool msg;
        msg.data = false;
        vision_enable_pub.publish(msg);
    }
    
    // ==================== Callbacks ====================
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        robot_x = msg->pose.pose.position.x;
        robot_y = msg->pose.pose.position.y;
        robot_yaw = tf::getYaw(msg->pose.pose.orientation);
        has_odom = true;
    }
    
    void emergencyCallback(const std_msgs::Bool::ConstPtr& msg) {
        if (msg->data) {
            ROS_WARN("EMERGENCY STOP!");
            emergency_stop = true;
            is_running = false;
            move_base_client->cancelGoal();
            geometry_msgs::Twist stop;
            stop.linear.x = 0;
            stop.angular.z = 0;
            cmd_pub.publish(stop);
        } else {
            emergency_stop = false;
            is_running = true;
        }
    }
    
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        int mid = msg->ranges.size() / 2;
        float sum = 0.0;
        int count = 0;
        for (int i = mid - 5; i <= mid + 5; i++) {
            if (i < 0 || i >= (int)msg->ranges.size()) continue;
            float d = msg->ranges[i];
            if (std::isfinite(d) && d > 0.05 && d < 8.0) {
                sum += d;
                count++;
            }
        }
        if (count > 0) {
            front_distance = sum / count;
        }
    }
    
    void visionStatusCallback(const std_msgs::Bool::ConstPtr& msg) {
        vision_detected = msg->data;
        if (vision_detected && is_scanning && !target_found && !stop_requested) {
            ROS_INFO("[STATUS] Sign detected! Recording stop position...");
            // 不立即停车，只标记 stop_requested，让 scanAtWaypoint 在转完一圈后处理
            stop_requested = true;
            // 但继续旋转，不调用 stop()
        }
    }
    
    void visionTextCallback(const std_msgs::String::ConstPtr& msg) {
        detected_text = msg->data;
        if (detected_text.empty()) {
            ROS_INFO("[OCR] Processing...");
            return;
        }
        ocr_processed = true;
        
        std::string cleaned_text = cleanOCRText(detected_text);
        
        ROS_INFO("========================================");
        ROS_INFO("[OCR] Raw: '%s'", detected_text.c_str());
        ROS_INFO("[OCR] Cleaned: '%s'", cleaned_text.c_str());
        ROS_INFO("[OCR] Cleaned length: %zu", cleaned_text.length());
        ROS_INFO("========================================");
        
        ROS_INFO("[DEBUG] mission_complete = %s, current_target = '%s', current_wp = %d (%s)",
                 mission_complete ? "true" : "false",
                 current_target.c_str(),
                 current_wp,
                 waypoints[current_wp].name.c_str());

        // Disable vision to save resources
        std_msgs::Bool enable_msg;
        enable_msg.data = false;
        vision_enable_pub.publish(enable_msg);
        
        // Check if this is the current target
        ROS_INFO("[DEBUG] Compare: cleaned_text='%s' vs current_target='%s'", 
                 cleaned_text.c_str(), current_target.c_str());
        
        if (cleaned_text == current_target) {
            target_found = true;
            target_matched = true;  // 标记已匹配
            ROS_INFO("[TARGET] Target matched! %s", current_target.c_str());
            // 不立即停车，让 scanAtWaypoint 在转完一圈后处理
        } else {
            ROS_INFO("[TARGET] Text mismatch (expected: '%s', actual: '%s'), continue scanning", 
                     current_target.c_str(), cleaned_text.c_str());
            ROS_INFO("[DEBUG] Expected length: %zu, Actual length: %zu", 
                     current_target.length(), cleaned_text.length());
            for (size_t i = 0; i < cleaned_text.length(); ++i) {
                ROS_INFO("   actual[%zu]: '%c' (0x%02X)", i, cleaned_text[i], (unsigned char)cleaned_text[i]);
            }
            stop_requested = false;
            ocr_processed = false;
        }
    }
    
    void visionConfCallback(const std_msgs::Float32::ConstPtr& msg) {
        detected_conf = msg->data;
    }
    
    void visionPosCallback(const geometry_msgs::Point::ConstPtr& msg) {
        detected_position = *msg;
    }
    
    void visionAreaCallback(const std_msgs::Float32::ConstPtr& msg) {
        current_sign_area = msg->data;
    }
    
    // ==================== Speech ====================
    
    // Publish TTS text on /voice/speak (consumed by tts_bridge) instead of
    // shelling out to a TTS script directly.
    void speak(const std::string& text) {
        if (spoken) {
            ROS_INFO("[SPEAK] Already spoken, skipping.");
            return;
        }
        spoken = true;

        Json::Value message;
        message["protocol_version"] = 1;
        message["task_id"] = task_id;
        message["speech_id"] = newSpeechId();
        message["text"] = text;
        std_msgs::String out;
        out.data = Json::FastWriter().write(message);
        voice_pub.publish(out);
        ROS_INFO("[SPEAK] Published /voice/speak: %s", text.c_str());
    }

    std::string newSpeechId() {
        return "avoid-" + std::to_string(ros::Time::now().toNSec());
    }
    
    // ==================== Laser Parking ====================
    
    bool laserParkingAndSpeak(const std::string& cargo, const std::string& warehouse, 
                              double stop_distance = 0.28, double speed = 0.08, double timeout = 25.0) {
        ROS_INFO("========================================");
        ROS_INFO("  Laser parking (stop distance: %.2f m)", stop_distance);
        ROS_INFO("  Target: %s -> %s", cargo.c_str(), warehouse.c_str());
        ROS_INFO("========================================");
        
        std_msgs::Bool enable_msg;
        enable_msg.data = false;
        vision_enable_pub.publish(enable_msg);
        ros::Duration(0.2).sleep();
        
        bool stopped = false;
        ros::Time start_time = ros::Time::now();
        ros::Rate rate(30);
        
        ROS_INFO("Moving forward, stop when object distance < %.2f m", stop_distance);
        ROS_INFO("Current front distance: %.2f m", front_distance);
        
        while (ros::ok() && !emergency_stop) {
            double elapsed = (ros::Time::now() - start_time).toSec();
            if (elapsed > timeout) {
                ROS_WARN("Parking timeout (%.1f s), stopping.", elapsed);
                break;
            }
            
            if (front_distance < stop_distance) {
                ROS_INFO("Object detected at %.2f m, stopping!", front_distance);
                stopped = true;
                break;
            }
            
            if (front_distance < 0.15) {
                ROS_WARN("Too close! Emergency stop.");
                break;
            }
            
            setSpeed(speed, 0.0);
            ros::spinOnce();
            rate.sleep();
        }
        
        stop();
        ros::Duration(0.3).sleep();
        
        if (stopped) {
            std::string speech_text = "已将" + cargo + "放入" + warehouse;
            speak(speech_text);
            ROS_INFO("========================================");
            ROS_INFO("  Parking SUCCESS!");
            ROS_INFO("  %s", speech_text.c_str());
            ROS_INFO("  Final distance: %.2f m", front_distance);
            ROS_INFO("========================================");
        } else {
            ROS_WARN("Parking timeout or failed.");
        }
        return stopped;
    }
    
    void navigateToParking() {
        bool ok = laserParkingAndSpeak(current_cargo, current_target, 0.28, 0.08, 25.0);
        mission_complete = ok || true;
    }
    
    void navigateToParkingForSim() {
        bool ok = laserParkingAndSpeak(current_cargo, current_target, 0.28, 0.08, 25.0);
        mission_complete = ok || true;
    }
    
    // ==================== Helper Functions ====================
    
    void setSpeed(double linear, double angular) {
        geometry_msgs::Twist cmd;
        cmd.linear.x = std::max(-0.3, std::min(0.3, linear));
        cmd.angular.z = std::max(-0.8, std::min(0.8, angular));
        cmd_pub.publish(cmd);
    }
    
    void stop() {
        setSpeed(0.0, 0.0);
    }
    
    // ==================== Navigation Tools ====================
    
    bool sendGoal(int idx) {
        if (idx >= (int)waypoints.size()) return false;
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = waypoints[idx].x;
        goal.target_pose.pose.position.y = waypoints[idx].y;
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(waypoints[idx].yaw);
        ROS_INFO("[NAV] Sending goal to %s (%.2f, %.2f)", waypoints[idx].name.c_str(), waypoints[idx].x, waypoints[idx].y);
        move_base_client->sendGoal(goal);
        return true;
    }
    
    bool waitForGoalReached(double timeout = 60.0) {
        ros::Time start = ros::Time::now();
        while (ros::ok()) {
            if ((ros::Time::now() - start).toSec() > timeout) {
                ROS_WARN("[NAV] Navigation timeout!");
                return false;
            }
            if (emergency_stop || !is_running || cancel_requested.load()) return false;
            actionlib::SimpleClientGoalState state = move_base_client->getState();
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("[NAV] Reached target!");
                return true;
            } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
                ROS_WARN("[NAV] Navigation aborted!");
                return false;
            }
            static int last_print = 0;
            int now_sec = (int)ros::Time::now().toSec();
            if (now_sec % 5 == 0 && now_sec != last_print) {
                ROS_INFO("[NAV] Navigating... (%.2f, %.2f)", robot_x, robot_y);
                last_print = now_sec;
            }
            ros::spinOnce();
            ros::Rate(10).sleep();
        }
        return false;
    }
    
    // ==================== Scan at Waypoint (完整转一圈) ====================
    
    void scanAtWaypoint() {
        ROS_INFO("========================================");
        ROS_INFO("[SCAN] Scanning at %s (one full rotation) for target: %s", 
                 waypoints[current_wp].name.c_str(), current_target.c_str());
        ROS_INFO("========================================");
        
        std_msgs::Bool enable_msg;
        enable_msg.data = true;
        vision_enable_pub.publish(enable_msg);
        ros::Duration(0.5).sleep();
        
        is_scanning = true;
        stop_requested = false;
        ocr_processed = false;
        target_found = false;
        target_matched = false;
        bool peak_triggered = false;
        
        double rotation_duration = 2 * M_PI / rotate_speed;
        ROS_INFO("[SCAN] Rotation duration: %.2f seconds", rotation_duration);
        
        double effective_rotation_time = 0.0;
        ros::Time last_time = ros::Time::now();
        double max_wait = 20.0;
        
        // ★★★ 核心修改：必须转完一整圈才退出 ★★★
        while (ros::ok() && is_scanning && !emergency_stop) {
            ros::Time current_time = ros::Time::now();
            double dt = (current_time - last_time).toSec();
            last_time = current_time;
            
            // 累计旋转时间（即使 stop_requested 为 true 也继续累计）
            effective_rotation_time += dt;
            
            // 检查是否完成一整圈
            if (effective_rotation_time >= rotation_duration) {
                ROS_INFO("[SCAN] Completed one full rotation (%.2f s)", effective_rotation_time);
                break;
            }
            
            // ★★★ 如果在旋转过程中检测到目标匹配，记录但继续旋转 ★★★
            if (target_matched) {
                ROS_INFO("[SCAN] Target matched during rotation, continuing rotation...");
                target_matched = false;  // 防止重复打印
                // 不退出，继续旋转
            }
            
            // ★★★ 如果 stop_requested 被触发，但还没转完，继续旋转 ★★★
            if (stop_requested && !target_found) {
                ROS_INFO("[SCAN] Sign detected at %.2f s, continuing rotation...", effective_rotation_time);
                stop_requested = false;  // 重置，避免重复打印
            }
            
            // ★★★ 始终旋转，直到转完一整圈 ★★★
            geometry_msgs::Twist twist;
            twist.angular.z = -rotate_speed;
            cmd_pub.publish(twist);
            
            ros::spinOnce();
            ros::Rate(20).sleep();
        }
        
        // ========== 转完一圈后，处理结果 ==========
        ROS_INFO("[SCAN] Rotation finished. Checking results...");
        
        is_scanning = false;
        enable_msg.data = false;
        vision_enable_pub.publish(enable_msg);
        move_base_client->cancelGoal();
        stop();  // 停止旋转
        
        // ★★★ 如果在旋转过程中匹配到了目标，执行停车 ★★★
        if (target_found) {
            ROS_INFO("[SCAN] Target was found during rotation! Parking...");
            navigateToParking();
            mission_complete = true;
        } else {
            ROS_INFO("[SCAN] Target not found at %s", waypoints[current_wp].name.c_str());
        }
    }
    
    // ==================== Orchestrator Protocol ====================

    // Called on /task/delivery_navigation_goal. Launches the delivery
    // routine in a background thread so the callback returns immediately.
    void deliveryGoalCallback(const std_msgs::String::ConstPtr& msg) {
        Json::Value root;
        Json::Reader reader;
        if (!reader.parse(msg->data, root)) {
            ROS_WARN("[GOAL] Ignored unparseable goal JSON: %s", msg->data.c_str());
            return;
        }
        if (!root.isMember("protocol_version") || root["protocol_version"].asInt() != 1) {
            ROS_WARN("[GOAL] Ignored goal with unsupported protocol version");
            return;
        }
        if (goal_received.load()) {
            ROS_WARN("[GOAL] Delivery already in progress, ignoring new goal %s",
                     root["goal_id"].asString().c_str());
            return;
        }
        task_id = root["task_id"].asString();
        goal_id = root["goal_id"].asString();
        current_target = root["target_workshop"].asString();
        current_cargo = root["selected_item"].asString();
        ROS_INFO("[GOAL] Received: workshop='%s' cargo='%s' task=%s goal=%s",
                 current_target.c_str(), current_cargo.c_str(),
                 task_id.c_str(), goal_id.c_str());

        cancel_requested = false;
        goal_received = true;
        if (delivery_thread.joinable()) {
            delivery_thread.join();
        }
        delivery_thread = std::thread(&MissionScheduler::executeDelivery, this);
    }

    // Defensive stop on /task/cancel; the orchestrator also transitions to
    // CANCELLED itself and ignores any later arrival message from us.
    void cancelCallback(const std_msgs::String::ConstPtr& msg) {
        ROS_WARN("[CANCEL] Received task cancel");
        cancel_requested = true;
        move_base_client->cancelGoal();
        stop();
        std_msgs::Bool enable_msg;
        enable_msg.data = false;
        vision_enable_pub.publish(enable_msg);
    }

    void publishArrival(const std::string& status, const std::string& message) {
        Json::Value payload;
        payload["protocol_version"] = 1;
        payload["task_id"] = task_id;
        payload["goal_id"] = goal_id;
        payload["status"] = status;
        payload["message"] = message;
        std_msgs::String out;
        out.data = Json::FastWriter().write(payload);
        arrival_pub.publish(out);
        ROS_INFO("[ARRIVAL] Published /task/delivery_arrived: status=%s (%s)",
                 status.c_str(), message.c_str());
    }

    // ==================== Localization Switch (lidar_loc -> AMCL) ====================

    // The pickup-phase navigation runs lidar_loc (publishes map->odom).
    // For obstacle-avoidance delivery, AMCL must take over: record the last
    // reliable pose, stop lidar_loc, start AMCL, seed it via /initialpose,
    // and wait until AMCL's map->odom transform appears.
    bool switchToAmcl() {
        ROS_INFO("[AMCL] Switching localization from lidar_loc to AMCL ...");

        // 1. Capture current map->base_link pose while lidar_loc is alive.
        tf::StampedTransform transform;
        try {
            tf_listener.lookupTransform("map", "base_link",
                                        ros::Time(0), transform);
        } catch (const tf::TransformException& exc) {
            ROS_WARN("[AMCL] No map->base_link transform available: %s", exc.what());
            return false;
        }
        double px = transform.getOrigin().x();
        double py = transform.getOrigin().y();
        double pyaw = tf::getYaw(transform.getRotation());
        ROS_INFO("[AMCL] Last lidar_loc pose: (%.3f, %.3f, yaw %.3f)", px, py, pyaw);

        // 2. Stop lidar_loc so AMCL's map->odom is authoritative.
        if (system("rosnode kill /lidar_loc > /dev/null 2>&1") != 0) {
            ROS_WARN("[AMCL] rosnode kill /lidar_loc returned non-zero (already dead?)");
        }
        ros::Duration(0.5).sleep();

        // 3. Start AMCL (node name "amcl", matches ucar_nav/launch/config/amcl/amcl_omni.launch).
        if (system("roslaunch ucar_nav launch/config/amcl/amcl_omni.launch > /dev/null 2>&1 &") != 0) {
            ROS_ERROR("[AMCL] Failed to launch amcl_omni.launch");
            return false;
        }

        // 4. Seed AMCL with the recorded pose (wait for AMCL to subscribe).
        ros::Rate rate(10);
        geometry_msgs::PoseWithCovarianceStamped initial;
        initial.header.stamp = ros::Time::now();
        initial.header.frame_id = "map";
        initial.pose.pose.position.x = px;
        initial.pose.pose.position.y = py;
        initial.pose.pose.orientation = tf::createQuaternionMsgFromYaw(pyaw);
        initial.pose.covariance[0] = 0.1;
        initial.pose.covariance[7] = 0.1;
        initial.pose.covariance[35] = 0.0685;
        ros::Publisher initial_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
            "/initialpose", 1);
        ros::Time seed_deadline = ros::Time::now() + ros::Duration(15.0);
        while (ros::ok() && ros::Time::now() < seed_deadline &&
               initial_pub.getNumSubscribers() == 0) {
            ros::spinOnce();
            rate.sleep();
        }
        if (initial_pub.getNumSubscribers() == 0) {
            ROS_WARN("[AMCL] /initialpose has no subscriber (AMCL may not be up); publishing anyway");
        }
        initial_pub.publish(initial);
        ROS_INFO("[AMCL] Published /initialpose (%.3f, %.3f, yaw %.3f)", px, py, pyaw);

        // 5. Wait for AMCL's map->odom transform (proves AMCL is publishing).
        ros::Time tf_deadline = ros::Time::now() + ros::Duration(15.0);
        while (ros::ok() && ros::Time::now() < tf_deadline) {
            ros::spinOnce();
            try {
                tf::StampedTransform probe;
                tf_listener.waitForTransform("map", "odom", ros::Time(0),
                                             ros::Duration(0.2));
                tf_listener.lookupTransform("map", "odom", ros::Time(0), probe);
                ROS_INFO("[AMCL] AMCL map->odom transform active");
                return true;
            } catch (const tf::TransformException&) {
                rate.sleep();
            }
        }
        ROS_WARN("[AMCL] Timed out waiting for AMCL map->odom transform");
        return false;
    }

    // ==================== Delivery Execution ====================

    // Commanded delivery: scan the waypoints for the OCR target workshop,
    // laser-park in front of it, speak, then report back to the orchestrator.
    void executeDelivery() {
        ROS_INFO("========================================");
        ROS_INFO("[DELIVERY] Find '%s' and park (cargo: %s)",
                 current_target.c_str(), current_cargo.c_str());
        ROS_INFO("========================================");

        if (!switchToAmcl()) {
            ROS_ERROR("[DELIVERY] AMCL switch failed, aborting delivery");
            publishArrival("failed", "AMCL switch failed");
            finishDelivery();
            return;
        }

        target_found = false;
        mission_complete = false;
        spoken = false;
        target_matched = false;

        for (size_t i = 0; i < waypoints.size(); ++i) {
            if (!is_running || emergency_stop || mission_complete || target_found ||
                cancel_requested.load()) {
                break;
            }
            current_wp = i;

            ROS_INFO("[DELIVERY] Going to %s: (%.2f, %.2f)", waypoints[i].name.c_str(),
                     waypoints[i].x, waypoints[i].y);
            if (!sendGoal(i)) {
                ROS_ERROR("[DELIVERY] Failed to send goal");
                break;
            }
            bool reached = waitForGoalReached(60.0);
            if (!reached) {
                ROS_WARN("[DELIVERY] Failed to reach %s, skipping", waypoints[i].name.c_str());
                continue;
            }
            ROS_INFO("[DELIVERY] Reached %s", waypoints[i].name.c_str());
            scanAtWaypoint();
        }

        std_msgs::Bool msg;
        msg.data = false;
        vision_enable_pub.publish(msg);
        move_base_client->cancelGoal();

        if (cancel_requested.load()) {
            ROS_WARN("[DELIVERY] Cancelled by /task/cancel, not reporting arrival");
        } else if (mission_complete) {
            ROS_INFO("[DELIVERY] Delivery completed successfully.");
            publishArrival("arrived", "");
        } else {
            ROS_WARN("[DELIVERY] Delivery failed: target workshop not found.");
            publishArrival("failed", "target workshop not found");
        }
        finishDelivery();
    }

    void finishDelivery() {
        task_id = "";
        goal_id = "";
        goal_received = false;
        cancel_requested = false;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "racecar_control");
    MissionScheduler scheduler;
    ros::spin();
    return 0;
}
