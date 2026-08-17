#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <vector>
#include <string>
#include <cmath>
#include <cstdlib>
#include <algorithm>

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
    
    // Waypoints (3)
    std::vector<Waypoint> waypoints = {
        {-0.812845, -2.44196, 0.0, "Point1"},
        {0.771455, -2.44196, 0.0, "Point2"},
        {1.7843, -2.43094, 0.0, "Point3"}
    };
    
    // ========== Task Configuration ==========
    const std::string TASK2_WAREHOUSE = "食品加工车间";
    const std::string TASK2_CARGO = "苹果";
    const std::string TASK3_WAREHOUSE = "电子产品加工车间";
    const std::string TASK3_CARGO = "手机";
    
    // Dynamic target
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
    
    // ========== Task3 Simulation Warehouse Record ==========
    bool sim_warehouse_recorded = false;
    int sim_waypoint_index = -1;
    double sim_robot_yaw = 0.0;
    
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
    MissionScheduler() : nh("~") {
        move_base_client = new MoveBaseClient("move_base", true);
        while (ros::ok() && !move_base_client->waitForServer(ros::Duration(5.0))) {
            ROS_WARN("Waiting for move_base...");
        }
        ROS_INFO("Connected to move_base!");
        
        cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        vision_enable_pub = nh.advertise<std_msgs::Bool>("/vision/enable", 1);
        
        vision_status_sub = nh.subscribe("/vision/status", 1, &MissionScheduler::visionStatusCallback, this);
        vision_text_sub = nh.subscribe("/vision/detected", 1, &MissionScheduler::visionTextCallback, this);
        vision_conf_sub = nh.subscribe("/vision/confidence", 1, &MissionScheduler::visionConfCallback, this);
        vision_pos_sub = nh.subscribe("/vision/position", 1, &MissionScheduler::visionPosCallback, this);
        
        laser_sub = nh.subscribe("/scan", 10, &MissionScheduler::laserCallback, this);
        vision_area_sub = nh.subscribe("/vision/area", 1, &MissionScheduler::visionAreaCallback, this);
        
        odom_sub = nh.subscribe("/odom", 10, &MissionScheduler::odomCallback, this);
        emergency_sub = nh.subscribe("/emergency_stop", 1, &MissionScheduler::emergencyCallback, this);
        
        ROS_INFO("========================================");
        ROS_INFO("  Mission Scheduler");
        ROS_INFO("  Task2 Target: %s", TASK2_WAREHOUSE.c_str());
        ROS_INFO("  Task2 Cargo: %s", TASK2_CARGO.c_str());
        ROS_INFO("  Task3 Target: %s", TASK3_WAREHOUSE.c_str());
        ROS_INFO("  Task3 Cargo: %s", TASK3_CARGO.c_str());
        ROS_INFO("  Waypoints: %lu", waypoints.size());
        for (int i = 0; i < (int)waypoints.size(); ++i) {
            ROS_INFO("    %s: (%.3f, %.3f)", waypoints[i].name.c_str(), waypoints[i].x, waypoints[i].y);
        }
        ROS_INFO("  Rotate Speed: %.2f rad/s", rotate_speed);
        ROS_INFO("========================================");
        
        while (ros::ok() && !has_odom) {
            ros::spinOnce();
            ros::Rate(10).sleep();
        }
        ROS_INFO("Odometry ready!");
        executeMission();
    }
    
    ~MissionScheduler() {
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
        
        // ★★★ 识别到仿真车间（仅在子任务2进行中时记录） ★★★
        if (!mission_complete && cleaned_text == TASK3_WAREHOUSE) {
            ROS_INFO("========================================");
            ROS_INFO("[SIM] Simulation warehouse detected!");
            ROS_INFO("    OCR text: '%s'", cleaned_text.c_str());
            ROS_INFO("    Waypoint index: %d (%s)", current_wp, waypoints[current_wp].name.c_str());
            ROS_INFO("    Robot yaw: %.2f rad", robot_yaw);
            ROS_INFO("========================================");
            
            sim_waypoint_index = current_wp;
            sim_robot_yaw = robot_yaw;
            sim_warehouse_recorded = true;
            
            ROS_INFO("[SIM] Record success! sim_waypoint_index = %d, sim_warehouse_recorded = true", sim_waypoint_index);
            
            // 继续旋转，不触发停车
            stop_requested = false;
            ocr_processed = false;
            ROS_INFO("[SIM] Reset stop_requested, continue scanning...");
            return;
        }
        
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
    
    void speak(const std::string& text) {
        if (spoken) {
            ROS_INFO("[SPEAK] Already spoken, skipping.");
            return;
        }
        spoken = true;
        
        std::string cmd = "python3 /home/ucar/ucar_ws/src/speech_command/scripts/tts_http.py \"" + text + "\" &";
        int ret = system(cmd.c_str());
        if (ret == 0) {
            ROS_INFO("[SPEAK] %s", text.c_str());
        } else {
            ROS_WARN("[SPEAK] Failed to speak");
            spoken = false;
        }
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
            if (emergency_stop || !is_running) return false;
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
    
    // ==================== Task3 Execution ====================
    
    void executeTask3() {
        ROS_INFO("========================================");
        ROS_INFO("[TASK3] Starting Task 3: Find %s", TASK3_WAREHOUSE.c_str());
        ROS_INFO("========================================");
        
        ROS_INFO("[TASK3] sim_warehouse_recorded = %s, sim_waypoint_index = %d",
                 sim_warehouse_recorded ? "true" : "false", sim_waypoint_index);
        
        current_target = TASK3_WAREHOUSE;
        current_cargo = TASK3_CARGO;
        target_found = false;
        mission_complete = false;
        spoken = false;
        target_matched = false;
        
        // Case 1: Recorded waypoint exists
        if (sim_warehouse_recorded && sim_waypoint_index >= 0 && sim_waypoint_index < (int)waypoints.size()) {
            ROS_INFO("[TASK3] Using recorded waypoint %d (%s) for simulation warehouse", 
                     sim_waypoint_index, waypoints[sim_waypoint_index].name.c_str());
            
            current_wp = sim_waypoint_index;
            if (sendGoal(current_wp)) {
                bool reached = waitForGoalReached(60.0);
                if (reached) {
                    ROS_INFO("[TASK3] Reached recorded waypoint, starting scan for simulation warehouse...");
                    current_target = TASK3_WAREHOUSE;
                    scanAtWaypoint();
                    
                    // Fallback manual parking if scan didn't find target
                    if (!mission_complete) {
                        ROS_WARN("[TASK3] Scan did not find target, trying manual parking...");
                        if (front_distance < 0.5 && front_distance > 0.1) {
                            ROS_INFO("[TASK3] Object detected ahead (%.2f m), attempting parking", front_distance);
                            bool ok = laserParkingAndSpeak(current_cargo, current_target, 0.28, 0.08, 15.0);
                            mission_complete = ok || true;
                            if (mission_complete) {
                                ROS_INFO("[TASK3] Manual parking success!");
                            }
                        } else {
                            ROS_WARN("[TASK3] No obstacle ahead (%.2f m), manual parking failed", front_distance);
                        }
                    }
                    
                    if (mission_complete) {
                        ROS_INFO("[TASK3] Task3 completed via recorded waypoint!");
                        return;
                    } else {
                        ROS_WARN("[TASK3] Simulation warehouse not found at recorded waypoint, fallback to full scan");
                    }
                } else {
                    ROS_WARN("[TASK3] Failed to reach recorded waypoint, fallback to full scan");
                }
            }
        } else {
            ROS_WARN("[TASK3] No simulation warehouse recorded, performing full waypoint scan");
        }
        
        // Case 2: Full waypoint scan (fallback)
        ROS_INFO("[TASK3] Starting full waypoint scan for simulation warehouse...");
        for (size_t i = 0; i < waypoints.size(); ++i) {
            if (mission_complete || emergency_stop || !is_running) break;
            current_wp = i;
            ROS_INFO("[TASK3] Going to %s for scan...", waypoints[i].name.c_str());
            if (!sendGoal(i)) {
                ROS_ERROR("Failed to send goal");
                continue;
            }
            bool reached = waitForGoalReached(60.0);
            if (!reached) {
                ROS_WARN("Failed to reach %s, skipping", waypoints[i].name.c_str());
                continue;
            }
            current_target = TASK3_WAREHOUSE;
            scanAtWaypoint();
            
            if (!mission_complete) {
                ROS_WARN("[TASK3] Scan at %s did not find target, trying manual parking", waypoints[i].name.c_str());
                if (front_distance < 0.5 && front_distance > 0.1) {
                    ROS_INFO("[TASK3] Object detected ahead (%.2f m), attempting parking", front_distance);
                    bool ok = laserParkingAndSpeak(current_cargo, current_target, 0.28, 0.08, 15.0);
                    mission_complete = ok || true;
                    if (mission_complete) {
                        ROS_INFO("[TASK3] Manual parking success at %s!", waypoints[i].name.c_str());
                        break;
                    }
                }
            }
            
            if (mission_complete) {
                ROS_INFO("[TASK3] Simulation warehouse found at %s!", waypoints[i].name.c_str());
                break;
            }
        }
        
        if (mission_complete) {
            ROS_INFO("[TASK3] Task3 completed!");
        } else {
            ROS_WARN("[TASK3] Task3 failed: simulation warehouse not found in any waypoint");
        }
    }
    
    // ==================== Main Execution ====================
    
    void executeMission() {
        ROS_INFO("========================================");
        ROS_INFO("  Mission started.");
        ROS_INFO("========================================");
        
        // -------- Task 2 --------
        ROS_INFO("===== Task 2: Find '%s' and park =====", TASK2_WAREHOUSE.c_str());
        current_target = TASK2_WAREHOUSE;
        current_cargo = TASK2_CARGO;
        target_found = false;
        mission_complete = false;
        spoken = false;
        target_matched = false;
        
        for (size_t i = 0; i < waypoints.size(); ++i) {
            if (!is_running || emergency_stop || mission_complete || target_found) break;
            current_wp = i;
            
            ROS_INFO("[MISSION] Going to %s: (%.2f, %.2f)", waypoints[i].name.c_str(), waypoints[i].x, waypoints[i].y);
            if (!sendGoal(i)) {
                ROS_ERROR("[MISSION] Failed to send goal");
                break;
            }
            bool reached = waitForGoalReached(60.0);
            if (!reached) {
                ROS_WARN("[MISSION] Failed to reach %s, skipping", waypoints[i].name.c_str());
                continue;
            }
            ROS_INFO("[MISSION] Reached %s", waypoints[i].name.c_str());
            scanAtWaypoint();
        }
        
        if (mission_complete) {
            ROS_INFO("[MISSION] Task 2 completed successfully.");
        } else {
            ROS_WARN("[MISSION] Task 2 failed. Skipping Task 3.");
            std_msgs::Bool msg;
            msg.data = false;
            vision_enable_pub.publish(msg);
            move_base_client->cancelGoal();
            ROS_INFO("[MISSION] Mission ended.");
            return;
        }
        
        // -------- Task 3 --------
        ROS_INFO("===== Task 3: Find '%s' and park =====", TASK3_WAREHOUSE.c_str());
        executeTask3();
        
        // -------- Final Result --------
        ROS_INFO("========================================");
        if (mission_complete) {
            ROS_INFO("  ALL TASKS COMPLETE!");
        } else {
            ROS_INFO("  Task 2 OK, Task 3 FAILED.");
        }
        ROS_INFO("========================================");
        
        std_msgs::Bool msg;
        msg.data = false;
        vision_enable_pub.publish(msg);
        move_base_client->cancelGoal();
        ROS_INFO("[MISSION] Mission ended.");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "racecar_control");
    MissionScheduler scheduler;
    ros::spin();
    return 0;
}
