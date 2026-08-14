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
    
    const std::string TARGET_WAREHOUSE = "食品加工车间";
    const std::string CARGO_NAME = "苹果";
    double goal_tolerance = 0.08;
    double rotate_speed = 0.3;  // rad/s
    
    // State
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
    bool peak_triggered = false;
    
    // 视觉反馈
    double current_sign_area = 0.0;
    std::string detected_text = "";
    double detected_conf = 0.0;
    geometry_msgs::Point detected_position;
    
    double robot_x, robot_y, robot_yaw;
    int current_wp = 0;

    // ===== 子任务3 (仿真车间) =====
    const std::string SIM_WAREHOUSE = "日用品加工车间";
    const std::string SIM_CARGO = "牙刷";
    bool sim_warehouse_found = false;
    int sim_waypoint_index = -1;
    double sim_robot_yaw = 0.0;          // ★ 记录朝向
    bool sim_task_complete = false;
    bool sim_scanning_mode = false;

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
        ROS_INFO("  Target: %s", TARGET_WAREHOUSE.c_str());
        ROS_INFO("  Cargo: %s", CARGO_NAME.c_str());
        ROS_INFO("  Sim Target: %s", SIM_WAREHOUSE.c_str());
        ROS_INFO("  Sim Cargo: %s", SIM_CARGO.c_str());
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
        if (vision_detected && is_scanning && !stop_requested) {
            if (!sim_scanning_mode && target_found) {
                return;
            }
            ROS_INFO("🎯 Peak detected! Stopping rotation...");
            geometry_msgs::Twist stop;
            stop.linear.x = 0;
            stop.angular.z = 0;
            cmd_pub.publish(stop);
            move_base_client->cancelGoal();
            stop_requested = true;
            peak_triggered = true;
            ROS_INFO("Robot stopped, waiting for OCR...");
        }
    }
    
    void visionTextCallback(const std_msgs::String::ConstPtr& msg) {
        detected_text = msg->data;
        if (detected_text.empty()) {
            ROS_INFO("OCR processing...");
            return;
        }
        ocr_processed = true;
        ROS_INFO("OCR result: %s", detected_text.c_str());
        
        if (detected_text == TARGET_WAREHOUSE && !sim_scanning_mode) {
            target_found = true;
            ROS_INFO("✅ Target matched! Parking...");
            geometry_msgs::Twist stop;
            stop.linear.x = 0;
            stop.angular.z = 0;
            cmd_pub.publish(stop);
            move_base_client->cancelGoal();
            std_msgs::Bool enable_msg;
            enable_msg.data = false;
            vision_enable_pub.publish(enable_msg);
            navigateToParking();
            return;
        }
        
        if (detected_text == SIM_WAREHOUSE) {
            if (sim_scanning_mode) {
                ROS_INFO("SIM warehouse detected during sim scanning.");
            } else {
                // ★ 子任务2扫描时，记录航点索引和当前朝向
                if (!sim_warehouse_found) {
                    sim_warehouse_found = true;
                    sim_waypoint_index = current_wp;
                    sim_robot_yaw = robot_yaw;   // 记录当前机器人朝向
                    ROS_INFO("📝 Sim warehouse detected at waypoint %d, yaw=%.3f rad (recorded)", 
                             sim_waypoint_index, sim_robot_yaw);
                }
            }
            return;
        }
        
        ROS_INFO("❌ Text not matched (expected: %s), will continue scanning.", TARGET_WAREHOUSE.c_str());
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
    
    // ==================== 语音播报 ====================
    void speak(const std::string& text) {
        if (spoken) {
            ROS_INFO("Already spoken, skipping.");
            return;
        }
        spoken = true;
        
        std::string cmd = "python3 /home/ucar/ucar_ws/src/speech_command/scripts/tts_http.py \"" + text + "\" &";
        int ret = system(cmd.c_str());
        if (ret == 0) {
            ROS_INFO("Speaking: %s", text.c_str());
        } else {
            ROS_WARN("Failed to speak: %s", text.c_str());
            spoken = false;
        }
    }
    
    // ==================== 激光直走停车（子任务2） ====================
    void navigateToParking() {
        ROS_INFO("========================================");
        ROS_INFO("  Lidar-based parking (detect obstacle)");
        ROS_INFO("========================================");
        
        double FORWARD_SPEED = 0.08;
        double STOP_DISTANCE = 0.20;
        double MAX_TIME = 10.0;
        
        std_msgs::Bool enable_msg;
        enable_msg.data = false;
        vision_enable_pub.publish(enable_msg);
        ros::Duration(0.2).sleep();
        
        bool stopped = false;
        ros::Time start_time = ros::Time::now();
        ros::Rate rate(30);
        
        ROS_INFO("Moving forward, stop when object distance < %.2f m", STOP_DISTANCE);
        ROS_INFO("Current front distance: %.2f m", front_distance);
        
        while (ros::ok() && !emergency_stop) {
            double elapsed = (ros::Time::now() - start_time).toSec();
            if (elapsed > MAX_TIME) {
                ROS_WARN("Parking timeout (%.1f s), stopping.", elapsed);
                break;
            }
            
            if (front_distance < STOP_DISTANCE) {
                ROS_INFO("Object detected at %.2f m, stopping!", front_distance);
                stopped = true;
                break;
            }
            
            if (front_distance < 0.15) {
                ROS_WARN("Too close! Emergency stop.");
                break;
            }
            
            setSpeed(FORWARD_SPEED, 0.0);
            
            static int last_print = 0;
            int now_sec = (int)ros::Time::now().toSec();
            if (now_sec % 1 == 0 && now_sec != last_print) {
                ROS_INFO("Distance: %.2f m (target: %.2f)", front_distance, STOP_DISTANCE);
                last_print = now_sec;
            }
            
            ros::spinOnce();
            rate.sleep();
        }
        
        stop();
        ros::Duration(0.3).sleep();
        
        if (stopped) {
            mission_complete = true;
            std::string speech_text = "已将" + CARGO_NAME + "放入" + TARGET_WAREHOUSE;
            speak(speech_text);
            ROS_INFO("========================================");
            ROS_INFO("  🎉 MISSION COMPLETE!");
            ROS_INFO("  %s", speech_text.c_str());
            ROS_INFO("  Final distance: %.2f m", front_distance);
            ROS_INFO("========================================");
        } else {
            ROS_WARN("Parking timeout, stopping anyway.");
            mission_complete = true;
        }
    }
    
    // ==================== 子任务3停车 ====================
    void simPark() {
        ROS_INFO("========================================");
        ROS_INFO("  Sim Task: Lidar-based parking");
        ROS_INFO("========================================");
        
        double FORWARD_SPEED = 0.08;
        double STOP_DISTANCE = 0.30;
        double MAX_TIME = 25.0;
        
        std_msgs::Bool enable_msg;
        enable_msg.data = false;
        vision_enable_pub.publish(enable_msg);
        ros::Duration(0.2).sleep();
        
        bool stopped = false;
        ros::Time start_time = ros::Time::now();
        ros::Rate rate(30);
        
        ROS_INFO("Moving forward, stop when object distance < %.2f m", STOP_DISTANCE);
        ROS_INFO("Current front distance: %.2f m", front_distance);
        
        while (ros::ok() && !emergency_stop) {
            double elapsed = (ros::Time::now() - start_time).toSec();
            if (elapsed > MAX_TIME) {
                ROS_WARN("Sim parking timeout (%.1f s), stopping.", elapsed);
                break;
            }
            
            if (front_distance < STOP_DISTANCE) {
                ROS_INFO("Object detected at %.2f m, stopping!", front_distance);
                stopped = true;
                break;
            }
            
            if (front_distance < 0.15) {
                ROS_WARN("Too close! Emergency stop.");
                break;
            }
            
            setSpeed(FORWARD_SPEED, 0.0);
            
            static int last_print = 0;
            int now_sec = (int)ros::Time::now().toSec();
            if (now_sec % 1 == 0 && now_sec != last_print) {
                ROS_INFO("Distance: %.2f m (target: %.2f)", front_distance, STOP_DISTANCE);
                last_print = now_sec;
            }
            
            ros::spinOnce();
            rate.sleep();
        }
        
        stop();
        ros::Duration(0.3).sleep();
        
        if (stopped) {
            sim_task_complete = true;
            std::string speech_text = "仿真任务已完成，已将" + SIM_CARGO + "放入" + SIM_WAREHOUSE;
            std::string cmd = "python3 /home/ucar/ucar_ws/src/speech_command/scripts/tts_http.py \"" + speech_text + "\" &";
            system(cmd.c_str());
            ROS_INFO("========================================");
            ROS_INFO("  🎉 SIM TASK COMPLETE!");
            ROS_INFO("  %s", speech_text.c_str());
            ROS_INFO("  Final distance: %.2f m", front_distance);
            ROS_INFO("========================================");
        } else {
            ROS_WARN("Sim parking timeout, stopping anyway.");
            sim_task_complete = true;
        }
    }
    
    // ==================== 辅助函数 ====================
    void setSpeed(double linear, double angular) {
        geometry_msgs::Twist cmd;
        cmd.linear.x = std::max(-0.3, std::min(0.3, linear));
        cmd.angular.z = std::max(-0.8, std::min(0.8, angular));
        cmd_pub.publish(cmd);
    }
    
    void stop() {
        setSpeed(0.0, 0.0);
    }
    
    // ==================== 旋转到指定角度 ====================
    bool rotateToAngle(double target_yaw) {
        ROS_INFO("🔄 Rotating to target yaw: %.3f rad", target_yaw);
        double current_yaw = robot_yaw;
        double diff = target_yaw - current_yaw;
        while (diff > M_PI) diff -= 2 * M_PI;
        while (diff < -M_PI) diff += 2 * M_PI;
        
        double speed = 0.5;
        double eps = 0.05;
        ros::Rate rate(50);
        while (ros::ok() && !emergency_stop && std::abs(diff) > eps) {
            geometry_msgs::Twist twist;
            twist.linear.x = 0;
            twist.angular.z = (diff > 0) ? speed : -speed;
            cmd_pub.publish(twist);
            ros::spinOnce();
            rate.sleep();
            current_yaw = robot_yaw;
            diff = target_yaw - current_yaw;
            while (diff > M_PI) diff -= 2 * M_PI;
            while (diff < -M_PI) diff += 2 * M_PI;
        }
        stop();
        ROS_INFO("✅ Rotation complete, final yaw: %.3f rad", robot_yaw);
        return true;
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
        ROS_INFO("Sending goal to %s (%.2f, %.2f)", waypoints[idx].name.c_str(), waypoints[idx].x, waypoints[idx].y);
        move_base_client->sendGoal(goal);
        return true;
    }
    
    bool waitForGoalReached(double timeout = 60.0) {
        ros::Time start = ros::Time::now();
        while (ros::ok()) {
            if ((ros::Time::now() - start).toSec() > timeout) {
                ROS_WARN("Navigation timeout!");
                return false;
            }
            if (emergency_stop || !is_running) return false;
            actionlib::SimpleClientGoalState state = move_base_client->getState();
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Reached target!");
                return true;
            } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
                ROS_WARN("Navigation aborted!");
                return false;
            }
            static int last_print = 0;
            int now_sec = (int)ros::Time::now().toSec();
            if (now_sec % 5 == 0 && now_sec != last_print) {
                ROS_INFO("Navigating... (%.2f, %.2f)", robot_x, robot_y);
                last_print = now_sec;
            }
            ros::spinOnce();
            ros::Rate(10).sleep();
        }
        return false;
    }
    
    // ==================== 子任务2扫描 ====================
    void scanAtWaypoint() {
        ROS_INFO("========================================");
        ROS_INFO("Scanning at %s (one full rotation)", waypoints[current_wp].name.c_str());
        ROS_INFO("========================================");
        
        std_msgs::Bool enable_msg;
        enable_msg.data = true;
        vision_enable_pub.publish(enable_msg);
        ros::Duration(0.5).sleep();
        
        is_scanning = true;
        stop_requested = false;
        ocr_processed = false;
        target_found = false;
        bool peak_triggered = false;
        
        double rotation_duration = 2 * M_PI / rotate_speed;
        ROS_INFO("Rotation duration: %.2f seconds", rotation_duration);
        
        double effective_rotation_time = 0.0;
        ros::Time last_time = ros::Time::now();
        double max_wait = 20.0;
        
        while (ros::ok() && is_scanning && !mission_complete && !emergency_stop) {
            ros::Time current_time = ros::Time::now();
            double dt = (current_time - last_time).toSec();
            last_time = current_time;
            
            if (stop_requested && !target_found) {
                ROS_INFO("🛑 Peak detected! Waiting for OCR result...");
                peak_triggered = true;
                
                ros::Time wait_start = ros::Time::now();
                while (ros::ok() && !ocr_processed && (ros::Time::now() - wait_start).toSec() < max_wait) {
                    ros::spinOnce();
                    ros::Rate(20).sleep();
                    static int last_wait_print = 0;
                    int now_sec = (int)ros::Time::now().toSec();
                    if (now_sec % 1 == 0 && now_sec != last_wait_print) {
                        ROS_INFO("⏳ Waiting for OCR... (%.1f s)", (ros::Time::now() - wait_start).toSec());
                        last_wait_print = now_sec;
                    }
                }
                
                if (target_found) {
                    ROS_INFO("✅ Target matched! Entering parking...");
                    is_scanning = false;
                    navigateToParking();
                    mission_complete = true;
                    break;
                } else {
                    ROS_WARN("❌ OCR not matched or timeout. Continue rotating...");
                    stop_requested = false;
                    ocr_processed = false;
                    continue;
                }
            }
            
            if (!stop_requested) {
                effective_rotation_time += dt;
            }
            
            if (effective_rotation_time >= rotation_duration) {
                ROS_INFO("Completed one full rotation (%.2f s)", effective_rotation_time);
                break;
            }
            
            if (!stop_requested) {
                geometry_msgs::Twist twist;
                twist.angular.z = -rotate_speed;
                cmd_pub.publish(twist);
            }
            
            ros::spinOnce();
            ros::Rate(20).sleep();
        }
        
        is_scanning = false;
        stop_requested = false;
        enable_msg.data = false;
        vision_enable_pub.publish(enable_msg);
        move_base_client->cancelGoal();
        stop();
        
        if (target_found) {
            ROS_INFO("Target found at %s!", waypoints[current_wp].name.c_str());
        } else {
            ROS_INFO("Target not found at %s", waypoints[current_wp].name.c_str());
        }
    }
    
    // ==================== 子任务3扫描（备选） ====================
    void scanForSimWarehouse(int waypoint_idx) {
        ROS_INFO("========================================");
        ROS_INFO("Scanning for SIM warehouse at %s (one full rotation)", waypoints[waypoint_idx].name.c_str());
        ROS_INFO("========================================");

        if (!sendGoal(waypoint_idx)) {
            ROS_ERROR("Failed to send goal to waypoint %d", waypoint_idx);
            return;
        }
        bool reached = waitForGoalReached(60.0);
        if (!reached) {
            ROS_WARN("Failed to reach %s, abort sim scanning", waypoints[waypoint_idx].name.c_str());
            return;
        }

        std_msgs::Bool enable_msg;
        enable_msg.data = true;
        vision_enable_pub.publish(enable_msg);
        ros::Duration(0.5).sleep();

        sim_scanning_mode = true;
        is_scanning = true;
        stop_requested = false;
        target_found = false;
        ocr_processed = false;
        bool sim_found = false;

        double rotation_duration = 2 * M_PI / rotate_speed;
        ROS_INFO("Rotation duration: %.2f seconds", rotation_duration);

        double effective_rotation_time = 0.0;
        ros::Time last_time = ros::Time::now();
        double max_wait = 20.0;

        while (ros::ok() && is_scanning && !emergency_stop && !sim_task_complete) {
            ros::Time current_time = ros::Time::now();
            double dt = (current_time - last_time).toSec();
            last_time = current_time;

            if (stop_requested && !sim_found) {
                ROS_INFO("🛑 Peak detected! Waiting for OCR result...");
                
                ros::Time wait_start = ros::Time::now();
                while (ros::ok() && !ocr_processed && (ros::Time::now() - wait_start).toSec() < max_wait) {
                    ros::spinOnce();
                    ros::Rate(20).sleep();
                }
                if (detected_text == SIM_WAREHOUSE) {
                    sim_found = true;
                    ROS_INFO("✅ SIM warehouse matched! Parking...");
                    is_scanning = false;
                    enable_msg.data = false;
                    vision_enable_pub.publish(enable_msg);
                    simPark();
                    break;
                } else {
                    ROS_WARN("❌ Not SIM warehouse (got '%s'), continue rotating...", detected_text.c_str());
                    stop_requested = false;
                    ocr_processed = false;
                    continue;
                }
            }

            if (!stop_requested) {
                effective_rotation_time += dt;
            }

            if (effective_rotation_time >= rotation_duration) {
                ROS_INFO("Completed full rotation, no SIM warehouse found.");
                break;
            }

            if (!stop_requested) {
                geometry_msgs::Twist twist;
                twist.angular.z = -rotate_speed;
                cmd_pub.publish(twist);
            }

            ros::spinOnce();
            ros::Rate(20).sleep();
        }

        is_scanning = false;
        sim_scanning_mode = false;
        stop_requested = false;
        enable_msg.data = false;
        vision_enable_pub.publish(enable_msg);
        move_base_client->cancelGoal();
        stop();

        if (!sim_found) {
            ROS_WARN("SIM warehouse not found at waypoint %d", waypoint_idx);
        }
    }
    
    // ==================== 子任务3主逻辑 ====================
    void executeSimTask() {
        if (sim_task_complete || mission_complete == false) return;
        ROS_INFO("========================================");
        ROS_INFO("  Starting SIM Task (sub-task 3)");
        ROS_INFO("========================================");
        
        // ★ 如果之前记录了仿真车间的位置和朝向，直接转向停车
        if (sim_warehouse_found && sim_waypoint_index >= 0 && sim_waypoint_index < (int)waypoints.size()) {
            ROS_INFO("Sim warehouse recorded at waypoint %d with yaw %.3f rad.", 
                     sim_waypoint_index, sim_robot_yaw);
            ROS_INFO("Navigating to waypoint %d and rotate to recorded yaw.", sim_waypoint_index);
            
            if (!sendGoal(sim_waypoint_index)) {
                ROS_ERROR("Failed to send goal to waypoint %d", sim_waypoint_index);
                return;
            }
            if (!waitForGoalReached(60.0)) {
                ROS_WARN("Failed to reach waypoint %d, fallback to scanning.", sim_waypoint_index);
                scanForSimWarehouse(sim_waypoint_index);
                return;
            }
            
            rotateToAngle(sim_robot_yaw);
            simPark();
            
            if (sim_task_complete) {
                ROS_INFO("✅ SIM TASK COMPLETED SUCCESSFULLY (direct parking).");
                return;
            }
        }
        
        // 如果未记录（或上面失败），走原来的重新扫描逻辑
        ROS_INFO("No sim warehouse recorded or direct parking failed, scanning all waypoints...");
        bool found = false;
        for (int i = 0; i < (int)waypoints.size(); ++i) {
            if (sim_task_complete) break;
            ROS_INFO("Scanning waypoint %d for sim warehouse...", i);
            scanForSimWarehouse(i);
            if (sim_task_complete) {
                found = true;
                break;
            }
        }
        if (!found) {
            ROS_WARN("Sim warehouse not found in any waypoint. Sim task failed.");
        }
        
        if (sim_task_complete) {
            ROS_INFO("✅ SIM TASK COMPLETED SUCCESSFULLY.");
        } else {
            ROS_INFO("❌ SIM TASK FAILED.");
        }
    }
    
    // ==================== Main Execution ====================
    void executeMission() {
        ROS_INFO("Mission started. Target: %s", TARGET_WAREHOUSE.c_str());
        ROS_INFO("Cargo: %s", CARGO_NAME.c_str());
        
        for (size_t i = 0; i < waypoints.size(); ++i) {
            if (!is_running || emergency_stop || mission_complete || target_found) break;
            current_wp = i;
            
            ROS_INFO("Going to %s: (%.2f, %.2f)", waypoints[i].name.c_str(), waypoints[i].x, waypoints[i].y);
            if (!sendGoal(i)) {
                ROS_ERROR("Failed to send goal");
                break;
            }
            bool reached = waitForGoalReached(60.0);
            if (!reached) {
                ROS_WARN("Failed to reach %s, skipping", waypoints[i].name.c_str());
                continue;
            }
            ROS_INFO("Reached %s", waypoints[i].name.c_str());
            scanAtWaypoint();
        }
        
        ROS_INFO("========================================");
        if (mission_complete) {
            ROS_INFO("  MISSION COMPLETE!");
            ROS_INFO("  Parked at %s", TARGET_WAREHOUSE.c_str());
        } else if (target_found) {
            ROS_INFO("  Target found, heading to parking...");
        } else {
            ROS_INFO("  MISSION FAILED!");
            ROS_INFO("  Target %s not found.", TARGET_WAREHOUSE.c_str());
        }
        ROS_INFO("========================================");
        
        std_msgs::Bool msg;
        msg.data = false;
        vision_enable_pub.publish(msg);
        move_base_client->cancelGoal();
        
        if (mission_complete && !sim_task_complete) {
            executeSimTask();
        }
        
        ROS_INFO("Mission ended.");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "racecar_control");
    MissionScheduler scheduler;
    ros::spin();
    return 0;
}
