/**
 * @file localization_node.cpp
 * @brief This node will handle the localization part, Checkpoint 1 and 3
 * @author Siddhesh Phadke
 */


#include <iostream>
#include <functional>
#include <string>
// #include <cmath> // for M_PI, sin, cos
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "eufs_msgs/msg/car_state.hpp"
#include "eufs_msgs/msg/wheel_speeds_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "slam_project/utils.hpp"






/**
 * @class LocalizationNode class
 * @brief Inherited from rclcpp/rclcpp.hpp/Node class
 */
class LocalizationNode : public rclcpp::Node {
    std::string rviz_marker_calculated_state_topic = "/slam_model/calculated_topic/state";
    std::string rviz_marker_ground_truth_state_topic = "/slam_model/ground_truth/state"; // topic name for rviz arrow publishing
    std::string rviz_marker_ground_truth_cones_topic = "/slam_model/ground_truth/cones";
    std::string imu_topic = "/imu"; // topic name for imu data
    std::string ground_truth_state_topic = "/ground_truth/state"; // 
    std::string ros_can_wheel_speeds_topic = "/ros_can/wheel_speeds"; //
    std::string ground_truth_cones_topic = "/ground_truth/cones";


    cublasHandle_t handle;
    std::vector<std::vector<double>> state_history;
    std::vector<std::vector<double>> landmarks_state;
    sensor_msgs::msg::Imu::SharedPtr imu_current_reading;
    double i_gear, r_wheel, sec_history, nanosec_history;

    
    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber; // imu subscriber
    rclcpp::Subscription<eufs_msgs::msg::CarState>::SharedPtr ground_truth_state_subscriber; // ground truth state subscriber
    rclcpp::Subscription<eufs_msgs::msg::WheelSpeedsStamped>::SharedPtr wheel_speeds_subscriber; // ros_can wheel speeds subscriber
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_ground_truth_state_marker_pub; // ground truth publisher
    visualization_msgs::msg::Marker rviz_ground_truth_state_arrow_marker = visualization_msgs::msg::Marker(); // ground truth arrow marker
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_calculated_state_marker_pub; // ground truth publisher
    visualization_msgs::msg::Marker rviz_calculated_state_arrow_marker = visualization_msgs::msg::Marker(); // ground truth arrow marker

public:
    /**
     * @details Constructor of LocalizationNode class. Calls parent's (Node) constructor, passes
     * name of the node as arguement
     * @param None
     * @returns None
     */
    LocalizationNode() : Node("localization_node") {
        // Initialize handle
        cublasCreate(&(this->handle));

        // Initialize variables
        this->state_history = std::vector<std::vector<double>>(3, std::vector<double>(1, 0.0));
        this->i_gear = 60;
        this->r_wheel=0.25;
        this->sec_history=0.0;
        this->nanosec_history=0.0;

        /* Initializing subscribers, publishers and timers */

        // Subscribing to data (imu, ground truth, wheel speed)
        this->imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
            this->imu_topic, 10, std::bind(&LocalizationNode::imu_callback, this, std::placeholders::_1)
        );
        this->ground_truth_state_subscriber = this->create_subscription<eufs_msgs::msg::CarState>(
            this->ground_truth_state_topic, 10, std::bind(&LocalizationNode::ground_truth_state_callback, this, std::placeholders::_1)
        );
        this->wheel_speeds_subscriber = this->create_subscription<eufs_msgs::msg::WheelSpeedsStamped>(
            this->ros_can_wheel_speeds_topic, 10, std::bind(&LocalizationNode::ros_can_wheel_speeds_callback, this, std::placeholders::_1)
        );


        // Ground truth and calculated state publisher
        this->rviz_ground_truth_state_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>(
            this->rviz_marker_ground_truth_state_topic, 10
        );
        this->rviz_calculated_state_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>(
            this->rviz_marker_calculated_state_topic, 10
        );
        

        /* Initializing Ground Truth Arrow Marker*/
        {
        this->rviz_ground_truth_state_arrow_marker.header.frame_id = "map";
        this->rviz_ground_truth_state_arrow_marker.header.stamp = this->now();
        
        this->rviz_ground_truth_state_arrow_marker.ns = "arrow";
        this->rviz_ground_truth_state_arrow_marker.id = 1;
        this->rviz_ground_truth_state_arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
        this->rviz_ground_truth_state_arrow_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Set position (start of arrow)
        this->rviz_ground_truth_state_arrow_marker.pose.position.x = 0.0;
        this->rviz_ground_truth_state_arrow_marker.pose.position.y = 0.0;
        this->rviz_ground_truth_state_arrow_marker.pose.position.z = 0.0;

        // Ser Orientaion
        auto q = Utils::getQuaternion(0.0, 0.0, 0.0);
        this->rviz_ground_truth_state_arrow_marker.pose.orientation.x = q.x();
        this->rviz_ground_truth_state_arrow_marker.pose.orientation.y = q.y();
        this->rviz_ground_truth_state_arrow_marker.pose.orientation.z = q.z();
        this->rviz_ground_truth_state_arrow_marker.pose.orientation.w = q.w();

        // Set arrow scale
        this->rviz_ground_truth_state_arrow_marker.scale.x = 1.0;  // Arrow length
        this->rviz_ground_truth_state_arrow_marker.scale.y = 0.2;  // Shaft width
        this->rviz_ground_truth_state_arrow_marker.scale.z = 0.2;  // Head width

        // Set color (RGBA)
        this->rviz_ground_truth_state_arrow_marker.color.r = 0.0;  // Blue
        this->rviz_ground_truth_state_arrow_marker.color.g = 0.0;
        this->rviz_ground_truth_state_arrow_marker.color.b = 1.0;
        this->rviz_ground_truth_state_arrow_marker.color.a = 1.0;  // Fully visible

        this->rviz_ground_truth_state_arrow_marker.lifetime = rclcpp::Duration::from_seconds(0);  // Infinite lifetime

        LOG_INFO("Initialisd an ground truth state arrow marker. frame_id='map'")
        }
        /* Initializing Ground Truth Arrow Marker*/
        {
            this->rviz_calculated_state_arrow_marker.header.frame_id = "map";
            this->rviz_calculated_state_arrow_marker.header.stamp = this->now();
            
            this->rviz_calculated_state_arrow_marker.ns = "arrow";
            this->rviz_calculated_state_arrow_marker.id = 1;
            this->rviz_calculated_state_arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
            this->rviz_calculated_state_arrow_marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Set position (start of arrow)
            this->rviz_calculated_state_arrow_marker.pose.position.x = 0.0;
            this->rviz_calculated_state_arrow_marker.pose.position.y = 0.0;
            this->rviz_calculated_state_arrow_marker.pose.position.z = 0.0;
    
            // Ser Orientaion
            auto q = Utils::getQuaternion(0.0, 0.0, 0.0);
            this->rviz_calculated_state_arrow_marker.pose.orientation.x = q.x();
            this->rviz_calculated_state_arrow_marker.pose.orientation.y = q.y();
            this->rviz_calculated_state_arrow_marker.pose.orientation.z = q.z();
            this->rviz_calculated_state_arrow_marker.pose.orientation.w = q.w();
    
            // Set arrow scale
            this->rviz_calculated_state_arrow_marker.scale.x = 1.0;  // Arrow length
            this->rviz_calculated_state_arrow_marker.scale.y = 0.2;  // Shaft width
            this->rviz_calculated_state_arrow_marker.scale.z = 0.2;  // Head width
    
            // Set color (RGBA)
            this->rviz_calculated_state_arrow_marker.color.r = 1.0;  // Blue
            this->rviz_calculated_state_arrow_marker.color.g = 0.0;
            this->rviz_calculated_state_arrow_marker.color.b = 0.0;
            this->rviz_calculated_state_arrow_marker.color.a = 1.0;  // Fully visible
    
            this->rviz_calculated_state_arrow_marker.lifetime = rclcpp::Duration::from_seconds(0);  // Infinite lifetime
    
            LOG_INFO("Initialisd an calculated state arrow marker. frame_id='map'")
            }
    }

    ~LocalizationNode(){
        LOG_INFO("Shutting localization_node down, destroying the cublas handle")
        cublasDestroy(this->handle);

    }
    


private:
    /**
     * @brief Callback function of subscriber of IMU Data
     * @details 1. Updates Car's position in car's frame, and publishes it as an rviz arrow over rviz_marker_topic
     * @param msg of type const sensor_msgs::msg::Imu::SharedPtr msg
     * @returns void
     */
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg){
        this->imu_current_reading = msg;
    }


    /**
     * @brief Callback function of subscriber of ground_truth_state data
     * @param msg of type const eufs_msgs::msg::CarState::SharedPtr
     * @returns void
     */
    void ground_truth_state_callback(const eufs_msgs::msg::CarState::SharedPtr msg){
        // Publishing car's orientation in car's frame
        {

            this->rviz_ground_truth_state_arrow_marker.pose.orientation.x = msg->pose.pose.orientation.x;
            this->rviz_ground_truth_state_arrow_marker.pose.orientation.y = msg->pose.pose.orientation.y;
            this->rviz_ground_truth_state_arrow_marker.pose.orientation.z = msg->pose.pose.orientation.z;
            this->rviz_ground_truth_state_arrow_marker.pose.orientation.w = msg->pose.pose.orientation.w;
            this->rviz_ground_truth_state_arrow_marker.pose.position.x = msg->pose.pose.position.x;
            this->rviz_ground_truth_state_arrow_marker.pose.position.y = msg->pose.pose.position.y;
            this->rviz_ground_truth_state_arrow_marker.pose.position.z = msg->pose.pose.position.z;
            this->rviz_ground_truth_state_marker_pub->publish(this->rviz_ground_truth_state_arrow_marker);
        }
    }

    /**
     * @brief Callback function of subscriber of ros_can_wheels_speeds Data. Calculates estimated current state of the bot without error estimation
     * @param msg of type const eufs_msgs::msg::WheelSpeedsStamped
     * @returns void
     */
    void ros_can_wheel_speeds_callback(const eufs_msgs::msg::WheelSpeedsStamped::SharedPtr msg){

        // Performing motion update for checkpoint1
        double d_t;
        if(this->sec_history==0.0 || this->nanosec_history==0.0){d_t=0.0;}
        else{
            d_t= msg->header.stamp.sec-this->sec_history + (msg->header.stamp.nanosec-this->nanosec_history)/pow(10, 9);
        }
        this->sec_history = msg->header.stamp.sec;
        this->nanosec_history = msg->header.stamp.nanosec;
        
        double vx = (M_PI * this->r_wheel * (msg->speeds.lb_speed + msg->speeds.rb_speed))/this->i_gear;
        double yaw = this->imu_current_reading->orientation.z;
        double yaw_rate = this->imu_current_reading->angular_velocity.z;
        std::vector<std::vector<double>> velocity_transorm_dt = {{cos(yaw) * vx * d_t},
                                                                {sin(yaw) * vx * d_t},
                                                                {yaw_rate * d_t}};
       

        // Calculating new state
        std::vector<std::vector<double>> new_state = Utils::matrixAdd2D(&(this->state_history), &(velocity_transorm_dt), &(this->handle));

        
        this->state_history = new_state; // Update state_history with the new state

        tf2::Quaternion q = Utils::getQuaternion(0.0, 0.0, new_state[2][0]);
        this->rviz_calculated_state_arrow_marker.pose.orientation.x = q.getX();
        this->rviz_calculated_state_arrow_marker.pose.orientation.y = q.getY();
        this->rviz_calculated_state_arrow_marker.pose.orientation.z = q.getZ();
        this->rviz_calculated_state_arrow_marker.pose.orientation.w = q.getW();

        this->rviz_calculated_state_arrow_marker.pose.position.x = new_state[0][0];
        this->rviz_calculated_state_arrow_marker.pose.position.y = new_state[1][0];
        this->rviz_calculated_state_marker_pub->publish(this->rviz_calculated_state_arrow_marker);
    }
    
};







// Spin the Localization node
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationNode>());
    rclcpp::shutdown();
    return 0;
}