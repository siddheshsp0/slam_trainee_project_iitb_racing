/**
 * @file localization_node_ekf.cpp
 * @brief This node performs localization, given given map, using EKF algorithm
 * @author Siddhesh Phadke
*/

#include <iostream>
#include <functional>
#include <string>
#include <cmath> // for M_PI, sin, cos

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "eufs_msgs/msg/car_state.hpp"
#include "eufs_msgs/msg/wheel_speeds_stamped.hpp"
#include "eufs_msgs/msg/cone_array_with_covariance.hpp"
#include "dv_msgs/msg/indexed_track.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "slam_project/data_assoc_trainee.hpp"
#include "slam_project/utils.hpp"



/**
 * @brief Node Class
 */
class LocalizationNodeEKF : public rclcpp::Node {
    // Class variables declaration:

    // Topics:
        // From rosbag or eufs sim
    const std::string ground_truth_cones_topic = "/ground_truth/cones"; // ground truth of cones in frame of car
    const std::string ground_truth_state_topic = "/ground_truth/state"; // ground truth position of the car (Red marker)
    const std::string imu_topic = "/imu"; // imu data for yaw rate
    const std::string perception_cones_topic = "/perception/cones"; // Cones detected by perception
    const std::string ros_can_wheel_speeds_topic = "/ros_can/wheel_speeds"; // wheel speed data for Vx estimation

        // To publish data over
    const std::string map_cones_topic="/slam_project/ground_truth/cones"; // Publishes Cones (world frame)
    const std::string map_car_topic="/slam_project/ground_truth/car"; // Publishes actual position of car
    const std::string estimated_car_topic="/slam_project/estimation/car"; // Publishes estimated position of car using ekf (Green arrow)

    // EKF Variables:
    dv_msgs::msg::IndexedTrack::SharedPtr perception_record;
    sensor_msgs::msg::Imu::SharedPtr imu_record;
    bool perception_available=false;

    Eigen::Matrix<double, 3, 1> mu_t_1, mu_t, mu_t_bar, u_t, mu_t_no_ekf;
    Eigen::Matrix<double, 3, 3> sigma_t_1, sigma_t, sigma_t_bar, G_t, Q_t_l, V_t, Q_t;
    Eigen::Matrix<double, 2, 2> R_t;

    // Other variables
    double sec_history=0.0, nanosec_history=0.0;
    const double i_gear=60.0, r_wheel=0.25;

    bool start=true; // Checks if its the first iteration





    // Publishers and Subscribers and timers
        // rosbag/eufs sim topics subscribers
    rclcpp::Subscription<eufs_msgs::msg::CarState>::SharedPtr ground_truth_state_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<dv_msgs::msg::IndexedTrack>::SharedPtr perception_sub;
    rclcpp::Subscription<eufs_msgs::msg::WheelSpeedsStamped>::SharedPtr ros_can_wheel_speeds_sub;

        // My publishers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_cones_pub; // Publishes Cones (world frame)
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr map_car_pub; // Publishes actual position of car
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr estimated_car_pub; // Publishes estimated position of car using ekf

        // Timers
    rclcpp::TimerBase::SharedPtr cones_pub_timer;





public:
    LocalizationNodeEKF() : Node("localization_node_ekf"){
        // Init vars
        this->Q_t_l<<0.78, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0 ,0.0, 0.00005; // Tweakable
        this->R_t<<25e-5, 0.0, 0.0, 25e-5; // Tweakable
        this->sigma_t_1<<1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6; // Very small cause robot almost certainly knows its initial position
        this->mu_t_1<<0.0, 0.0, 0.0;
        this->mu_t_no_ekf<<0.0, 0.0, 0.0;



        // Init subs and pubs
        
        this->map_cones_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            this->map_cones_topic,
            10);
        this->map_car_pub = this->create_publisher<visualization_msgs::msg::Marker>(this->map_car_topic,10);
        this->estimated_car_pub = this->create_publisher<visualization_msgs::msg::Marker>(this->estimated_car_topic,10);


        this->ground_truth_state_sub = this->create_subscription<eufs_msgs::msg::CarState>(this->ground_truth_state_topic, 10, std::bind(&LocalizationNodeEKF::ground_truth_state_sub_callback, this, std::placeholders::_1));
        this->imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(this->imu_topic, 10, std::bind(&LocalizationNodeEKF::imu_sub_callback, this, std::placeholders::_1));
        this->perception_sub = this->create_subscription<dv_msgs::msg::IndexedTrack>(this->perception_cones_topic, 10, std::bind(&LocalizationNodeEKF::perception_sub_callback, this, std::placeholders::_1));  
        this->ros_can_wheel_speeds_sub = this->create_subscription<eufs_msgs::msg::WheelSpeedsStamped>(this->ros_can_wheel_speeds_topic, 10, std::bind(&LocalizationNodeEKF::ros_can_wheel_speeds_sub_callback, this, std::placeholders::_1));  

        this->cones_pub_timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&LocalizationNodeEKF::cones_pub_timer_callback, this));

    }




private:
// FUNCTION DEFINITIONS
    /**
     * @brief Publishes an arrow marker
     * @param id Marker id
     * @param pos double array [x,y,z] of position of the marker
     * @param color double array [r,g,b,a] color of marker
     * @param z
     * @param w
     * @param publisher Publisher object SharedPtr
     */
    void publishArrow(int id, std::vector<double> pos, std::vector<double> color, double x, double y, double z, double w,rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher){

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map"; // or "base_link" or any TF frame you have
        marker.header.stamp = this->now();
        marker.ns = "arrow";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        // Set position and orientation
        marker.pose.position.x = pos[0];
        marker.pose.position.y = pos[1];
        marker.pose.position.z = pos[2];
        marker.pose.orientation.x=x;
        marker.pose.orientation.y=y;
        marker.pose.orientation.z=z;
        marker.pose.orientation.w=w;
        // Scale of the arrow (shaft length, shaft diameter, head diameter)
        marker.scale.x = 1.0;  // shaft length
        marker.scale.y = 0.1;  // shaft diameter
        marker.scale.z = 0.1;  // head diameter
        // Color (RGBA)
        marker.color.r = color[0];
        marker.color.g = color[1];
        marker.color.b = color[2];
        marker.color.a = color[3];
        marker.lifetime = rclcpp::Duration(0, 0); // 0 means marker stays forever
        
        publisher->publish(marker);
    }
    
    /**
     * @brief Publishes marker array
     */
    void publishMarkerArray(std::vector<std::vector<double>> positions_colours,rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher, int del_markers=0, std::vector<double> scales={0.25,0.25,0.5}){
        visualization_msgs::msg::MarkerArray marker_array;
        if(del_markers){
            visualization_msgs::msg::Marker del_marker;
            del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
            marker_array.markers.push_back(del_marker);
        }
        int i;
        for (i=0; i < positions_colours.size(); i++){
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->now();
            marker.ns = "cylinders";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = positions_colours.at(i).at(0);
            marker.pose.position.y = positions_colours.at(i).at(1);
            marker.pose.position.z = positions_colours.at(i).at(2);
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = scales.at(0); // diameter x
            marker.scale.y = scales.at(1); // diameter y
            marker.scale.z = scales.at(2); // radius

            marker.color.a = 1.0;  // Don't forget to set alpha!
            marker.color.r = positions_colours.at(i).at(3);
            marker.color.g = positions_colours.at(i).at(4);
            marker.color.b = positions_colours.at(i).at(5);

            marker_array.markers.push_back(marker);
        }
        publisher->publish(marker_array);
        // LOG_INFO("DONE")

    }



// CALLBACK DEFINITIONS
        // Callbacks for ground truth pubs
    /**
     * @brief cones publisher callback
     */
    void cones_pub_timer_callback(){
        std::vector<std::vector<double>> points(0, std::vector<double>(6, 0.0));
        for(auto landmark:known_landmarks){
            std::vector<double> point = {landmark.x(), landmark.y(), landmark.z(), 0.0, 1.0, 0.0};
            points.push_back(point);
        }
        this->publishMarkerArray(points, this->map_cones_pub, 0);
        // LOG_INFO("Published marker")
    }
    /**
     * @brief Publishes ground truth of the car
     */
    void ground_truth_state_sub_callback(eufs_msgs::msg::CarState::SharedPtr msg){
        if(this->start) {
            this->start=false;
            this->mu_t_1<<msg->pose.pose.position.x,msg->pose.pose.position.y,2 * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        }
        this->publishArrow(0,{msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z}, {1.0, 0.0, 0.0, 1.0}, 0.0, 0.0, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w ,this->map_car_pub);
    };

    /**
     * @brief to and stores imu data
     */
    void imu_sub_callback(const sensor_msgs::msg::Imu::SharedPtr msg){
        this->imu_record=msg;
    }
    /**
     * @brief to and stores imu data
     */
    void perception_sub_callback(const dv_msgs::msg::IndexedTrack::SharedPtr msg){
        this->perception_record = msg;
        this->perception_available=true;
    }
    
    /**
     * @brief wheel speed call back. Implementation of EKF
     */
    void ros_can_wheel_speeds_sub_callback(const eufs_msgs::msg::WheelSpeedsStamped::SharedPtr msg){
        double d_t;
        if(this->sec_history==0.0 && this->nanosec_history==0.0){d_t=0.0;}
        else{
            d_t= msg->header.stamp.sec + (msg->header.stamp.nanosec)/pow(10, 9) - (this->sec_history + (this->nanosec_history)/pow(10, 9));
        }
        // LOG_INFO(std::to_string(d_t).c_str())
        this->sec_history = msg->header.stamp.sec;
        this->nanosec_history = msg->header.stamp.nanosec;


        double vx = (M_PI * this->r_wheel * (msg->speeds.lb_speed + msg->speeds.rb_speed))/this->i_gear;
        // double Phi = this->mu_t_1(2,0); // Changed the source of input for Phi
        double Phi = 2 * atan2(this->imu_record->orientation.z, this->imu_record->orientation.w);
        double Phi_dot = this->imu_record->angular_velocity.z;

        // Prediction
        this->G_t<<1, 0.0, -vx*sin(Phi)*d_t, 0.0, 1, vx*cos(Phi)*d_t, 0.0, 0.0, d_t;
        this->V_t<<cos(Phi)*d_t,(-sin(Phi)*d_t),0.0,sin(Phi)*d_t, cos(Phi)*d_t, 0.0, 0.0, 0.0, 1;
        this->Q_t = this->V_t*this->Q_t_l*this->V_t.transpose();
        Eigen::Matrix<double, 3, 1> motion_update;
        motion_update<<vx*cos(Phi)*d_t, vx*sin(Phi)*d_t, Phi_dot*d_t; 
        this->mu_t_bar = this->mu_t_1 + motion_update;
        this->mu_t_bar(2,0) = Utils::normalizeAngle(this->mu_t_bar(2,0)); // Normalizing Angle
        this->sigma_t_bar = this->G_t*this->sigma_t_1*this->G_t.transpose() + this->Q_t;

        // Update

        if(this->perception_available){
        // if(false){
            std::vector<int> data_assoc = performDataAssociation({{this->mu_t_bar(0,0)}, {this->mu_t_bar(1,0)}, {this->mu_t_bar(2,0)}}, this->perception_record->track, this->R_t);
            this->perception_available = false;
            for (int i = 0; i < data_assoc.size(); i++)
            {
                if(data_assoc.at(i)==-1){continue;}
                Eigen::Matrix<double, 2, 1> z_t_j;
                z_t_j<<this->perception_record->track.at(i).location.x,this->perception_record->track.at(i).location.y;
                
                Eigen::Matrix<double, 2, 1> delta;
                delta<<(known_landmarks.at(i).x()-this->mu_t_bar(0,0)),(known_landmarks.at(i).y()-this->mu_t_bar(1,0));
                double q = delta.transpose() * delta;
                Eigen::Matrix<double, 2, 1> z_t_j_hat;
                z_t_j_hat<<sqrt(q),atan2(delta(1,0), delta(0,0));

                Eigen::Matrix<double, 2, 3> H_t;
                H_t<<(-1*delta(0,0))*sqrt(q),(-1*delta(1,0))*sqrt(q), 0.0, delta(1,0), -1*delta(0,0), -q;
                
                Eigen::Matrix<double, 3, 2> K_t; // K_t's dimensions are (dim of state vector)x(dim of measurement vector)
                K_t=this->sigma_t_bar * H_t.transpose() * (H_t * this->sigma_t_bar* H_t.transpose() + this->R_t).inverse();

                this->mu_t_bar = this->mu_t_bar + K_t * (z_t_j - z_t_j_hat);
                this->mu_t_bar(2,0) = Utils::normalizeAngle(this->mu_t_bar(2,0)); // Normalizing Angle
                this->sigma_t_bar = (Eigen::MatrixXd::Identity(3,3) - K_t*H_t)*this->sigma_t_bar;
            }
        }
        this->mu_t = this->mu_t_bar;
        this->sigma_t = this->sigma_t_bar;

        
        // Publishing
        auto q = Utils::getQuaternion(0.0, 0.0, Phi);
        this->publishArrow(1, {this->mu_t(0,0), this->mu_t(1,0), this->mu_t(2,0)}, {0.0, 1.0, 0.0, 1.0}, 0.0, 0.0, q.z(), q.w(), this->estimated_car_pub);
        // Return step: Updating for next iteration
        this->mu_t_1 = this->mu_t;
        this->sigma_t_1 = this->sigma_t;
        

    }
    
};






// Spin the Localization node
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationNodeEKF>());
    rclcpp::shutdown();
    return 0;
}