#pragma once

#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <queue>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int16.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

using namespace std;

class Relative_2_UTM : public rclcpp::Node {
    public:
        Relative_2_UTM();
        void calc_relative_2_UTM();

    private:
        void heading_cb(const std_msgs::msg::Float64::SharedPtr msg);
        void car_UTM_cb(const geometry_msgs::msg::PointStamped::SharedPtr msg);
        void narrow_object_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
        
        //calculation relative to UTM
        void narrow_cal_obj_UTM(float c_heading, double sensor_distanse[2], vector<vector<double>> object_cen, double utm_gps[2]);
        void narrow_color_cone();

        //publish funcion
        void narrow_obstacle_UTM_Pub(std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> narrow_obstacle_UTM);

        //pub
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr narrow_obstacle_UTM;

        //sub
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr heading_sub; //heading
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr car_utm_sub; //utm

        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr narrow_obstacle_sub;
 
        //class member variable
        double inter_sensor_distanse[2] {0.35, 0}; //차량 내 센서 위치 고려
        vector<vector<double>> obj_cen; //물체 중심 상대좌표 저장
        vector<vector<double>> obj_UTM; //물체 중심 절대좌표 저장
        double c_UTM[2] {}; // car_UTM [x, y]
        float c_heading = 0.0; // car_heading(radian)
        bool check_cb_sub[2] {}; // 차량 UTM,heading이 콜백되고 있는지 확인
        double tmp_distance = 0.0;
        std::chrono::milliseconds loop_duration; // 1초
};