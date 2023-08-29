#include "relative_2_utm/relative_2_UTM.hpp"

std::vector<double> calculateVectorSubtraction(const std::vector<double>& v1, const std::vector<double>& v2) {
    std::vector<double> result(2);
    result[0] = v1[0] - v2[0];
    result[1] = v1[1] - v2[1];
    return result;
}

double calculateDistance(const std::vector<double>& v1, const std::vector<double>& v2) {
    std::vector<double> diff = calculateVectorSubtraction(v1, v2);
    return sqrt(diff[0] * diff[0] + diff[1] * diff[1]);
}

Relative_2_UTM::Relative_2_UTM() : Node("relative_2_utm") {
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();

    //sub
    heading_sub = this->create_subscription<std_msgs::msg::Float64>("/Local/heading", qos_profile, std::bind(&Relative_2_UTM::heading_cb,this,std::placeholders::_1)); //현재 차량 헤딩(라디안)
    car_utm_sub = this->create_subscription<geometry_msgs::msg::PointStamped>("/Local/utm", qos_profile, std::bind(&Relative_2_UTM::car_UTM_cb,this,std::placeholders::_1)); //현재 차량 UTM
    narrow_obstacle_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("/LiDAR/center_color", qos_profile, std::bind(&Relative_2_UTM::narrow_object_cb,this,std::placeholders::_1)); //장애물 상대좌표
    big_obstacle_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("/LiDAR/bigcone_point",qos_profile,std::bind(&Relative_2_UTM::big_object_cb,this,std::placeholders::_1));
    mission_sub = this->create_subscription<std_msgs::msg::Int16>("/Planning/mission",qos_profile,std::bind(&Relative_2_UTM::mission_cb,this,std::placeholders::_1));
    obq_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("/Vision/parking_points",qos_profile,std::bind(&Relative_2_UTM::obq_cb,this,std::placeholders::_1));
    prl_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("/LiDAR/bigcone_point",qos_profile,std::bind(&Relative_2_UTM::prl_cb,this,std::placeholders::_1));
    small_obstacle_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>("/LiDAR/object_cen",qos_profile,std::bind(&Relative_2_UTM::small_object_cb,this,std::placeholders::_1));

    //pub
    narrow_obstacle_UTM = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Planning/narrow_object_UTM", qos_profile); //장애물 절대좌표
    big_obstacle_UTM = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Planning/big_object_UTM", qos_profile); //장애물 절대좌표
    obq_obstacle_UTM = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Planning/obq_points", qos_profile); //장애물 절대좌표
    prl_point_UTM = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Planning/prl_points", qos_profile); //주차위치 절대좌표
    small_obstacle_UTM = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Planning/small_object_UTM", qos_profile); //장애물 절대좌표

    fill_n(mission_state,30,false);
    loop_duration = std::chrono::milliseconds(20);
    
}

void Relative_2_UTM::calc_relative_2_UTM() {
    while(rclcpp::ok()) {

        rclcpp::spin_some(shared_from_this());
        cout << "노드 실행" << endl;
        std::this_thread::sleep_for(loop_duration);
        // cout << check_cb_sub[0] << check_cb_sub[1] << endl;
        if(!check_cb_sub[0] || !check_cb_sub[1]) {continue;} //waiting for utm and heading
        if(mission == 0) {continue;} //waiting for mission number
        if(!mission_state[mission]) {continue;} //waiting for mission points
        switch(mission) {
            case 1:
                cout << "narrow" <<endl; //협로
                narrow_cal_obj_UTM(c_heading, inter_sensor_distanse, obj_cen, c_UTM);
                narrow_obstacle_UTM_Pub(narrow_obstacle_UTM);
                fill_n(check_cb_sub, 2, false);
                break;
            case 6:
                cout << "obq_points" <<endl;
                obq_cal_obj_UTM(c_heading, inter_sensor_distanse, obj_cen, c_UTM);
                obq_point_UTM_Pub(obq_obstacle_UTM); 
                fill_n(check_cb_sub, 2, false);
                break;
            case 11:
                cout << "small_obj" <<endl; //정적소형
                small_cal_obj_UTM(c_heading, inter_sensor_distanse, obj_cen, c_UTM);
                small_obstacle_UTM_Pub(small_obstacle_UTM); 
                fill_n(check_cb_sub, 2, false);
                break;
            case 12:
                cout << "big_obj" <<endl; //정적대형
                big_cal_obj_UTM(c_heading, inter_sensor_distanse, obj_cen, c_UTM);
                big_obstacle_UTM_Pub(big_obstacle_UTM); 
                fill_n(check_cb_sub, 2, false);
            case 14:
                cout << "prl_points" <<endl; //평행주차
                prl_cal_obj_UTM(c_heading, inter_sensor_distanse, obj_cen, c_UTM);
                prl_parking_UTM_Pub(prl_point_UTM); 
                fill_n(check_cb_sub, 2, false);
            default:
                cout << "no mission" << endl;
        }
        fill_n(mission_state,30,false);

    }

    printf("finish\n");
    return;
}

void Relative_2_UTM::narrow_obstacle_UTM_Pub(std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> narrow_obstacle_UTM) {
    std_msgs::msg::Float64MultiArray array;
    array.data.clear();

    for(int i=0; i< obj_UTM.size(); i++) {
        array.data.push_back(obj_UTM[i][0]);
        array.data.push_back(obj_UTM[i][1]);
        array.data.push_back(obj_UTM[i][2]);
    }

    narrow_obstacle_UTM->publish(array);

    obj_UTM.clear();
}

void Relative_2_UTM::big_obstacle_UTM_Pub(std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> big_obstacle_UTM) {
    std_msgs::msg::Float64MultiArray array;
    array.data.clear();

    for(int i=0; i< obj_UTM.size(); i++) {
        array.data.push_back(obj_UTM[i][0]);
        array.data.push_back(obj_UTM[i][1]);
    }

    big_obstacle_UTM->publish(array);

    obj_UTM.clear();
}

void Relative_2_UTM::obq_point_UTM_Pub(std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> obq_obstacle_UTM) {
    std_msgs::msg::Float64MultiArray array;
    array.data.clear();

    for(int i=0; i< obj_UTM.size(); i++) {
        array.data.push_back(obj_UTM[i][0]);
        array.data.push_back(obj_UTM[i][1]);
    }

    obq_obstacle_UTM->publish(array);

    obj_UTM.clear();
}

void Relative_2_UTM::prl_parking_UTM_Pub(std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> prl_point_UTM) {
    std_msgs::msg::Float64MultiArray array;
    array.data.clear();

    for(int i=0; i< obj_UTM.size(); i++) {
        array.data.push_back(obj_UTM[i][0]);
        array.data.push_back(obj_UTM[i][1]);
    }

    prl_point_UTM->publish(array);

    obj_UTM.clear();
}

void Relative_2_UTM::small_obstacle_UTM_Pub(std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> small_obstacle_UTM) {
    std_msgs::msg::Float64MultiArray array;
    array.data.clear();

    for(int i=0; i< obj_UTM.size(); i++) {
        array.data.push_back(obj_UTM[i][0]);
        array.data.push_back(obj_UTM[i][1]);
    }

    small_obstacle_UTM->publish(array);

    obj_UTM.clear();
}
void Relative_2_UTM::heading_cb(const std_msgs::msg::Float64::SharedPtr msg) {
    c_heading = msg->data;
    // cout << "heading come" << endl;
    check_cb_sub[0] = true;
}

void Relative_2_UTM::car_UTM_cb(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    c_UTM[0] = msg->point.x;
    c_UTM[1] = msg->point.y;
    // cout << "UTM come" << endl;
    check_cb_sub[1] = true;
}

void Relative_2_UTM::narrow_object_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    int cluster_count = 0;
    obj_cen.clear();
    vector<double> tmp_obj_cen;
    
    if (msg->data.size() == 0) {return;}

    for(int i = 0; i < msg->data.size(); i++) {
        if(i%3 != 2) {   // x,y
            tmp_obj_cen.push_back(msg->data[i]);
        }
        else {// color : 2, 5, 8, 11 ---
            tmp_obj_cen.push_back(msg->data[i]);
            obj_cen.push_back(tmp_obj_cen);
            tmp_obj_cen.clear();
        }
    }
    mission_state[1] = true;
}

void Relative_2_UTM::big_object_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    int cluster_count = 0;
    obj_cen.clear();
    vector<double> tmp_obj_cen;
    
    for(int i = 0; i < msg->data.size(); i++) {
        if(i%2 == 0) {  // x

            tmp_obj_cen.push_back(msg->data[i]);
        }
        else {   // y
            tmp_obj_cen.push_back(msg->data[i]);
            obj_cen.push_back(tmp_obj_cen);
            tmp_obj_cen.clear();
        }
    }
    if(msg->data[0] == 777.0){
        mission_state[12] = false;
    }
    else{
        mission_state[12] = true;
    }
    
}

void Relative_2_UTM::obq_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    int cluster_count = 0;
    obj_cen.clear();
    vector<double> tmp_obj_cen;
    
    for(int i = 0; i < msg->data.size(); i++) {
        if(i%2 == 0) {  // x

            tmp_obj_cen.push_back(msg->data[i]);
        }
        else {   // y
            tmp_obj_cen.push_back(msg->data[i]);
            obj_cen.push_back(tmp_obj_cen);
            tmp_obj_cen.clear();
        }
    }
    mission_state[6] = true;
}

void Relative_2_UTM::prl_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    int cluster_count = 0;
    obj_cen.clear();
    vector<double> tmp_obj_cen;
    
    for(int i = 0; i < msg->data.size(); i++) {
        if(i%2 == 0) {  // x

            tmp_obj_cen.push_back(msg->data[i]);
        }
        else {   // y
            tmp_obj_cen.push_back(msg->data[i]);
            obj_cen.push_back(tmp_obj_cen);
            tmp_obj_cen.clear();
        }
    }
    mission_state[14] = true;
}

void Relative_2_UTM::small_object_cb(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    int cluster_count = 0;
    obj_cen.clear();
    vector<double> tmp_obj_cen;
    
    for(int i = 0; i < msg->data.size(); i++) {
        if(i%2 == 0) {  // x

            tmp_obj_cen.push_back(msg->data[i]);
        }
        else {   // y
            tmp_obj_cen.push_back(msg->data[i]);
            obj_cen.push_back(tmp_obj_cen);
            tmp_obj_cen.clear();
        }
    }
    mission_state[11] = true;
}

void Relative_2_UTM::narrow_cal_obj_UTM(float c_heading, double sensor_distanse[2], vector<vector<double>> object_cen, double utm_gps[2]) {
    cout << c_heading << endl;
    
    for(int i =0; i < object_cen.size(); i++) {
        double x = sensor_distanse[0] + object_cen[i][0];
        double y = sensor_distanse[1] + object_cen[i][1];
        float c = obj_cen[i][2];

        double tmp_x = x*cos(c_heading) - y*sin(c_heading);
        double tmp_y = x*sin(c_heading) + y*cos(c_heading);

        vector<double> tmp_UTM;
        tmp_UTM.push_back(tmp_x + c_UTM[0]);
        tmp_UTM.push_back(tmp_y + c_UTM[1]);
        tmp_UTM.push_back(c);
        obj_UTM.push_back(tmp_UTM);

        cout << x << ", " << y <<endl;
    }
    cout << "------------------ " << endl << endl;
}

void Relative_2_UTM::narrow_color_cone(){
    queue<int> q;

    for(int i=0; i< obj_UTM.size(); i++) {
        if(obj_UTM[i][2] != 0){
            q.push(i);
        }
    }

    //bfs
    while(!q.empty()){
        int node = q.front();
        q.pop();
        for(int i=0; i<obj_UTM.size(); i++){
            if(calculateDistance(obj_UTM[i],obj_UTM[node]) < 1 && obj_UTM[i][2] == 0){
                cout << "push" << i <<endl;
                q.push(i);
                obj_UTM[i][2] = obj_UTM[node][2];
            }
        }
    }

    int yellow_count = 0;
    int blue_count = 0;

    for(int i=0; i< obj_UTM.size(); i++) {
        if(obj_UTM[i][2] == 1){
            blue_count +=1;
        }
        else if(obj_UTM[i][2] == 2){
            yellow_count +=1;
        }
    }

    if(blue_count == 0 && yellow_count != 0){
        for(int i=0; i< obj_UTM.size(); i++) {
            if(obj_UTM[i][2] == 0){
                obj_UTM[i][2] = 1;
            }
        }
    }
    else if(blue_count != 0 && yellow_count == 0){
        for(int i=0; i< obj_UTM.size(); i++) {
            if(obj_UTM[i][2] == 0){
                obj_UTM[i][2] = 2;
            }
        }
    }
    else if(blue_count == 0 && yellow_count == 0){
        
    }

    for(int i = 0; i<obj_UTM.size(); i++){
        cout << obj_UTM[i][0] << "," << obj_UTM[i][1] << "," << obj_UTM[i][2] << endl;
    }

}

void Relative_2_UTM::big_cal_obj_UTM(float c_heading, double sensor_distanse[2], vector<vector<double>> object_cen, double utm_gps[2]) {
    cout << c_heading << endl;
    
    for(int i =0; i < object_cen.size(); i++) {
        double x = sensor_distanse[0] + object_cen[i][0];
        double y = sensor_distanse[1] + object_cen[i][1];

        double tmp_x = x*cos(c_heading) - y*sin(c_heading);
        double tmp_y = x*sin(c_heading) + y*cos(c_heading);

        vector<double> tmp_UTM;
        tmp_UTM.push_back(tmp_x + c_UTM[0]);
        tmp_UTM.push_back(tmp_y + c_UTM[1]);
        obj_UTM.push_back(tmp_UTM);

        cout << x << ", " << y <<endl;
    }
    cout << "------------------ " << endl << endl;
}

void Relative_2_UTM::obq_cal_obj_UTM(float c_heading, double sensor_distanse[2], vector<vector<double>> object_cen, double utm_gps[2]) {
    cout << c_heading << endl;
    
    for(int i =0; i < object_cen.size(); i++) {
        double x = sensor_distanse[0] + object_cen[i][0];
        double y = sensor_distanse[1] + object_cen[i][1];

        double tmp_x = x*cos(c_heading) - y*sin(c_heading);
        double tmp_y = x*sin(c_heading) + y*cos(c_heading);

        vector<double> tmp_UTM;
        tmp_UTM.push_back(tmp_x + c_UTM[0]);
        tmp_UTM.push_back(tmp_y + c_UTM[1]);
        obj_UTM.push_back(tmp_UTM);

        cout << x << ", " << y <<endl;
    }
    cout << "------------------ " << endl << endl;
}

void Relative_2_UTM::prl_cal_obj_UTM(float c_heading, double sensor_distanse[2], vector<vector<double>> object_cen, double utm_gps[2]) {
    cout << c_heading << endl;
    
    for(int i =0; i < object_cen.size(); i++) {
        double x = sensor_distanse[0] + object_cen[i][0];
        double y = sensor_distanse[1] + object_cen[i][1];

        double tmp_x = x*cos(c_heading) - y*sin(c_heading);
        double tmp_y = x*sin(c_heading) + y*cos(c_heading);

        vector<double> tmp_UTM;
        tmp_UTM.push_back(tmp_x + c_UTM[0]);
        tmp_UTM.push_back(tmp_y + c_UTM[1]);
        obj_UTM.push_back(tmp_UTM);

        cout << x << ", " << y <<endl;
    }
    cout << "------------------ " << endl << endl;
}
void Relative_2_UTM::small_cal_obj_UTM(float c_heading, double sensor_distanse[2], vector<vector<double>> object_cen, double utm_gps[2]) {
    cout << c_heading << endl;
    
    for(int i =0; i < object_cen.size(); i++) {
        double x = sensor_distanse[0] + object_cen[i][0];
        double y = sensor_distanse[1] + object_cen[i][1];

        double tmp_x = x*cos(c_heading) - y*sin(c_heading);
        double tmp_y = x*sin(c_heading) + y*cos(c_heading);

        vector<double> tmp_UTM;
        tmp_UTM.push_back(tmp_x + c_UTM[0]);
        tmp_UTM.push_back(tmp_y + c_UTM[1]);
        obj_UTM.push_back(tmp_UTM);

        cout << x << ", " << y <<endl;
    }
    cout << "------------------ " << endl << endl;
}

//미션 번호 callback
void Relative_2_UTM::mission_cb(const std_msgs::msg::Int16::SharedPtr msg) {
    mission = msg->data;
}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Relative_2_UTM>();
    node->calc_relative_2_UTM();

    rclcpp::shutdown();
    return 0;
}
