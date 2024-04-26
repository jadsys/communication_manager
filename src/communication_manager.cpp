/**
* @file     communication_manager.cpp
* @brief    通信マネージャー（CommunicationManager）の定義ソースファイル
* @author   S.Kumada
* @date     2023/12/7
* @note     ロボットとRDR、上位系間のデータのやり取りに関して、メッセージのフォーマット変換やデータの精査等を行うクラスの実装
*/

#include "communication_manager/communication_manager.hpp"

// Global


// Common
CommunicationManager::CommunicationManager(ros::NodeHandle &node)
{
    ros::NodeHandle param_node("~");
    std::string robot_id = "";
    std::string robot_info_topic = "";


    http_manager_    = new HttpManager(node);
    mqtt_manager_    = new MqttManager(node);
    put_ego_map_     = new nav_msgs::OccupancyGrid();
    put_socio_map_   = new nav_msgs::OccupancyGrid();

    is_undelivered_robot_state_ = false;
    is_undelivered_path_plan_   = false;
    is_undelivered_lidar_2d_    = false;
    is_undelivered_lidar_3d_    = false;

    is_sb_put_ego_map_       = false;
    is_sb_put_socio_map_     = false;
    is_published_path_plan_  = false;
    is_status_navi_          = false;

    retry_get_position_times_ = 0;

    // ロボット名の読み込み
    getParam(param_node, "robot_id", robot_id_, std::string(DEF_ROBOT_ID));

    // ロボット種別情報の読み込み
    getParam(param_node, "robot_type", robot_type_, std::string(DEF_ROBOT_TYPE));

    // 地図の場所情報読み込み
    getParam(param_node, "map_location", map_location_, std::string(DEF_MAP_LOCATION));

    // ループレートの読み込み
    getParam(param_node, "loop_rate", loop_rate_, ROS_RATE_30HZ);

}

CommunicationManager::~CommunicationManager()
{
    delete(http_manager_);
    delete(mqtt_manager_);
    delete(put_ego_map_);
    delete(put_socio_map_);
}

void CommunicationManager::manageCom(void)
{
    ros::Rate rate = loop_rate_;

    while(ros::ok())
    {
        rate.sleep(); // スリープ
        ros::spinOnce();   // コールバック処理
        mqtt_manager_->pubMqttTopic();
        mqtt_manager_->pubRobotTopic();

    }

    return;
}