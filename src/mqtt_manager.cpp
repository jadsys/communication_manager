/**
* @file     mqtt_manager.cpp
* @brief    ROS-MQTT間データマネージャーMqttManagerの定義ソースファイル
* @author   S.Kumada
* @date     2023/12/7
* @note     ROS⇔MQTT間のフォーマット変換やデータの精査等を行うクラスの実装
*/

#include "communication_manager/mqtt_manager.hpp"

MqttManager::MqttManager(ros::NodeHandle &node)
{

    ros::NodeHandle param_node("~");

    // パラメータ読み込み
    // ロボット名の読み込み
    getParam(param_node, "robot_id", robot_id_, std::string(DEF_ROBOT_ID));

    // ロボット種別情報の読み込み
    getParam(param_node, "robot_type", robot_type_, std::string(DEF_ROBOT_TYPE));
    
    // ロボットを動作させる際の環境
    getParam(param_node, "space_info", space_info_, std::string(DEF_SPACE_INFOMATION));

    // 地図の場所情報読み込み
    getParam(param_node, "map_location", map_location_, std::string(DEF_MAP_LOCATION));

// ロボットのステータス情報
    // for ロボット
    std::string sub_robot_state_topic;
    getParam(param_node, "robot_state/for_robot_topic_name", sub_robot_state_topic, std::string(DEF_SUB_ROBOT_STATE_TOPIC_NAME));
    sub_robo_status_        = node.subscribe(sub_robot_state_topic, ROS_QUEUE_SIZE_1, &MqttManager::recvRoboStatusCB, this);

    // To MQTTブリッジ
    std::string pub_robot_state_topic;
    getParam(param_node, "robot_state/to_bridge_topic_name", pub_robot_state_topic, std::string(DEF_PUB_ROBOT_STATE_TOPIC_NAME));
    pub_robot_state_        = node.advertise<uoa_poc4_msgs::r_pub_robot_state>("/" + robot_id_ + pub_robot_state_topic, ROS_QUEUE_SIZE_1, true);
    getParam(param_node, "robot_state/publish_rate", pub_rate_robot_state_, ROS_NON_RATE);

// 経路
    // for ロボット
    std::string sub_path_plan_topic;
    getParam(param_node, "path_plan/for_robot_topic_name", sub_path_plan_topic, std::string(DEF_SUB_PATH_PLAN_TOPIC_NAME));
    sub_local_path_plan_    = node.subscribe("/" + robot_id_ + sub_path_plan_topic, ROS_QUEUE_SIZE_1, &MqttManager::recvPathPlanCB, this);
    // To MQTTブリッジ
    std::string pub_path_plan_topic;
    getParam(param_node, "path_plan/to_bridge_topic_name", pub_path_plan_topic, std::string(DEF_PUB_PATH_PLAN_TOPIC_NAME));    
    pub_local_path_plan_ = node.advertise<uoa_poc4_msgs::r_pub_local_path_plan>("/" + robot_id_ + pub_path_plan_topic, ROS_QUEUE_SIZE_1, true);
    getParam(param_node, "path_plan/publish_rate", pub_rate_path_plan_, ROS_NON_RATE);

// LiDAR2D点群
    // for ロボット
    std::string sub_lidar_2d_topic;
    getParam(param_node, "lidar_2d/for_robot_topic_name", sub_lidar_2d_topic, std::string(DEF_SUB_LIDAR_2D_TOPIC_NAME));
    sub_lidar_data_2d_      = node.subscribe("/" + robot_id_ + sub_lidar_2d_topic, ROS_QUEUE_SIZE_1, &MqttManager::recvLaserScanCB, this );
    // To MQTTブリッジ
    std::string pub_lidar_2d_topic;
    getParam(param_node, "lidar_2d/to_bridge_topic_name", pub_lidar_2d_topic, std::string(DEF_PUB_LIDAR_2D_TOPIC_NAME));
    pub_lidar_data_2d_      = node.advertise<uoa_poc5_msgs::r_pub_laser_scan>("/" + robot_id_ + pub_lidar_2d_topic, ROS_QUEUE_SIZE_1, true);
    getParam(param_node, "lidar_2d/publish_rate", pub_rate_lidar_2d_, ROS_NON_RATE);

// LiDAR3D点群
    // for ロボット
    std::string sub_lidar_3d_topic;
    getParam(param_node, "lidar_3d/for_robot_topic_name", sub_lidar_3d_topic, std::string(DEF_SUB_LIDAR_3D_TOPIC_NAME));
    sub_lidar_data_3d_      = node.subscribe("/" + robot_id_ + sub_lidar_3d_topic, ROS_QUEUE_SIZE_1, &MqttManager::recvPointCloud2CB, this );
    // To MQTTブリッジ
    std::string pub_lidar_3d_topic;
    getParam(param_node, "lidar_3d/to_bridge_topic_name", pub_lidar_3d_topic, std::string(DEF_PUB_LIDAR_3D_TOPIC_NAME));
    pub_lidar_data_3d_      = node.advertise<uoa_poc5_msgs::r_pub_point_cloud>("/" + robot_id_ + pub_lidar_3d_topic, ROS_QUEUE_SIZE_1, true);
    getParam(param_node, "lidar_3d/publish_rate", pub_rate_lidar_3d_, ROS_NON_RATE);

// 緊急停止指示応答
    // for ロボット
    std::string sub_emg_cmd_exe_topic;
    getParam(param_node, "emg_cmd_exe/for_robot_topic_name", sub_emg_cmd_exe_topic, std::string(DEF_SUB_EMG_CMD_EXE_TOPIC_NAME));
    sub_emg_cmd_exe_        = node.subscribe(sub_emg_cmd_exe_topic, ROS_QUEUE_SIZE_1, &MqttManager::recvEmgCmdExeCB, this);
    // To MQTTブリッジ
    std::string pub_emg_cmd_exe_topic;
    getParam(param_node, "emg_cmd_exe/to_bridge_topic_name", pub_emg_cmd_exe_topic, std::string(DEF_PUB_EMG_CMD_EXE_TOPIC_NAME));
    pub_emg_cmd_exe_        = node.advertise<uoa_poc6_msgs::r_pub_emg_cmd_exe>("/" + robot_id_ + pub_emg_cmd_exe_topic, ROS_QUEUE_SIZE_1, true);
    getParam(param_node, "emg_cmd_exe/publish_rate", pub_rate_emg_cmd_exe_, ROS_NON_RATE);

// 緊急停止指示
    // To ロボット
    std::string pub_emg_cmd_topic;
    getParam(param_node, "emg_cmd/to_robot_topic_name", pub_emg_cmd_topic, std::string(DEF_SUB_EMG_CMD_TOPIC_NAME));
    pub_emg_cmd_        = node.advertise<uoa_poc3_msgs::r_emergency_command>(pub_emg_cmd_topic, ROS_QUEUE_SIZE_1, true);
    getParam(param_node, "emg_cmd/publish_rate", pub_rate_emg_cmd_, ROS_NON_RATE);
    // For MQTTブリッジ
    std::string sub_emg_cmd_topic;
    getParam(param_node, "emg_cmd/for_bridge_topic_name", sub_emg_cmd_topic, std::string(DEF_PUB_EMG_CMD_TOPIC_NAME));
    sub_emg_cmd_        = node.subscribe("/" + robot_id_ + sub_emg_cmd_topic, ROS_QUEUE_SIZE_1, &MqttManager::recvEmgCmdCB, this);

// 地図の更新通知
    // For MQTTブリッジ
    std::string sub_map_update_notify_topic;
    getParam(param_node, "map_update_notify/for_bridge_topic_name", sub_map_update_notify_topic, std::string(DEF_SUB_MAP_UPDATE_NOTIFY_TOPIC_NAME));
    sub_map_update_notify_   = node.subscribe("/" + robot_id_ + "/ros_bridge/sub_map_update_notify", ROS_QUEUE_SIZE_1, &MqttManager::subMapUpdateCB, this);
}

MqttManager::~MqttManager()
{
    // 処理なし
}

// For Robot
void MqttManager::recvRoboStatusCB(const uoa_poc3_msgs::r_stateConstPtr& msg)
{ // ロボットの情報受信

    uoa_poc4_msgs::r_pub_robot_state robot_state;

    robot_state.id      = msg->id;
    robot_state.type    = msg->type;    
    robot_state.space   = space_info_;    
    robot_state.time    = msg->time;
    robot_state.mode    = msg->mode;
    if(!msg->errors.empty())
    {
        robot_state.errors  = msg->errors.back();
    }
    robot_state.pose.position    = msg->pose.point;
    
    tf::Quaternion quat=tf::createQuaternionFromRPY(msg->pose.angle.roll, msg->pose.angle.pitch, msg->pose.angle.yaw);
    quaternionTFToMsg(quat, robot_state.pose.orientation);
    
    robot_state.covariance = msg->covariance; // boost::arrayは=でコピー可
    robot_state.battery = msg->battery;

    robot_state_ = robot_state;
    
    // 配信頻度が無制限の場合は受信と同時に配信する
    if(pub_rate_robot_state_ == 0) 
    { // 配信頻度が無制限(つまり受信していれば実行)
        pubRobotStatus();
    }
    else 
    { // 配信頻度に沿って配信する
        is_undelivered_robot_state_ = true; 
    }
}

void MqttManager::recvPathPlanCB(const nav_msgs::PathConstPtr& msg)
{ // ロボットの情報受信

    uoa_poc4_msgs::r_pub_local_path_plan robot_plan;

    robot_plan.id      = robot_id_;
    robot_plan.type    = robot_type_;    
    robot_plan.space   = space_info_;
    robot_plan.time    = iso8601ex();
    robot_plan.poses   = msg->poses;

    if(robot_state_.mode == MODE_NAVI )
    { // 最新のステータスがnavi時
        robot_plan_ = robot_plan;

        // 配信頻度が無制限の場合は受信と同時に配信する
        if(pub_rate_path_plan_ == 0) 
        { // 配信頻度が無制限(つまり受信していれば実行)
            pubPathPlan();
        }
        else 
        { // 配信頻度に沿って配信する
            is_undelivered_path_plan_ = true; 
        }
    }
}

void MqttManager::recvLaserScanCB(const sensor_msgs::LaserScan& msg)
{
    uoa_poc5_msgs::r_pub_laser_scan lidar_scan;

    lidar_scan.id               = robot_id_; 
    lidar_scan.type             = robot_type_; 
    lidar_scan.space            = space_info_;     
    lidar_scan.time             = iso8601ex(); 
    lidar_scan.scan_start_time  = rosTimeToIso8601(msg.header.stamp);
    lidar_scan.angle_min        = msg.angle_min;
    lidar_scan.angle_max        = msg.angle_max;
    lidar_scan.angle_increment  = msg.angle_increment;
    lidar_scan.time_increment   = msg.time_increment;
    lidar_scan.scan_time        = msg.scan_time;
    lidar_scan.range_min        = msg.range_min;
    lidar_scan.range_max        = msg.range_max;
    lidar_scan.ranges           = msg.ranges;
    lidar_scan.intensities      = msg.intensities;

    lidar_scan_ = lidar_scan;

     // 配信頻度が無制限の場合は受信と同時に配信する
    if(pub_rate_lidar_2d_ == 0) 
    { // 配信頻度が無制限(つまり受信していれば実行)
        pubLaserScan();
    }
    else 
    { // 配信頻度に沿って配信する
        is_undelivered_lidar_2d_ = true;
    }
    
    return ;
}

void MqttManager::recvPointCloud2CB(const sensor_msgs::PointCloud2& msg)
{
    uoa_poc5_msgs::r_pub_point_cloud point_cloud;

    point_cloud.id              = robot_id_;
    point_cloud.type            = robot_type_;
    point_cloud.space           = space_info_;
    point_cloud.time            = iso8601ex();
    point_cloud.scan_start_time = rosTimeToIso8601(msg.header.stamp);
    point_cloud.height          = msg.height;
    point_cloud.width           = msg.width;
    point_cloud.fields          = msg.fields;
    point_cloud.is_bigendian    = msg.is_bigendian;
    point_cloud.point_step      = msg.point_step;
    point_cloud.row_step        = msg.row_step;
    point_cloud.data            = msg.data;
    point_cloud.is_dense        = msg.is_dense;

    point_cloud_ = point_cloud;

    // 配信頻度が無制限の場合は受信と同時に配信する
    if(pub_rate_lidar_3d_ == 0) 
    { // 配信頻度が無制限(つまり受信していれば実行)
        pubPointCloud();
    }
    else 
    { // 配信頻度に沿って配信する
        is_undelivered_lidar_3d_ = true;
    }

    return ;
}

void MqttManager::recvEmgCmdExeCB(const uoa_poc3_msgs::r_emergency_result& msg)
{
    uoa_poc6_msgs::r_pub_emg_cmd_exe pub_emg_cmd_exe;

    pub_emg_cmd_exe.id                      = robot_id_;
    pub_emg_cmd_exe.type                    = robot_type_;
    pub_emg_cmd_exe.space                   = space_info_;
    pub_emg_cmd_exe.time                    = iso8601ex();
    pub_emg_cmd_exe.received_time           = msg.received_time;
    pub_emg_cmd_exe.received_emergency_cmd  = msg.received_emergency_cmd;
    pub_emg_cmd_exe.result                  = msg.result;

    for(const auto& e : msg.errors)
    {
        pub_emg_cmd_exe.errors.push_back(e);
    }

    emg_cmd_exe_ = pub_emg_cmd_exe;

    // 配信頻度が無制限の場合は受信と同時に配信する
    if(pub_rate_emg_cmd_exe_ == 0) 
    { // 配信頻度が無制限(つまり受信していれば実行)
        pubEmgCmdExe();
    }
    else 
    { // 配信頻度に沿って配信する
        is_undelivered_emg_cmd_exe_ = true;
    }

    return ;
}

void MqttManager::recvEmgCmdCB(const uoa_poc6_msgs::r_sub_emg_cmd& msg)
{
    uoa_poc3_msgs::r_emergency_command pub_emg_cmd;

    if( msg.id == robot_id_ && // 降ってくるロボットのIDが一致してる場合
        msg.type == robot_type_ && // 降ってくるロボットの種別が一致してる場合
        msg.space == space_info_)
    { // 空間情報が同じ場合
        pub_emg_cmd.id              = robot_id_;
        pub_emg_cmd.type            = robot_type_;
        pub_emg_cmd.time            = iso8601ex();
        pub_emg_cmd.emergency_cmd   = msg.emergency_cmd;
        
        emg_cmd_ = pub_emg_cmd;

        // 配信頻度が無制限の場合は受信と同時に配信する
        if(pub_rate_emg_cmd_ == 0) 
        { // 配信頻度が無制限(つまり受信していれば実行)
            
            pubEmgCmd();
        }
        else 
        { // 配信頻度に沿って配信する
            is_undelivered_emg_cmd_ = true;
        }

    }
    else
    { // 空間情報が違う場合
        // 無視したことを通知
        ROS_INFO("Message received was ignored because it was for a \"%s\"", space_info_.c_str());

    
    }

    return ;
}

// To mqtt_bridge
void MqttManager::pubMqttTopic()
{
    // ロボットステータス配信
    if(is_undelivered_robot_state_ && 
     checkLastPublishTime(pub_rate_robot_state_, last_pub_time_robot_state_)) // 配信間隔以上
    { // 未配信のデータが存在するかつ配信頻度しきい値以上
        // publish
        pubRobotStatus();

        // 最後に配信した時間の更新
        last_pub_time_robot_state_ = ros::Time::now();
    }
    // 経路情報
    if(is_undelivered_path_plan_ && 
      checkLastPublishTime(pub_rate_path_plan_, last_pub_time_path_plan_))
    { // 未配信のデータが存在するかつ配信頻度しきい値以上
        // publish
        pubPathPlan();
        // 最後に配信した時間の更新
        last_pub_time_path_plan_ = ros::Time::now();
    }
    // LiDARの2D点群
    if(is_undelivered_lidar_2d_ && 
      checkLastPublishTime(pub_rate_lidar_2d_, last_pub_time_lidar_2d_))
    { // 未配信のデータが存在するかつ配信頻度しきい値以上
        // publish
        pubLaserScan();
        // 最後に配信した時間の更新
        last_pub_time_lidar_2d_ = ros::Time::now();
    }
    // LiDARの3D点群
    if(is_undelivered_lidar_3d_ && 
      checkLastPublishTime(pub_rate_lidar_3d_, last_pub_time_lidar_3d_))
    { // 未配信のデータが存在するかつ配信頻度しきい値以上
        // publish
        pubPointCloud();
        // 最後に配信した時間の更新
        last_pub_time_lidar_3d_ = ros::Time::now();
    }
    // 緊急停止指示の応答
    if(is_undelivered_emg_cmd_exe_ && 
      checkLastPublishTime(pub_rate_emg_cmd_exe_, last_pub_time_emg_cmd_exe_))
    { // 未配信のデータが存在するかつ配信頻度しきい値以上
        // publish
        pubEmgCmdExe();
        // 最後に配信した時間の更新
        last_pub_time_emg_cmd_exe_ = ros::Time::now();
    }

}

// To Robot
void MqttManager::pubRobotTopic()
{
    // 緊急停止指示
    if(is_undelivered_emg_cmd_ && 
      checkLastPublishTime(pub_rate_emg_cmd_, last_pub_time_emg_cmd_))
    { // 未配信のデータが存在するかつ配信頻度しきい値以上
        // publish
        pubEmgCmd();
        // 最後に配信した時間の更新
        last_pub_time_emg_cmd_ = ros::Time::now();
    }
}

void MqttManager::pubRobotStatus()
{
    pub_robot_state_.publish(robot_state_);
    is_undelivered_robot_state_ = false;

    return;
}

void MqttManager::pubPathPlan()
{
    pub_local_path_plan_.publish(robot_plan_);
    is_undelivered_path_plan_ = false;

    return;
}

void MqttManager::pubLaserScan()
{
    pub_lidar_data_2d_.publish(lidar_scan_);
    is_undelivered_lidar_2d_ = false;

    return;
}

void MqttManager::pubPointCloud()
{
    pub_lidar_data_3d_.publish(point_cloud_);
    is_undelivered_lidar_3d_ = false;

    return;
}

void MqttManager::pubEmgCmdExe()
{
    pub_emg_cmd_exe_.publish(emg_cmd_exe_);
    is_undelivered_emg_cmd_exe_ = false;

    return;
}

void MqttManager::pubEmgCmd()
{
    pub_emg_cmd_.publish(emg_cmd_);
    is_undelivered_emg_cmd_ = false;

    return;
}

// For mqtt_bridge
// #if 1
// 地図の更新通知受信コールバック関数
void MqttManager::subMapUpdateCB(const uoa_poc4_msgs::r_sub_map_update_notify& msg)
{
//     uoa_poc4_msgs::r_req_get_mapdata req_mapdata;

//     if( robot_id_       == msg.id &&
//         map_location_   == msg.location_info.location &&
//         last_revision_  != msg.revision )
//     { // 地図の場所が一緒かつリビジョン番号が更新された場合に地図の更新があったものとして判断
//         req_mapdata.info.header.time        = iso8601ex();
//         req_mapdata.info.location.location  = map_location_;
//         req_mapdata.info.system.data_type   = DEF_SYSTEM_TYPE;
//         req_mapdata.info.system.latest      = true;
//         req_mapdata.info.system.map_layer   = msg.map_layer;
//         req_mapdata.info.system.revision    = msg.revision;
//         req_mapdata.is_format_location      = true;
//         req_mapdata.is_use_latest           = true;

//         ROS_INFO("----------- Get MAP UPDATE START ------------");

//         reqGetMapdata(req_mapdata);

//         ROS_INFO("----------- Get MAP UPDATE END ------------");

//         // 最終リビジョン番号を更新
//         last_revision_ = msg.revision;
}
// #if DEBUG
//     else
//     {
//         ROS_WARN("subMapUpdateCB : Not Map Updated");
//     }
// #endif

//     return;
// }
// #else
// void MqttManager::subMapUpdateCB(const uoa_poc4_msgs::r_sub_map_update_notify& msg)
// {
//     uoa_poc4_msgs::r_req_get_mapdata req_mapdata;

//     if( robot_id_       == msg.info.header.id &&
//         map_location_   == msg.info.location.location &&
//         last_revision_  != msg.info.system.revision )
//     { // 地図の場所が一緒かつリビジョン番号が更新された場合に地図の更新があったものとして判断
//         req_mapdata.info.header.time        = iso8601ex();
//         req_mapdata.info.location.location  = map_location_;
//         req_mapdata.info.system.data_type   = DEF_SYSTEM_TYPE;
//         req_mapdata.info.system.latest      = true;
//         req_mapdata.info.system.map_layer   = msg.info.system.map_layer;
//         req_mapdata.info.system.revision    = msg.info.system.revision;
//         req_mapdata.is_format_location      = true;
//         req_mapdata.is_use_latest           = true;

//         ROS_INFO("----------- Get MAP UPDATE START ------------");

//         reqGetMapdata(req_mapdata);

//         ROS_INFO("----------- Get MAP UPDATE END ------------");

//         // 最終リビジョン番号を更新
//         last_revision_ = msg.info.system.revision;
//     }
// #if DEBUG
//     else
//     {
//         ROS_WARN("subMapUpdateCB : Not Map Updated");
//     }
// #endif

//     return;
// }
// #endif
