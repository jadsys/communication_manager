/**
* @file     http_manager.cpp
* @brief    ROS-HTTP間データマネージャーHttpManagerの定義ソースファイル
* @author   S.Kumada
* @date     2023/12/7
* @note     ROS⇔HTTP間のフォーマット変換やデータの精査等を行うクラスの実装
*/

#include "communication_manager/http_manager.hpp"

// デバック用
void showHeaderValue(uoa_poc4_msgs::r_header header)
{
    ROS_INFO("----- Header -----");
    ROS_INFO(" id : %s",    header.id.c_str());
    ROS_INFO(" type : %s",  header.type.c_str());
    ROS_INFO(" space : %s", header.space.c_str());
    ROS_INFO(" time : %s",  header.time.c_str());
    ROS_INFO("------------------");
}

void showLocationInfo(uoa_poc4_msgs::r_map_location_info info)
{
    ROS_INFO("----- Location Info -----");
    ROS_INFO(" location : %s",  info.location.c_str());
    ROS_INFO(" lat : %lf",      info.lat);
    ROS_INFO(" lon : %lf",      info.lon);
    ROS_INFO(" azimuth : %lf",  info.azimuth);
    ROS_INFO("-------------------------");
}

void showMapSystemInfo(uoa_poc4_msgs::r_map_system_info info)
{
    ROS_INFO("----- Map System Info -----");
    ROS_INFO(" data_type : %s", info.data_type.c_str());
    ROS_INFO(" map_layer : %s", info.map_layer.c_str());
    ROS_INFO(" revision : %s",  info.revision.c_str());
    ROS_INFO(" latest : %s",    info.latest ? "true" : "false");
    ROS_INFO("---------------------------");
}

void showMapDataInfo(nav_msgs::OccupancyGrid map)
{
    char date[64];
    time_t t = map.info.map_load_time.sec;

    ROS_INFO("----- Map Data Info -----");

    strftime(date, sizeof(date), "%Y/%m/%d %a %H:%M:%S", localtime(&t));
    ROS_INFO(" date : %s",          date);
    ROS_INFO(" resolution : %f",    map.info.resolution);
    ROS_INFO(" width : %ui",        map.info.width);
    ROS_INFO(" height : %ui",       map.info.height);
    ROS_INFO(" Origin X : %lf",     map.info.origin.position.x);
    ROS_INFO(" Origin Y : %lf",     map.info.origin.position.y);
    ROS_INFO(" Origin QX : %lf",    map.info.origin.orientation.x);
    ROS_INFO(" Origin QY: %lf",     map.info.origin.orientation.y);
    ROS_INFO(" Origin QZ : %lf",    map.info.origin.orientation.z);
    ROS_INFO(" Origin QW: %lf",     map.info.origin.orientation.w);
    ROS_INFO("-------------------------");    
}

//**************************************
// メソッド
//**************************************
// Common
HttpManager::HttpManager(ros::NodeHandle &node)
{
    ros::NodeHandle param_node("~");
    std::string http_req = "";
    std::string http_res = "";
    cnt_retry_get_position = 10;
    

    // put_ego_map_     = new nav_msgs::OccupancyGrid();
    // put_socio_map_   = new nav_msgs::OccupancyGrid();
    ego_map_= new nav_msgs::OccupancyGrid();
    socio_map_= new nav_msgs::OccupancyGrid();
    static_layer_map_= new nav_msgs::OccupancyGrid();
    semi_static_layer_map_= new nav_msgs::OccupancyGrid();
    exclusion_zone_layer_map_= new nav_msgs::OccupancyGrid();
    environment_map_    = new nav_msgs::OccupancyGrid();
    reference_map_  = new nav_msgs::OccupancyGrid();


    // is_status_navi_          = false;

    // ロボット名の読み込み
    getParam(param_node, "robot_id", robot_id_, std::string(DEF_ROBOT_ID));

    // 地図のTFフレーム名
    getParam(param_node, "map_frame_id", map_frame_id_, std::string(DEF_MAP_FLAME_NAME));

    // TFのトップ階層
    getParam(param_node, "tf_prefix", tf_prefix_, std::string(DEF_FRAME_ID));
    if( !tf_prefix_.empty() &&
        (tf_prefix_.at(tf_prefix_.size()-1) != '/'))
    { // tf_prefixが設定されており、"/"が入っていない場合
        tf_prefix_ = tf_prefix_ + "/";
    }

    // ロボット種別情報の読み込み
    getParam(param_node, "robot_type", robot_type_, std::string(DEF_ROBOT_TYPE));

    // ロボット機種名の読み込み
    getParam(param_node, "robot_name", robot_name_, std::string(DEF_ROBOT_ID));

    // 地図の場所情報読み込み
    getParam(param_node, "map_location", map_location_, std::string(DEF_MAP_LOCATION));
    
    // 空間情報読み込み
    getParam(param_node, "space_info", space_info_, std::string(DEF_SPACE_INFOMATION));

    // ロボット内部地図の受信トピック名の読み込み
    getParam(param_node, "map_topic/ego_map_topic_name",            ego_map_topic_name_,                std::string(DEF_EGO_MAP_TOPIC_NAME));     // エゴ地図のトピック名の読み込み
    getParam(param_node, "map_topic/sosio_map_topic_name",          socio_map_topic_name_,              std::string(DEF_SOSIO_MAP_TOPIC_NAME));   //    ソシオ地図のトピック名の読み込み
    getParam(param_node, "map_topic/static_map_topic_name",         static_layer_map_topic_name_,       std::string(DEF_STATIC_LAYER_MAP_TOPIC_NAME));// 静的レイヤ地図のトピック名の読み込み
    getParam(param_node, "map_topic/quasi_static_map_topic_name",   quasi_static_layer_map_topic_name_, std::string(DEF_QUASI_STATIC_LAYER_MAP_TOPIC_NAME));    // 準静的レイヤ地図のトピック名の読み込み
    getParam(param_node, "map_topic/no_entry_map_topic_name",       exclusion_zone_layer_map_topic_name_, std::string(DEF_EXCLUSION_ZONE_LAYER_MAP_TOPIC_NAME));      // 進入禁止レイヤ地図のトピック名の読み込み
    
    // シミュレータ生成地図の受信トピック名の読み込み
    getParam(param_node, "map_topic/environment_map_topic_name",    environment_map_topic_name_,    std::string(DEF_EXCLUSION_ZONE_LAYER_MAP_TOPIC_NAME));      // 進入禁止レイヤ地図のトピック名の読み込み
    getParam(param_node, "map_topic/reference_map_topic_name",      reference_map_topic_name_,      std::string(DEF_EXCLUSION_ZONE_LAYER_MAP_TOPIC_NAME));      // 進入禁止レイヤ地図のトピック名の読み込み

// ロボットの機体情報をRDRに登録するAPI
    // リトライ間隔、最大回数の読み込み
    // getParam(param_node, "retry_interval_time", retry_time_, DEF_RETRY_TIME);    // リトライ間隔時間を読み込み
    // getParam(param_node, "max_num_of_retry", retry_counter_, DEF_RETRY_COUNT);    // リトライ最大回数を読み込み
    // For Robot
    sub_robo_info_           = node.subscribe("/robot_bridge/" + robot_id_ + "/" + "robo_info", ROS_QUEUE_SIZE_1, &HttpManager::recvRoboInfoCB, this);
    // To http_bridge
    std::string pub_req_robot_info_topic_name;
    getParam(param_node, "robot_info/to_bridge_topic_name", pub_req_robot_info_topic_name, std::string(DEF_REQ_ROBOT_INFO_TOPIC_NAME));
    pub_req_robot_info_      = node.advertise<uoa_poc4_msgs::r_req_robot_info>("/" + robot_id_ + pub_req_robot_info_topic_name, ROS_QUEUE_SIZE_1, true);
    // For http_bridge
    std::string sub_res_robot_info_topic_name;
    getParam(param_node, "robot_info/for_bridge_topic_name", sub_res_robot_info_topic_name, std::string(DEF_RES_ROBOT_INFO_TOPIC_NAME));
    sub_res_robot_info_      = node.subscribe("/" + robot_id_ + sub_res_robot_info_topic_name, ROS_QUEUE_SIZE_1, &HttpManager::resRobotInfoCB, this);
    // To Robot
    /*ロボットには返さない*/

// 地図のデータリスト（最新10件）を取得するAPI
    // リトライ間隔、最大回数の読み込み
    // For Robot
    // To http_bridge
    std::string pub_req_map_data_list_topic_name;
    getParam(param_node, "get_map_data_list/to_bridge_topic_name", pub_req_map_data_list_topic_name, std::string(DEF_REQ_MAP_DATA_LIST_TOPIC_NAME));
    pub_req_map_data_list_      = node.advertise<uoa_poc4_msgs::r_req_mapdata_list>("/" + robot_id_ + pub_req_map_data_list_topic_name, ROS_QUEUE_SIZE_2, true);
    // For http_bridge
    std::string sub_res_map_data_list_topic_name;
    getParam(param_node, "get_map_data_list/for_bridge_topic_name", sub_res_map_data_list_topic_name, std::string(DEF_RES_MAP_DATA_LIST_TOPIC_NAME));
    sub_res_map_data_list_   = node.subscribe("/" + robot_id_ + sub_res_map_data_list_topic_name, ROS_QUEUE_SIZE_1, &HttpManager::resMapdataListCB, this);
    // To Robot

// 地図をRDRへPUTするAPI
    // リトライ間隔、最大回数の読み込み

    // For Robot & Simulator
    std::string sub_put_map_data_topic_name;
    getParam(param_node, "put_map_data/for_robot_topic_name", sub_put_map_data_topic_name, std::string(DEF_FOR_RBT_PUT_MAP_DATA_TOPIC_NAME));
    sub_put_map_data_               = node.subscribe(sub_put_map_data_topic_name, ROS_QUEUE_SIZE_1, &HttpManager::recvPutMapdataCB, this);

    // For Robot
    is_sb_put_ego_map_              = false;
    is_sb_put_socio_map_            = false;
    is_sb_put_static_layer_         = false;
    is_sb_put_quasi_static_layer_   = false;
    is_sb_put_exclusion_zone_layer_ = false;
    sub_ego_map_                    = node.subscribe<nav_msgs::OccupancyGrid>("/" + robot_id_ + ego_map_topic_name_, ROS_QUEUE_SIZE_1, boost::bind(&HttpManager::recvMapCB, this, _1, DEF_EGO_MAP_NAME));
    sub_socio_map_                  = node.subscribe<nav_msgs::OccupancyGrid>("/" + robot_id_ + socio_map_topic_name_, ROS_QUEUE_SIZE_1, boost::bind(&HttpManager::recvMapCB, this, _1, DEF_SOCIO_MAP_NAME));
    sub_static_layer_map_           = node.subscribe<nav_msgs::OccupancyGrid>("/" + robot_id_ + static_layer_map_topic_name_, ROS_QUEUE_SIZE_1, boost::bind(&HttpManager::recvMapCB, this, _1, DEF_STATIC_LAYER_MAP_NAME));
    sub_quasi_static_layer_map_     = node.subscribe<nav_msgs::OccupancyGrid>("/" + robot_id_ + quasi_static_layer_map_topic_name_, ROS_QUEUE_SIZE_1, boost::bind(&HttpManager::recvMapCB, this, _1, DEF_QUASI_STATIC_LAYER_MAP_NAME));
    sub_exclusion_zone_layer_map_   = node.subscribe<nav_msgs::OccupancyGrid>("/" + robot_id_ + exclusion_zone_layer_map_topic_name_, ROS_QUEUE_SIZE_1, boost::bind(&HttpManager::recvMapCB, this, _1, DEF_EXCLUSION_ZONE_LAYER_MAP_NAME));
    
    // For Simulator
    is_sb_put_environment_map_      = false;
    is_sb_put_reference_map_        = false;
    sub_environment_map_    = node.subscribe<nav_msgs::OccupancyGrid>("/" + robot_id_ + environment_map_topic_name_, ROS_QUEUE_SIZE_1, boost::bind(&HttpManager::recvMapCB, this, _1, DEF_ENVIRONMENT_MAP_NAME));
    sub_reference_map_      = node.subscribe<nav_msgs::OccupancyGrid>("/" + robot_id_ + reference_map_topic_name_, ROS_QUEUE_SIZE_1, boost::bind(&HttpManager::recvMapCB, this, _1, DEF_REFERENCE_MAP_NAME));

    // To http_bridge
    if(getParam(param_node, "manual_settings/put_map_linked_revision", set_map_linked_revision_, std::string("")))
    {
        ROS_WARN("Attention: Manual settings for \"put_map_linked_revision\" are effective! If not in use, please modify the configuration file.");
    }
    std::string pub_req_put_map_data_topic_name;
    getParam(param_node, "put_map_data/to_bridge_topic_name", pub_req_put_map_data_topic_name, std::string(DEF_REQ_PUT_MAP_DATA_TOPIC_NAME));
    pub_req_put_map_data_       = node.advertise<uoa_poc4_msgs::r_req_put_mapdata>("/" + robot_id_ + pub_req_put_map_data_topic_name, ROS_QUEUE_SIZE_2, true);
    
    // For http_bridge
    std::string sub_res_put_map_data_topic_name;
    getParam(param_node, "put_map_data/for_bridge_topic_name", sub_res_put_map_data_topic_name, std::string(DEF_RES_PUT_MAP_DATA_TOPIC_NAME));
    sub_res_put_map_data_       = node.subscribe("/" + robot_id_ + sub_res_put_map_data_topic_name, ROS_QUEUE_SIZE_1, &HttpManager::resPutMapdataCB, this);
    
    // To Robot
    /* ロボットには返さない */

// 補正値をRDRへPUTするAPI
    // リトライ間隔、最大回数の読み込み
    // For simulator
    is_sb_put_map_pose_correct_value_ = false;
    std::string sub_map_pose_correct_topic_name;
    getParam(param_node, "map_pose_correct_topic", sub_map_pose_correct_topic_name, std::string(DEF_FOR_SIM_CORRECT_INFO_TOPIC_NAME));
    sub_correct_info_           = node.subscribe("/" + robot_id_ + sub_map_pose_correct_topic_name, ROS_QUEUE_SIZE_1, &HttpManager::recvMapCorrectInfoCB, this);

    std::string sub_put_correct_info_topic_name;
    getParam(param_node, "put_correct_info/for_robot_topic_name", sub_put_correct_info_topic_name, std::string(DEF_FOR_SIM_PUT_CORRECT_INFO_TOPIC_NAME));
    sub_put_correct_info_       = node.subscribe("/" + robot_id_ + sub_put_correct_info_topic_name, ROS_QUEUE_SIZE_1, &HttpManager::recvPutCorrectInfoCB, this);
    
    // To http_bridge
    // ターゲットとソース地図のリビジョン番号の手動設定
    if(getParam(param_node, "manual_settings/put_correct_val_linked_source_map_revision", source_map_revision_number_, std::string("")))
    {   
        ROS_WARN("Attention: Manual settings for \"put_correct_val_linked_source_map_revision\" are effective! If not in use, please modify the configuration file.");
    }
    if(getParam(param_node, "manual_settings/put_correct_val_linked_target_map_revision", target_map_revision_number_, std::string("")))
    {
        ROS_WARN("Attention: Manual settings for \"put_correct_val_linked_target_map_revision\" are effective! If not in use, please modify the configuration file.");
    }
    std::string pub_req_put_correct_info_topic_name;
    getParam(param_node, "put_correct_info/to_bridge_topic_name", pub_req_put_correct_info_topic_name, std::string(DEF_REQ_PUT_CORRECT_INFO_TOPIC_NAME));
    pub_req_put_correct_info_   = node.advertise<uoa_poc6_msgs::r_req_put_map_pose_correct>("/" + robot_id_ + pub_req_put_correct_info_topic_name, ROS_QUEUE_SIZE_1, true);
    
    // For http_bridge
    std::string pub_res_put_correct_info_topic_name;
    getParam(param_node, "put_correct_info/for_bridge_topic_name", pub_res_put_correct_info_topic_name, std::string(DEF_RES_PUT_CORRECT_INFO_TOPIC_NAME));
    sub_res_put_correct_info_   = node.subscribe("/" + robot_id_ + pub_res_put_correct_info_topic_name, ROS_QUEUE_SIZE_1, &HttpManager::resPutCorrectInfoCB, this);
    
    // To simulator
    /* シミュレータには返さない */

// 地図をRDRからGETするAPI
    // リトライ間隔、最大回数の読み込み
    // For Robot
    sub_get_map_data_     = node.subscribe("/robot_bridge/" + robot_id_ + "/get_map_data", ROS_QUEUE_SIZE_1, &HttpManager::recvGetMapdataCB, this );
    
    // To http_bridge
    std::string pub_req_get_map_data_topic_name;
    getParam(param_node, "get_map_data/to_bridge_topic_name", pub_req_get_map_data_topic_name, std::string(DEF_REQ_GET_MAP_DATA_TOPIC_NAME));
    pub_req_get_map_data_           = node.advertise<uoa_poc4_msgs::r_req_get_mapdata>("/" + robot_id_ + pub_req_get_map_data_topic_name, ROS_QUEUE_SIZE_2, true);
    
    // For http_bridge
     std::string sub_res_get_map_data_topic_name;
    getParam(param_node, "get_map_data/for_bridge_topic_name", sub_res_get_map_data_topic_name, std::string(DEF_RES_GET_MAP_DATA_TOPIC_NAME));
    sub_res_get_map_data_    = node.subscribe("/" + robot_id_ + sub_res_get_map_data_topic_name, ROS_QUEUE_SIZE_1, &HttpManager::resGetMapdataCB, this);

    // To Robot
    pub_recv_map_info_                  = node.advertise<uoa_poc5_msgs::r_recv_map_info>("/robot_bridge/" + robot_id_ + "/layer_map_update_notify", ROS_QUEUE_SIZE_3, true);
    pub_ego_map_data_                   = node.advertise<nav_msgs::OccupancyGrid>("/" + robot_id_ + ego_map_topic_name_, ROS_QUEUE_SIZE_1, true);
    pub_socio_map_data_                 = node.advertise<nav_msgs::OccupancyGrid>("/" + robot_id_ + socio_map_topic_name_, ROS_QUEUE_SIZE_1, true);
    pub_static_layer_map_data_          = node.advertise<nav_msgs::OccupancyGrid>("/" + robot_id_ + static_layer_map_topic_name_, ROS_QUEUE_SIZE_1, true);
    pub_quasi_static_layer_map_data_    = node.advertise<nav_msgs::OccupancyGrid>("/" + robot_id_ + quasi_static_layer_map_topic_name_, ROS_QUEUE_SIZE_1, true);
    pub_exclusion_zone_layer_map_data_  = node.advertise<nav_msgs::OccupancyGrid>("/" + robot_id_ + exclusion_zone_layer_map_topic_name_, ROS_QUEUE_SIZE_1, true);
    
    // To simulator
    pub_environment_map_data_ = node.advertise<nav_msgs::OccupancyGrid>("/" + robot_id_ + environment_map_topic_name_, ROS_QUEUE_SIZE_1, true);
    pub_reference_map_data_   = node.advertise<nav_msgs::OccupancyGrid>("/" + robot_id_ + reference_map_topic_name_, ROS_QUEUE_SIZE_1, true);

// RDRに登録されている環境地図に紐づくレイヤー地図1件を取得するAPI
    // リトライ間隔、最大回数の読み込み
    // For Robot
    sub_get_layer_map_data_     = node.subscribe("/robot_bridge/" + robot_id_ + "/get_layer_map_data", ROS_QUEUE_SIZE_1, &HttpManager::recvGetLayerMapdataCB, this );
    
    // To http_bridge
    std::string pub_req_get_layer_map_data_topic_name;
    getParam(param_node, "get_layer_map_data/to_bridge_topic_name", pub_req_get_layer_map_data_topic_name, std::string(DEF_REQ_GET_LAYER_MAP_DATA_TOPIC_NAME));
    pub_req_get_layer_map_data_ = node.advertise<uoa_poc4_msgs::r_req_get_mapdata>("/" + robot_id_ + pub_req_get_layer_map_data_topic_name, ROS_QUEUE_SIZE_2, true);
    
    // For http_bridge
    std::string sub_req_get_layer_map_data_topic_name;
    getParam(param_node, "get_layer_map_data/for_bridge_topic_name", sub_req_get_layer_map_data_topic_name, std::string(DEF_RES_GET_LAYER_MAP_DATA_TOPIC_NAME));
    sub_res_get_layer_map_data_    = node.subscribe("/" + robot_id_ + sub_req_get_layer_map_data_topic_name, ROS_QUEUE_SIZE_1, &HttpManager::resGetMapdataCB, this);
    
    // To Robot
        /* 地図をRDRからGETするAPIと共有 */

// 物体認識システムで認識したロボットの推定位置を取得するAPI
    // リトライ間隔、最大回数の読み込み
    // For Robot
    sub_get_position_data_     = node.subscribe("/robot_bridge/" + robot_id_ + "/get_position_data", ROS_QUEUE_SIZE_1, &HttpManager::recvGetPositiondataCB, this );
    
    // To http_bridge
    // 位置取得対象のロボット検索名
    getParam(param_node, "get_robot_position_name", get_robot_position_name_, std::string(DEF_ROBOT_NAME));
    std::string pub_req_get_position_data_topic_name;
    getParam(param_node, "get_cam_position_data/to_bridge_topic_name", pub_req_get_position_data_topic_name, std::string(DEF_REQ_GET_POSITION_TOPIC_NAME));
    pub_req_get_position_data_  = node.advertise<uoa_poc4_msgs::r_req_get_position_data>("/" + robot_id_ + pub_req_get_position_data_topic_name, ROS_QUEUE_SIZE_1, true);
    
    // For http_bridge
    std::string sub_res_get_position_data_topic_name;
    getParam(param_node, "get_cam_position_data/for_bridge_topic_name", sub_res_get_position_data_topic_name, std::string(DEF_RES_GET_POSITION_TOPIC_NAME));
    sub_res_position_data_   = node.subscribe("/" + robot_id_ + sub_res_get_position_data_topic_name, ROS_QUEUE_SIZE_1, &HttpManager::resReqPositiondataCB, this );
    
    // To Robot
    pub_initial_position_data_  = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("/" + robot_id_ + "/initialpose", ROS_QUEUE_SIZE_1, true); 


// 準静的物体の更新位置を取得するAPI
    // リトライ間隔、最大回数の読み込み
    // For sim
    std::string sub_get_object_location_topic_name;
    getParam(param_node, "get_object_location/for_robot_topic_name", sub_get_object_location_topic_name, std::string(DEF_TO_SIM_GET_OBJECT_LOCATION_TOPIC_NAME));
    
    sub_get_object_location_     = node.subscribe("/" + robot_id_ + sub_get_object_location_topic_name, ROS_QUEUE_SIZE_1, &HttpManager::recvGetObjectLocationCB, this );
    
    // To http_bridge
    std::string pub_req_get_object_location_topic_name;
    getParam(param_node, "get_object_location/to_bridge_topic_name", pub_req_get_object_location_topic_name, std::string(DEF_REQ_GET_OBJECT_LOCATION_TOPIC_NAME));
    pub_req_get_object_location_  = node.advertise<uoa_poc6_msgs::r_req_change_obj_loc>("/" + robot_id_ + pub_req_get_object_location_topic_name, ROS_QUEUE_SIZE_1, true);
    
    // For http_bridge
    std::string sub_res_object_location_topic_name;
    getParam(param_node, "get_object_location/for_bridge_topic_name", sub_res_object_location_topic_name, std::string(DEF_RES_GET_OBJECT_LOCATION_TOPIC_NAME));
    sub_res_get_object_location_   = node.subscribe("/" + robot_id_ + sub_res_object_location_topic_name, ROS_QUEUE_SIZE_1, &HttpManager::resObjectLocationCB, this );
    
    // To sim
    std::string pub_get_object_location_topic_name;
    getParam(param_node, "get_object_location/to_robot_topic_name", pub_get_object_location_topic_name, std::string(DEF_FOR_SIM_GET_OBJECT_LOCATION_TOPIC_NAME));
    pub_get_object_location_  = node.advertise<uoa_poc6_msgs::r_objects_location>("/" +  robot_id_ + pub_get_object_location_topic_name, ROS_QUEUE_SIZE_1, true); 

// 補正値をRDRから取得するAPI
    // リトライ間隔、最大回数の読み込み
    // For robot
    std::string sub_get_correct_info_topic_name;
    getParam(param_node, "get_correct_info/for_robot_topic_name", sub_get_correct_info_topic_name, std::string(DEF_FOR_RBT_GET_CORRECT_INFO_TOPIC_NAME));
    sub_get_correct_info_       = node.subscribe("/" + robot_id_ + sub_get_correct_info_topic_name, ROS_QUEUE_SIZE_1, &HttpManager::recvGetCorrectInfoCB, this);
    
    // To http_bridge
    std::string pub_req_get_correct_info_topic_name;
    getParam(param_node, "get_correct_info/to_bridge_topic_name", pub_req_get_correct_info_topic_name, std::string(DEF_REQ_GET_CORRECT_INFO_TOPIC_NAME));
    pub_req_get_correct_info_   = node.advertise<uoa_poc6_msgs::r_req_get_map_pose_correct>("/" + robot_id_ + pub_req_get_correct_info_topic_name, ROS_QUEUE_SIZE_1, true);
    
    // For http_bridge
    std::string pub_res_get_correct_info_topic_name;
    getParam(param_node, "get_correct_info/for_bridge_topic_name", pub_res_get_correct_info_topic_name, std::string(DEF_RES_GET_CORRECT_INFO_TOPIC_NAME));
    sub_res_get_correct_info_   = node.subscribe("/" + robot_id_ + pub_res_get_correct_info_topic_name, ROS_QUEUE_SIZE_1, &HttpManager::resGetCorrectInfoCB, this);
    
    // To robot
    std::string pub_get_correct_info_topic_name;
    getParam(param_node, "get_correct_info/to_robot_topic_name", pub_get_correct_info_topic_name, std::string(DEF_TO_RBT_GET_CORRECT_INFO_TOPIC_NAME));
    pub_get_correct_info_  = node.advertise<uoa_poc6_msgs::r_map_pose_correct_info>("/" + robot_id_ + pub_get_correct_info_topic_name, ROS_QUEUE_SIZE_1, true); 
    

}


HttpManager::~HttpManager()
{
    delete(ego_map_);
    delete(socio_map_);
    delete(static_layer_map_);
    delete(semi_static_layer_map_);
    delete(exclusion_zone_layer_map_);
    delete(environment_map_);
    delete(reference_map_);
}

std::string HttpManager::check_map_revision(std::string map_revision)
{
    std::string return_map_revison = "";
    if((!map_revision.empty()))
    {
        return_map_revison = map_revision;
    }     
    else
    {
        ROS_WARN("The revision number of the map is empty.");
    }

    return return_map_revison;
}

//**************************************
// シミュレータ Reqest受信関連メソッド
//**************************************
void HttpManager::recvPutCorrectInfoCB(const uoa_poc6_msgs::r_put_map_pose_correct& msg)
{ // 補正値データの配信要求受信コールバック
    ROS_INFO("----------- Put CORRECT INFO START ------------");

    std::string source_map_revision = "";
    std::string target_map_revision = "";
    
    if(is_sb_put_map_pose_correct_value_)
    { // 配信データ受信済み
        // 比較元のリビジョン番号
        ROS_INFO_STREAM("Sorce map layer is " + msg.source_map_layer + ".");
        if(!source_map_revision_number_.empty())
        {
            ROS_WARN("The source map revision number linked to the correction value has been manually set.");
            source_map_revision = source_map_revision_number_;
        }
        else if(msg.source_map_layer == DEF_ENVIRONMENT_MAP_NAME)
        {
            source_map_revision = check_map_revision(put_map_revision_.environment_map_revision);

        }
        else if(msg.source_map_layer == DEF_REFERENCE_MAP_NAME)
        {
            source_map_revision = check_map_revision(put_map_revision_.reference_map_revision);
        }
        else if(msg.source_map_layer == DEF_STATIC_LAYER_MAP_NAME)
        {
            source_map_revision = check_map_revision(put_map_revision_.static_map_revision);
        }
        else if(msg.source_map_layer == DEF_QUASI_STATIC_LAYER_MAP_NAME)
        {
            source_map_revision = check_map_revision(put_map_revision_.q_static_map_revision);
        }
        else if(msg.source_map_layer == DEF_EXCLUSION_ZONE_LAYER_MAP_NAME)
        {
            source_map_revision = check_map_revision(put_map_revision_.no_entry_map_revision);
        }

        // 比較先のリビジョン番号
        ROS_INFO_STREAM("Target map layer is " + msg.target_map_layer + ".");
        if(!target_map_revision_number_.empty())
        {
            ROS_WARN("The target map revision number linked to the correction value has been manually set.");
            source_map_revision = target_map_revision_number_;
        }
        else if(msg.target_map_layer == DEF_ENVIRONMENT_MAP_NAME)
        {
            target_map_revision = check_map_revision(put_map_revision_.environment_map_revision);
        }
        else if(msg.target_map_layer == DEF_REFERENCE_MAP_NAME)
        {
            target_map_revision = check_map_revision(put_map_revision_.reference_map_revision);
        }
        else if(msg.target_map_layer == DEF_STATIC_LAYER_MAP_NAME)
        {
            target_map_revision = check_map_revision(put_map_revision_.static_map_revision);
        }
        else if(msg.target_map_layer == DEF_QUASI_STATIC_LAYER_MAP_NAME)
        {
            target_map_revision = check_map_revision(put_map_revision_.q_static_map_revision);
        }
        else if(msg.target_map_layer == DEF_EXCLUSION_ZONE_LAYER_MAP_NAME)
        {
            target_map_revision = check_map_revision(put_map_revision_.no_entry_map_revision);
        }

        // 配信データの設定
        uoa_poc6_msgs::r_req_put_map_pose_correct correct_info;

        correct_info.map_identities.source_map.system.revision  = source_map_revision;  // 補正値算出元の地図のリビジョン番号
        correct_info.map_identities.source_map.system.map_layer = msg.source_map_layer; // 補正値算出元の地図の種別情報
        correct_info.map_identities.source_map.header.space     = msg.source_map_space; // 補正値算出元の地図の空間情報
        correct_info.map_identities.target_map.system.revision  = target_map_revision;  // 補正値算出先の地図のリビジョン番号
        correct_info.map_identities.target_map.system.map_layer = msg.target_map_layer; // 補正値算出先の地図の種別情報
        correct_info.map_identities.target_map.header.space     = msg.target_map_space; // 補正値算出先の地図の空間情報
        correct_info.rmse = last_pose_correct_value_.rmse;   // 2乗平均誤差
        correct_info.relative_position = last_pose_correct_value_.pose;  // 補正値

        // 補正値情報の登録リクエスト
        reqPutCorrectionInfo(correct_info);

        ROS_INFO("----------- Put CORRECT INFO END ------------");

    }
    else
    { // 配信する補正値が未受信の場合
        ROS_WARN("The revision number of the environmental map is empty.");
    }

    return;
}

void HttpManager::recvGetObjectLocationCB(const uoa_poc6_msgs::r_get_objects_location& msg)
{ // 準静的物体の最新の位置情報の取得要求の受信
    ROS_INFO("----------- Get Object Location CallBack START ------------");
    
    // リクエストデータの作成
    uoa_poc6_msgs::r_req_change_obj_loc change_obj_loc;
    change_obj_loc.range        = msg.range;
    change_obj_loc.latest       = msg.latest;
    
    // リクエストデータの送信
    reqGetObjectLocation(change_obj_loc);

    ROS_INFO("----------- Get Object Location CallBack END ------------");

}

void HttpManager::recvMapCorrectInfoCB(const uoa_poc6_msgs::r_map_pose_correct_info& msg)
{ // RDRへ配信する補正値データの受信コールバック関数
    ROS_INFO_STREAM("Subscribed map correction values");
    last_pose_correct_value_ = msg; // 内部保持
    is_sb_put_map_pose_correct_value_ = true; // 受信フラグ
}

//**************************************
// ROBOT Reqest受信関連メソッド
//**************************************
void HttpManager::recvRoboInfoCB(const uoa_poc3_msgs::r_info& msg)
{ // ロボットの情報受信
    ROS_INFO("----------- Put ROBOT INFO START ------------");

    uoa_poc4_msgs::r_req_robot_info robot_info;
    // ロボットの情報格納
    robot_info.header.id                = msg.id;
    robot_info.header.type              = msg.type;
    robot_info.header.space             = space_info_;
    robot_info.header.time              = iso8601ex();
    robot_info.size.robot_radius        = msg.robot_size.robot_radius; 
    robot_info.size.inflation_radius    = msg.robot_size.inflation_radius;
    robot_info.size.footprint           = msg.robot_size.footprint;

    // ロボットの情報の通知
    reqRobotInfo(robot_info);

    ROS_INFO("----------- Put ROBOT INFO END ------------");

    return;

}

void HttpManager::recvPutMapdataCB(const uoa_poc5_msgs::r_put_mapdata& msg)
{   // ロボットの地図配信要求の受信
    ROS_INFO("----------- Put Map Data CallBack START ------------");

    // リクエストデータの作成
    nav_msgs::OccupancyGrid *pub_map_p = nullptr;
    uoa_poc4_msgs::r_req_put_mapdata put_mapdata;
    std::string linked_revision = "";

    put_mapdata.map_info.header.id          = msg.robot_id;
    put_mapdata.map_info.header.type        = msg.robot_type;
    put_mapdata.map_info.header.space       = space_info_;
    put_mapdata.map_info.location.location  = msg.location;
    put_mapdata.map_info.system.map_layer   = msg.map_layer;
    if(!obj_position_update_time_.empty())
    { // 生成した時間は準静的物体のレイアウト変化の検知時刻と紐付ける
        ros::Time update_time;
        strTimeToRosTime(update_time, obj_position_update_time_);
        put_mapdata.map_data.info.map_load_time = update_time;
        put_mapdata.map_info.header.time        = obj_position_update_time_;
    }
    else
    {  // 準静的物体のレイアウト変化の検知時間が未取得の場合は現在時刻を入れる
        ROS_WARN_STREAM("The associated layout change time has not been acquired yet. Instead, the current time will be stored.");
        put_mapdata.map_info.header.time        = iso8601ex();
    }

    if(msg.map_layer == DEF_EGO_MAP_NAME && is_sb_put_ego_map_)
    { // エゴ地図
        pub_map_p = ego_map_;

        // 地図に紐づく環境地図のリビジョン番号の設定
        if(!set_map_linked_revision_.empty())
        {
            ROS_WARN("The revision numbers linked to the map have been manually set.");
            linked_revision = set_map_linked_revision_;
        }
        else
        {
            linked_revision = check_map_revision(put_map_revision_.environment_map_revision);
        }
    }
    else if(msg.map_layer == DEF_SOCIO_MAP_NAME && is_sb_put_socio_map_)
    { // ソシオ地図
        pub_map_p = socio_map_;
        
        // 地図に紐づく環境地図のリビジョン番号の設定
        if(!set_map_linked_revision_.empty())
        {
            ROS_WARN("The revision numbers linked to the map have been manually set.");
            linked_revision = set_map_linked_revision_;
        }
        else
        {
            linked_revision = check_map_revision(put_map_revision_.environment_map_revision);
        }
    }
    else if(msg.map_layer == DEF_STATIC_LAYER_MAP_NAME && is_sb_put_static_layer_)
    { // 静的レイヤ地図
        pub_map_p = static_layer_map_;
        
        // 地図に紐づく環境地図のリビジョン番号の設定
        if(!set_map_linked_revision_.empty())
        {
            ROS_WARN("The revision numbers linked to the map have been manually set.");
            linked_revision = set_map_linked_revision_;
        }
        else
        {
            linked_revision = check_map_revision(put_map_revision_.environment_map_revision);
        }
    }
    else if(msg.map_layer == DEF_QUASI_STATIC_LAYER_MAP_NAME && is_sb_put_quasi_static_layer_)
    { // 準静的レイヤ地図
        pub_map_p = semi_static_layer_map_;
        
        // 地図に紐づく環境地図のリビジョン番号の設定
        if(!set_map_linked_revision_.empty())
        {
            ROS_WARN("The revision numbers linked to the map have been manually set.");
            linked_revision = set_map_linked_revision_;
        }
        else
        {
            linked_revision = check_map_revision(put_map_revision_.environment_map_revision);
        }
    }
    else if(msg.map_layer == DEF_EXCLUSION_ZONE_LAYER_MAP_NAME && is_sb_put_exclusion_zone_layer_)
    { // 進入禁止レイヤ地図
        pub_map_p = exclusion_zone_layer_map_;
        
        // 地図に紐づく環境地図のリビジョン番号の設定
        if(!set_map_linked_revision_.empty())
        {
            ROS_WARN("The revision numbers linked to the map have been manually set.");
            linked_revision = set_map_linked_revision_;
        }
        else
        {
            linked_revision = check_map_revision(put_map_revision_.environment_map_revision);
        }
    }    
    else if(msg.map_layer == DEF_ENVIRONMENT_MAP_NAME && is_sb_put_environment_map_)
    { // 環境地図
        pub_map_p = environment_map_;
    }
    else if(msg.map_layer == DEF_REFERENCE_MAP_NAME && is_sb_put_reference_map_)
    { // リファレンス地図
        pub_map_p = reference_map_;
    }

    if(!pub_map_p)
    { // 配信する地図データが存在しない場合
        ROS_WARN("The map delivery was ignored because the map is undefined or has already been delivered");
    }
    else
    { // 配信する地図データが存在する場合
        ROS_INFO("Put %s maps to RDR. ", msg.map_layer.c_str());

        put_mapdata.map_info.system.linked_revision = linked_revision;
        put_mapdata.map_data = *pub_map_p;
        put_mapdata.is_format_location      = true;
        put_mapdata.is_use_latest           = true;

        // リクエストデータの送信
        reqPutMapdata(put_mapdata);
    }

    ROS_INFO("----------- Put Layer Map Data CallBack END ------------");
}

void HttpManager::recvGetMapdataCB(const uoa_poc5_msgs::r_get_mapdata& msg)
{   // ロボットの地図取得要求の受信
    ROS_INFO("----------- Get Map Data CallBack START ------------");

    // リクエストデータの作成
    uoa_poc4_msgs::r_req_get_mapdata get_mapdata;
    get_mapdata.info.header.time        = msg.time;
    get_mapdata.info.header.space       = msg.space;
    get_mapdata.info.system.revision    = msg.revision;
    get_mapdata.info.system.latest      = msg.latest;
    get_mapdata.info.system.map_layer   = msg.map_layer;
    get_mapdata.info.system.data_type   = DEF_SYSTEM_TYPE;
    get_mapdata.info.location.location  = msg.location;

    get_mapdata.is_format_location      = true;
    get_mapdata.is_use_latest           = true;

    // リクエストデータの送信
    reqGetMapdata(get_mapdata);

    ROS_INFO("----------- Get Map Data CallBack END ------------");
}

void HttpManager::recvGetLayerMapdataCB(const uoa_poc5_msgs::r_get_mapdata& msg)
{ // ロボットのレイヤ地図取得要求の受信
    ROS_INFO("----------- Get Layer Map Data CallBack START ------------");
    
    // リクエストデータの作成
    uoa_poc4_msgs::r_req_get_mapdata get_mapdata;
    get_mapdata.info.system.revision    = msg.revision;
    get_mapdata.info.system.map_layer   = msg.map_layer;
    
    // リクエストデータの送信
    reqGetLayerMapdata(get_mapdata);

    ROS_INFO("----------- Get Layer Map Data CallBack END ------------");

}

void HttpManager::recvGetPositiondataCB(const uoa_poc5_msgs::r_get_position_data& msg)
{ // ロボットの現在位置取得要求の受信
    ROS_INFO("----------- Get Position Data CallBack START ------------");
    
    // リクエストデータの作成
    uoa_poc4_msgs::r_req_get_position_data get_positiondata;
    get_positiondata.header.name = robot_id_;
    ROS_INFO_STREAM("get_positiondata.header.name " + std::string(robot_id_));

       
    // リクエストデータの送信
    reqGetPositiondata();
    
    ROS_INFO("----------- Get Position Data CallBack END ------------");

}

void HttpManager::recvMapCB(const nav_msgs::OccupancyGridConstPtr& msg, std::string map_layer)
{

    unsigned int map_size = msg->info.width * msg->info.height;
    nav_msgs::OccupancyGrid *map_p = nullptr;

    // 各地図のポインタを設定
    if(map_layer == DEF_EGO_MAP_NAME)
    {
        // ROS_INFO_STREAM("Subscribed" + std::string(DEF_EGO_MAP_NAME)   + " Map");

        is_sb_put_ego_map_  = true;
        map_p = ego_map_;
    }
    else if(map_layer == DEF_SOCIO_MAP_NAME)
    {
        // ROS_INFO_STREAM("Subscribed " + std::string(DEF_SOCIO_MAP_NAME) + " Map");

        is_sb_put_socio_map_    = true;
        map_p = socio_map_;
    }
    else if(map_layer == DEF_STATIC_LAYER_MAP_NAME)
    {
        ROS_INFO_STREAM("Subscribed " + std::string(DEF_STATIC_LAYER_MAP_NAME) + " Map");

        is_sb_put_static_layer_ = true;
        map_p = static_layer_map_;
    
    }
    else if(map_layer == DEF_QUASI_STATIC_LAYER_MAP_NAME)
    {
        ROS_INFO_STREAM("Subscribed " + std::string(DEF_QUASI_STATIC_LAYER_MAP_NAME) + " Map");

        is_sb_put_quasi_static_layer_   = true;
        map_p = semi_static_layer_map_;

    }
    else if(map_layer == DEF_EXCLUSION_ZONE_LAYER_MAP_NAME)
    {
        ROS_INFO_STREAM("Subscribed " + std::string(DEF_EXCLUSION_ZONE_LAYER_MAP_NAME) + " Map");

        is_sb_put_exclusion_zone_layer_ = true;
        map_p = exclusion_zone_layer_map_;

    }
    else if(map_layer == DEF_ENVIRONMENT_MAP_NAME)
    {
        ROS_INFO_STREAM("Subscribed " + std::string(DEF_ENVIRONMENT_MAP_NAME) + " Map");

        is_sb_put_environment_map_ = true;
        map_p = environment_map_;

    }
    else if(map_layer == DEF_REFERENCE_MAP_NAME)
    {
        ROS_INFO_STREAM("Subscribed " + std::string(DEF_REFERENCE_MAP_NAME) + " Map");

        is_sb_put_reference_map_ = true;
        map_p = reference_map_;

    }
    else 
    {
        ROS_WARN("Undefined map retrieved.");
    }

    // データ格納
    if(map_p != nullptr)
    {   // ポインタがNULLじゃない場合（設定できている場合）
        map_p->header = msg->header;
        map_p->info = msg->info;

        map_p->data.resize(map_size);
        map_p->data = msg->data;

    }


}

void HttpManager::recvGetCorrectInfoCB(const uoa_poc6_msgs::r_get_map_pose_correct& msg)
{ // 補正値の取得要求の受信CallBack関数
    ROS_INFO("----------- Get Correct Info CallBack START ------------");
    
    // リクエストデータの作成
    uoa_poc6_msgs::r_req_get_map_pose_correct get_correct_info;
    
    get_correct_info.map_identities.source_map.system.revision  = msg.map_identities.source_map.system.revision;
    get_correct_info.map_identities.source_map.system.map_layer = msg.map_identities.source_map.system.map_layer;
    get_correct_info.map_identities.source_map.header.space     = msg.map_identities.source_map.header.space;
    get_correct_info.map_identities.target_map.system.revision  = msg.map_identities.target_map.system.revision;
    get_correct_info.map_identities.target_map.system.map_layer = msg.map_identities.target_map.system.map_layer;
    get_correct_info.map_identities.target_map.header.space     = msg.map_identities.target_map.header.space;
       
    // リクエストデータの送信
    reqGetCorrectionInfo(get_correct_info);
    
    ROS_INFO("----------- Get Correct Info CallBack END ------------");

}

//**************************************
// HTTP Reqest受信関連メソッド
//**************************************
void HttpManager::reqRobotInfo(uoa_poc4_msgs::r_req_robot_info robot_info)
{ // ロボットの情報の通知

    // データのチェック
    if( !robot_info.header.id.empty() &&
        !robot_info.header.type.empty() &&
        !robot_info.header.space.empty() &&
        !robot_info.header.time.empty() &&
        !robot_info.size.robot_radius + DBL_EPSILON > 0.0  &&
        !robot_info.size.inflation_radius + DBL_EPSILON > 0.0)
    { // 必要なデータは格納されている

        ROS_INFO("----------- REQEST PUT ROBOT INFO ------------");

        // http_bridgeへパブリッシュ
        pub_req_robot_info_.publish(robot_info);

    }
#if DEBUG
    else
    {
        ROS_WARN("reqRobotInfo : Param Error !!");
    }
#endif

    return;
}

void HttpManager::reqMapdataList(uoa_poc4_msgs::r_req_mapdata_list map_list)
{ // 最新10件の地図データのリスト取得のリクエスト

    if( !map_list.info.header.id.empty() &&
        !map_list.info.header.type.empty() &&
        !map_list.info.location.location.empty() &&
        !map_list.info.system.map_layer.empty())
    { // 必要なデータは格納されている

        ROS_INFO("----------- REQEST GET MAP DATA LIST ------------");

        // http_bridgeへパブリッシュ
        pub_req_map_data_list_.publish(map_list);

    }
#if DEBUG
    else
    {
        ROS_WARN("reqMapdataList : Param Error !!");
    }
#endif

    return;
}

void HttpManager::reqPutMapdata(uoa_poc4_msgs::r_req_put_mapdata put_mapdata)
{

    if( !put_mapdata.map_info.header.id.empty() &&
        !put_mapdata.map_info.header.type.empty() &&
        !put_mapdata.map_info.header.space.empty() &&
        !put_mapdata.map_info.header.time.empty() &&
        !put_mapdata.map_info.location.location.empty() &&
        !put_mapdata.map_info.system.map_layer.empty() &&
        (is_sb_put_ego_map_ || is_sb_put_socio_map_ || is_sb_put_reference_map_ || is_sb_put_environment_map_ || is_sb_put_static_layer_ || is_sb_put_quasi_static_layer_ || is_sb_put_exclusion_zone_layer_) )
    {
        
        ROS_INFO("----------- REQEST PUT MAP DATA ------------");
        // パブリッシュ
        pub_req_put_map_data_.publish(put_mapdata);

        // フラグオフ
        if(put_mapdata.map_info.system.map_layer == DEF_EGO_MAP_NAME)
        { // エゴ地図
            is_sb_put_ego_map_ = false;
        }
        else if(put_mapdata.map_info.system.map_layer == DEF_SOCIO_MAP_NAME)
        { // ソシオ地図
            is_sb_put_socio_map_ = false;
        }
        else if(put_mapdata.map_info.system.map_layer == DEF_STATIC_LAYER_MAP_NAME)
        { // 静的レイヤ地図
            is_sb_put_static_layer_ = false;
        }
        else if(put_mapdata.map_info.system.map_layer == DEF_QUASI_STATIC_LAYER_MAP_NAME)
        { // 準静的レイヤ地図
            is_sb_put_quasi_static_layer_ = false;
        }
        else if(put_mapdata.map_info.system.map_layer == DEF_EXCLUSION_ZONE_LAYER_MAP_NAME)
        { // 進入禁止レイヤ地図
            is_sb_put_exclusion_zone_layer_ = false;
        }
        else if(put_mapdata.map_info.system.map_layer == DEF_ENVIRONMENT_MAP_NAME)
        { // 環境地図
            is_sb_put_environment_map_ = false;
        }
        else if(put_mapdata.map_info.system.map_layer == DEF_REFERENCE_MAP_NAME)
        { // リファレンス地図
            is_sb_put_reference_map_ = false;
        }
        else
        { // どの地図でもない
            ROS_WARN("Undefined map put reqest.");
        }
    }
#if DEBUG
    else
    {
        ROS_WARN("reqPutMapdata : Param Error !!");
    }
#endif

    return;
}

void HttpManager::reqGetMapdata(uoa_poc4_msgs::r_req_get_mapdata map_info)
{
     // データのチェック
    if( !map_info.info.system.data_type.empty() )
    { // 必要なデータは格納されている
    
        // パブリッシュ
        pub_req_get_map_data_.publish(map_info);

        // 最後に配信したデータの更新
        map_info_ = map_info; // 注:バッファは1個のみのため、制御側で順番に取得する必要がある
    
    }
#if DEBUG
    else
    {
        ROS_WARN("reqGetMapdata : Param Error !!");
    }
#endif 

    return;
}

void HttpManager::reqGetLayerMapdata(uoa_poc4_msgs::r_req_get_mapdata map_info)
{
    // データのチェック
    if( !map_info.info.system.map_layer.empty() &&
        !map_info.info.system.revision.empty())
    { // 必要なデータは格納されている
    
        // パブリッシュ
        pub_req_get_layer_map_data_.publish(map_info);
    
    }
#if DEBUG
    else
    {
        ROS_WARN("reqGetLayerMapdata : Param Error !!");
    }
#endif 

    return;
}

void HttpManager::reqGetPositiondata()
{ // ロボットの位置取得

    // リクエストメッセージデータの作成 
    uoa_poc4_msgs::r_req_get_position_data req_msg;
    req_msg.header.name = get_robot_position_name_;
    
    // HTTPブリッジノードへリクエストメッセージ送信
    pub_req_get_position_data_.publish(req_msg);
    
    return;
}

void HttpManager::reqPutCorrectionInfo(const uoa_poc6_msgs::r_req_put_map_pose_correct& correct_info)
{ // 地図の補正値の登録要求
    // データのチェック
    if( !correct_info.map_identities.source_map.system.map_layer.empty() && // 補正値算出元の地図の種別情報が入ってる
        !correct_info.map_identities.source_map.header.space.empty() && // 補正値算出元の地図の空間情報が入ってる
        !correct_info.map_identities.target_map.system.map_layer.empty() && // 補正値算出先の地図の種別情報が入ってる
        !correct_info.map_identities.target_map.header.space.empty() && // 補正値算出先の地図の空間情報が入ってる
        // !correct_info.relative_position &&  // 補正値が入っている
        !(correct_info.rmse < 0) ) // 2乗平均誤差がマイナス値ではない
    { // 必要なデータは格納されている
    
        // パブリッシュ
        pub_req_put_correct_info_.publish(correct_info);
    
    }
#if DEBUG
    else
    {
        ROS_WARN("reqPutCorrectionInfo : Param Error !!");
    }
#endif 

    return;
}

void HttpManager::reqGetObjectLocation(uoa_poc6_msgs::r_req_change_obj_loc change_obj_loc)
{ // 準静的物体の位置情報取得要求
    // データのチェック
    // if( !change_obj_loc.range.start.empty()  &&
    //     !change_obj_loc.range.end.empty())
    { // 必要なデータは格納されている

        // パブリッシュ
        pub_req_get_object_location_.publish(change_obj_loc);
    
    }
#if DEBUG
    // else
    // {
    //     ROS_WARN("reqGetObjectLocation : Param Error !!");
    // }
#endif 

    return;
}

void HttpManager::reqGetCorrectionInfo(const uoa_poc6_msgs::r_req_get_map_pose_correct& get_correct_info)
{ // 地図の補正値の登録要求
    // データのチェック
    // if( !get_correct_info.map_identities.source_map.system.map_layer.empty() && // 補正値算出元の地図の種別情報が入ってる
    //     !get_correct_info.map_identities.source_map.header.space.empty() && // 補正値算出元の地図の空間情報が入ってる
    //     !get_correct_info.map_identities.target_map.system.map_layer.empty() && // 補正値算出先の地図の種別情報が入ってる
    //     !get_correct_info.map_identities.target_map.header.space.empty() ) // 補正値算出先の地図の空間情報が入ってる
    // { // 必要なデータは格納されている
    
        // パブリッシュ
        pub_req_get_correct_info_.publish(get_correct_info);
    
//     }
// #if DEBUG
//     else
//     {
//         ROS_WARN("reqGetCorrectionInfo : Param Error !!");
//     }
// #endif 

    return;
}

//**************************************
// HTTP Response受信関連メソッド
//**************************************
void HttpManager::resRobotInfoCB(const uoa_poc4_msgs::r_res_robot_info& msg)
{ // ロボットの情報通知結果

    ROS_INFO("--------- RobotInfo Response msg START --------- ");
    
    if( msg.status_code == 200)
    { // ステータスコードが200番（正常受理）の場合
        ROS_INFO("Status Code : %d", msg.status_code);
    }
    else if(msg.status_code == 400)
    { // ステータスコードが400番（構文エラーによりリクエストが受理されなかった）の場合
        ROS_INFO("Status Code : %d", msg.status_code);
        ROS_WARN("There was a syntax error and the data could not be retrieved.");
    }
    else if(msg.status_code == 500)
    { // ステータスコードが500番（サーバーに接続出来なかったまたはサーバー側のエラー）の場合
        ROS_INFO("Status Code : %d", msg.status_code);
        ROS_WARN("Could not connect to the server for some reason.");

        // 最後に送信したデータでリトライ
        
        // ros::Duration(retry_time_).sleep();
        // reqRobotInfo(last_req_robot_info_);

    }
    else
    {
        ROS_INFO("Status Code : %d", msg.status_code);
        ROS_WARN("The process is not complete due to other errors.");
    }

    ROS_INFO("--------- RobotInfo Response msg END --------- ");
    
    return;
}

void HttpManager::resMapdataListCB(const uoa_poc4_msgs::r_res_mapdata_list& msg)
{ // 地図のデータリストのリクエスト結果

    ROS_INFO("--------- MapDataList Response msg START ---------");

    if( msg.status_code == 200)
    { // ステータスコードが200番（正常受理）の場合
        ROS_INFO("Status Code : %d", msg.status_code);

        // 最新の地図のリビジョン番号の取得
        // last_revision_ = msg.list.front().system.revision;

        for(const auto& e : msg.list)
        {
            // if(e.header.id == robot_id_)
            // { // ロボット固有の地図か確認する
            //     if(e.location)

            // }
            // showHeaderValue(e.header);
            // showLocationInfo(e.location);
            // showMapSystemInfo(e.system);
        }

    }
    else if(msg.status_code == 204)
    { // ステータスコードが204番（リクエストは受理されたが、該当データが無かった）の場合
        // ログ通知
        ROS_INFO("Status Code : %d", msg.status_code);
        ROS_WARN("No data matching the condition existed in the RDR.");
    }
    else if(msg.status_code == 400)
    { // ステータスコードが400番（構文エラーによりリクエストが受理されなかった）の場合
        ROS_INFO("Status Code : %d", msg.status_code);
        ROS_WARN("There was a syntax error and the data could not be retrieved.");
    }
    else if(msg.status_code == 500)
    { // ステータスコードが400番（サーバーに接続出来なかったまたはサーバー側のエラー）の場合
        ROS_INFO("Status Code : %d", msg.status_code);
        ROS_WARN("Could not connect to the server for some reason.");

        // リトライ
        
    }
    else
    {
        ROS_INFO("Status Code : %d", msg.status_code);
        ROS_WARN("The process is not complete due to other errors.");
    }
    
    ROS_INFO("--------- MapDataList Response msg END ---------");
    
    return;
}

void HttpManager::resPutMapdataCB(const uoa_poc4_msgs::r_res_put_mapdata& msg)
{

#if DEBUG
    ROS_INFO("--------- PutMapData Response msg START ---------");
#endif
    if( msg.status_code == 200)
    { // ステータスコードが200番（正常受理）の場合
        ROS_INFO("Status Code : %d", msg.status_code);

        // レスポンスメッセージの内容表示
        // showHeaderValue(msg.info.header);
        // showLocationInfo(msg.info.location);
        // showMapSystemInfo(msg.info.system);

        // リビジョン番号の管理
        if(msg.info.system.map_layer == DEF_ENVIRONMENT_MAP_NAME)
        {   // 環境地図
            put_map_revision_.environment_map_revision = msg.info.system.revision;
        }
        else if(msg.info.system.map_layer == DEF_REFERENCE_MAP_NAME)
        {   // リファレンス地図
            put_map_revision_.reference_map_revision = msg.info.system.revision;
        }
        else if(msg.info.system.map_layer == DEF_STATIC_LAYER_MAP_NAME)
        {   // 静的レイヤ地図
            put_map_revision_.static_map_revision = msg.info.system.revision;
        }
        else if(msg.info.system.map_layer == DEF_QUASI_STATIC_LAYER_MAP_NAME)
        {   // 準静的レイヤ地図
            put_map_revision_.q_static_map_revision = msg.info.system.revision;
        }
        else if(msg.info.system.map_layer == DEF_EXCLUSION_ZONE_LAYER_MAP_NAME)
        {   // 進入禁止レイヤ地図
            put_map_revision_.no_entry_map_revision = msg.info.system.revision;
        }
        else 
        {
            ROS_WARN("Received revision number for unknown map (name: %s, revision: %s).", msg.info.system.map_layer.c_str(), msg.info.system.revision.c_str());
        }
        // msg.info.header.time;
    }
    else if(msg.status_code == 400)
    { // ステータスコードが400番（構文エラーによりリクエストが受理されなかった）の場合
        ROS_INFO("Status Code : %d", msg.status_code);
        ROS_WARN("There was a syntax error and the data could not be retrieved.");
    }
    else if(msg.status_code == 500)
    { // ステータスコードが500番（サーバーに接続出来なかったまたはサーバー側のエラー）の場合
        ROS_INFO("Status Code : %d", msg.status_code);
        ROS_WARN("Could not connect to the server for some reason.");

        // リトライ
        
    }
    else
    {
        ROS_INFO("Status Code : %d", msg.status_code);
        ROS_WARN("The process is not complete due to other errors.");
    }

#if DEBUG    
    ROS_INFO("--------- PutMapData Response msg END ---------");
#endif

    return;
}

void HttpManager::resGetMapdataCB(const uoa_poc4_msgs::r_res_get_mapdata& msg)
{

#if DEBUG
    ROS_INFO("--------- GetMapData Response CallBack START ---------");
#endif
    if( msg.status_code == 200 )
    { // ステータスコードが200番（正常受理）の場合
        ROS_INFO("Status Code : %d", msg.status_code);
        
        // レスポンスメッセージの内容表示
        // showHeaderValue(msg.map_info.header);
        // showLocationInfo(msg.map_info.location);
        // showMapSystemInfo(msg.map_info.system);
        // showMapDataInfo(msg.map_data);

        // 内部地図の更新
        nav_msgs::OccupancyGrid acquired_map;
        uoa_poc5_msgs::r_recv_map_info info;
        
        info.revision = msg.map_info.system.revision;
        info.map_layer = msg.map_info.system.map_layer;
        pub_recv_map_info_.publish(info);

        acquired_map.header.frame_id = tf_prefix_ + map_frame_id_;
        acquired_map.header.stamp = ros::Time::now();
        acquired_map.info = msg.map_data.info;
        acquired_map.data.resize(msg.map_data.data.size());
        acquired_map.data = msg.map_data.data;

        if(msg.map_info.system.map_layer == DEF_EGO_MAP_NAME)
        {
            pub_ego_map_data_.publish(acquired_map);
        }
        else if(msg.map_info.system.map_layer == DEF_SOCIO_MAP_NAME)
        {
            pub_socio_map_data_.publish(acquired_map);
        }
        else if(msg.map_info.system.map_layer == DEF_STATIC_LAYER_MAP_NAME)
        {
            pub_static_layer_map_data_.publish(acquired_map);
        }
        else if(msg.map_info.system.map_layer == DEF_QUASI_STATIC_LAYER_MAP_NAME)
        {
            pub_quasi_static_layer_map_data_.publish(acquired_map);
        }
        else if(msg.map_info.system.map_layer == DEF_EXCLUSION_ZONE_LAYER_MAP_NAME)
        {
            pub_exclusion_zone_layer_map_data_.publish(acquired_map);
        }
        else if(msg.map_info.system.map_layer == DEF_ENVIRONMENT_MAP_NAME)
        {
            pub_environment_map_data_.publish(acquired_map);
        }
        else if(msg.map_info.system.map_layer == DEF_REFERENCE_MAP_NAME)
        {
            pub_reference_map_data_.publish(acquired_map);
        }
        else
        { // 存在しない地図の型
            std::string warn_msg = "Map \"" + msg.map_info.system.map_layer + "\" is not used by this robot.";
            ROS_WARN_STREAM(warn_msg);
        }
    }
    else if(msg.status_code == 204)
    { // ステータスコードが204番（リクエストは受理されたが、該当データが無かった）の場合
        // ログ通知
        ROS_INFO("Status Code : %d", msg.status_code);
        ROS_WARN("No data matching the condition existed in the RDR.");
    }
    else if(msg.status_code == 400)
    { // ステータスコードが400番（構文エラーによりリクエストが受理されなかった）の場合
        ROS_INFO("Status Code : %d", msg.status_code);
        ROS_WARN("There was a syntax error and the data could not be retrieved.");
    }
    else if(msg.status_code == 500)
    { // ステータスコードが400番（サーバーに接続出来なかったまたはサーバー側のエラー）の場合
        ROS_INFO("Status Code : %d", msg.status_code);
        ROS_WARN("Could not connect to the server for some reason.");
        
        // リトライ処理
        // for(unsigned int idx = 0; idx < max_retry_count; idx++)
        // {
        //     // 最終データの配信
        //     // リトライ頻度調整
        // }
    }
    else
    {
        ROS_INFO("Status Code : %d", msg.status_code);
        ROS_WARN("The process is not complete due to other errors.");
    }
    
#if DEBUG
    ROS_INFO("--------- GetMapData Response CallBack END ---------");
#endif
    return;
}

void HttpManager::resReqPositiondataCB(const uoa_poc5_msgs::r_res_get_position_data& msg)
{ // ロボットの位置取得

#if DEBUG
    ROS_INFO("--------- GetPosition Response CallBack START ---------");
#endif
    // unsigned int max_retry_count = retry_get_position_times_;    // 最大リトライ回数
    // double interval_time = retry_get_position_interval_;         // リトライ間隔
    
    
    if( msg.status_code == 200 )
    { // ステータスコードが200番（正常受理）の場合
#if DEBUG
        ROS_INFO("Status Code : %d", msg.status_code);
#endif
        // if(msg.name == robot_name_)
        // {
            // 受信時刻の検査(未実装)
            // strptime();

            // 位置データの格納 
            geometry_msgs::PoseWithCovarianceStamped position_msg;
            position_msg.header.frame_id = tf_prefix_ + map_frame_id_;
            position_msg.header.stamp    = ros::Time::now();
            position_msg.pose.pose       = msg.pose;

            if(msg.pose.orientation.z == 0.0 &&
               msg.pose.orientation.w == 0.0)
            {
                position_msg.pose.pose.orientation.w = 1.0;
            }

            memset(&position_msg.pose.covariance[0], 0, sizeof(position_msg.pose.covariance));  //共分散行列（散らばり具合を表す指標）
            position_msg.pose.covariance[0] = 0.25;
            position_msg.pose.covariance[7] = 0.25;
            position_msg.pose.covariance[35] = 0.06853891945200942;

            // 初期位置メッセージの送信
            pub_initial_position_data_.publish(position_msg);
        // }
        // else
        // { // 存在しない地図の型
        //     std::string warn_msg = "It's not a robot named  \"" + msg.name + "\".";
        //     ROS_WARN_STREAM(warn_msg);
        // }
    }
    else if(msg.status_code == 204)
    { // ステータスコードが204番（リクエストは受理されたが、該当データが無かった）の場合
        // ログ通知
        ROS_INFO("Status Code : %d", msg.status_code);
        ROS_WARN("No data matching the condition existed in the RDR.");
        if(cnt_retry_get_position > 0)
        {
            reqGetPositiondata();

            cnt_retry_get_position--;
        }
    }
    else if(msg.status_code == 400)
    { // ステータスコードが400番（構文エラーによりリクエストが受理されなかった）の場合
        ROS_INFO("Status Code : %d", msg.status_code);
        ROS_WARN("There was a syntax error and the data could not be retrieved.");
    }
    else if(msg.status_code == 500)
    { // ステータスコードが400番（サーバーに接続出来なかったまたはサーバー側のエラー）の場合
        ROS_INFO("Status Code : %d", msg.status_code);
        ROS_WARN("Could not connect to the server for some reason.");

        // リトライ処理
        // for(unsigned int idx = 0; idx < max_retry_count; idx++)
        // {
        //     // 最終データの配信

        //     // リトライ頻度調整

        // }
    }
    else
    {
        ROS_INFO("Status Code : %d", msg.status_code);
        ROS_WARN("The process is not complete due to other errors.");
    }

#if DEBUG
    ROS_INFO("--------- GetPosition Response CallBack END ---------");
#endif

}

void HttpManager::resPutCorrectInfoCB(const uoa_poc6_msgs::r_res_put_map_pose_correct& msg)
{ // 補正値登録要求の結果

#if DEBUG
    ROS_INFO("--------- PutCorrectionInfo Response CallBack START ---------");
#endif
    // unsigned int max_retry_count = retry_get_position_times_;    // 最大リトライ回数
    // double interval_time = retry_get_position_interval_;         // リトライ間隔
    
    
    if( msg.status_code == 200 )
    { // ステータスコードが200番（正常受理）の場合
        ROS_INFO("Status Code : %d", msg.status_code);
    }
    else if(msg.status_code == 204)
    { // ステータスコードが204番（リクエストは受理されたが、該当データが無かった）の場合
        // ログ通知
        ROS_INFO("Status Code : %d", msg.status_code);
        ROS_WARN("No data matching the condition existed in the RDR.");
    }
    else if(msg.status_code == 400)
    { // ステータスコードが400番（構文エラーによりリクエストが受理されなかった）の場合
        ROS_INFO("Status Code : %d", msg.status_code);
        ROS_WARN("There was a syntax error and the data could not be retrieved.");
    }
    else if(msg.status_code == 500)
    { // ステータスコードが400番（サーバーに接続出来なかったまたはサーバー側のエラー）の場合
        ROS_INFO("Status Code : %d", msg.status_code);
        ROS_WARN("Could not connect to the server for some reason.");

        // リトライ処理
        // for(unsigned int idx = 0; idx < max_retry_count; idx++)
        // {
        //     // 最終データの配信

        //     // リトライ頻度調整

        // }
    }
    else
    {
        ROS_INFO("Status Code : %d", msg.status_code);
        ROS_WARN("The process is not complete due to other errors.");
    }

#if DEBUG
    ROS_INFO("--------- PutCorrectionInfo Response CallBack END ---------");
#endif

}

void HttpManager::resObjectLocationCB(const uoa_poc6_msgs::r_res_change_obj_loc& msg)
{ // ロボットの位置取得

#if DEBUG
    ROS_INFO("--------- GetObjectLocation Response CallBack START ---------");
#endif
    // unsigned int max_retry_count = retry_get_position_times_;    // 最大リトライ回数
    // double interval_time = retry_get_position_interval_;         // リトライ間隔
    
    
    if( msg.status_code == 200 )
    { // ステータスコードが200番（正常受理）の場合
#if DEBUG
        ROS_INFO("Status Code : %d", msg.status_code);
#endif

        // 物体認識側のレイアウト変化検知時間の取得
        obj_position_update_time_ = msg.update_time;

        // 位置データの格納 
        uoa_poc6_msgs::r_objects_location pub_msg;
        pub_msg.objects.resize(msg.objects.size());
        pub_msg.objects = msg.objects;

        // 初期位置メッセージの送信
        pub_get_object_location_.publish(pub_msg);
    }
    else if(msg.status_code == 204)
    { // ステータスコードが204番（リクエストは受理されたが、該当データが無かった）の場合
        // ログ通知
        ROS_INFO("Status Code : %d", msg.status_code);
        ROS_WARN("No data matching the condition existed in the RDR.");
    }
    else if(msg.status_code == 400)
    { // ステータスコードが400番（構文エラーによりリクエストが受理されなかった）の場合
        ROS_INFO("Status Code : %d", msg.status_code);
        ROS_WARN("There was a syntax error and the data could not be retrieved.");
    }
    else if(msg.status_code == 500)
    { // ステータスコードが400番（サーバーに接続出来なかったまたはサーバー側のエラー）の場合
        ROS_INFO("Status Code : %d", msg.status_code);
        ROS_WARN("Could not connect to the server for some reason.");

        // リトライ処理
        // for(unsigned int idx = 0; idx < max_retry_count; idx++)
        // {
        //     // 最終データの配信

        //     // リトライ頻度調整

        // }
    }
    else
    {
        ROS_INFO("Status Code : %d", msg.status_code);
        ROS_WARN("The process is not complete due to other errors.");
    }

#if DEBUG
    ROS_INFO("--------- GetPosition Response CallBack END ---------");
#endif

}

void HttpManager::resGetCorrectInfoCB(const uoa_poc6_msgs::r_res_get_map_pose_correct& msg)
{ // 補正値取得要求の結果

#if DEBUG
    ROS_INFO("--------- GetCorrectionInfo Response CallBack START ---------");
#endif
    // unsigned int max_retry_count = retry_get_position_times_;    // 最大リトライ回数
    // double interval_time = retry_get_position_interval_;         // リトライ間隔
    
    
    if( msg.status_code == 200 )
    { // ステータスコードが200番（正常受理）の場合
        ROS_INFO("Status Code : %d", msg.status_code);
        // 位置データの格納 
        uoa_poc6_msgs::r_map_pose_correct_info pub_msg;
        pub_msg.header.stamp = ros::Time::now();
        pub_msg.header.frame_id = tf_prefix_ + map_frame_id_;
        pub_msg.pose = msg.relative_position;
        pub_msg.rmse = msg.rmse;

        // 初期位置メッセージの送信
        pub_get_correct_info_.publish(pub_msg);
    }
    else if(msg.status_code == 204)
    { // ステータスコードが204番（リクエストは受理されたが、該当データが無かった）の場合
        // ログ通知
        ROS_INFO("Status Code : %d", msg.status_code);
        ROS_WARN("No data matching the condition existed in the RDR.");
    }
    else if(msg.status_code == 400)
    { // ステータスコードが400番（構文エラーによりリクエストが受理されなかった）の場合
        ROS_INFO("Status Code : %d", msg.status_code);
        ROS_WARN("There was a syntax error and the data could not be retrieved.");
    }
    else if(msg.status_code == 500)
    { // ステータスコードが400番（サーバーに接続出来なかったまたはサーバー側のエラー）の場合
        ROS_INFO("Status Code : %d", msg.status_code);
        ROS_WARN("Could not connect to the server for some reason.");

        // リトライ処理
        // for(unsigned int idx = 0; idx < max_retry_count; idx++)
        // {
        //     // 最終データの配信

        //     // リトライ頻度調整

        // }
    }
    else
    {
        ROS_INFO("Status Code : %d", msg.status_code);
        ROS_WARN("The process is not complete due to other errors.");
    }

#if DEBUG
    ROS_INFO("--------- GetCorrectInfo Response CallBack END ---------");
#endif

}
