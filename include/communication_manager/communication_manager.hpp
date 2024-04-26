/**
* @file     communication_manager.hpp
* @brief    CommunicationManagerの定義ソースファイル
* @author   S.Kumada
* @date     2023/12/7
* @note     ロボットとRDR、上位系間のデータのやり取りに関して、メッセージのフォーマット変換やデータの精査等を行うクラスの定義
*/

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

// 共通関数定義
#include "communication_manager/utilitie.h"

// 各マネージャクラス
#include "http_manager.hpp"
#include "mqtt_manager.hpp"

// RDR通信結合用カスタムメッセージ定義
#include "uoa_poc3_msgs/r_state.h"
#include "uoa_poc4_msgs/r_pub_robot_state.h"
#include "uoa_poc4_msgs/r_pub_local_path_plan.h"
#include "uoa_poc4_msgs/r_sub_map_update_notify.h"
#include "uoa_poc5_msgs/r_pub_laser_scan.h"
#include "uoa_poc5_msgs/r_pub_point_cloud.h"


// define定義
#define DEF_RETRY_TIME          5.0 // 5秒
#define DEF_RETRY_COUNT         5 // 5回
#define DEF_EGO_MAP_NAME        "ego"
#define DEF_SOCIO_MAP_NAME      "socio"
#define DEF_MAP_FLAME_NAME      "map"


//  MODE種別
#define     MODE_STANDBY        "standby"
#define     MODE_NAVI           "navi"
#define     MODE_SUSPEND        "suspend"
#define     MODE_ERROR          "error"

/**
 * @brief   通信の管理クラス
 * @details MQTT・HTTP通信にて外部との通信を行う際のメッセージの橋渡し等、通信内容の管理を行う
 */
class CommunicationManager
{
  public:
    
  // Common
    /**
    * @brief   CommunicationManagerクラスのコンストラクタ
    * @details 初期化を行う
    */
    CommunicationManager(ros::NodeHandle &node);

    /**
    * @brief   CommunicationManagerクラスのデストラクタ
    * @details オブジェクトの破棄を行う
    */
    ~CommunicationManager();


    /**
     * @brief        メインループ処理
     * @param[in]    void
     * @return       void
     * @details      マネージャのメインループ処理。
     */
    void manageCom(void);


  private:
    // To mqtt_bridge
    double pub_rate_robot_state_; // MQTTでの配信頻度(ロボットステータス)[Hz]
    double pub_rate_path_plan_;   // MQTTでの配信頻度(経路情報)[Hz]
    double pub_rate_lidar_2d_;    // MQTTでの配信頻度(LiDARの2D点群)[Hz]
    double pub_rate_lidar_3d_;    // MQTTでの配信頻度(LiDARの3D点群)[Hz]
    bool is_undelivered_robot_state_;
    bool is_undelivered_path_plan_;
    bool is_undelivered_lidar_2d_;
    bool is_undelivered_lidar_3d_;
    uoa_poc4_msgs::r_pub_robot_state robot_state_;
    uoa_poc4_msgs::r_pub_local_path_plan robot_plan_;
    uoa_poc5_msgs::r_pub_laser_scan lidar_scan_;
    uoa_poc5_msgs::r_pub_point_cloud point_cloud_;
    ros::Time last_pub_time_robot_state_;
    ros::Time last_pub_time_path_plan_;
    ros::Time last_pub_time_lidar_2d_;
    ros::Time last_pub_time_lidar_3d_;
    ros::Publisher pub_robot_state_;
    ros::Publisher pub_local_path_plan_;
    ros::Publisher pub_lidar_data_2d_;
    ros::Publisher pub_lidar_data_3d_;
    
    // manager
    HttpManager *http_manager_;
    MqttManager *mqtt_manager_;

    nav_msgs::OccupancyGrid *put_ego_map_;
    nav_msgs::OccupancyGrid *put_socio_map_;
    
    bool is_sb_put_ego_map_;
    bool is_sb_put_socio_map_;
    bool is_published_path_plan_;
    bool is_status_navi_;

    std::string last_revision_; // 最終リビジョン番号

    std::string robot_id_;      // ロボットの機体名
    std::string frame_id_;      // ロボットのTFフレーム名
    std::string robot_type_;    // ロボットの機体種別
    std::string map_location_;  // ロボット内部の地図の場所情報
    std::string sub_ego_map_topic_name_;  // エゴ地図名
    std::string sub_socio_map_topic_name_;  // ソシオ地図名
    std::string pub_ego_map_topic_name_;  // エゴ地図名
    std::string pub_socio_map_topic_name_;  // ソシオ地図名
    int         retry_counter_; // リトライ回数
    unsigned int retry_get_position_times_;
    ros::Time retry_get_position_interval_;
    double loop_rate_;   // コールバック関数の読み取り頻度[Hz]
    double retry_time_;  // リトライ時に待つ時間
};