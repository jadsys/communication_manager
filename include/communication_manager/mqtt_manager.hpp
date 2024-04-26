/**
* @file     mqtt_manager.hpp
* @brief    ROS-MQTT間データマネージャーMqttManagerの定義ソースファイル
* @author   S.Kumada
* @date     2023/12/7
* @note     ROS⇔MQTT間のフォーマット変換やデータの精査等を行うクラスの定義
*/

#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

// 共通関数定義
#include "communication_manager/utilitie.h"


// RDR通信結合用カスタムメッセージ定義
#include "uoa_poc3_msgs/r_state.h"
#include "uoa_poc3_msgs/r_emergency_command.h"
#include "uoa_poc3_msgs/r_emergency_result.h"
#include "uoa_poc4_msgs/r_pub_robot_state.h"
#include "uoa_poc4_msgs/r_pub_local_path_plan.h"
#include "uoa_poc4_msgs/r_sub_map_update_notify.h"
#include "uoa_poc5_msgs/r_pub_laser_scan.h"
#include "uoa_poc5_msgs/r_pub_point_cloud.h"
#include "uoa_poc6_msgs/r_sub_emg_cmd.h"
#include "uoa_poc6_msgs/r_pub_emg_cmd_exe.h"

//  MODE種別
#define     MODE_STANDBY        "standby"
#define     MODE_NAVI           "navi"
#define     MODE_SUSPEND        "suspend"
#define     MODE_ERROR          "error"
#define     DEF_SUB_ROBOT_STATE_TOPIC_NAME        "/robot_bridge/megarover_01/state"
#define     DEF_PUB_ROBOT_STATE_TOPIC_NAME        "/ros_bridge/mqtt/pub_robot_state"
#define     DEF_SUB_PATH_PLAN_TOPIC_NAME          "/move_base/NavfnROS/plan"
#define     DEF_PUB_PATH_PLAN_TOPIC_NAME          "/ros_bridge/mqtt/pub_local_path_plan"
#define     DEF_SUB_LIDAR_2D_TOPIC_NAME           "/scan"
#define     DEF_PUB_LIDAR_2D_TOPIC_NAME           "/ros_bridge/mqtt/pub_laserscan"
#define     DEF_SUB_LIDAR_3D_TOPIC_NAME           "/velodyne_points"
#define     DEF_PUB_LIDAR_3D_TOPIC_NAME           "/ros_bridge/mqtt/pub_pointcloud"
#define     DEF_SUB_EMG_CMD_EXE_TOPIC_NAME        "/robot_bridge/megarover_01/emgexe"
#define     DEF_PUB_EMG_CMD_EXE_TOPIC_NAME        "/ros_bridge/mqtt/pub_emg_exe"
#define     DEF_SUB_EMG_CMD_TOPIC_NAME            "/ros_bridge/mqtt/sub_emg"
#define     DEF_PUB_EMG_CMD_TOPIC_NAME            "/robot_bridge/megarover_01/emg"
#define     DEF_SUB_MAP_UPDATE_NOTIFY_TOPIC_NAME  "/ros_bridge/sub_map_update_notify"

/**
 * @brief   MQTT通信の管理クラス
 * @details MQTT通信にて外部との通信を行う際のメッセージの橋渡し等、通信内容の管理を行う
 */
class MqttManager
{
  public:
    /**
    * @brief   MqttManagerクラスのコンストラクタ
    * @details 初期化を行う
    */
    MqttManager(ros::NodeHandle &node);

    /**
    * @brief   HttpManagerクラスのデストラクタ
    * @details オブジェクトの破棄を行う
    */
    ~MqttManager();

  // To mqtt_bridge
    /**
     * @brief        ロボット内部状態の配信関数
     * @param[in]    void
     * @return       void
     * @details      mqtt_bridgeへデータを配信する。
     */
    void pubRobotStatus();

    /**
     * @brief        ナビゲーション経路の配信関数
     * @param[in]    void
     * @return       void
     * @details      mqtt_bridgeへデータを配信する。
     */
    void pubPathPlan();

    /**
     * @brief        2Dスキャンデータの配信関数
     * @param[in]    void
     * @return       void
     * @details      mqtt_bridgeへデータを配信する。
     */
    void pubLaserScan();

    /**
     * @brief        3Dスキャンデータの配信関数
     * @param[in]    void
     * @return       void
     * @details      mqtt_bridgeへデータを配信する。
     */
    void pubPointCloud();

    /**
     * @brief        緊急停止指示応答の配信関数
     * @param[in]    void
     * @return       void
     * @details      mqtt_bridgeへデータを配信する。
     */
    void pubEmgCmdExe();

    /**
     * @brief        緊急停止指示の配信関数
     * @param[in]    void
     * @return       void
     * @details      ロボットへデータを配信する。
     */
    void pubEmgCmd();

    /**
     * @brief        配信頻度に従ったMQTTデータ配信処理
     * @param[in]    void
     * @return       void
     * @details      配信頻度に従って受信しているデータの配信処理を実行する。
     */
    void pubMqttTopic();

    /**
     * @brief        配信頻度に従ったロボット（シミュレータ含む）へのデータ配信処理
     * @param[in]    void
     * @return       void
     * @details      配信頻度に従って受信しているデータの配信処理を実行する。
     */
    void pubRobotTopic();

  // For mqtt_bridge
    /**
     * @brief        地図の更新要求の受信CallBack関数
     * @param[in]    (const uoa_poc4_msgs::r_sub_map_update_notify& msg　更新要求メッセージ
     * @return       bool true:取得成功, false:取得失敗
     * @details      MQTTにて受信した地図の更新要求メッセージを監視し、更新された場合は地図の取得要求を行う。
     */
    void subMapUpdateCB(const uoa_poc4_msgs::r_sub_map_update_notify& msg);

  // For Robot
    /**
     * @brief        ロボットの内部ステータス情報の受信CallBack関数
     * @param[in]    const uoa_poc3_msgs::r_stateConstPtr& msg　内部ステータス情報
     * @return       void
     * @details      ロボット統合管理ノードから送られてくる内部ステータスを、外部へと配信するための準備を行う。
     */
    void recvRoboStatusCB(const uoa_poc3_msgs::r_stateConstPtr& msg);


    /**
     * @brief        ロボットの経路情報の受信CallBack関数
     * @param[in]    const nav_msgs::PathConstPtr& msg　経路情報
     * @return       void
     * @details      move_baseにて求めたロボットの経路を受取、RDRへMQTTにて送信するためのメッセージ変換を行う
     */
    void recvPathPlanCB(const nav_msgs::PathConstPtr& msg);

    /**
     * @brief        LiDARの2D点群データの受信CallBack関数
     * @param[in]    const sensor_msgs::LaserScan& msg　受信した2Dのスキャン
     * @return       void
     * @details      RDRに送信する、LiDARの2D点群情報を受信し、内部に保持する。
     */
    void recvLaserScanCB(const sensor_msgs::LaserScan& msg);

    /**
     * @brief        LiDARの3D点群情報の受信CallBack関数
     * @param[in]    const sensor_msgs::PointCloud2& msg　受信した3Dポイントクラウド
     * @return       void
     * @details      RDRに送信する、LiDARの3D点群情報を受信し、内部に保持する。
     */
    void recvPointCloud2CB(const sensor_msgs::PointCloud2& msg);

    /**
     * @brief        緊急停止指示応答の受信CallBack関数
     * @param[in]    const uoa_poc3_msgs::r_emergency_result& msg　受信した緊急停止指示の応答
     * @return       void
     * @details      上位系に送信する、緊急停止指示の応答を受信し、内部に保持する。
     */
    void recvEmgCmdExeCB(const uoa_poc3_msgs::r_emergency_result& msg);

    /**
     * @brief        緊急停止指示の受信CallBack関数
     * @param[in]    const uoa_poc6_msgs::r_sub_emg_cmd& msg　受信した緊急停止指示
     * @return       void
     * @details      ロボットに送信する、緊急停止指示を受信し、内部に保持する。
     */
    void recvEmgCmdCB(const uoa_poc6_msgs::r_sub_emg_cmd& msg);

  private:

    std::string robot_id_;      // ロボットの機体名
    std::string robot_type_;    // ロボットの機体種別
    std::string map_location_;  // ロボット内部の地図の場所情報
    std::string space_info_;    // 空間情報

   // To mqtt_bridge
    double pub_rate_robot_state_; // MQTTでの配信頻度(ロボットステータス)[Hz]
    double pub_rate_path_plan_;   // MQTTでの配信頻度(経路情報)[Hz]
    double pub_rate_lidar_2d_;    // MQTTでの配信頻度(LiDARの2D点群)[Hz]
    double pub_rate_lidar_3d_;    // MQTTでの配信頻度(LiDARの3D点群)[Hz]
    double pub_rate_emg_cmd_exe_; // MQTTでの配信頻度(緊急停止指示コマンドの応答)[Hz]
    double pub_rate_emg_cmd_;     // MQTTでの配信頻度(緊急停止指示コマンド)[Hz]
    bool is_undelivered_robot_state_;
    bool is_undelivered_path_plan_;
    bool is_undelivered_lidar_2d_;
    bool is_undelivered_lidar_3d_;
    bool is_undelivered_emg_cmd_exe_;
    bool is_undelivered_emg_cmd_;
    uoa_poc4_msgs::r_pub_robot_state robot_state_;
    uoa_poc4_msgs::r_pub_local_path_plan robot_plan_;
    uoa_poc5_msgs::r_pub_laser_scan lidar_scan_;
    uoa_poc5_msgs::r_pub_point_cloud point_cloud_;
    uoa_poc6_msgs::r_pub_emg_cmd_exe emg_cmd_exe_;
    uoa_poc3_msgs::r_emergency_command emg_cmd_;
    ros::Time last_pub_time_robot_state_;
    ros::Time last_pub_time_path_plan_;
    ros::Time last_pub_time_lidar_2d_;
    ros::Time last_pub_time_lidar_3d_;
    ros::Time last_pub_time_emg_cmd_exe_;
    ros::Time last_pub_time_emg_cmd_;

    // To Robot
    ros::Publisher pub_emg_cmd_;

    // To mqtt_bridge
    ros::Publisher pub_robot_state_;
    ros::Publisher pub_local_path_plan_;
    ros::Publisher pub_lidar_data_2d_;
    ros::Publisher pub_lidar_data_3d_;
    ros::Publisher pub_emg_cmd_exe_;

    // For mqtt_bridge
    ros::Subscriber sub_emg_cmd_;
    ros::Subscriber sub_map_update_notify_;
    
    // For Robot
    ros::Subscriber sub_robo_status_;       // ロボットのテレメトリ情報のサブスクライバ
    ros::Subscriber sub_local_path_plan_;
    ros::Subscriber sub_lidar_data_2d_;
    ros::Subscriber sub_lidar_data_3d_;
    ros::Subscriber sub_emg_cmd_exe_;
};
#endif