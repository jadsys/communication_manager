/**
* @file     http_manager.hpp
* @brief    ROS-HTTP間データマネージャーHttpManagerの定義ヘッダファイル
* @author   S.Kumada
* @date     2023/12/7
* @note     ROS⇔HTTP間のフォーマット変換やデータの精査等を行うクラスの実装
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>// 初期位置トピックの型 https://demura.net/lecture/14011.html

// 共通関数定義
#include "communication_manager/utilitie.h"

#include "uoa_poc3_msgs/r_info.h"
#include "uoa_poc4_msgs/r_req_robot_info.h"
#include "uoa_poc4_msgs/r_res_robot_info.h"
#include "uoa_poc4_msgs/r_res_mapdata_list.h"
#include "uoa_poc4_msgs/r_req_mapdata_list.h"
#include "uoa_poc4_msgs/r_req_put_mapdata.h"
#include "uoa_poc4_msgs/r_res_put_mapdata.h"
#include "uoa_poc4_msgs/r_req_get_mapdata.h"
#include "uoa_poc4_msgs/r_res_get_mapdata.h"
#include "uoa_poc4_msgs/r_req_get_position_data.h"
#include "uoa_poc5_msgs/r_get_mapdata.h"
#include "uoa_poc5_msgs/r_put_mapdata.h"
#include "uoa_poc5_msgs/r_get_position_data.h"
#include "uoa_poc5_msgs/r_get_externally_position_data.h"
#include "uoa_poc5_msgs/r_res_get_position_data.h"
#include "uoa_poc5_msgs/r_recv_map_info.h"
#include "uoa_poc6_msgs/r_map_pose_correct_info.h"
#include "uoa_poc6_msgs/r_put_map_pose_correct.h"
#include "uoa_poc6_msgs/r_req_put_map_pose_correct.h"
#include "uoa_poc6_msgs/r_res_put_map_pose_correct.h"
#include "uoa_poc6_msgs/r_req_change_obj_loc.h"
#include "uoa_poc6_msgs/r_res_change_obj_loc.h"
#include "uoa_poc6_msgs/r_objects_location.h"
#include "uoa_poc6_msgs/r_get_objects_location.h"
#include "uoa_poc6_msgs/r_get_map_pose_correct.h"
#include "uoa_poc6_msgs/r_req_get_map_pose_correct.h"
#include "uoa_poc6_msgs/r_res_get_map_pose_correct.h"

#define DEF_ROBOT_ID                        "turtlebot_01"
#define DEF_ROBOT_TYPE                      "turtlebot"
#define DEF_ROBOT_NAME                      "Turtlebot"
#define DEF_MAP_LOCATION                    "lictia_1f"
#define DEF_SPACE_INFOMATION                "real"
#define DEF_SYSTEM_TYPE                     "ros"

// To http_bridge
#define DEF_REQ_ROBOT_INFO_TOPIC_NAME              "/ros_bridge/http/req/robot_info"
#define DEF_REQ_MAP_DATA_LIST_TOPIC_NAME           "/ros_bridge/http/req/map_data_list"
#define DEF_REQ_PUT_MAP_DATA_TOPIC_NAME            "/ros_bridge/http/req/put_map_data"
#define DEF_REQ_PUT_CORRECT_INFO_TOPIC_NAME        "/ros_bridge/http/req/put_correct_info"
#define DEF_REQ_GET_MAP_DATA_TOPIC_NAME            "/ros_bridge/http/req/get_map_data"
#define DEF_REQ_GET_LAYER_MAP_DATA_TOPIC_NAME      "/ros_bridge/http/req/get_layer_map_data"
#define DEF_REQ_GET_POSITION_TOPIC_NAME            "/ros_bridge/http/req/get_position_data"
#define DEF_REQ_GET_OBJECT_LOCATION_TOPIC_NAME     "/ros_bridge/http/req/get_object_location"
#define DEF_REQ_GET_CORRECT_INFO_TOPIC_NAME        "/ros_bridge/http/req/get_correct_info"

// For http_bridge
#define DEF_RES_ROBOT_INFO_TOPIC_NAME              "/ros_bridge/http/res/robot_info"
#define DEF_RES_MAP_DATA_LIST_TOPIC_NAME           "/ros_bridge/http/res/map_data_list"
#define DEF_RES_PUT_MAP_DATA_TOPIC_NAME            "/ros_bridge/http/res/put_map_data"
#define DEF_RES_PUT_CORRECT_INFO_TOPIC_NAME        "/ros_bridge/http/res/put_correct_info"
#define DEF_RES_GET_MAP_DATA_TOPIC_NAME            "/ros_bridge/http/res/get_map_data"
#define DEF_RES_GET_LAYER_MAP_DATA_TOPIC_NAME      "/ros_bridge/http/res/get_layer_map_data"
#define DEF_RES_GET_POSITION_TOPIC_NAME            "/ros_bridge/http/res/get_position_data"
#define DEF_RES_GET_OBJECT_LOCATION_TOPIC_NAME     "/ros_bridge/http/res/get_object_location"
#define DEF_RES_GET_CORRECT_INFO_TOPIC_NAME        "/ros_bridge/http/res/get_correct_info"

// To Robot
#define DEF_EGO_MAP_TOPIC_NAME                  "/map"
#define DEF_SOSIO_MAP_TOPIC_NAME                "/map_movebase"
#define DEF_STATIC_LAYER_MAP_TOPIC_NAME         "/static_layer_map"
#define DEF_QUASI_STATIC_LAYER_MAP_TOPIC_NAME   "/quasi_static_layer_map"
#define DEF_EXCLUSION_ZONE_LAYER_MAP_TOPIC_NAME "/no_entry_layer_map"
#define DEF_TO_RBT_GET_CORRECT_INFO_TOPIC_NAME  "/robot_bridge/correction_value"

// For Robot
#define DEF_FOR_RBT_PUT_MAP_DATA_TOPIC_NAME       "/robot_bridge/put_map_data"
#define DEF_FOR_RBT_GET_CORRECT_INFO_TOPIC_NAME   "/robot_bridge/get_correction_value"

// to sim
#define DEF_TO_SIM_GET_OBJECT_LOCATION_TOPIC_NAME   "/simulator_bridge/get_object_location"

// for sim
#define DEF_FOR_SIM_CORRECT_INFO_TOPIC_NAME     "/correction_info"
#define DEF_FOR_SIM_PUT_CORRECT_INFO_TOPIC_NAME  "/simulator_bridge/put_correction_info"
#define DEF_FOR_SIM_GET_OBJECT_LOCATION_TOPIC_NAME   "/simulator_bridge/object_location"

// queueサイズ
#define ROS_QUEUE_SIZE_1                    1
#define ROS_QUEUE_SIZE_2                    2
#define ROS_QUEUE_SIZE_3                    3
#define ROS_QUEUE_SIZE_10                   10

#define ROS_TIME_60_SEC                     60

#define DEF_EGO_MAP_NAME                    "ego"
#define DEF_SOCIO_MAP_NAME                  "socio"
#define DEF_STATIC_LAYER_MAP_NAME           "static_layer"
#define DEF_QUASI_STATIC_LAYER_MAP_NAME     "semi_static_layer"
#define DEF_EXCLUSION_ZONE_LAYER_MAP_NAME   "exclusion_zone_layer"
#define DEF_ENVIRONMENT_MAP_NAME            "environment"
#define DEF_REFERENCE_MAP_NAME              "reference"
#define DEF_MAP_FLAME_NAME                  "map"

/**
 * @brief 地図のリビジョン番号格納用構造体
 */
typedef struct stMapRevision
{
  std::string environment_map_revision; //!< 環境地図のリビジョン番号
  std::string reference_map_revision;   //!< リファレンス地図のリビジョン番号
  std::string static_map_revision;      //!< 静的レイヤ地図のリビジョン番号
  std::string q_static_map_revision;    //!< 準静的レイヤ地図のリビジョン番号
  std::string no_entry_map_revision;    //!< 進入禁止レイヤ地図のリビジョン番号
  
} MapRevision;

enum MAP_TYPE
{
  ENVIRONMENT_MAP,  //!< 環境地図
  REFERENCE_MAP,    //!< リファレンス地図
  STATIC_MAP,       //!< 静的レイヤ地図
  Q_STATIC_MAP,     //!< 準静的レイヤ地図
  NO_ENTRY_MAP,     //!< 進入禁止レイヤ地図
  
  MAX_MAP_IDX       //!< インデックス値
};

/**
 * @brief   HTTP通信の管理クラス
 * @details HTTP通信にて外部との通信を行う際のメッセージの橋渡し等、通信内容の管理を行う
 */
class HttpManager
{
  public:

// Common
    /**
    * @brief   HttpManagerクラスのコンストラクタ
    * @details 初期化を行う
    */
    HttpManager(ros::NodeHandle &node);

    /**
    * @brief   HttpManagerクラスのデストラクタ
    * @details オブジェクトの破棄を行う
    */
    ~HttpManager();

    /**
     * @brief        リビジョン番号のチェック関数
     * @param[in]    std::string map_revision 地図のリビジョン番号
     * @return       void
     * @details      リビジョン番号をチェックし、空じゃない場合はそのリビジョン番号を返す
     */
    std::string check_map_revision(std::string map_revision);

// HTTP Reqest送信関連メソッド

    /**
     * @brief        ロボットの機体情報送信関数
     * @param[in]    uoa_poc4_msgs::r_req_robot_info robot_info ロボットの機体情報
     * @return       void
     * @details      ロボットの機体情報を外部システムに通知するメッセージを配信する。
     */
    void reqRobotInfo(uoa_poc4_msgs::r_req_robot_info robot_info);

    /**
     * @brief        RDRに登録された地図の一覧の取得要求の送信関数
     * @param[in]    uoa_poc4_msgs::r_req_mapdata_list map_list リクエストするデータ
     * @return       void
     * @details      RDRに登録済みの地図の一覧の取得要求を送信する。
     */
    void reqMapdataList(uoa_poc4_msgs::r_req_mapdata_list map_list);
    
    /**
     * @brief        RDRに更新した地図の登録要求の送信関数
     * @param[in]    uoa_poc4_msgs::r_req_put_mapdata put_mapdata RDRに登録したい地図データ
     * @return       void
     * @details      RDRに更新した地図の登録要求を送信する。
     */
    void reqPutMapdata(uoa_poc4_msgs::r_req_put_mapdata put_mapdata);

    /**
     * @brief        RDRに登録された地図の取得要求の送信関数
     * @param[in]    uoa_poc4_msgs::r_req_get_mapdata map_info 取得したい地図の情報
     * @return       void
     * @details      RDRに登録済みの地図の取得要求を送信する。
     */
    void reqGetMapdata(uoa_poc4_msgs::r_req_get_mapdata map_info);

    /**
     * @brief        RDRに登録された地図の取得要求の送信関数
     * @param[in]    uoa_poc4_msgs::r_req_get_mapdata map_info 取得したい地図の情報
     * @return       void
     * @details      RDRに登録済みの地図の取得要求を送信する。
     */
    void reqGetLayerMapdata(uoa_poc4_msgs::r_req_get_mapdata map_info);

    /**
     * @brief        ロボットの位置情報取得要求の送信関数
     * @param[in]    const uoa_poc4_msgs::r_req_robot_info robot_info　ロボットの機体情報
     * @return       void
     * @details      外部システムによって観測されたロボットの現在位置情報の取得要求を配信する。
     */
    void reqGetPositiondata();

    /**
     * @brief        地図の補正値の登録要求の送信関数
     * @param[in]    const uoa_poc6_msgs::r_req_put_map_pose_correct& correct_info　補正値情報
     * @return       void
     * @details      地図の補正値をRDRに登録するための登録要求を送信する。
     */
    void reqPutCorrectionInfo(const uoa_poc6_msgs::r_req_put_map_pose_correct& correct_info);

    /**
     * @brief        準静的物体の位置情報取得要求の送信関数
     * @param[in]    uoa_poc6_msgs::r_req_change_obj_loc　ロボットの機体情報
     * @return       void
     * @details      準静的物体の位置変化情報の取得要求を配信する。
     */
    void reqGetObjectLocation(uoa_poc6_msgs::r_req_change_obj_loc change_obj_loc);

    /**
     * @brief        地図の補正値の取得要求の送信関数
     * @param[in]    const uoa_poc6_msgs::r_req_put_map_pose_correct& correct_info　補正値情報
     * @return       void
     * @details      地図の補正値をRDRから取得するための要求を送信する。
     */
    void reqGetCorrectionInfo(const uoa_poc6_msgs::r_req_get_map_pose_correct& get_correct_info);

// HTTP Response受信関連メソッド
    /**
     * @brief        機体情報送信結果の受信CallBack関数
     * @param[in]    const uoa_poc4_msgs::r_res_robot_info& msg　送信結果
     * @return       void
     * @details      機体情報をリクエストし、レスポンスメッセージから成功の可否を判断する。
     */
    void resRobotInfoCB(const uoa_poc4_msgs::r_res_robot_info& msg);

    /**
     * @brief        地図の一覧の取得要求送信結果の受信CallBack関数
     * @param[in]    const uoa_poc4_msgs::r_res_mapdata_list&　msg　送信結果
     * @return       void
     * @details      地図の一覧の取得要求をリクエストし、レスポンスメッセージから登録された地図一覧を把握する。
     */
    void resMapdataListCB(const uoa_poc4_msgs::r_res_mapdata_list& msg);

   /**
     * @brief        地図の登録要求送信結果の受信CallBack関数
     * @param[in]    const uoa_poc4_msgs::r_res_put_mapdata& msg　送信結果
     * @return       void
     * @details      地図の登録要求をリクエストし、レスポンスメッセージから登録要求の結果を把握する。
     */
    void resPutMapdataCB(const uoa_poc4_msgs::r_res_put_mapdata& msg);

    /**
     * @brief        地図の取得要求送信結果の受信CallBack関数
     * @param[in]    const uoa_poc4_msgs::r_res_get_mapdata& msg　送信結果
     * @return       void
     * @details      地図の取得要求をリクエストし、レスポンスメッセージから登録された地図を取得する。
     */
    void resGetMapdataCB(const uoa_poc4_msgs::r_res_get_mapdata& msg);

    /**
     * @brief        ロボットの位置取得結果の受信CallBack関数
     * @param[in]    const uoa_poc4_msgs::r_req_get_position_data& msg　送信結果
     * @return       void
     * @details      対象のロボットの位置情報取得要求をリクエストし、レスポンスメッセージから位置情報を取得する。
     */
    void resReqPositiondataCB(const uoa_poc5_msgs::r_res_get_position_data& msg);

    /**
     * @brief        地図の補正値の登録要求のレスポンス受信CallBack関数
     * @param[in]    const uoa_poc6_msgs::r_res_put_map_pose_correct msg　送信結果 
     * @return       void
     * @details      環境地図とリファレンス地図の補正値情報の登録要求の結果を取得する。
     */
    void resPutCorrectInfoCB(const uoa_poc6_msgs::r_res_put_map_pose_correct& msg);

    /**
     * @brief        準静的物体の位置情報取得結果の受信CallBack関数
     * @param[in]    const uoa_poc6_msgs::r_res_change_obj_loc& msg　送信結果
     * @return       void
     * @details      地図の取得要求をリクエストし、レスポンスメッセージから登録された地図を取得する。
     */
    void resObjectLocationCB(const uoa_poc6_msgs::r_res_change_obj_loc& msg);

    /**
     * @brief        地図の補正値の取得要求のレスポンス受信CallBack関数
     * @param[in]    const uoa_poc6_msgs::r_res_get_map_pose_correct msg　送信結果 
     * @return       void
     * @details      地図の補正値情報の取得要求の結果を取得する。
     */
    void resGetCorrectInfoCB(const uoa_poc6_msgs::r_res_get_map_pose_correct& msg);

// Robotからのリクエスト要求受信関連メソッド
    /**
     * @brief        機体情報送信のCallBack関数
     * @param[in]    const uoa_poc3_msgs::r_info& msg　要求データ
     * @return       void
     * @details      ロボット統合管理ノードから送られてくる機体情報を、外部へと送信するための準備を行う。
     */
    void recvRoboInfoCB(const uoa_poc3_msgs::r_info& msg);

    /**
     * @brief        各地図配信要求のCallBack関数
     * @param[in]    const uoa_poc5_msgs::r_put_mapdata& msg　要求データ
     * @return       void
     * @details      ロボット内部の各地図を配信する。
     */
    void recvPutMapdataCB(const uoa_poc5_msgs::r_put_mapdata& msg);
    
    /**
     * @brief        各地図取得のCallBack関数
     * @param[in]    const uoa_poc5_msgs::r_get_mapdata& msg　要求データ
     * @return       void
     * @details      ロボット内部の地図の取得・更新のための各地図を取得する。
     */
    void recvGetMapdataCB(const uoa_poc5_msgs::r_get_mapdata& msg);
    
    /**
     * @brief        環境地図のリビジョン番号に対応するレイヤ地図取得のCallBack関数
     * @param[in]    const uoa_poc5_msgs::r_get_mapdata& msg　要求データ
     * @return       void
     * @details      ナビゲーション実施時に上位系で使用している環境地図に対応したレイヤ地図を取得する。
     */
    void recvGetLayerMapdataCB(const uoa_poc5_msgs::r_get_mapdata& msg);
        
    /**
     * @brief        ロボットの現在位置取得のCallBack関数
     * @param[in]    const uoa_poc5_msgs::r_get_positiondata& msg　要求データ
     * @return       void
     * @details      RDRからロボットの現在位置を取得する。
     */
    void recvGetPositiondataCB(const uoa_poc5_msgs::r_get_position_data& msg);
    
    /**
     * @brief        ロボット内部の地図の受信CallBack関数
     * @param[in]    const nav_msgs::OccupancyGridConstPtr& msg　受信した地図
     * @return       void
     * @details      RDRに送信する、ロボット内部地図を受信し、内部に保持する。
     */
    void recvMapCB(const nav_msgs::OccupancyGridConstPtr& msg, std::string map_layer);

    /**
     * @brief        補正値の取得要求の受信CallBack関数
     * @param[in]    const uoa_poc6_msgs::r_get_map_pose_correct& msg　補正値取得要求
     * @return       void
     * @details      RDRから地図の補正値情報を取得する。
     */
    void recvGetCorrectInfoCB(const uoa_poc6_msgs::r_get_map_pose_correct& msg);


// シミュレータからのリクエスト要求受信関連メソッド
    /**
     * @brief        補正値データの配信要求のCallBack関数
     * @param[in]    const uoa_poc6_msgs::r_req_put_map_pose_correct& msg　要求データ
     * @return       void
     * @details      環境地図とリファレンス地図間の補正値を配信する。
     */
    void recvPutCorrectInfoCB(const uoa_poc6_msgs::r_put_map_pose_correct& msg);
    
    /**
     * @brief        準静的物体の位置情報取得のCallBack関数
     * @param[in]    const uoa_poc6_msgs::r_get_objects_location& msg　要求データ
     * @return       void
     * @details      RDRから物体認識システムによって更新される準静的物体の位置情報を取得する。
     */
    void recvGetObjectLocationCB(const uoa_poc6_msgs::r_get_objects_location& msg);
    
    /**
     * @brief        補正値の受信CallBack関数
     * @param[in]    const uoa_poc6_msgs::r_map_pose_correct_info& msg　受信した補正値
     * @return       void
     * @details      RDRに送信する補正値を受信し、内部に保持する。
     */
    void recvMapCorrectInfoCB(const uoa_poc6_msgs::r_map_pose_correct_info& msg);


// Robotへのレスポンス結果配信関連メソッド
  private:
    // パブリッシャ
    // To http_bridge
    ros::Publisher pub_req_robot_info_;
    ros::Publisher pub_req_map_data_list_;
    ros::Publisher pub_req_put_map_data_;
    ros::Publisher pub_req_get_map_data_;
    ros::Publisher pub_req_get_layer_map_data_;
    ros::Publisher pub_req_get_position_data_;
    ros::Publisher pub_req_put_correct_info_;
    ros::Publisher pub_req_get_correct_info_;
    ros::Publisher pub_req_get_object_location_;
    std::string get_robot_position_name_; // ロボットの位置検索名

    // To Robot
    ros::Publisher pub_recv_map_info_;
    ros::Publisher pub_ego_map_data_;
    ros::Publisher pub_socio_map_data_;
    ros::Publisher pub_static_layer_map_data_;
    ros::Publisher pub_quasi_static_layer_map_data_;
    ros::Publisher pub_exclusion_zone_layer_map_data_;
    ros::Publisher pub_environment_map_data_;
    ros::Publisher pub_reference_map_data_;
    ros::Publisher pub_initial_position_data_;
    ros::Publisher pub_get_correct_info_;
    std::string ego_map_topic_name_;                  // エゴ地図名
    std::string socio_map_topic_name_;                // ソシオ地図名
    std::string static_layer_map_topic_name_;         // 静的レイヤ地図名
    std::string quasi_static_layer_map_topic_name_;   // 準静的レイヤ地図名
    std::string exclusion_zone_layer_map_topic_name_; // 進入禁止レイヤ地図名
    std::string environment_map_topic_name_; // 進入禁止レイヤ地図名
    std::string reference_map_topic_name_; // 進入禁止レイヤ地図名

    // To sim
    ros::Publisher pub_get_object_location_;

    // サブスクライバ
    // For http_bridge
    ros::Subscriber sub_res_robot_info_;
    ros::Subscriber sub_res_map_data_list_;
    ros::Subscriber sub_res_put_map_data_;
    ros::Subscriber sub_res_get_map_data_;
    ros::Subscriber sub_res_position_data_;         // ロボットの位置取得のレスポンスデータのサブスクライバ
    ros::Subscriber sub_res_get_layer_map_data_;
    ros::Subscriber sub_res_put_correct_info_;      // 地図の補正値配信のレスポンスデータのサブスクライバ
    ros::Subscriber sub_res_get_correct_info_;      // 地図の補正値取得のレスポンスデータのサブスクライバ
    ros::Subscriber sub_res_get_object_location_; 

    // For Robot
    ros::Subscriber sub_robo_info_;                 // ロボット情報のサブスクライバ
    ros::Subscriber sub_get_map_data_;              // 地図取得要求のサブスクライバ
    ros::Subscriber sub_get_layer_map_data_;        // レイヤ地図取得要求のサブスクライバ
    ros::Subscriber sub_get_position_data_;         // ロボットの位置取得要求のサブスクライバ
    ros::Subscriber sub_ego_map_;                   // エゴ地図のサブスクライバ
    ros::Subscriber sub_socio_map_;                 // ソシオ地図のサブスクライバ
    ros::Subscriber sub_static_layer_map_;          // 静的レイヤ地図のサブスクライバ
    ros::Subscriber sub_quasi_static_layer_map_;    // 準静的レイヤ地図のサブスクライバ
    ros::Subscriber sub_exclusion_zone_layer_map_;  // 進入禁止レイヤ地図のサブスクライバ
    ros::Subscriber sub_put_map_data_;              // レイヤ地図の配置要求のサブスクライバ
    ros::Subscriber sub_get_correct_info_;          // 補正情報の取得要求のサブスクライバ


    // For Sim
    ros::Subscriber sub_correct_info_;              // 補正情報のサブスクライバ
    ros::Subscriber sub_put_correct_info_;          // 補正情報の配信要求のサブスクライバ
    ros::Subscriber sub_environment_map_;           // レイヤ地図の配置要求のサブスクライバ
    ros::Subscriber sub_reference_map_;             // リファレンス地図の配置要求のサブスクライバ
    ros::Subscriber sub_get_object_location_;       // 準静的物体の位置変化情報の取得要求のサブスクライバ
        
    bool is_sb_put_ego_map_;    // 未配信のエゴ地図あり
    bool is_sb_put_socio_map_;  // 未配信のソシオ地図あり
    bool is_sb_put_static_layer_;
    bool is_sb_put_quasi_static_layer_;
    bool is_sb_put_exclusion_zone_layer_;
    bool is_sb_put_environment_map_;
    bool is_sb_put_reference_map_;
    bool is_sb_put_map_pose_correct_value_;

    // 最後に配信したメッセージデータのバッファ
    uoa_poc4_msgs::r_req_robot_info  robot_info_;
    uoa_poc4_msgs::r_req_get_mapdata map_info_;
    uoa_poc4_msgs::r_req_get_mapdata layer_map_info_;
    uoa_poc6_msgs::r_map_pose_correct_info last_pose_correct_value_;
    nav_msgs::OccupancyGrid *ego_map_;
    nav_msgs::OccupancyGrid *socio_map_;
    nav_msgs::OccupancyGrid *static_layer_map_;
    nav_msgs::OccupancyGrid *semi_static_layer_map_;
    nav_msgs::OccupancyGrid *exclusion_zone_layer_map_;
    nav_msgs::OccupancyGrid *environment_map_;
    nav_msgs::OccupancyGrid *reference_map_;

    // data保持用
    MapRevision put_map_revision_; // 配信した地図のリビジョン番号
    MapRevision get_map_revision_; // 取得した地図のリビジョン番号

    std::string robot_id_;      // ロボットの機体名
    std::string map_frame_id_;  // 地図のTFフレーム名
    std::string tf_prefix_;     // TFのトップ階層
    std::string robot_type_;    // ロボットの機体種別
    std::string robot_name_;    // ロボット名
    std::string map_location_;  // ロボット内部の地図の場所情報
    std::string space_info_;    // 地図の空間情報
    std::string linked_revision_;  // 対象の地図とリンクしてる地図のリビジョン番号
    std::string set_map_linked_revision_; // 地図配信時の紐付いた環境地図のリビジョン番号の手動設定用
    std::string target_map_revision_number_; // 補正値配信時の紐付いたターゲット地図のリビジョン番号の手動設定用
    std::string source_map_revision_number_; // 補正値配信時の紐付いたソース地図のリビジョン番号の手動設定用
    std::string obj_position_update_time_; // 物体認識側のレイアウト変化検知時間の保持用
    
    // retryカウンタ
    unsigned int cnt_retry_get_position;
};
