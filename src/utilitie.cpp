/**
* @file     utilitie.cpp
* @brief    各ユーティリティ関数の実装ソースファイル
* @author   S.Kumada
* @date     2023/06/18
* @note     utilitie.hで定義された関数の実装
*/

#include "communication_manager/utilitie.h"

bool checkLastPublishTime(double freq, ros::Time last_pub_time)
{
    bool result = false;
    // 配信間隔時間(nsec)
    uint64_t pub_interval_time_ = freq == 0 ? 0 : 1 / freq * std::pow(10, 9);
    
    uint32_t nsec_part = pub_interval_time_ % 1000000000UL; // 1秒で割った余り
    uint32_t sec_part = pub_interval_time_ / 1000000000UL; // 1秒で割ったときの整数部


    // 時間差
    ros::Duration time_diff = ros::Time::now() - last_pub_time;

    // 周期以上判定
    if(time_diff.sec > sec_part ||  // 周期秒より大きい
        time_diff.sec == sec_part && time_diff.nsec >= nsec_part) // 周期秒と一致かつ周期ナノ秒以上
    {
#if DEBUG
        // ROS_INFO("Diff time: %fl[s]", time_diff);
        // double cycle = 1.0 / (time_diff.sec + time_diff.nsec * 0.000000001);
        // ROS_INFO("Target cycle: %fl[Hz] Actual cycle: %fl[Hz]", freq, cycle);
#endif

        result = true;
    }
    
    return result;
}

std::string rosTimeToIso8601(ros::Time time)
{
    struct timespec my_time; // 時刻格納変数(エポック秒)
    char iso_time[40];
    char time_zone[10];
    char converted_time[70];

    my_time.tv_sec = time.sec;
    my_time.tv_nsec = time.nsec;

    strftime(iso_time, sizeof(iso_time)-1,"%FT%T", localtime(&my_time.tv_sec)); // iso時間へ変換
    strftime(time_zone, sizeof(time_zone)-1,"%z", localtime(&my_time.tv_sec)); // タイムゾーン設定
    
    sprintf(converted_time, "%s.%03lu%s", iso_time, (my_time.tv_nsec+500)/1000, time_zone); // 結合

    std::string result_str_time = converted_time;

    return (result_str_time);
}

bool strTimeToRosTime(ros::Time &res_time, const std::string str_time, std::string format) 
{
    bool isSuccess = false;

    std::istringstream ss(str_time);
    std::tm tm = {};

    // フォーマットに合わせてパース
    ss >> std::get_time(&tm, format.c_str());

    if (ss.fail()) {
        // パースに失敗した場合のエラーハンドリング
        throw std::runtime_error("Failed to parse time.");
    }
    
    // ナノ秒の小数部分を取得
    double fractional_seconds;
    ss >> fractional_seconds;

    // タイムゾーン情報を取得
    int tz_minutes;
    ss >> tz_minutes;

    // タイムゾーン情報を秒に変換
    int tz_seconds = tz_minutes * 60;

    // tm 構造体から std::chrono::system_clock::time_point に変換
    std::chrono::system_clock::time_point tp = std::chrono::system_clock::from_time_t(std::mktime(&tm));

    // タイムゾーンを適用
    tp += std::chrono::seconds(static_cast<long long>(fractional_seconds));

    // std::chrono::system_clock::time_point から ros::Time に変換
    // res_time.sec = std::chrono::duration_cast<std::chrono::seconds>(tp.time_since_epoch()).count();
    // res_time.nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch()).count() % 1000000000;

    // std::chrono::system_clock::time_point から ros::Time に変換
    std::chrono::duration<long long> sec = std::chrono::duration_cast<std::chrono::seconds>(tp.time_since_epoch());
    std::chrono::duration<long long, std::nano> nsec = tp.time_since_epoch() - sec;

    ros::Time ros_time;
    ros_time.sec = sec.count();
    ros_time.nsec = nsec.count();

    // // チェック
    isSuccess = true;

    return isSuccess;
}

std::string iso8601ex(void)
{
    int ch;
    char iso_time[40];
    char time_zone[10];
    char dest[70];
    struct timeval myTime;
    struct tm *time_st;

    memset(iso_time, 0, sizeof(iso_time));
    memset(time_zone, 0, sizeof(time_zone));
    memset(dest, 0, sizeof(dest));

    gettimeofday(&myTime, NULL);
    time_st = localtime(&myTime.tv_sec);

    ch = strftime(iso_time, sizeof(iso_time)-1,"%FT%T", time_st);
    ch = strftime(time_zone, sizeof(time_zone)-1,"%z", time_st);

    sprintf(dest, "%s.%03lu%s", iso_time, (myTime.tv_usec+500)/1000, time_zone);

    std::string time_str = dest;

    return( time_str );
}
