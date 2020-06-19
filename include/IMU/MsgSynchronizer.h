#ifndef MSGSYNCHRONIZER_H
#define MSGSYNCHRONIZER_H

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <utility>
#include <IMU/imudata.h>
#include <IMU/IMUPreintegrator.h>
#include <IMU/NavState.h>
#include <livox_ros_driver/CustomMsg.h>
#include <livox_ros_driver/CustomPoint.h>
#include <newmsg/X1Msg.h>
#include <queue>
#include <mutex>
#include <thread>
#include <memory>
#include <condition_variable>

using namespace std;

class ImuMsgFetcher
{
public:
    enum Status{
        NOTINIT = 0,
        INIT,
        NORMAL
    };

    ImuMsgFetcher():_status(NOTINIT), imuStartTime(0){};
    ~ImuMsgFetcher() = default;

    void addImuMsg(const sensor_msgs::ImuConstPtr &imumsg){
        unique_lock<mutex> lock(_mutexIMUQueue);
      _imuMsgQueue.push(imumsg);
      if(_status == NOTINIT){
        imuStartTime = imumsg->header.stamp.toNSec();
        _status = INIT;
      }
    }

    void addLidarMsg(const livox_ros_driver::CustomMsgConstPtr& lidarmsg);
    void addLidarMsg(const newmsg::X1MsgConstPtr& lidarmsg);

    bool getRecentLidarMsg(livox_ros_driver::CustomMsgConstPtr& lidarmsg){
        unique_lock<mutex> lock(_mutexLidarQueue);
      if(lidarQueneSize > 2){
          lidarmsg = _lidarMsgQueue.front();
        _lidarMsgQueue.pop();
        lidarQueneSize--;
        return true;
      }
      return false;
    }

    bool getRecentLidarMsg(newmsg::X1MsgConstPtr& lidarmsg){
        unique_lock<mutex> lock(_mutexLidarQueue);
      if(lidarQueneSize > 0){
          lidarmsg = _lidarMsgQueueX1.front();
        _lidarMsgQueueX1.pop();
        lidarQueneSize--;
        return true;
      }
      return false;
    }

    bool fetchImuMsgs(double startTime, double endTime, std::vector<sensor_msgs::ImuConstPtr> &vimumsgs);

    unsigned long sizeOfImuQueue(){
        unique_lock<mutex> lock(_mutexIMUQueue);
      return _imuMsgQueue.size();
    }

    const std::queue<sensor_msgs::ImuConstPtr>& get_IMU_queue(){
        return _imuMsgQueue;
    }

    const std::queue<newmsg::X1MsgConstPtr>& get_X1_queue(){
        return _lidarMsgQueueX1;
    }

    void Set_Status(int i){
      switch(i){
        case 0  :
          _status = NOTINIT;
          break;
        case 1  :
          _status = INIT;
          break;

        default :
          _status = NORMAL;
      }
    }

private:
    std::mutex _mutexIMUQueue;
    std::mutex _mutexLidarQueue;
    std::queue<sensor_msgs::ImuConstPtr> _imuMsgQueue;
    std::queue<livox_ros_driver::CustomMsgConstPtr> _lidarMsgQueue;
    std::queue<newmsg::X1MsgConstPtr> _lidarMsgQueueX1;
    size_t volatile lidarQueneSize = 0;
    unsigned long imuStartTime;
    Status _status;
    const float ot = 0.1;
};

#endif // MSGSYNCHRONIZER_H
