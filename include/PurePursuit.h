#ifndef PUREPURSUIT_H
#define PUREPURSUIT_H

#include <string>
#include <cmath>
#include <vector>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class PPControl {
    public:
        // Constructor
        PPControl(const ros::NodeHandle &nh);
        // Destructor
        virtual ~PPControl();template<typename A, typename B>
        
        double getDistance(A punto1, B punto2){
            return sqrt(pow(punto1.x - punto2.x,2) + pow(punto1.y - punto2.y,2) + pow(punto1.z - punto2.z,2));
        }

    private:
        // Obtención de la trayectoria
        void getPath(const nav_msgs::Path &path);
        void getOdom(const nav_msgs::Odometry& odom);
        void timerCallback(const ros::TimerEvent& event);
        geometry_msgs::PoseStamped getPose();
        geometry_msgs::TransformStamped getTF_BLMap();
        void getPose_MapBL(const geometry_msgs::Pose& pose_map);
        double getPoseDist(const geometry_msgs::PoseStamped& pose);


        int getWayPoint();

        ros::NodeHandle _nh;
        ros::Subscriber _pathSub;
        ros::Subscriber _odomSub;
        ros::Publisher _cmdVelPub;

        std::string _pathTopic;
        std::string _odomTopic;
        std::string _cmdVelTopic;
        std::string _robotFrameid;

        tf::TransformListener _tfListener;
        tf2_ros::Buffer _tfBuffer;
        geometry_msgs::TransformStamped _tf_BLMap;
        ros::Timer _timer;

        double _velocity;
        double _controlfreq;
        int _index;
        bool _recorrido;
        int _nextWP;

        double _ld;

        // Variables relacionadas con el path actual
        nav_msgs::Path _cpath;
        std::string _pathFrameid;
        std::string _mapFrameid;
        int _pathlength;
        geometry_msgs::Pose _cpathFinalPose;

        geometry_msgs::Twist _cVel;
        geometry_msgs::PoseStamped _cPose;

};

#endif