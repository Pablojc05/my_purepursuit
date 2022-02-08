#ifndef PUREPURSUIT_H
#define PUREPURSUIT_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_listener.h>

class PPControl {
    public:
        // Constructor
        PPControl(const ros::NodeHandle &nh);
        // Destructor
        virtual ~PPControl();

    private:
        // Obtención de la trayectoria
        void getPath(const nav_msgs::Path &path);
        void getOdom(const nav_msgs::Odometry& odom);
        void timerCallback(const ros::TimerEvent& event);

        ros::NodeHandle _nh;
        ros::Subscriber _pathSub;
        ros::Subscriber _odomSub;
        ros::Publisher _cmdVelPub;

        std::string _pathTopic;
        std::string _odomTopic;
        std::string _cmdVelTopic;

        tf::TransformListener _tfListener;
        ros::Timer _timer;

        double _velocity;
        double _controlfreq;
        int _index;
        bool _recorrido;

        // Variables relacionadas con el path actual
        nav_msgs::Path _cpath;
        std::string _pathFrameid;
        int _pathlength;
        geometry_msgs::Pose _cpathFinalPose;

        geometry_msgs::Twist _cVel;
        geometry_msgs::PoseStamped _cPose;

};

#endif