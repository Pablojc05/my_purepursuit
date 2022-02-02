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

class PPControl {
    public:
        // Constructor
        PPControl(const ros::NodeHandle &nh);
        // Destructor
        virtual ~PPControl();

    private:
        // Obtención de la trayectoria
        void getPath(const nav_msgs::Path &path);

        ros::NodeHandle _nh;
        ros::Subscriber _pathSub;
        ros::Publisher _cmdVelPub;
        double _velocity;
        int _index;
        bool _recorrido;
        nav_msgs::Path _path;    
};

#endif