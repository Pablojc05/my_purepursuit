#include "include/PurePursuit.h"


    // Constructor
    PPControl::PPControl(const ros::NodeHandle &nh): _nh(nh){
        _pathSub = _nh.subscribe(_pathTopic,100,&PPControl::getPath,this); // "/move_base/TrajectoryPlannerROS/local_plan"
        _odomSub = _nh.subscribe(_odomTopic,100,&PPControl::getOdom,this);
        _cmdVelPub = _nh.advertise<geometry_msgs::Twist>(_cmdVelTopic, 100);
        _timer = _nh.createTimer(ros::Duration(1.0/_controlfreq),&PPControl::timerCallback, this);
    }

    PPControl::~PPControl(){};

    void PPControl::getPath(const nav_msgs::Path &path){
        _path = path;
        _index = 0;
        // Comprobamos que la trayectoria no esté vacía
        if (path.poses.size()>0){
            _recorrido=false;
        }
        else{
            _recorrido=true;
            std::cout << "Trayectoria vacía, ingrese nueva trayectoria" << std::endl;
        }
    }

    void PPControl::getOdom(const nav_msgs::Odometry& odom){

    }

    void PPControl::timerCallback(const ros::TimerEvent& event){
        
    }


