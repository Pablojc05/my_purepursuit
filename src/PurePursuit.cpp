#include "include/PurePursuit.h"


    // Constructor
    PPControl::PPControl(const ros::NodeHandle &nh): _nh(nh){
        _pathSub = _nh.subscribe("/move_base/TrajectoryPlannerROS/local_plan",100,&PPControl::getPath,this);
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


