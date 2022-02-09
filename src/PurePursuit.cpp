#include "PurePursuit.h"
// #include </home/jborrego/catkin_ws_pp/src/my_purepursuit/include/PurePursuit.h>


    // Constructor
    PPControl::PPControl(const ros::NodeHandle &nh): _nh(nh){
        _pathSub = _nh.subscribe(_pathTopic,100,&PPControl::getPath,this); // "/move_base/TrajectoryPlannerROS/local_plan"
        _odomSub = _nh.subscribe(_odomTopic,100,&PPControl::getOdom,this);
        _cmdVelPub = _nh.advertise<geometry_msgs::Twist>(_cmdVelTopic, 100);
        _timer = _nh.createTimer(ros::Duration(1.0/_controlfreq),&PPControl::timerCallback, this);
    }

    PPControl::~PPControl(){};

    void PPControl::getPath(const nav_msgs::Path &path){
        _cpath = path;
        _pathFrameid = path.header.frame_id;
        _pathlength = path.poses.size();
        _cpathFinalPose = path.poses.back().pose;
        _index = 0;

        // Comprobamos que la trayectoria no esté vacía
        if (_pathlength>0){
            _recorrido=false;
            // Obtengo el primer punto con el que empezará el algoritmo
        }
        else{
            _recorrido=true;
            std::cout << "Trayectoria vacía, ingrese nueva trayectoria" << std::endl;
        }
    }

    void PPControl::getOdom(const nav_msgs::Odometry& odom){

        // Guardo los campos cabecera y pose de la odometria actual
        _cPose.header=odom.header;
        _cPose.pose=odom.pose.pose;
        // Guardo la velocidad actual dada por la odometría
        _cVel=odom.twist.twist;
    }

    void PPControl::timerCallback(const ros::TimerEvent& event){
        
    }

    geometry_msgs::PoseStamped PPControl::getPose(){

        geometry_msgs::PoseStamped tfPose;

        // Intentamos transformar la pose actual al frame_id del path
        try {
            // La pose transformada tendra ya dicho frame_id
		    _tfListener.transformPose(_pathFrameid, _cPose, tfPose);
        }

        // Si falla lanzamos una excepcion
        catch (tf::TransformException& exception) {
            ROS_ERROR_STREAM("Error en PPControl::getPose: " << 
            exception.what());
        }

        return tfPose;
    }

    double PPControl::getPoseDist(const geometry_msgs::PoseStamped& pose){

        geometry_msgs::PoseStamped origen = getPose();
        geometry_msgs::PoseStamped tfPose; 

        // Intentamos transformar la pose que recibe como argumento al frame_id del path
        try {
            // La pose transformada tendra ya dicho frame_id
		    _tfListener.transformPose(_pathFrameid, _cPose, tfPose);
        }

        // Si falla lanzamos una excepcion
        catch (tf::TransformException& exception) {
            ROS_ERROR_STREAM("Error en PPControl::getPoseDist: " << 
            exception.what());
            return -1;
        }

        /* Calculamos la distancia desde la pose origen (la actual) hasta
           la pose que acabamos de transformar mediante vectores que contienen
           las coordenadas (x,y,z) de cada pose */  
        tf::Vector3 org(origen.pose.position.x, origen.pose.position.y, origen.pose.position.z);
        tf::Vector3 dest(tfPose.pose.position.x,tfPose.pose.position.y, tfPose.pose.position.z);

        return tf::tfDistance(org,dest);
    }

    int PPControl::getWayPoint(){

        if (!_cpath.poses.empty()){
            int wp = 0;
            /* Inicializo la distancia minima a la distancia desde
            la pose actual hasta la primera pose del path actual */
            double dist_min = getPoseDist(_cpath.poses.front());

        }
    }


