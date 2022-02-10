#include "PurePursuit.h"
// #include </home/jborrego/catkin_ws_pp/src/my_purepursuit/include/PurePursuit.h>


    // Constructor
    PPControl::PPControl(const ros::NodeHandle &nh): _nh(nh), _nextWP(-1){
        _pathSub = _nh.subscribe(_pathTopic,1,&PPControl::getPath,this); // "/move_base/TrajectoryPlannerROS/local_plan"
        _odomSub = _nh.subscribe(_odomTopic,1,&PPControl::getOdom,this);
        _cmdVelPub = _nh.advertise<geometry_msgs::Twist>(_cmdVelTopic, 1);
        _timer = _nh.createTimer(ros::Duration(1.0/_controlfreq),&PPControl::timerCallback, this);
    }

    PPControl::~PPControl(){};

    void PPControl::getPath(const nav_msgs::Path &path){

        /* Si se recibe un nuevo path y tiene el frame_id del mapa se sobreescribe el anterior */
        if (path.header.frame_id==_mapFrameid){
            _cpath = path;
            _pathFrameid = path.header.frame_id;
            _pathlength = path.poses.size();
            _cpathFinalPose = path.poses.back().pose;
            _nextWP = 0;

            // Comprobamos que la trayectoria no esté vacía
            if (_pathlength>0){
                _recorrido=false;
                // Obtengo el primer punto con el que empezará el algoritmo
            }
            else{
                _recorrido=true;
                ROS_WARN_STREAM("Trayectoria vacía, ingrese nueva trayectoria");
            }
        }
        else    ROS_WARN_STREAM("El frame_id del path debe ser " << _mapFrameid << 
                " pero esta publicado en el frame " << path.header.frame_id);
        
    }

    void PPControl::getOdom(const nav_msgs::Odometry& odom){

        // Guardo los campos cabecera y pose de la odometria actual
        _cPose.header=odom.header;
        _cPose.pose=odom.pose.pose;
        // Guardo la velocidad actual dada por la odometría
        _cVel=odom.twist.twist;

        
        _tf_BLMap = getTF_BLMap();



    }

    geometry_msgs::TransformStamped PPControl::getTF_BLMap(){
        
        tf2_ros::TransformListener tfListener(_tfBuffer);
        geometry_msgs::TransformStamped tfGeom;
        try{
            tfGeom = _tfBuffer.lookupTransform(_mapFrameid, _robotFrameid, ros::Time(0));
        }
        
        catch (tf2::TransformException &ex) {
            ROS_WARN_STREAM("Error en PPControl::getTF_BLMap: " << ex.what());
        }

        return tfGeom;
    }

    void PPControl::getPose_MapBL(const geometry_msgs::Pose& pose_map){
        
        geometry_msgs::TransformStamped tfGeom;
        // probar con tf2::transform 
        // tf2::Transform tf;
        // tf.setRotation(tf2::Quaternion(pose_map.orientation.x, pose_map.orientation.y, pose_map.orientation.z, pose_map.orientation.w));
        // tf.setOrigin(tf2::Vector3(pose_map.position.x, pose_map.position.y, pose_map.position.z));
        // tf2::Transform tf;
        // tf.setRotation(tf2::Quaternion(_tf_BLMap.transform.rotation.x, _tf_BLMap.transform.rotation.y, _tf_BLMap.transform.rotation.z, _tf_BLMap.transform.rotation.w));
        // tf.setOrigin(tf2::Vector3(_tf_BLMap.transform.translation.x, _tf_BLMap.transform.translation.y, _tf_BLMap.transform.translation.z));
        tf2::Stamped<tf2::Transform> tf;
        tf2::fromMsg(_tf_BLMap, tf);
        
        
        // tf.inverse()
        // return tfGeom;
    }

    int PPControl::getWayPoint(){

        // if (!_cpath.poses.empty()){
        //     int wp = 0;
        //     /* Inicializo la distancia minima a la distancia desde
        //     la pose actual hasta la primera pose del path actual */
        //     double dist_min = getPoseDist(_cpath.poses.front());

        // }

        geometry_msgs::Vector3 pos_robot = _tf_BLMap.transform.translation;

        for (int i=0; i < _cpath.poses.size(); i++){
            geometry_msgs::Point pos_wp = _cpath.poses[i].pose.position;
            
            if (getDistance(pos_wp, pos_robot) > _ld){

            }
        }
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


