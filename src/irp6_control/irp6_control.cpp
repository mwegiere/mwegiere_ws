#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <serwo/SerwoInfo.h>
#include <cmath>
#include <vector>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <tf/transform_listener.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <actionlib/client/simple_action_client.h>

//Define a marker to be published in a topic and visualize in Rviz
//Used to visualize grid plate
//konwencja
//T_aB - pozycja ukladu a w ukladzie B
//w mojej notacji a stoi na dole, a B stoi na gorze
class Irp6Control
{
public:
    Irp6Control(ros::NodeHandle &nh);
    //dane o obrazie otrzymane z topica
    std::vector<double> T_gC_vector;
    int found;

    int frequency; //czestotliwosc wysylania sterowania dla robota

    std::vector<double> error; //chyby dla 6 stopni swobody
    std::vector<double> minError; //minimalny uchyb przy ktorym zostanie wyliczona nowa predkosc

    std::vector<double> newVel; //wektro predkosci dla 6 stopni swobody
    std::vector<double> newAcceleration; //wektro przyspieszenia dla 6 stopni swobody

    std::vector<double> p; //wartosc p dla regulatora PID
    std::vector<double> i; //wartosc p dla regulatora PID
    std::vector<double> d; //wartosc p dla regulatora PID

    std::vector<double> maxVel;
    std::vector<double> maxAcceleration;
    std::vector<double> new_pos_cartesian;

    //kartezajanskie ograniczenia pozycji jako trojki xyz
    //w ukladzie zwiazanym z baza robota
    //przetestowac ograniczenia na prawdziwym robocie
    std::vector<double> min_pos; //xyz
    std::vector<double> max_pos; //xyz

    std::vector<double> w_vector[3][8];

    void limits_test();
    void callback();
    void calculateNewVel();
    void calculateNewCartesianPose();
    void calculateNewAcceleration();
    void checkVelocityLimits();
    void checkAccelerationLimits();
    void checkCartesianLimits();
    void setZeroValues();
    void setNewVelocity();
    void calculateAndSetNewValues();
    void TestLimits();
    void regulatroDecomposition();

    void moveToJointPosition(std::vector<double>, double);

   
private:

};
void Irp6Control::moveToJointPosition(std::vector<double> pos, double time_from_start){

    actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > trajectory_action_("/irp6p_arm/spline_trajectory_action_joint", true);

    trajectory_action_.waitForServer();
    //std::cout<<trajectory_action_.isServerConnected()<<std::endl;

    control_msgs::FollowJointTrajectoryGoal jointGoal;

    jointGoal.trajectory.joint_names.push_back("joint1");
    jointGoal.trajectory.joint_names.push_back("joint2");
    jointGoal.trajectory.joint_names.push_back("joint3");
    jointGoal.trajectory.joint_names.push_back("joint4");
    jointGoal.trajectory.joint_names.push_back("joint5");
    jointGoal.trajectory.joint_names.push_back("joint6");

    //number of waypoints
    jointGoal.trajectory.points.resize(1);

    int ind = 0;
    jointGoal.trajectory.points[ind].positions.resize(6);

    jointGoal.trajectory.points[ind].positions[0] = pos[1];
    jointGoal.trajectory.points[ind].positions[1] = pos[2];
    jointGoal.trajectory.points[ind].positions[2] = pos[3];
    jointGoal.trajectory.points[ind].positions[3] = pos[4];
    jointGoal.trajectory.points[ind].positions[4] = pos[5];
    jointGoal.trajectory.points[ind].positions[5] = pos[6];

    jointGoal.trajectory.points[ind].time_from_start = ros::Duration(time_from_start);

    jointGoal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.2);

    trajectory_action_.sendGoal(jointGoal);

    trajectory_action_.waitForResult();

    //std::cout<<trajectory_action_.getState().toString();
    ROS_INFO("Sent Goal");
}

Irp6Control::Irp6Control(ros::NodeHandle &nh)
  {
    found = 1;

    frequency = 500;

    error.push_back(0);
    error.push_back(0);
    error.push_back(0);
    error.push_back(0);
    error.push_back(0);
    error.push_back(0);

    minError.push_back(0.01);
    minError.push_back(0.01);
    minError.push_back(0.01);
    minError.push_back(0.01);
    minError.push_back(0.01);
    minError.push_back(0.01);

    newVel.push_back(0);
    newVel.push_back(0);
    newVel.push_back(0);
    newVel.push_back(0);
    newVel.push_back(0);
    newVel.push_back(0);

    newAcceleration.push_back(0);
    newAcceleration.push_back(0);
    newAcceleration.push_back(0);
    newAcceleration.push_back(0);
    newAcceleration.push_back(0);
    newAcceleration.push_back(0);

    p.push_back(0);
    p.push_back(0.5);
    p.push_back(0);
    p.push_back(0);
    p.push_back(0);
    p.push_back(0);

    i.push_back(0);
    i.push_back(0);
    i.push_back(0);
    i.push_back(0);
    i.push_back(0);
    i.push_back(0);

    d.push_back(0);
    d.push_back(0);
    d.push_back(0);
    d.push_back(0);
    d.push_back(0);
    d.push_back(0);

    maxVel.push_back(0.05);
    maxVel.push_back(0.05);
    maxVel.push_back(0.05);
    maxVel.push_back(0.0);
    maxVel.push_back(0.0);
    maxVel.push_back(0.0);

    maxAcceleration.push_back(25.0);
    maxAcceleration.push_back(25.0);
    maxAcceleration.push_back(25.0);
    maxAcceleration.push_back(0.0);
    maxAcceleration.push_back(0.0);
    maxAcceleration.push_back(0.0);

    new_pos_cartesian.push_back(0.0);
    new_pos_cartesian.push_back(0.0);
    new_pos_cartesian.push_back(0.0);

    min_pos.push_back(0.8);
    min_pos.push_back(-0.3);
    min_pos.push_back(1.2);

    max_pos.push_back(0.9);
    max_pos.push_back(0.3);
    max_pos.push_back(1.4);

    /*w_vector[0] = {max_pos[0], max_pos[1], min_pos[2]};
    w_vector[1] = {min_pos[0], max_pos[1], min_pos[2]};
    w_vector[2] = {min_pos[0], min_pos[1], min_pos[2]};
    w_vector[3] = {max_pos[0], min_pos[1], min_pos[2]};
    w_vector[4] = {max_pos[0], max_pos[1], max_pos[2]};
    w_vector[5] = {min_pos[0], max_pos[1], max_pos[2]};
    w_vector[6] = {min_pos[0], min_pos[1], max_pos[2]};
    w_vector[7] = {max_pos[0], min_pos[1], max_pos[2]};*/

  }


int main(int argc, char **argv) {  
  ros::init(argc, argv,"serwovision_control");
  ros::NodeHandle nh;
  Irp6Control control(nh);

  std::vector<double> pos;
  pos.push_back(0.0);
  pos.push_back(-1.57079632679);
  pos.push_back(0.0);
  pos.push_back(0.0);
  pos.push_back(4.71238898038);
  pos.push_back(1.57079632679);

  control.moveToJointPosition(pos,10.0);
  ros::spinOnce();
  /*while (ros::ok())
  {
    control.moveToJointPosition();
    ros::spinOnce();
  }*/

  return 0;
}

