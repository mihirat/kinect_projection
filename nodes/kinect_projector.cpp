#include "ros/ros.h"
#include <iostream>

#include <fstream>
#include <sstream>

#include <limits>
#include <vector>
#include <cmath>
#include <algorithm>
#include <stdlib.h>
#include <time.h>
#include <string>

#include <pluginlib/class_loader.h>
#include "sensor_msgs/JointState.h"

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayTrajectory.h>

// Robot state publishing
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>

// execution
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/RobotState.h>


using namespace std;

void diffTraj(vector< vector<double> >& traj, vector< vector<double> >& diffedtraj, double dt)
{
	for (int i=0;i<traj.size()-1;i++){
		vector<double> tmp, tmp2, diffed;
		tmp = traj[i];
		tmp2 = traj[i+1];

		for(int j= 0;j<tmp.size();j++){
			double value = (tmp2[j] - tmp[j]) / dt;
			diffed.push_back(value);
		}
		diffedtraj.push_back(diffed);
	}
	vector<double> last = diffedtraj.back();
	
	for(int j= 0;j<last.size();j++){
		double value = last[j] /2;
		last[j] = value;
	}
	diffedtraj.push_back(last);
}

void vector2moveit(vector< vector<double> >& traj, moveit_msgs::RobotTrajectory& moveit_traj, 
	moveit_msgs::RobotState& state, double dt, vector<string> joint_names, string frame_id)
{
	trajectory_msgs::JointTrajectory joint_traj = trajectory_msgs::JointTrajectory();
	trajectory_msgs::MultiDOFJointTrajectory multi_joint_traj;
	sensor_msgs::JointState initPoint;
	sensor_msgs::MultiDOFJointState multi_initPoint;
	
	// set parameters to header for joint trajectory
	std_msgs::Header header = std_msgs::Header();
	int seq = 0;
	int sec = 0;
	int nsec = 0;
	
	ros::Time stamp(sec);
	stamp.sec = sec;
	stamp.nsec = nsec;
	header.seq = seq;
	header.stamp = stamp;
	header.frame_id = frame_id;
		
	vector<trajectory_msgs::JointTrajectoryPoint> points;
	vector<double> empty;
	
	// calc acceleraton from velocities
	vector< vector<double> > vel, accel; 
	diffTraj(traj, vel, dt);
	diffTraj(vel, accel, dt);
	
	for(int i=0;i<traj.size();i++){
		trajectory_msgs::JointTrajectoryPoint point = trajectory_msgs::JointTrajectoryPoint();
		point.positions = traj[i];
		point.velocities = vel[i];
		point.accelerations = accel[i];
		point.effort = empty;
		double time = dt * i;
		ros::Duration tfs(time);
		point.time_from_start = tfs;
		
		points.push_back(point);		
	}
	vector<double>  initPos, initVel; 
	initPos = traj[0];
	initVel = vel[0];
	
	// set parameters to joint trajectory
	joint_traj.header = header;
	joint_traj.joint_names = joint_names;
	joint_traj.points = points;	
	
	// set parameters to multi dof joint trajectory
	frame_id = "";
	header.frame_id = frame_id;
	vector<string> empty_s;
	vector<trajectory_msgs::MultiDOFJointTrajectoryPoint> empty_multipoints;

	multi_joint_traj.header = header;
	multi_joint_traj.joint_names = empty_s;
	multi_joint_traj.points = empty_multipoints;
	
	moveit_traj.joint_trajectory = joint_traj;
	moveit_traj.multi_dof_joint_trajectory = multi_joint_traj;	
	
	// set vaiables to state
	initPoint.header = header;
	initPoint.name = joint_names;
	initPoint.position = initPos;
	initPoint.velocity = initVel;
	initPoint.effort = empty;
	
	multi_initPoint.header = header;
	multi_initPoint.joint_names = empty_s;
		
	state.joint_state = initPoint;
	state.multi_dof_joint_state = multi_initPoint;
}


void executePathRequest(ros::ServiceClient client, moveit_msgs::RobotTrajectory& traj, bool wfe)
{
	moveit_msgs::ExecuteKnownTrajectory srv;
	srv.request.trajectory = traj;
	srv.request.wait_for_execution = wfe;
	
	if(client.call(srv))
	{
		cout << "!!Execute Motion!!" << endl;
	}else
	{
		cout << "Failed path execution. something wrong..." << endl;
	}
}


bool loadCSV(const string filename, vector< vector<double> >& table, const char delimiter = ',')
{
    // open file
    fstream filestream(filename.c_str());
    if (!filestream.is_open())
    {
        //if failed, finish
        return false;
    }
    // load file contents
    while (!filestream.eof())
    {
        // read 1 line
        string buffer;
        filestream >> buffer;
       
        string::size_type sz;
        //separate and add to list
        vector<double> record;              
        istringstream streambuffer(buffer);  
        string token;                        
        while (getline(streambuffer, token, delimiter))
        {
			double value = atof(token.c_str());
            record.push_back(value);
        }  
        table.push_back(record);
    }
    return true;
}

void modifyCoordinate(vector< vector<double> >& traj)
{
	vector<double> modvalues(2, 0.0);
	modvalues[0] = -1.57; // shoulder x-y
	modvalues[1] = 3.14; // shoulder z
	//~ modvalues[3] = 0; // elbow
	
	for(int i=0;i<traj.size();i++){
		vector<double> tmp = traj[i];

		for(int k=0;k<tmp.size();k++){
			tmp[k] = - tmp[k];
		}
		for(int k=0;k<2;k++){
			tmp[k] = tmp[k] + modvalues[k];
			tmp[k+7] = tmp[k+7] + modvalues[k];
		}
		traj[i] = tmp;
	}
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "kinect_projector");
	ros::NodeHandle n;
	
	// load robot description
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
	kinematic_state->setToDefaultValues();

	int dims = 14;
	double dt = 0.1;

	// pr2 joint names
	string jnames[] = {"l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint", "r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"};
	vector<string> joint_names(jnames, jnames + dims);
	string frame_id = "/odom_combined";

	// prepare to show planned path
	ros::Publisher display_publisher = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	ros::ServiceClient ExecutePath_client = n.serviceClient<moveit_msgs::ExecuteKnownTrajectory>("execute_kinematic_path");
	
	ros::Rate loop_rate(10);
	bool isFirst = true; // to avoid non-subscription problem

	while (ros::ok()){
		if(!isFirst){
			vector< vector<double> >  traj_both;

			// load .csv			
			cout<<"which motion? (0: hadouken, 1: jojo, 2: boxing, 3: banzai, 4: wish, 5: compose)"<<endl;
			int lequel;
			cin >> lequel;
			string filename;
			switch (lequel){
				case 0:
				{
					filename = ros::package::getPath("kinect_projection") + "/data/hadouken.csv";
					break;
				}
				default:
				{
					filename = ros::package::getPath("kinect_projection") + "/data/kinect.csv";
					break;
				}
								
			}
			
			bool status = false;
			status = loadCSV(filename, traj_both);
			if(!status){
				cout << "kinect csv loading failed." << endl;
				return -1;
			}
			
			traj_both.pop_back();
			modifyCoordinate(traj_both);
			
			int len = traj_both.size();
			cout << "loaded trajectory length is " << len << endl;						
			if (len != 0){
													
				moveit_msgs::RobotTrajectory moveit_traj;
				moveit_msgs::RobotState state;
				vector2moveit(traj_both, moveit_traj, state, dt, joint_names, frame_id);

				bool wait_for_execution = true;			
				moveit_msgs::DisplayTrajectory display_trajectory;
					
				cout<<"Execute motion? (y/n)"<<endl;
				char answer;
				cin >> answer;
				if (answer == 'y'){
					display_trajectory.trajectory_start = state;
					display_trajectory.trajectory.push_back(moveit_traj);
					display_publisher.publish(display_trajectory);
								
					ros::Duration(len*dt).sleep();
					executePathRequest(ExecutePath_client, moveit_traj, wait_for_execution);		
				}			
			}
		}
		else{
			ros::Duration(1.0).sleep();
			isFirst = false;
		}			
        ros::spinOnce();
    	loop_rate.sleep();
  	}

	return 0;
}
