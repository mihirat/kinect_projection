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
	
	// pseudo
	for(int j= 0;j<last.size();j++){
		double value = last[j] /4;
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
	
	// calc acceleraton and velocity 
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

double AngleBetweenTwoVectors(vector<double> vecA, vector<double> vecB)
{
	double inner = vecA[0] * vecB[0] + vecA[1] * vecB[1] + vecA[2] * vecB[2];
	double sqrtA = sqrt(vecA[0] * vecA[0] + vecA[1] * vecA[1] + vecA[2] * vecA[2]);
	double sqrtB = sqrt(vecB[0] * vecB[0] + vecB[1] * vecB[1] + vecB[2] * vecB[2]);
	double cos = inner / (sqrtA * sqrtB);
	return double(acos(cos));	
}

vector<double> getAngles(vector<double> pos)
{
	vector<double> angle;
	double x = pos[0];
	double y = pos[1];
	double z = pos[2];
	
	double r = sqrt(x*x + y*y + z*z);
	double t = atan2(y,x);
	double p = acos(z/r);
	angle.push_back(t);
	angle.push_back(p);
	return angle;
}

vector<double> getDegsElbow(const vector<double>& pos){

	vector<double> degs;
	vector<double> Lsh2el,Lel2hand,Rsh2el,Rel2hand;

	for(int i=0;i<3;i++){
		Lsh2el.push_back(pos[3+i]-pos[0+i]);
		Lel2hand.push_back(pos[6+i]-pos[3+i]);
		Rsh2el.push_back(pos[12+i]-pos[9+i]);
		Rel2hand.push_back(pos[15+i]-pos[12+i]);
	}

	double degL = AngleBetweenTwoVectors(Lsh2el, Lel2hand);	
	double degR = AngleBetweenTwoVectors(Rsh2el, Rel2hand);
	
	degs.push_back(degL);
	degs.push_back(degR);
	return degs;
}

vector<double> getDegsShoulder(const vector<double>& pos){

	vector<double> degs;
	vector<double> Lsh2el, Rsh2el;
	for(int i=0;i<3;i++){
		Lsh2el.push_back(pos[3+i]-pos[0+i]);
		Rsh2el.push_back(pos[12+i]-pos[9+i]);
	}
	vector<double> degsL = getAngles(Lsh2el);
	vector<double> degsR = getAngles(Rsh2el);
	
	degs.push_back(degsL[0]);
	degs.push_back(degsL[1]);
	degs.push_back(degsR[0]);
	degs.push_back(degsR[1]);
	return degs;
}

void smoothing(vector< vector<double> >& postraj, int window)
{
	vector< vector<double> > smoothed = postraj;
	int dim = postraj[0].size();
	for(int d=0;d<dim;d++){
		for(int l=0;l<postraj.size()-window;l++){

			int current = 0;
			for(int w=0;w<window;w++){
				current = current + postraj[l+w][d];
			}
			smoothed[l][d] = current / window;
		}
	}
	postraj = smoothed;
}

void changePos2Angle(vector< vector<double> >& postraj)
{
	vector< vector<double> > angletraj;
	for(int i=0;i<postraj.size();i++){
		vector<double> tmp = postraj[i];
		vector<double> degs_elbow = getDegsElbow(tmp);
		vector<double> degs_shoulder = getDegsShoulder(tmp);
		vector<double> eefangles(3,0.0);
		degs_shoulder.insert(degs_shoulder.begin() + 2, degs_elbow[0]);
		degs_shoulder.insert(degs_shoulder.begin() + 2, 0.0);
		degs_shoulder.insert(degs_shoulder.begin() + 4, eefangles.begin(),eefangles.end());
		degs_shoulder.push_back(0.0);
		degs_shoulder.push_back(degs_elbow[1]);
		degs_shoulder.insert(degs_shoulder.end(), eefangles.begin(),eefangles.end());

		angletraj.push_back(degs_shoulder);
	}
	postraj = angletraj;
}


void modifyCoordinate(vector< vector<double> >& traj)
{
	vector<double> modvalues(2, 0.0);
	modvalues[0] = -1.57; // shoulder x-y
	modvalues[1] = 3.14; // shoulder z
	
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
	int window = 5;

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
			cout<<"which motion? (1: jojo, 2: boxing, 3: conduct)"<<endl;
			int lequel;
			cin >> lequel;
			string filename;
			switch (lequel){
				case 0:
				{
					filename = ros::package::getPath("kinect_projection") + "/data/hadouken.csv";
					break;
				}
				case 1:
				{
					filename = ros::package::getPath("kinect_projection") + "/data/jojo.csv";
					break;
				}
				case 2:
				{
					filename = ros::package::getPath("kinect_projection") + "/data/boxing.csv";
					break;
				}
				case 3:
				{
					filename = ros::package::getPath("kinect_projection") + "/data/conduct.csv";
					break;
				}
				default:
				{
					filename = ros::package::getPath("kinect_projection") + "/data/jojo.csv";
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
			
			// something wrong
			//~ smoothing(traj_both,window);
			changePos2Angle(traj_both);
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
					// visualize planned path
					display_trajectory.trajectory_start = state;
					display_trajectory.trajectory.push_back(moveit_traj);
					display_publisher.publish(display_trajectory);
								
					ros::Duration(len*dt/2).sleep();
					// execute the path
					executePathRequest(ExecutePath_client, moveit_traj, wait_for_execution);		
				}
				else{
					cout << "finish process... " << endl;
					return 0;
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
