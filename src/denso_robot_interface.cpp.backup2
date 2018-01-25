/*
 * Copyright @ CERLAB
 * this is a MoveIt interface for robot trajectory
 * execution
 */

#include <vector>
#include <algorithm>
#include <fstream>

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/Mesh.h>

#include <std_msgs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <denso_robot_interface/MarkState.h>
#include <denso_robot_interface/ClearStates.h>
#include <denso_robot_interface/DeleteState.h>
#include <denso_robot_interface/ShowStates.h>
#include <denso_robot_interface/ExecuteTrajectory.h>
#include <denso_robot_interface/GoTo.h>
#include <denso_robot_interface/Translation.h>
#include <denso_robot_interface/Upload.h>
#include <denso_robot_interface/JointState.h>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>

class DensoRobotInterface {
 private:
  // frame transformer
  std::string target_frame;
  tf::TransformListener tf_listener;

  // buffer for trajectory (both in Joint Space and Cartesian Space)
  std::vector<std::string> waypoints;
  std::vector<geometry_msgs::Pose> pose_buffer;

  // current pose for the end effector
  geometry_msgs::Pose current_pose;
  std::vector<double> current_jnt_value;
  boost::shared_mutex jnt_update_lock;
  boost::shared_mutex pose_update_lock;

  // ros publisher and subscriber
  ros::NodeHandle nh, pnh;
  ros::Publisher  eef_pose;
  ros::Publisher  jnt_values;

  // services provided
  ros::ServiceServer mark_state, clear_states, show_states;
  ros::ServiceServer execute_trajectory;
  ros::ServiceServer go_to;
  ros::ServiceServer translation;
  ros::ServiceServer upload_states;
  ros::ServiceServer download_states;

  // moveit interface
  moveit::planning_interface::MoveGroupInterface manipulator;
  moveit::planning_interface::PlanningSceneInterface planning_scene;

 public:
  /*
   * Initialization
   * this will setup all the topics and services this node provides
   */
  DensoRobotInterface() : manipulator("arm"), nh("denso_robot_interface"), pnh("~") {
    ROS_INFO("[rbt_trj] Initializing robot_interface node...");

    // publish end effector pose
    ROS_INFO("[rbt_trj] Setting up pose/jnt_state publisher...");
    this->eef_pose = this->nh.advertise<geometry_msgs::PoseStamped>("eef_pose", 10);
    this->jnt_values = this->nh.advertise<denso_robot_interface::JointState>("jnt_values", 10);
    ROS_INFO("[rbt_trj] Done.");

    // start services
    ROS_INFO("[rbt_trj] Starting services...");
    this->mark_state = this->nh.advertiseService("mark_state", &DensoRobotInterface::mark_stateCb, this);
    this->clear_states = this->nh.advertiseService("clear_states", &DensoRobotInterface::clear_statesCb, this);
    this->show_states = this->nh.advertiseService("show_states", &DensoRobotInterface::show_statesCb, this);
    this->execute_trajectory = this->nh.advertiseService("execute_trajectory", &DensoRobotInterface::execute_trajectoryCb, this);
    this->go_to = this->nh.advertiseService("go_to", &DensoRobotInterface::go_toCb, this);
    this->translation = this->nh.advertiseService("translation", &DensoRobotInterface::translationCb, this);
    this->upload_states = nh.advertiseService("upload_states", &DensoRobotInterface::upload_statesCb, this);
    // TODO(Bosch)
    // Service: download states
    ROS_INFO("[rbt_trj] Done.");

    // load buildbox into planning scene
    // std::map<std::string, std::string> models;
    // std::string model_ref_frame = this->pnh.param<std::string>("workspace/ref_frame", "robot_work_frame");
    // if (this->pnh.getParam("workspace/meshes", models) == false) {
    //   ROS_WARN("[rbt_trj] Workcell environment not loaded, yaml file not found.");
    // } else {
    //   ROS_INFO("[rbt_trj] Loading workspace environment...");
    //   for (std::map<std::string, std::string>::iterator it = models.begin(); it != models.end(); ++ it) {
    //     moveit_msgs::CollisionObject collision_object;
    //     collision_object.header.frame_id = model_ref_frame;
    //     collision_object.id = it->first;

    //     shapes::Mesh *cell = shapes::createMeshFromResource(it->second);
    //     if (!cell) {
    //       ROS_WARN("[rbt_trj] Work cell environment not loaded. File not found.");
    //       continue;
    //     }
    //     shapes::ShapeMsg mesh_msg;
    //     shapes::constructMsgFromShape(cell, mesh_msg);
    //     shape_msgs::Mesh mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
    //     collision_object.meshes.resize(1);
    //     collision_object.mesh_poses.resize(1);

    //     collision_object.meshes[0] = mesh;
    //     collision_object.mesh_poses[0].position.x = 0.f;
    //     collision_object.mesh_poses[0].position.y = 0.f;
    //     collision_object.mesh_poses[0].position.z = 0.f;
    //     collision_object.mesh_poses[0].orientation.w = 1.0;
    //     collision_object.mesh_poses[0].orientation.x = 0.f;
    //     collision_object.mesh_poses[0].orientation.y = 0.f;
    //     collision_object.mesh_poses[0].orientation.z = 0.f;

    //     collision_object.operation = collision_object.ADD;

    //     this->planning_scene.applyCollisionObject(collision_object);
    //   }

    //   ROS_INFO("[rbt_trj] Done.");
    // }

    // start state monitor
    ROS_INFO("[rbt_trj] Starting state monitor...");
    this->manipulator.startStateMonitor();
    ROS_INFO("[rbt_trj] Done.");
    ROS_INFO("[rbt_trj] Alright! Everyone is happy. Robot interface is online!");

    ROS_INFO("[rbt_trj] -------------------------------------------------");
    ROS_INFO("[rbt_trj] Here is some info w.r.t move_group:");
    // joint names
    ROS_INFO("\t[Joint Names]:");
    std::vector<std::string> joint_names = this->manipulator.getJointNames();
    for (int jnt_index = 0; jnt_index < joint_names.size(); jnt_index ++) {
      ROS_INFO("\t\t[%d]: %s", jnt_index + 1, joint_names[jnt_index].c_str());
    }
    // planning frame
    ROS_INFO("\t[Planning Frame]:");
    ROS_INFO("\t\t%s", this->manipulator.getPlanningFrame().c_str());
    // default planner id
    this->manipulator.setPlannerId("RRTConnectkConfigDefault");
    ROS_INFO("\t[Default PlannerID]:");
    ROS_INFO("\t\t%s", this->manipulator.getDefaultPlannerId("arm").c_str());
    // end effector name
    ROS_INFO("\t[End Effector]:");
    ROS_INFO("\t\t%s", this->manipulator.getEndEffector().c_str());

    // finished
    ROS_INFO("[rbt_trj] -------------------------------------------------");
  }

  /* Function @ mark_stateCb
   * callback function for mark_state service
   * this will mark down the current joint values and EEF pose
   */
  bool mark_stateCb(denso_robot_interface::MarkState::Request &req,
                    denso_robot_interface::MarkState::Response &res) {
    if (req.name == "") {
      res.success = false;
      res.reason = "Please give this waypoint a name.";
      return false;
    }

    // mark joint values
    this->manipulator.rememberJointValues(req.name);
    this->waypoints.push_back(req.name);

    res.success = true;
    res.reason = "Success!";

    return true;
  }

  /*
   * Function @ clear_statesCb
   * callback function for clear_states service
   * this will clear all the states that you marked down
   */
  bool clear_statesCb(denso_robot_interface::ClearStates::Request &req, 
                      denso_robot_interface::ClearStates::Response &res) {
    if (!this->waypoints.size()) {
      res.success = false;
      res.reason = "Well, you haven't marked down any states yet!";
    } else {
      for (std::vector<std::string>::iterator it = this->waypoints.begin(); it != this->waypoints.end(); ++ it) {
        this->manipulator.forgetJointValues(*it);
      }
      this->waypoints.clear();

      res.success = true;
      res.reason = "Success!";
    }

    return true;
  }

  /*
   * Function @ delete_stateCb
   * callback functino for delete_states service
   * this will delete a specific state according to the name given
   */
  bool delete_stateCb(denso_robot_interface::DeleteState::Request &req, 
                      denso_robot_interface::DeleteState::Response &res) {
    if (req.name == "") {
      res.success = false;
      res.reason = "Please specific the name of the state, I can't read your mind!";
    } else {
      std::vector<std::string>::iterator target = std::find(this->waypoints.begin(), this->waypoints.end(), req.name);
      if (target == this->waypoints.end()) {
        res.success = false;
        res.reason = "The name seems not to be within the states list...";
      } else {
        // the name is valid
        this->manipulator.forgetJointValues(req.name);
        this->waypoints.erase(target);
        res.success = true;
        res.reason = "Success!";
      }
    }

    return true;
  }

  /*
   * Function @ show_statesCb
   * callback for show_states service
   * this will print all the saved states in the cmd window
   */
  bool show_statesCb(denso_robot_interface::ShowStates::Request &req, 
                      denso_robot_interface::ShowStates::Response &res) {
    // compose the response
    if (!this->waypoints.size()) {
      res.success = false;
      res.reason = "Haven't stored any state yet. Have you called 'mark_state' service before?";
      return true;
    }

    res.states = this->waypoints;

    res.success = true;
    res.reason = "Success!";
    return true;
  }

  /*
   * Function @ execute_trajectoryCb
   * callback function for execute_trajectory service
   * this will execute the trajetory in order
   */
  bool execute_trajectoryCb(denso_robot_interface::ExecuteTrajectory::Request &req, 
                            denso_robot_interface::ExecuteTrajectory::Response &res) {
    if (!this->waypoints.size()) {
      res.success = false;
      res.reason = "You haven't marked down any states yet. Have you called the 'mark_state' service?";
      return true;
    }

    // execute in order
    int num_of_waypoints = (int)(this->waypoints.size()), cur_waypoint = 0;
    bool success = false;
    moveit::planning_interface::MoveGroupInterface::Plan next_waypoint;
    ROS_INFO("[rbt_trj] Executing trajectory...");
    for (std::vector<std::string>::iterator it = this->waypoints.begin(); it != this->waypoints.end(); ++ it) {
      cur_waypoint += 1;
      this->manipulator.setNamedTarget(*it);
      if (this->manipulator.plan(next_waypoint) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_INFO("[rbt_trj] Executing %s...", (*it).c_str());
        success = (this->manipulator.execute(next_waypoint) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        // if (!success) {
        //   res.success = false;
        //   res.percentage = (1.0 * (cur_waypoint - 1) / num_of_waypoints);
        //   res.reason = "Failed to execute the plan for " + *it;
        //   return true;
        // }
        // execution is a success
        ROS_INFO("[rbt_trj] Done. %lf of the trajectory has been executed.", (1.0 * cur_waypoint / num_of_waypoints));
      } else {
        res.success = false;
        res.percentage = (1.0 * (cur_waypoint - 1) / num_of_waypoints);
        res.reason = "Failed to retrieve a plan from MoveIt for " + *it;
        return true;
      }
    }

    // trajectory execution is a success
    res.success = true;
    res.percentage = (1.0 * cur_waypoint / num_of_waypoints);
    res.reason = "Success!";
    return true;
  }

  /*
   * Function @ go_toCb
   * callback function for go_to service
   * this will try to reach a state in cartesian space
   */
  bool go_toCb(denso_robot_interface::GoTo::Request &req, 
                denso_robot_interface::GoTo::Response &res) {
    moveit::planning_interface::MoveGroupInterface::Plan next_position;
    this->manipulator.setPoseTarget(req.next);
    if (this->manipulator.plan(next_position) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      ROS_INFO("[rbt_trj] Executing your pose target...");
      bool success = (this->manipulator.execute(next_position) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      // if (!success) {
      //   res.success = false;
      //   res.reason = "Failed to execute the plan.";
      //   return true;
      // }
    } else {
      res.success = false;
      res.reason = "Failed to plan the trajectory for this pose...Try to set a state which is not far away from the current state.";
      return true;
    }

    res.success = true;
    res.reason = "Success!";
    return true;
  }

  /*
   * Function @ translationCb
   * callback function for translation service
   * this will only translate the EEF thorugh cartesian space
   * useful is you just want to change the XYZ position of the EEF
   */
  bool translationCb(denso_robot_interface::Translation::Request &req, 
                      denso_robot_interface::Translation::Response &res) {
    if (!req.trajectory.size()) {
      res.success = false;
      res.reason = "Trajectory received is empty...";
      return true;
    }

    if ((req.trajectory.size() % 3) != 0) {
      res.success =false;
      res.reason = "Trajectory length should be multiple of 3. Since it should be [X1, Y1, Z1, X2, Y2, Z2, ...].";
      return true;
    }

    for (std::vector<double>::iterator it = req.trajectory.begin(); it != req.trajectory.end(); ++ it) {
      if (*it > 10) {
        // we don't have a 50m table!!
        res.success = false;
        res.reason = "Too large values detected! The matric of the trajectory should be in meters.";
        return true;
      }
    }

    // get the current robot orientation
    ROS_INFO("[rbt_trj] Ready to execute the translation...");
    geometry_msgs::Pose waypoint;
    {
      boost::shared_lock<boost::shared_mutex> read_lock(this->pose_update_lock);
      waypoint = this->current_pose;
    }
    ROS_INFO("[rbt_trj] Currently @ (%lf, %lf, %lf).", waypoint.position.x, waypoint.position.y, waypoint.position.z);

    std::vector<geometry_msgs::Pose> trj;
    for (int index = 0; index < req.trajectory.size() / 3; index ++) {
      waypoint.position.x = req.trajectory[index * 3];
      waypoint.position.y = req.trajectory[index * 3 + 1];
      waypoint.position.z = req.trajectory[index * 3 + 2];
      trj.push_back(waypoint);
    }

    ROS_INFO("[rbt_trj] Computing trajectory...");
    moveit_msgs::RobotTrajectory actions;
    double fraction = this->manipulator.computeCartesianPath(trj, 0.015, 0.0, actions);
    ROS_INFO("[rbt_trj] Done. (%.2f%% acheived)", fraction * 100.0);

    ROS_INFO("[rbt_trj] Executing the trajectory...");
    moveit::planning_interface::MoveGroupInterface::Plan exe_plan;
    exe_plan.trajectory_ = actions;
    bool success = (this->manipulator.execute(exe_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // if (success) {ROS_INFO("[rbt_trj] Done.");}
    // else {ROS_ERROR("[rbt_trj] Failed to move according to the trajectory..."); return;}

    res.success = true;
    res.percentage = fraction;
    res.reason = "Success!";

    return true;
  }

  /*
   * Function @ upload_statesCb
   * callback function for upload_states callback
   * this will save all the marked-down states to the yaml file
   */
  bool upload_statesCb(denso_robot_interface::Upload::Request &req, 
                        denso_robot_interface::Upload::Response &res) {
    if (req.file_name == "") {
      res.success = false;
      res.reason = "Please give your YAML file a name!";
      return true;
    }

    if (!this->waypoints.size()) {
      res.success = false;
      res.reason = "Haven't stored any state yet. Have you called 'mark_state' service before?";
      return true;
    }

    // compose the file path
    std::string full_path = ros::package::getPath("denso_robot_interface");
    full_path += ("/yaml/" + req.file_name + ".yaml");

    // compose the YAML data (jnt_space)
    std::ofstream yaml_file;
    yaml_file.open(full_path.c_str());

    yaml_file << "waypoints:\n";
    yaml_file << "\tjoint_space:\n";

    std::map<std::string, std::vector<double> > remembered_jnt_values = this->manipulator.getRememberedJointValues();
    for (std::vector<std::string>::iterator it = this->waypoints.begin(); it != this->waypoints.end(); ++ it) {
      yaml_file << "\t\t" << *it << ": [";
      int index = 0;
      for (index = 0; index < remembered_jnt_values[*it].size() - 1; index ++) {
        yaml_file << remembered_jnt_values[*it][index] << ", ";
      }
      yaml_file << remembered_jnt_values[*it][index] << "]\n";
    }

    // TODO (Bosch)
    // do we need to save the cartesian space into YAML file?

    yaml_file.close();

    res.success = true;
    res.reason = "Success!";

    return true;
  }

  /*
   */
  // void trajectoryCb(const blackbox::TrajectoryConstPtr &trajectory) {
  //   ROS_INFO("[rbt_trj] Trajectory received, forwarding to MoveIt...");

  //   // check trajectory validness
  //   ROS_INFO("[rbt_trj] Checking validness of the trajectory...");
  //   std::string trj_ref_frame = trajectory->header.frame_id;
  //   int robot_dim = trajectory->robot_dim;
  //   if ((trajectory->trajectory.size() % robot_dim) != 0) {
  //     ROS_ERROR("[rbt_trj] Trajectory size doesn't match trajectory dimension!");
  //     return;
  //   } else {
  //     ROS_INFO("[rbt_trj] Trajectory valid.");
  //   }

  //   // compose transformation matrix
  //   ROS_INFO("[rbt_trj] Looking for transformation matrix from %s to %s.", trj_ref_frame.c_str(), this->target_frame.c_str());
  //   tf::StampedTransform work2rob;
  //   Eigen::Affine3d transformer;
  //   try {
  //     this->tf_listener.waitForTransform(this->target_frame, trj_ref_frame, ros::Time::now(), ros::Duration(4.f));
  //     this->tf_listener.lookupTransform(this->target_frame, trj_ref_frame, ros::Time::now(), work2rob);
  //   } catch (tf::LookupException &err) {
  //     ROS_ERROR("[rbt_trj] Lookup [Work Frame --> Robot Frame] falied.");
  //     return;
  //   } catch (tf::ExtrapolationException &err) {
  //     ROS_ERROR("[rbt_trj] Lookup [Work Frame --> Robot Frame] falied.");
  //     return;
  //   } catch (tf::TransformException &err) {
  //     ROS_ERROR("[rbt_trj] Lookup [Work Frame --> Robot Frame] falied.");
  //     return;
  //   }
  //   tf::transformTFToEigen(work2rob, transformer);
  //   ROS_INFO("[rbt_trj] Done.");

  //   // move to the start-approaching point
  //   ROS_INFO("[rbt_trj] Approaching start point...");
  //   this->manipulator.setJointValueTarget(this->start_approach);

  //   moveit::planning_interface::MoveGroup::Plan exe_plan;
  //   if (this->manipulator.plan(exe_plan)) {
  //     // got a plan, now execute
  //     bool success = this->manipulator.execute(exe_plan);
  //     if (success)  {ROS_INFO("[rbt_trj] Done.");}
  //     else          {ROS_ERROR("[rbt_trj] Failed to approach start point!"); return;}
  //   } else {
  //     ROS_ERROR("[rbt_trj] No plan generated for approaching start point!");
  //     return;
  //   }

  //   // cartesian plan
  //   geometry_msgs::Pose waypoint;
  //   {
  //     boost::shared_lock<boost::shared_mutex> read_lock(this->pose_update_lock);
  //     waypoint = this->current_pose;
  //   }
  //   ROS_INFO("[rbt_trj] Currently @ P(%lf, %lf, %lf).", waypoint.position.x, waypoint.position.y, waypoint.position.z);

  //   Eigen::Affine3d scaler = Eigen::Affine3d::Identity();
  //   scaler.scale(0.001);

  //   int trj_length = trajectory->trajectory.size() / robot_dim;
  //   std::vector<float> trj = trajectory->trajectory;

  //   std::vector<geometry_msgs::Pose> waypoints;
  //   for (int trj_index = 0; trj_index < trj_length; trj_index ++) {
  //     // move to each waypoint
  //     Eigen::Vector3d point(trj[trj_index * robot_dim], trj[trj_index * robot_dim + 1], trj[trj_index * robot_dim + 2]);
  //     point = transformer * scaler * point;
  //     waypoint.position.x = point(0);
  //     waypoint.position.y = point(1);
  //     waypoint.position.z = point(2);
  //     waypoints.push_back(waypoint);
  //   }

  //   ROS_INFO("[rbt_trj] Computing waypoints for the rest of the trajectory...");
  //   moveit_msgs::RobotTrajectory actions;
  //   double fraction = this->manipulator.computeCartesianPath(waypoints, 0.015, 0.0, actions);
  //   ROS_INFO("[rbt_trj] Done. (%.2f%% acheived)", fraction * 100.0);

  //   ROS_INFO("[rbt_trj] Executing the trajectory...");
  //   moveit::planning_interface::MoveGroup::Plan exe_plan_2;
  //   exe_plan_2.trajectory_ = actions;
  //   bool success = this->manipulator.execute(exe_plan_2);
  //   if (success) {ROS_INFO("[rbt_trj] Done.");}
  //   else {ROS_ERROR("[rbt_trj] Failed to move according to the trajectory..."); return;}

  //   // move to the ending pose
  //   {
  //     boost::shared_lock<boost::shared_mutex> read_lock(this->pose_update_lock);
  //     waypoint = this->current_pose;
  //   }
  //   ROS_INFO("[rbt_trj] Currently @ P(%lf, %lf, %lf).", waypoint.position.x, waypoint.position.y, waypoint.position.z);
  //   ROS_INFO("[rbt_trj] Returning to initial position...");
  //   waypoint.position.z = 0.20740845427;
  //   this->manipulator.setPoseTarget(waypoint);
  //   if (this->manipulator.plan(exe_plan)) {
  //     // got a plan, now execute
  //     bool success = this->manipulator.execute(exe_plan);
  //     if (!success) {ROS_ERROR("[rbt_trj] Failed to return to initial point!"); return;}
  //   } else {
  //     ROS_ERROR("[rbt_trj] No plan generated for returning to initial point!");
  //     return;
  //   }
  //   for (int end_approach_index = 0; end_approach_index < this->end_approach.size(); end_approach_index ++) {
  //     this->manipulator.setJointValueTarget(this->end_approach[end_approach_index]);

  //     if (this->manipulator.plan(exe_plan)) {
  //       bool success = this->manipulator.execute(exe_plan);
  //       if (!success) {ROS_ERROR("[rbt_trj] Failed to return to initial point!"); return;}
  //     } else {
  //       ROS_ERROR("[rbt_trj] No plan generated for end pose!");
  //       return;
  //     }
  //   }
  //   ROS_INFO("[rbt_trj] Done.");
  //   ROS_INFO("[rbt_trj] Layer Done. Waiting for the next frame...");

  //   return;
  // }

  void publish_states(void) {
    // get EEF pose
    geometry_msgs::PoseStamped cur_eef_pose = this->manipulator.getCurrentPose();
    {
      boost::upgrade_lock<boost::shared_mutex> lock(this->pose_update_lock);
      boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(lock);
      this->current_pose = cur_eef_pose.pose;
    }

    // get joint values
    {
      boost::upgrade_lock<boost::shared_mutex> lock(this->jnt_update_lock);
      boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(lock);
      this->manipulator.getCurrentState()->copyJointGroupPositions(this->manipulator.getCurrentState()->getJointModelGroup(this->manipulator.getName()), this->current_jnt_value);
    }

    // publish
    denso_robot_interface::JointState jnt_state;
    jnt_state.jnt_names = this->manipulator.getJointNames();
    jnt_state.jnt_values = this->current_jnt_value;
    this->jnt_values.publish(jnt_state);
    this->eef_pose.publish(cur_eef_pose);
    return;
  }
};

int main(int argc, char **argv) {
  // initialize the node
  ros::init(argc, argv, "denso_robot_interface");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // bring up the node
  DensoRobotInterface denso_robot_interface;

  // do the publishing here
  ros::Rate rate(100);
  while (ros::ok()) {
    denso_robot_interface.publish_states();
    rate.sleep();
  }
  spinner.stop();

  return 0;
}
