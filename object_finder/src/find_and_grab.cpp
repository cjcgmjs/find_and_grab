// example_object_finder_action_client: 
// wsn, April, 2016
// illustrates use of object_finder action server called "objectFinderActionServer"

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <object_finder/objectFinderAction.h>
#include <object_grabber/object_grabberAction.h>
#include <part_codes/part_codes.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cartesian_planner/cart_moveAction.h>
#include <cartesian_planner/cart_motion_commander.h>
#include <xform_utils/xform_utils.h>

#include <object_manipulation_properties/object_ID_codes.h>
using namespace std;

void grab(void);

std::vector <geometry_msgs::PoseStamped> g_perceived_object_poses;
ros::Publisher *g_pose_publisher;

int g_found_object_code;
void objectFinderDoneCb(const actionlib::SimpleClientGoalState& state,
        const object_finder::objectFinderResultConstPtr& result) {

    geometry_msgs::PoseStamped perceived_object_pose;
    g_perceived_object_poses.clear();
    ROS_INFO(" objectFinderDoneCb: server responded with state [%s]", state.toString().c_str());
    g_found_object_code=result->found_object_code;
    ROS_INFO("got object code response = %d; ",g_found_object_code);
    if (g_found_object_code==object_finder::objectFinderResult::OBJECT_CODE_NOT_RECOGNIZED) {
        ROS_WARN("object code not recognized");
    }
    else if (g_found_object_code==object_finder::objectFinderResult::OBJECT_FOUND) {
        //ROS_INFO("found objects!");
        int n_objects = result->object_poses.size();
        for (int i_object=0;i_object<n_objects;i_object++) {
         //g_perceived_object_pose= result->object_pose; //MAKE MORE GENERAL FOR  POSES
            ROS_INFO("object %d: ",i_object);
            perceived_object_pose = result->object_poses[i_object];
            ROS_INFO("   pose x,y,z = %f, %f, %f",perceived_object_pose.pose.position.x,
                 perceived_object_pose.pose.position.y,
                 perceived_object_pose.pose.position.z);

            ROS_INFO("   quaternion x,y,z, w = %f, %f, %f, %f",perceived_object_pose.pose.orientation.x,
                 perceived_object_pose.pose.orientation.y,
                 perceived_object_pose.pose.orientation.z,
                 perceived_object_pose.pose.orientation.w);
            g_perceived_object_poses.push_back(perceived_object_pose);
        }
         //g_pose_publisher->publish(g_perceived_object_pose);
    }
    else {
        ROS_WARN("object not found!");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "find_and_grab"); // name this node 
    ros::NodeHandle nh; //standard ros node handle    
    
    
    actionlib::SimpleActionClient<object_finder::objectFinderAction> object_finder_ac("beta_object_finder_action_service", true);
    
    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = object_finder_ac.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to object_finder action server"); // if here, then we connected to the server; 
    ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("triad_display_pose", 1, true); 
    g_pose_publisher = &pose_publisher;
    object_finder::objectFinderGoal object_finder_goal;
    //object_finder::objectFinderResult object_finder_result;
    int part_index;
    cout<<"enter index for desired part:"<<endl<<"GEARBOX_TOP = 1"<<endl<<"GEARBOX_BOTTOM = 2"<<endl<<"BOLT = 3"<<endl<<"SMALL_GEAR = 4"<<endl<<"LARGE_GEAR = 5"<<endl<<"TOTE=6"<<endl;
    cout<<"enter  0 for fake part (hardcoded pose: "<<endl;
    cout<<"enter part index: ";
    cin>>part_index;
    switch(part_index) {
        case 0: object_finder_goal.object_id = part_codes::part_codes::FAKE_PART;
            break;
        case 1: object_finder_goal.object_id = part_codes::part_codes::GEARBOX_TOP;
        break;
        case 2: object_finder_goal.object_id = part_codes::part_codes::GEARBOX_BOTTOM;
        break;
        case 3: object_finder_goal.object_id = part_codes::part_codes::BOLT;
        break;
        case 4: object_finder_goal.object_id = part_codes::part_codes::SMALL_GEAR;
        break;
        case 5: object_finder_goal.object_id = part_codes::part_codes::LARGE_GEAR;
        break;
        case 6: object_finder_goal.object_id = part_codes::part_codes::TOTE;
        break;
        default:
            ROS_WARN("unknown part code; quitting");
            return 1;
    }
       

     ROS_INFO("sending goal to find object: ");
	    object_finder_ac.sendGoal(object_finder_goal,&objectFinderDoneCb); 
	    
	    bool finished_before_timeout = object_finder_ac.waitForResult(ros::Duration(10.0));
	    if (!finished_before_timeout) {
	        ROS_WARN("giving up waiting on result ");
	        return 1;
	    }       
        
    if (g_found_object_code == object_finder::objectFinderResult::OBJECT_FOUND)   {
        ROS_INFO("found object(s)!");
        ROS_WARN("size of g_perceived_object_poses %i", int(g_perceived_object_poses.size()));
        grab();
        return 0;
    }    
    else {
        ROS_WARN("object not found!:");
        return 1;
    }

}


XformUtils xformUtils; //type conversion utilities

int g_object_grabber_return_code;
actionlib::SimpleActionClient<object_grabber::object_grabberAction> *g_object_grabber_ac_ptr;
bool g_got_callback = false;

void objectGrabberDoneCb(const actionlib::SimpleClientGoalState& state,
        const object_grabber::object_grabberResultConstPtr& result) {
    ROS_INFO(" objectGrabberDoneCb: server responded with state [%s]", state.toString().c_str());
    g_object_grabber_return_code = result->return_code;
    ROS_INFO("got result output = %d; ", g_object_grabber_return_code);
    g_got_callback=true; //flag that action server has called back
}


//test fnc to specify object pick-up and drop-off frames;
//should get pick-up frame from perception, and drop-off frame from perception or task

void set_example_object_frames(geometry_msgs::PoseStamped &object_poseStamped,
        geometry_msgs::PoseStamped &object_dropoff_poseStamped) {
    //hard code an object pose; later, this will come from perception
    //specify reference frame in which this pose is expressed:
    //will require that "system_ref_frame" is known to tf
    object_poseStamped.header.frame_id = "system_ref_frame"; //set object pose; ref frame must be connected via tf
    object_poseStamped.pose.position.x = 0.5;
    object_poseStamped.pose.position.y = -0.35;
    object_poseStamped.pose.position.z = 0.7921; //-0.125; //pose w/rt world frame
    object_poseStamped.pose.orientation.x = 0;
    object_poseStamped.pose.orientation.y = 0;
    object_poseStamped.pose.orientation.z = 0.842;
    object_poseStamped.pose.orientation.w = 0.54;
    object_poseStamped.header.stamp = ros::Time::now();

    object_dropoff_poseStamped = object_poseStamped; //specify desired drop-off pose of object
    
}

void move_to_waiting_pose() {
        ROS_INFO("sending command to move to waiting pose");
        g_got_callback=false; //reset callback-done flag
        object_grabber::object_grabberGoal object_grabber_goal;
        object_grabber_goal.action_code = object_grabber::object_grabberGoal::MOVE_TO_WAITING_POSE;
        g_object_grabber_ac_ptr->sendGoal(object_grabber_goal, &objectGrabberDoneCb);
}

void grab_object(geometry_msgs::PoseStamped object_pickup_poseStamped) {
    ROS_INFO("sending a grab-object command");
    g_got_callback=false; //reset callback-done flag
    object_grabber::object_grabberGoal object_grabber_goal;
    object_grabber_goal.action_code = object_grabber::object_grabberGoal::GRAB_OBJECT; //specify the action to be performed 
    object_grabber_goal.object_id = ObjectIdCodes::TOY_BLOCK_ID; // specify the object to manipulate                
    object_grabber_goal.object_frame = object_pickup_poseStamped; //and the object's current pose
    object_grabber_goal.grasp_option = object_grabber::object_grabberGoal::DEFAULT_GRASP_STRATEGY; //from above
    object_grabber_goal.speed_factor = 1.0;
    ROS_INFO("sending goal to grab object: ");
    g_object_grabber_ac_ptr->sendGoal(object_grabber_goal, &objectGrabberDoneCb);
}

void   dropoff_object(geometry_msgs::PoseStamped object_dropoff_poseStamped) {
    ROS_INFO("sending a dropoff-object command");
    object_grabber::object_grabberGoal object_grabber_goal;
    object_grabber_goal.action_code = object_grabber::object_grabberGoal::DROPOFF_OBJECT; //specify the action to be performed 
    object_grabber_goal.object_id = ObjectIdCodes::TOY_BLOCK_ID; // specify the object to manipulate                
    object_grabber_goal.object_frame = object_dropoff_poseStamped; //and the object's current pose
    object_grabber_goal.grasp_option = object_grabber::object_grabberGoal::DEFAULT_GRASP_STRATEGY; //from above
    object_grabber_goal.speed_factor = 1.0;
    ROS_INFO("sending goal to dropoff object: ");
    g_object_grabber_ac_ptr->sendGoal(object_grabber_goal, &objectGrabberDoneCb);
} 


void grab(void) {
/*    ArmMotionCommander arm_motion_commander;
    XformUtils xformUtils;
    Eigen::VectorXd joint_angles;
    Eigen::Vector3d dp_displacement;
    int rtn_val;
    int njnts;
    
    arm_motion_commander.send_test_goal(); // send a test command
    
    //send a command to plan a joint-space move to pre-defined pose:
    ROS_INFO("commanding move to waiting pose");
    rtn_val=arm_motion_commander.plan_move_to_waiting_pose();
    
    //send command to execute planned motion
    rtn_val=arm_motion_commander.execute_planned_path();  

/**/

    //try planning a joint-space motion to this new joint-space pose:
    /*rtn_val=arm_motion_commander.plan_jspace_path_current_to_qgoal(joint_angles);*/
    geometry_msgs::PoseStamped object_pickup_poseStamped = g_perceived_object_poses[0];
    geometry_msgs::PoseStamped object_dropoff_poseStamped = object_pickup_poseStamped;
    object_dropoff_poseStamped.pose.orientation.z = 1;
    object_dropoff_poseStamped.pose.orientation.w = 0;
    
    //specify object pick-up and drop-off frames using simple test fnc
    //more generally, pick-up comes from perception and drop-off comes from task
    //set_example_object_frames(object_pickup_poseStamped, object_dropoff_poseStamped);

    //instantiate an action client of object_grabber_action_service:
    actionlib::SimpleActionClient<object_grabber::object_grabberAction> object_grabber_ac("object_grabber_action_service", true);
    g_object_grabber_ac_ptr = &object_grabber_ac; // make available to fncs
    ROS_INFO("waiting for server: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = object_grabber_ac.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to object_grabber action server"); // if here, then we connected to the server; 

    //move to waiting pose
    move_to_waiting_pose();
    while(!g_got_callback) {
        ROS_INFO("waiting on move...");
        ros::Duration(0.5).sleep(); //could do something useful
    }

    grab_object(object_pickup_poseStamped);
    while(!g_got_callback) {
        ROS_INFO("waiting on grab...");
        ros::Duration(0.5).sleep(); //could do something useful
    }    

    dropoff_object(object_dropoff_poseStamped);
    while(!g_got_callback) {
        ROS_INFO("waiting on dropoff...");
        ros::Duration(0.5).sleep(); //could do something useful
    }  
   
}


