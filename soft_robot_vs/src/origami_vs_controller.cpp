#include "ros/ros.h"
#include <math.h>
#include <iostream>
#include <vector>
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"
#include <eigen3/Eigen/Dense>
#include "sensor_msgs/JointState.h"
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <cstdlib>
#include "soft_robot_vs/TwoModuleJacobian.h"
/**** Formualtion 
 *  Camera Intrinsic Matrix K -(pixel frame = K * Camera frame)
 *   [Z*U Z*V Z] =  K * [Xc Yc Z]
 * 
 *  K =[f 0 C;
 *      0 f C;
 *      0 0 1]
 *   
 *   U_pixel = (f/Z) * Xc  +  C
 *   V_pixel = (f/Z) * Yc  +  C
 * 
 *  Planar (2D) => constant Depth => Z_dot = 0
 * 
 *  Pixel velocities = [u_dot ,v_dot]
 *  u_dot = (f/Z) * Xc_dot -----1
 *  v_dot = (f/Z) * Yc_dot -----2
 *  
 *  Xc_dot = -V_x  - (w_y*Z) + (w_z*Y) ----> 3
 *  Yc_dot = -V_y  - (w_z*X) + (w_x*Z) ----> 4
 *  
 * Substitue 3 & 4 in 1 & 2 
 * 
 * [u_dot v_dot]  = J_img * ee_vel
 * 
 *  J_img(2*6) << -(f/Z), 0, 0, 0,-f, (V_pixel - C),
            0, -(f/Z) , 0 , f ,0 , (C - U_pixel);

 * In this case depth is constant,so J_img = f/Z  => [u_dot v_dot] = f/z * [x_dot y_dot]
 * 
 * ****Robot Jacobian is derived w.r.t Camera center Frame ***
 * 
 * J_robot << ((l1*cos(theta1))+(l2*cos(theta1+theta2))), ((l2*cos(theta1+theta2))),
              ((l1*sin(theta1))+(l2*sin(theta1+theta2))) ,((l2*sin(theta1+theta2)));

 * Proportional Controller Gains 
 * lamda = [lamda_1 , lamda_2]
 * 
 *  Proportional Controller Law - Pixel Position error is proportional to pixel velocities
 *
 * Combined_Jacobian(2*2) = J_image(2*6) * J_image(6*2)
 * 
 * joint_vel = lamda * inverse(Combined_Jacobian) * error_vec;
 * 
 * ****/

float ee_pos_x;  // Feature cordinates in Pixel frame
float ee_pos_y;  // Feature cordinates in Image frame
float theta1;    // Joint -1 angle
float theta2;    // Joint -2 angle
std_msgs::Bool end_flag; // Terminate Control loop

/***Callback for End Effector Marker Point Topic***/
void ee_feature_callback(const std_msgs::Float64MultiArray &msg){
    ee_pos_x = msg.data.at(0);
    ee_pos_y = msg.data.at(1);
}


void control_loop_callback(const std_msgs::Bool msg){
     end_flag = msg;
}


int main(int argc, char **argv){

    // ROS initialization
    ros::init(argc, argv, "origami_vs_controller");
    ros::NodeHandle n;
    
    ros::Rate rateController = ros::Rate(10);

    // Initializing ROS publishers
    ros::Publisher control_lengths_pub = n.advertise<std_msgs::Float64MultiArray>("/origamibot/control_lengths",1);
    ros::Publisher err_pub = n.advertise<std_msgs::Float64MultiArray>("/origamibot/servoing_error", 1);
    ros::Publisher end_flag_pub = n.advertise<std_msgs::Bool>("/origamibot/control_loop_end_flag", 1);

    // Subscriber Node for End effector feature pose
    ros::Subscriber ee_feature = n.subscribe("ee_feature_pose",1,ee_feature_callback);
    ros::Subscriber control_loop = n.subscribe("/control_loop_end_flag",1,control_loop_callback);
    ros::Duration(3).sleep(); 

    // Service Clients
    ros::ServiceClient jac_client = n.serviceClient<soft_robot_vs::TwoModuleJacobian>("two_module_jacobian");
    soft_robot_vs::TwoModuleJacobian  jacobian_msg;
    
    // Declarations   
    std_msgs::Float64MultiArray err_msg; //Error msg topic
    Eigen::Vector2f error_vec;
    Eigen::Vector2f joint_vel;
    Eigen::MatrixXf J_robot(6,6); // Robot Jacobian
    float err;  // Norm of Pixel(x,y) Errors
    float ee_img_x; // Feature cordinates in Image frame
    float ee_img_y; // Feature cordinates in Image frame
    int step;
    std::vector<float> control_lengths(6); //link-1 length
    float d = 0;
    float goal_pos_x; //Goal Feature positions  
    float goal_pos_y; // 
    Eigen::Vector2f lamda; // Proportional Gain //1.5
    /*** Extract parameters from config.yaml file ***/
    n.getParam("origamibot/control/goal_pos_x",goal_pos_x);
    n.getParam("origamibot/control/goal_pos_y",goal_pos_y);
    n.getParam("origamibot/control/lambda_1",lamda[0]);
    n.getParam("origamibot/control/lambda_2",lamda[1]);
    n.getParam("origamibot/simulation/module_d",d);

    /** Initialize**/
    end_flag.data = false;
    end_flag_pub.publish(end_flag);

    error_vec[0] = goal_pos_x - ee_pos_x;
    error_vec[1] = goal_pos_y - ee_pos_y;
    err = sqrt((error_vec[0]*error_vec[0]) + (error_vec[1]*error_vec[1]));

    err_msg.data.clear();
    err_msg.data.push_back(err);
    err_msg.data.push_back(error_vec[1]);
    err_pub.publish(err_msg);



    // control_lengths_pub.publish(j1_vel);

    std::cout<<" Servoing Started "<<std::endl;

    /********** Control loop **************/
    while(int(end_flag.data) == 0){
   
        // update error 
        error_vec[0] =  (goal_pos_x - ee_pos_x);
        error_vec[1] = (goal_pos_y - ee_pos_y);
        err = sqrt((error_vec[0]*error_vec[0]) + (error_vec[1]*error_vec[1]));

        if((error_vec[0] != goal_pos_x) &&  (error_vec[1] != goal_pos_y)) // To avoid high error during initial loop condition
        {
            // Publish current error to plot data
            err_msg.data.clear();
            err_msg.data.push_back(err);
            err_msg.data.push_back(error_vec[1]);
            err_pub.publish(err_msg);

            // Service message request for Jacobian
            jacobian_msg.request.d  = d;
            jacobian_msg.request.l1 = control_lengths[0];
            jacobian_msg.request.l2 = control_lengths[1];
            jacobian_msg.request.l3 = control_lengths[2];
            jacobian_msg.request.l4 = control_lengths[3];
            jacobian_msg.request.l5 = control_lengths[4];
            jacobian_msg.request.l6 = control_lengths[5];

            /**Robot Jacobian**/
            // Get current Jacobian
            if(jac_client.call(jacobian_msg)){
                step = jacobian_msg.response.step;
                for(int i = 0; i< (step*step); i++){
                    J_robot << jacobian_msg.response.jv.at(i);
                }
            }
            std::cout<<"robot "<<J_robot<<std::endl;
          
            // /** Proportional Control Law **/
            joint_vel = (lamda.cwiseProduct(J_robot.inverse() * error_vec));

            

            // current_lengths = prev_lengths + joint_vel * delta_time;
            // prev_lengths = current_lengths;


            // // Publish control lengths
            // control_lengths_pub.publish(current_lengths);
        }
        ros::spinOnce();
        rateController.sleep();
    
    }

    std::cout<<" Servoing Completed "<<std::endl;
    ros::spin();
    return 0;
}