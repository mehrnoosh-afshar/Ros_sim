#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include "geometry_msgs/Twist.h"
#include <vector>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
int drive_robot(float lin_x, float ang_z)
{
    ball_chaser::DriveToTarget srv;

    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

   if (client.call(srv))
    {
       ROS_INFO("Succeed to call service add_two_ints");
     }
     else
    {
       ROS_ERROR("Failed to call service add_two_ints");
       return 1;
    }
  return 0;
}

// This callback function continuously executes and reads the image data

int retun_colum_number_of_ball_center(std::vector<std::vector<int>> &image_pixel_vector){
  
  int row_num= image_pixel_vector[0].size(); 
  int col_num = image_pixel_vector.size();
  int max_col=0 ;
  int max_col_sum =0;
  std::vector<int> image_pixel_row(col_num,0);
  for (int col =0 ; col < col_num; ++col){
       for (int row=0; row < row_num; ++row){
            image_pixel_row[col]+= image_pixel_vector[row][col]; }       
    
       if (image_pixel_row[col] > max_col_sum){
            max_col_sum = image_pixel_row[col];
            max_col = col;
        }
    }

  return max_col;
}





void process_image_callback(const sensor_msgs::Image img)
{
    
    int white_pixel = 255;

    bool find = false;
    int point_pixel=0;

    int step_left_size = img.step/3;
    int step_right_size = img.step*2/3;
    

    std::vector<int> image_pixel_row(img.step,0);
    std::vector<std::vector<int>> image_pixel_vector(img.height,image_pixel_row);

    for(int i= 0 ; i <img.step * img.height; ++i ){

        int col = i % img.step;
        int row = i / img.step;

        if (img.data[i] == white_pixel ){
            find = true;
            image_pixel_vector[row][col] = 1;
        }
    }

    int center_col = retun_colum_number_of_ball_center(image_pixel_vector);

   if (find==true){
     
      if (center_col < step_left_size){
        // left
        ROS_INFO("BALL is %3d of %3d: LEFT", center_col, img.step);
        drive_robot(0.02, -0.15);
        }

        else if (center_col > step_right_size){
        // right
        ROS_INFO("BALL is %3d of %3d: RIGHT", center_col, img.step);
        drive_robot(0.02, 0.15);
        }
    
       else{
        // straight
        ROS_INFO("BALL is %3d of %3d: FRONT", center_col, img.step);
         drive_robot(0.25, 0.0);
         }
   }

     

}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
