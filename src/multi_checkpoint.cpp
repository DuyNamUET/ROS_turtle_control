/******************************************************************
** run demo:                                                     **
** $ rosrun ROS_turte_control multi_checkpoint [list checkpoint] **
** each two number is one checkpoint                             **
******************************************************************/
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

#define PI 3.1415926536

turtlesim::Pose current_pose;

struct CheckPoint{
    double goal_x, goal_y;
    double distance;
    bool state;
};

//Get distance
double distanceLinear(const turtlesim::Pose &pose, const CheckPoint &cp)
{
    double linear_x = sqrt(pow(cp.goal_x - pose.x, 2) + pow(cp.goal_y - pose.y, 2));
    if(abs(linear_x < 0.01)) linear_x == 0.0;
    return linear_x;
}

double distanceAngular(const turtlesim::Pose &pose, const CheckPoint &cp)
{
    double linear_x = distanceLinear(pose, cp);
    if(linear_x == 0) return 0.0;
    else
    {
        return asin((cos(pose.theta) * (cp.goal_y - pose.y) - sin(pose.theta) * (cp.goal_x - pose.x))
                    / linear_x);
    }   
}

double sign(const turtlesim::Pose &pose, const CheckPoint &cp)
{
    double scalar = acos((cos(pose.theta) * (cp.goal_x - pose.x) + sin(pose.theta) * (cp.goal_y - pose.y))
                    / distanceLinear(pose, cp));
    if(scalar > PI / 2) return -1.0;
    else return 1.0;
}

//Get velocity
geometry_msgs::Twist getVelocity(const turtlesim::Pose &pose, const CheckPoint &cp)
{
    double linear_x = distanceLinear(pose, cp);
    double angular_z = distanceAngular(pose, cp);
    double sign_ = sign(pose, cp);

    geometry_msgs::Twist vel;
    vel.linear.x = (linear_x > 1.0) ? 1.5 * linear_x * sign_ : 1.0 * sign_;
    vel.angular.z = 20.0 * angular_z * sign_;
    return vel;
}

geometry_msgs::Twist stop()
{
    geometry_msgs::Twist vel;
    vel.linear.x = 0.0;
    vel.angular.z = 0.0;
    return vel;
}

//Check done
void isDone(const turtlesim::Pose &pose, CheckPoint& cp)
{
    if(distanceLinear(pose, cp) < 0.01)
    {
        cp.state = true;        
        ROS_INFO("target has done: %0.1f %0.1f", cp.goal_x, cp.goal_y);
    }
}

//Point to the checkpoint whose position is nearest
CheckPoint* p_minCheckPoint(const turtlesim::Pose &pose, CheckPoint arr_cp[], int len)
{
    // CheckPoint *p_cp = &arr_cp[0];
    // for(int i = 1; i < len; i++)
    // {
    //     if(p_cp->distance > arr_cp[i].distance) p_cp = &arr_cp[i];
    // }
    // if(p_cp->state) p_cp = NULL;
    // return p_cp;
    CheckPoint *p_cp = NULL;
    for(int i = 0; i < len; i++)
    {
        if(!arr_cp[i].state)
        {
            arr_cp[i].distance = distanceLinear(pose, arr_cp[i]);
            if(p_cp == NULL) p_cp = &arr_cp[i];
            else
            {
                if(p_cp->distance > arr_cp[i].distance) p_cp = &arr_cp[i];
            }
        }
    }
    return p_cp;
}

//Update posision of turtle 
void poseUpdate(const turtlesim::Pose::ConstPtr& pose)
{
    current_pose = *pose;
    if(current_pose.theta > PI) current_pose.theta -= 2 * PI;
    if(current_pose.theta < -PI) current_pose.theta += 2 * PI;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gogo");
    ros::NodeHandle node;

    ros::Time start = ros::Time::now();
    
    ros::Subscriber sub = node.subscribe("/turtle1/pose", 100, &poseUpdate);
    ros::Publisher pub = node.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);

    geometry_msgs::Twist vel_msg;

    int len = (argc - 1) / 2;
    CheckPoint arr_cp[len];
    CheckPoint *p_cp_target;
    for(int i = 0; i < len; i++)
    {
        arr_cp[i].goal_x = atof(argv[2 * i + 1]);
        arr_cp[i].goal_y = atof(argv[2 * i + 2]);
        arr_cp[i].state = false;
    }

    ros::Rate rate(100);
    while(node.ok())
    {
        // for(int i = 0; i < len; i++)
        // {
        //     if(arr_cp[i].state) arr_cp[i].distance = DISTANCE_MAX;
        //     else arr_cp[i].distance = distanceLinear(current_pose, arr_cp[i]);
        // }

        p_cp_target = p_minCheckPoint(current_pose, arr_cp, len);

        if(p_cp_target == NULL)
        {
            pub.publish(stop());
            ROS_INFO("all done!!");
            break;
        }
        else
        {
            pub.publish(getVelocity(current_pose, *p_cp_target));    
            isDone(current_pose, *p_cp_target);
        }

        rate.sleep();
        ros::spinOnce();
    }
    ros::Time finish = ros::Time::now();
    ROS_INFO("total time: %f", (finish - start).toSec());
    return 0;
}