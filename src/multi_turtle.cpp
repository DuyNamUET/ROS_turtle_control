/*********************************************************************
** run demo: rosrun [package] [type] [list]                         **
** [list] is array of number (argv(1) to argv(end))                 **
** argv(1) is number of turtles that we have                        **
** from argv(2) to argv(2 * argv(1) + 1) is position of each turtle **
** from argv(2 * (argv(1) + 1)) is position of each checkpoint      **
*********************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Pose.h>

#define PI 3.1415926536

class TTurtle
{
    public:
    ros::Subscriber sub;
    ros::Publisher pub;
    turtlesim::Pose pose;

    void poseCallback(const turtlesim::Pose::ConstPtr& msg)
    {
        pose = *msg;
        if(pose.theta > PI) pose.theta -= 2 * PI;
        if(pose.theta < -PI) pose.theta += 2 * PI;
    }
};

struct CheckPoint
{
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
    vel.linear.x = (linear_x > 1.5) ? 5 * linear_x * sign_ : 1.5 * sign_;
    vel.angular.z = 25.0 * angular_z * sign_;
    return vel;
}

geometry_msgs::Twist stop()
{
    geometry_msgs::Twist vel;
    vel.linear.x = 0.0;
    vel.angular.z = 0.0;
    return vel;
}

CheckPoint *p_minCheckPoint(const turtlesim::Pose &pose, CheckPoint arr_cp[], int len)
{
    CheckPoint *p = NULL;
    for(int i = 0; i < len; i++)
    {
        if(!arr_cp[i].state)
        {
            arr_cp[i].distance = distanceLinear(pose, arr_cp[i]);
            if(p == NULL) p = &arr_cp[i];
            else
            {
                if(p->distance > arr_cp[i].distance) p = &arr_cp[i];
            }
        }
    }
    if(!(p == NULL)) p->state = true;
    return p;
}

void checkDone(const turtlesim::Pose &pose, CheckPoint* &p)
{
    if(distanceLinear(pose, *p) < 0.01)
    {
        ROS_INFO("target has done: %0.1f %0.1f", p->goal_x, p->goal_y);
        p = NULL;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "multi");
    ros::NodeHandle node;

    ros::Time start = ros::Time::now();

    int num = atoi(argv[1]);
    TTurtle tturtle[num];
    CheckPoint* p[num];
    for(int i = 0; i < num; i++)
    {
        std::stringstream s;
        s << "turtle" << i + 1;
        std::string name = s.str();

        if(i != 0)
        {
            ros::service::waitForService("spawn");
            ros::ServiceClient spawner = node.serviceClient<turtlesim::Spawn>("spawn");
            turtlesim::Spawn turtle;
            turtle.request.x = rand() % 12;
            turtle.request.y = rand() % 12;
            spawner.call(turtle);
        }

        tturtle[i].sub = node.subscribe(name + "/pose", 100, &TTurtle::poseCallback, &tturtle[i]);
        tturtle[i].pub = node.advertise<geometry_msgs::Twist>(name + "/cmd_vel", 100);

        p[i] = NULL;
    }

    int len = (argc - 2) / 2;
    CheckPoint arr_cp[len];
    for(int i = 0; i < len; i++)
    {
        arr_cp[i].goal_x = atof(argv[2 * (i + 1)]);
        arr_cp[i].goal_y = atof(argv[2 * (i + 1) +1]);
        arr_cp[i].state = false;
        // ROS_INFO("%f %f", arr_cp[i].goal_x, arr_cp[i].goal_y);
    }

    ros::Rate rate(50);
    while(node.ok())
    {
        int count = 0;
        for(int i = 0; i < num; i++)
        {
            if(p[i] == NULL)
            {
                tturtle[i].pub.publish(stop());
                p[i] = p_minCheckPoint(tturtle[i].pose, arr_cp, len);
                if(p[i] == NULL) count++;
                // ROS_INFO("%f %f",p[i]->goal_x, p[i]->goal_y);
            }
            else
            {
                tturtle[i].pub.publish(getVelocity(tturtle[i].pose, *p[i]));
                checkDone(tturtle[i].pose, p[i]);
            }
        }

        if(count == num) break;

        rate.sleep();
        ros::spinOnce();    
    }
    ros::Time finish = ros::Time::now();
    ROS_INFO("total time: %f", (finish - start).toSec());
    
    return 0;
}
