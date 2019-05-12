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

#define MAX_LINEAR 10.0
#define MAX_ANGULAR 4.0

class TTurtle
{
    public:
    int index;
    ros::Subscriber sub;
    ros::Publisher pub;
    turtlesim::Pose pose;

    void poseCallback(const turtlesim::Pose::ConstPtr& msg)
    {
        pose = *msg;
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
    double angular_z;
    angular_z = asin((cos(pose.theta) * (cp.goal_y - pose.y) - sin(pose.theta) * (cp.goal_x - pose.x))
                        / distanceLinear(pose, cp));
    if(distanceLinear(pose, cp)  == 0.0) angular_z = 0.0;
    return angular_z;
}

//Get velocity
geometry_msgs::Twist getVelocity(const turtlesim::Pose &pose, const CheckPoint &cp)
{
    geometry_msgs::Twist vel;
    vel.linear.x = 1.5 * distanceLinear(pose, cp);
    vel.angular.z = 20.0 * distanceAngular(pose, cp);
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
            turtle.request.x = atof(argv[2 * i]);
            turtle.request.y = atof(argv[2 * i + 1]);
            spawner.call(turtle);
        }

        tturtle[i].index = i;
        tturtle[i].sub = node.subscribe(name + "/pose", 100, &TTurtle::poseCallback, &tturtle[i]);
        tturtle[i].pub = node.advertise<geometry_msgs::Twist>(name + "/cmd_vel", 100);

        p[i] = NULL;
    }

    int len = (argc - 2 - 2 * (num - 1)) / 2;
    CheckPoint arr_cp[len];
    for(int i = 0; i < len; i++)
    {
        arr_cp[i].goal_x = atof(argv[2 * (i + num)]);
        arr_cp[i].goal_y = atof(argv[2 * (i + num) + 1]);
        arr_cp[i].state = false;
    }

    ros::Rate rate(100);
    while(node.ok())
    {
        for(int i = 0; i < num; i++)
        {
            if(p[i] == NULL) p[i] = p_minCheckPoint(tturtle[i].pose, arr_cp, len);
            else
            {
                tturtle[i].pub.publish(getVelocity(tturtle[i].pose, *p[i]));
                checkDone(tturtle[i].pose, p[i]);
            }
        }

        rate.sleep();
        ros::spinOnce();    
    }
    
    return 0;
}

