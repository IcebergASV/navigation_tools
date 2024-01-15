#include <ros/ros.h>
#include <task_master/TaskStatus.h>
#include <task_master/TaskGoalPosition.h>
#include <task_master/Task.h>
#include <prop_mapper/PropArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>

class MagellansRoute {
public:
    MagellansRoute(): nh_(""), private_nh_("~")
    {
        // ROS parameters
        private_nh_.param<double>("safety_dist", safety_dist, 1.0);

        private_nh_.param<std::string>("local_pose_topic", local_pose_topic_, "/mavros/local_position/pose");

        // ROS subscribers
        prop_map_ = nh_.subscribe("props", 10, &MagellansRoute::propMapCallback, this);
        global_pos_ = nh_.subscribe(local_pose_topic_, 10, &MagellansRoute::globalPositionCallback, this);
        task_to_exec_ = nh_.subscribe("task_to_execute", 10, &MagellansRoute::magellansRouteCallback, this);

        // ROS publishers
        task_status_ = nh_.advertise<task_master::TaskStatus>("task_status", 10);
        task_goal_position_ = nh_.advertise<task_master::TaskGoalPosition>("task_goal_position", 10);
    }

    void spin() {
        ros::Rate rate(10);
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber prop_map;
    ros::Subscriber global_pos_;
    ros::Subscriver task_to_exec_;
    ros::Publisher task_status_;
    ros::Publisher task_goal_position_;

    std::string local_pose_topic_;
    std::string TAG = "MAGELLANS_ROUTE: ";

    geometry_msgs::PoseStamped current_pos_;
    prop_mapper::PropArray props_;
    task_master::TaskGoalPosition goal_pos_;

    double safety_dist;

    enum States {NOT_STARTED, FIND_GATE, MOVE_TO_GATE, AVOIDING_OBSTACLE, CIRCLING, COMPLETE};
    States status = States::NOT_STARTED;


    bool findObstacle(prop_mapper::Prop &obstacle) {
        prop_mapper::Prop obstacle;
        obstacle.prop_label = "no obstacle"; // better way to do this?
        double dist_to_obstacle = 1000; // arbitrary high number
        double angle, a_x, a_y, b_x, b_y, c_x, c_y, d_x, d_y;
        // could make the danger zone a message?

        getDangerZone(angle, a_x, a_y, b_x, b_y, c_x, c_y, d_x, d_y);

        for(int i = 0; i <  props_.props.size(); i++) {
            if(props_.props[i].prop_label == "yellow_marker" || props_.props[i].prop_label == "black_marker") {
                double p_x = props_.props[i].vector.x;
                double p_y = props_.props[i].vector.y;

                if(obstacleInDangerZone(p_x,p_y,a_x,a_y,b_x,b_y,c_x,c_y,d_x,d_y)) {
                    double dist_to_i = sqrt(pow(current_pos_.pose.pose.position.x - p_x, 2) + pow(current_pos_.pose.pose.position.y - p_y, 2));
                    if (dist_to_i < dist_to_obstacle) {
                        obstacle = props_.props[i];
                    }
                }
            }
        }

        if (obstacle.prop_label != "no obstacle") {
            return true;
        }
        else {
            return false;
        }
    }

    void getDangerZone(double angle, double, a_x, double a_y, double b_x, double b_y, double c_x, double c_y, double d_x, double d_y) {
        //double heading = getGazeboHeading(current_pos_.pose.orientation);
        if (heading < M_PI_2) {
            ROS_DEBUG_STREAM("heading < 90deg.");
            angle = M_PI_2 - heading;
            a_x = current_pos_.pose.pose.position.x - (1.335 * cos(angle));
            a_y = current_pos_.pose.pose.position.y + (1.335 * sin(angle));
            b_x = a_x + (5 * sin(angle));
            b_y = a_y + (5 * cos(angle));
            c_x = b_x + (2.67 * cos(angle));
            c_y = b_y - (2.67 * sin(angle));
            d_x = c_x - (5 * sin(angle));
            d_y = c_y - (5 * cos(angle));
        }
        else if (heading >= M_PI_2 && heading < M_PI) {
            ROS_DEBUG_STREAM("90deg < heading < 180deg.");
            angle = M_PI - heading;
            a_x = current_pos_.pose.pose.position.x - (1.335 * cos(angle));
            a_y = current_pos_.pose.pose.position.y - (1.335 * sin(angle));
            b_x = a_x - (5 * sin(angle));
            b_y = a_y + (5 * cos(angle));
            c_x = b_x + (2.67 * cos(angle));
            c_y = b_y + (2.67 * sin(angle));
            d_x = c_x + (5 * sin(angle));
            d_y = c_y - (5 * cos(angle));
        }
        else if (heading >= M_PI && heading < (M_PI+M_PI_2)) {
            ROS_DEBUG_STREAM("180deg < heading < 270deg.");
            angle = M_PI + M_PI_2 - heading;
            a_x = current_pos_.pose.pose.position.x + (1.335 * cos(angle));
            a_y = current_pos_.pose.pose.position.y - (1.335 * sin(angle));
            b_x = a_x - (5 * sin(angle));
            b_y = a_y - (5 * cos(angle));
            c_x = b_x - (2.67 * cos(angle));
            c_y = b_y + (2.67 * sin(angle));
            d_x = c_x + (5 * sin(angle));
            d_y = c_y + (5 * cos(angle));
        }
        else { //  heading between 3PI/2 and 2PI
            ROS_DEBUG_STREAM("270deg < heading < 360deg.");
            angle = 2*M_PI - heading;
            a_x = current_pos_.pose.pose.position.x + (1.335 * cos(angle));
            a_y = current_pos_.pose.pose.position.y + (1.335 * sin(angle));
            b_x = a_x + (5 * sin(angle));
            b_y = a_y - (5 * cos(angle));
            c_x = b_x - (2.67 * cos(angle));
            c_y = b_y - (2.67 * sin(angle));
            d_x = c_x - (5 * sin(angle));
            d_y = c_y + (5 * cos(angle));
        }
    }

    bool obstacleInDangerZone(double px, double py, double ax, double ay, double bx, double by, double cx, double cy, double dx, double dy) {
        double APx = px - ax;
        double APy = py - ay;
        double ABx = bx - ax;
        double ABy = by - ay;
        double ADx = dx - ax;
        double ADy = dy - ay;

        double dotAPAB = APx * ABx + APy * ABy;
        double dotABAB = ABx * ABx + ABy * ABy;
        double dotAPAD = APx * ADx + APy * ADy;
        double dotADAD = ADx * ADx + ADy * ADy;

        return (0 <= dotAPAB && dotAPAB <= dotABAB && 0 <= dotAPAD && dotAPAD <= dotADAD);
    }

    void magellansRouteCallback(const task_master::Task msg) {
        if(msg.current_task == task_master::Task::AVOID_CROWDS) {
            task_master::TaskStatus taskStatus;
            prop_mapper::Prop obstacle;
            switch (status)
            {
            case States::NOT_STARTED: {
                // wait for a valid gate?
                // set state to FIND_GATE

            }
                break;
            case States::FIND_GATE: {
                // find nearest valid gate
                // if gate found, set state to MOVE_TO_GATE
                // else, set state to CIRCLING

                ROS_INFO_STREAM(TAG << "Looking for a gate");

                prop_mapper::Prop green_marker;
                prop_mapper::Prop red_marker;

                /* would be beneficial to make gate functions publicly accessable
                if (findGate(green_marker, red_marker)) {
                    goal_pos_.point = findMidpoint(green_marker, red_marker);
                    green_id_ = green_marker.id;
                    red_id_ = red_marker.id;
                    status = States::MOVE_TO_GATE;
                }
                */
            }
                break;
            case States::MOVE_TO_GATE: {
                // set goal to midpoint of gate
                // if at destination, set state to FIND_GATE
                // else if obstacle in path, set state to AVOIDING_OBSTACLE
                // else if not at destination, stay in current state

                ROS_INFO_STREAM(TAG << "Moving to gate and searching for obstacles");

                /*
                if(!isReached()) {
                    ROS_DEBUG_STREAM(TAG << "Gate not reached yet");
                    setDestination();
                    ros::Rate rate(10);
                    rate.sleep();
                }

                if(isReached()) {
                    status = States::FIND_GATE;
                    ROS_DEBUG_STREAM(TAG << "Gate reached");
                }
                */

                if(findObstacle(obstacle)) {
                    status = States::AVOIDING_OBSTACLE;
                    ROS_DEBUG_STREAM(TAG << "Obstacle found");
                }
            }
                break;
            case States::AVOIDING_OBSTACLE: {
                // determine shortest path around obstacle
                // if around obstacle, set state to MOVE_TO_GATE
                // else stay in current state
                task_master::TaskGoalPosition goal = goal_pos_;
                double theta, wpf_x, wpf_y, wpl_x, wpl_y, wpr_x, wpr_y, wpb_x, wpb_y;

                //
                // CONTINUE HERE
                //
            }
                break;
            case States::CIRCLING: {
                // for each yellow buoy in prop_map, circle 1 time
                // set state to COMPLETE
            }
                break;
            case States::COMPLETE: {
                ROS_INFO_STREAM(TAG << "Magellans Route Complete");
                taskStatus.status = task_master::TaskStatus::COMPLETE;
                task_status_.publish(taskStatus);
            }
                break;
            
            default:
                break;
            }
        }
    }

    void propMapCallback(const prop_mapper::PropArray msg) {
        props_ = msg;
    }

    void localPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pos_ = *msg;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "magellans_route_ctrl");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
        ros::console::notifyLLoggerLevelsChanged();

    MagellansRoute magellans_route;

    magellans_route.spin();

    return 0;
}