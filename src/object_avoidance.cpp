#include <ros/ros.h>
#include <task_master/TaskGoalPosition.h>
#include <prop_mapper/Prop.h>
#include <prop_mapper/PropArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <cmath>


class ObjectAvoidance {
public:
    ObjectAvoidance(): nh_(""), private_nh_("~")
    {
        // ROS parameters
        // start point, end point, obstacle point, L/R default, acceptable distance from obstacles
        private_nh_.param<double>("start_pos_x", start_pos_x, 0.0); // boat's starting position in x direction
        private_nh_.param<double>("start_pos_y", start_pos_y, 0.0); // boat's starting position in y direction
        private_nh_.param<double>("end_pos_x", end_pos_x, 10.0); // boat's goal position in x direction
        private_nh_.param<double>("end_pos_y", end_pos_y, 10.0); // boat's goal position in y direction
        private_nh_.param<double>("obstacle_x", obstacle_x, 5.5); // obstacle's position in x direction
        private_nh_.param<double>("obstacle_y", obstacle_y, 5.0); // obstacle's position in y direction

        private_nh_.param<bool>("avoid_left", avoid_left, true); // boolean for what is the default direction to avoid (is this even needed?)
        private_nh_.param<double>("safety_dist", safety_dist, 1.0); // minimum distance we want to be away from obstacles

        // ROS subscribers
        prop_map_ = nh_.subscribe("props", 10, &ObjectAvoidance::propMapCallback, this);
        global_pos_ = nh_.subscribe("mavros/global_position/local", 10, &ObjectAvoidance::globalPositionCallback, this);
        task_goal_position_ = nh_.subscribe("task_goal_position", 10, &ObjectAvoidance::goalPositionCallback, this);

        // ROS publishers
        waypoint_ = nh_.advertise<task_master::TaskGoalPosition>("task_goal_position", 10);
    }

    void spin() {
        ros::Rate rate(10);
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

    prop_mapper::Prop findObstacle() {
        // want to find nearest obstacle, so do distance comparison to make sure we have the nearest obstacle
        prop_mapper::Prop obstacle;
        obstacle.prop_label = "no obstacle"; // if we find no obstacle, return a prop with this label
        double dist_to_obstacle = 1000; // set arbitrarily high distance
        // get bounding box for 'danger zone'
        double heading = getGazeboHeading(current_pos_.pose.pose.orientation); // in radians
        double theta;
        double gamma;
        double a_x;
        double a_y;
        double b_x;
        double b_y;
        double d_x;
        double d_y;
        double c_x;
        double c_y;

        if (heading < M_PI_2) {
            theta = heading;
            gamma = M_PI_2 - theta;
            a_x = current_pos_.pose.pose.position.x - (1.335 * sin(gamma));
            a_y = current_pos_.pose.pose.position.y - (1.335 * cos(gamma));
            b_x = a_x - (5 * cos(gamma));
            b_y = a_y + (5 * sin(gamma));
            d_x = current_pos_.pose.pose.position.x + (1.335 * cos(theta));
            d_y = current_pos_.pose.pose.position.y + (1.335 * sin(theta));
            c_x = d_x - (5 * sin(theta));
            c_y = d_y + (5 * cos(theta));
        }
        else if (heading >= M_PI_2 && heading < M_PI) {
            theta = M_PI - heading;
            gamma = M_PI_2 - theta;
            a_x = current_pos_.pose.pose.position.x + (1.335 * cos(theta));
            a_y = current_pos_.pose.pose.position.y - (1.335 * sin(theta));
            b_x = a_x - (5 * sin(theta));
            b_y = a_y - (5 * cos(theta));
            d_x = current_pos_.pose.pose.position.x - (1.335 * sin(gamma));
            d_y = current_pos_.pose.pose.position.y + (1.335 * cos(gamma));
            c_x = d_x - (5 * cos(gamma));
            c_y = d_y - (5 * sin(gamma));
        }
        else if (heading >= M_PI && heading < (M_PI+M_PI_2)) {
            theta = heading - M_PI;
            gamma = M_PI_2 - theta;
            a_x = current_pos_.pose.pose.position.x + (1.335 * sin(gamma));
            a_y = current_pos_.pose.pose.position.y + (1.335 * cos(gamma));
            b_x = a_x + (5 * cos(gamma));
            b_y = a_y + (5 * sin(gamma));
            d_x = current_pos_.pose.pose.position.x - (1.335 * cos(theta));
            d_y = current_pos_.pose.pose.position.y - (1.335 * sin(theta));
            c_x = d_x + (5 * sin(gamma));
            c_y = d_y - (5 * cos(gamma));
        }
        else { //  heading between 3PI/2 and 2PI
            theta =  2*M_PI - heading;
            gamma = M_PI_2 - theta;
            a_x = current_pos_.pose.pose.position.x - (1.335 * cos(theta));
            a_y = current_pos_.pose.pose.position.y + (1.335 * sin(theta));
            b_x = a_x + (5 * sin(theta));
            b_y = a_y + (5 * cos(theta));
            d_x = current_pos_.pose.pose.position.x + (1.335 * sin(gamma));
            d_y = current_pos_.pose.pose.position.y - (1.335 * cos(gamma));
            c_x = d_x + (5 * cos(gamma));
            c_y = d_y + (5 * sin(gamma));
        }


        for (int i = 0; i < sizeof(props_.props); i++) {
            if (props_.props[i].prop_label == "obstacle") {
                double p_x = props_.props[i].vector.x;
                double p_y = props_.props[i].vector.y;
                // determine barycentric coordinates
                double bary_a = ((c_y-d_y)*(p_x-d_x)+(d_x-c_x)*(p_y-d_y))/((c_y-d_y)*(a_x-d_x)+(d_x-c_x)*(a_y-d_y));
                double bary_b = ((d_y-a_y)*(p_x-d_x)+(a_x-d_x)*(p_y-d_y))/((c_y-d_y)*(a_x-d_x)+(d_x-c_x)*(a_y-d_y));
                double bary_c = ((a_y-b_y)*(p_x-a_x)+(b_x-a_x)*(p_y-a_y))/((a_y-b_y)*(c_x-a_x)+(b_x-a_x)*(c_y-a_y));
                double bary_d = ((b_y-c_y)*(p_x-a_x)+(c_x-a_x)*(p_y-a_y))/((a_y-b_y)*(c_x-a_x)+(b_x-a_x)*(c_y-a_y));
                // if prop within 5 meters of the front of the boat and prop label == 'obstacle' and distance to the obstacle < dist_to_obstacle
                    // then props_[i] is the closest obstacle to the boat
                // if a,b,c,d between 0 and 1 and a+b+c+d=1, then prop is an obstacle that needs to be avoided
                if (bary_a>=0 && bary_a<=1 && bary_b>=0 && bary_b<=1 && bary_c>=0 && bary_c<=1 && bary_d>=0 && bary_d<=1 && bary_a+bary_b+bary_c+bary_d>0.99 && bary_a+bary_b+bary_c+bary_d<1.01) { // range should be set as a parameter
                    double dist_to_i = sqrt(pow(current_pos_.pose.pose.position.x - p_x, 2) + pow(current_pos_.pose.pose.position.y - p_y, 2));
                    if (dist_to_i < dist_to_obstacle) {
                        obstacle = props_.props[i];
                    }
                }
            }
        }
        return obstacle;
    }

    void avoidObstacle(prop_mapper::Prop obstacle) {
        // store current goal, so we can set goal again when the obstacle is cleared.
        if (!obstacleNearGate(obstacle)) {
            task_master::TaskGoalPosition goal = goal_pos_; // only do this if obstacle is not near a gate buoy
            double heading = getGazeboHeading(current_pos_.pose.pose.orientation); // in radians
            double theta;
            double wpf_x;
            double wpf_y;
            double wpl_x;
            double wpl_y;
            double wpr_x;
            double wpr_y;
            double wpb_x;
            double wpb_y;
            

            if (heading < M_PI_2) {
                theta = heading;
                wpf_x = obstacle.vector.x + sin(theta);
                wpf_y = obstacle.vector.y - cos(theta);
                wpl_x = obstacle.vector.x - cos(theta);
                wpl_y = obstacle.vector.y - sin(theta);
                wpr_x = obstacle.vector.x + cos(theta);
                wpr_y = obstacle.vector.y + sin(theta);
                wpb_x = obstacle.vector.x - sin(theta);
                wpb_y = obstacle.vector.y + cos(theta);

            }
            else if (heading >= M_PI_2 && heading < M_PI) {
                theta = M_PI - heading;
                wpf_x = obstacle.vector.x + sin(theta);
                wpf_y = obstacle.vector.y + cos(theta);
                wpl_x = obstacle.vector.x + cos(theta);
                wpl_y = obstacle.vector.y - sin(theta);
                wpr_x = obstacle.vector.x - cos(theta);
                wpr_y = obstacle.vector.y + sin(theta);
                wpb_x = obstacle.vector.x - sin(theta);
                wpb_y = obstacle.vector.y - cos(theta);
            }
            else if (heading >= M_PI && heading < (M_PI+M_PI_2)) {
                theta = heading - M_PI;
                wpf_x = obstacle.vector.x - sin(theta);
                wpf_y = obstacle.vector.y + cos(theta);
                wpl_x = obstacle.vector.x + cos(theta);
                wpl_y = obstacle.vector.y + sin(theta);
                wpr_x = obstacle.vector.x - cos(theta);
                wpr_y = obstacle.vector.y - sin(theta);
                wpb_x = obstacle.vector.x + sin(theta);
                wpb_y = obstacle.vector.y - cos(theta);
            }
            else { //  heading between 3PI/2 and 2PI
                theta =  2*M_PI - heading;
                wpf_x = obstacle.vector.x - sin(theta);
                wpf_y = obstacle.vector.y - cos(theta);
                wpl_x = obstacle.vector.x - cos(theta);
                wpl_y = obstacle.vector.y + sin(theta);
                wpr_x = obstacle.vector.x + cos(theta);
                wpr_y = obstacle.vector.y - sin(theta);
                wpb_x = obstacle.vector.x + sin(theta);
                wpb_y = obstacle.vector.y + cos(theta);
            }


            if ((sqrt(pow(current_pos_.pose.pose.position.x - wpl_x, 2) + pow(current_pos_.pose.pose.position.y - wpl_y, 2))) < (sqrt(pow(current_pos_.pose.pose.position.x - wpr_x, 2) + pow(current_pos_.pose.pose.position.y - wpr_y, 2)))) {
                // go leftt
                setDestination(wpf_x, wpf_y);
                setDestination(wpl_x, wpl_y);
                setDestination(wpb_x, wpb_y);
                // return to original goal
            }            
        }

        // IF OBSTACLE IS NOT NEAR A GATE BUOY (near implies not enough room to pass through as gates can be as small as 6ft, want 1m space for the boat)
        // determine the optimal direction to avoid. calculate the shortest distance to either side of the obstacle.
            // find waypoint behind the obstacle, store it
                // wp_back = obstacle + 1 meter in direction of boats vector
            // find waypoint to both sides of the obstacle, travel to shortest one (or take path necessary if marker is nearby)
                // wp_left = obstacle + 1 meter perpendicular left to boats vector
                // dist_left = wp_left - boat
                // wp_right = obstacle + 1 meter perpendicular right to boats vector
                // dist_right = wp_right - boat
                // set wp_side to the wp corresponding to the shorter distance and travel to it
            // once arrived at side waypoint, set to behind waypoint
                // travel to wp_back
            // once arrived at behind waypoint, reset waypoint to original
                // travel to goal

        // IF OBSTACLE IS NEAR A GATE BUOY
        // determine what path allows the boat to travel
            // if marker is to the left and behind the obstacle, have to avoid right then pass through gate
                // set wp_side, then set a waypoint to the goal
            // if marker is to the right and behind the obstacle, have to avoid left then pass through gate
                // set wp_side, then set a waypoint to the goal
            // if marker is to the left and in front of the obstacle, set a waypoint as the midpoint of the obstacle and the right buoy
            // if marker is to the right and in front of the obstacle, set a waypoint as the midpoint of the obstacle and the left buoy
    }

    double getGazeboHeading(const geometry_msgs::Quaternion q) {
        // yaw (z-axis rotation)
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);

        // convert to gazebo heading
        double heading = yaw - (M_PI/2);
        if (heading < 0)
        {
            heading = heading + (2*M_PI);
        }
        return heading;
    }

    bool obstacleNearGate(prop_mapper::Prop obstacle) {
        for (int i = 0; i < sizeof(props_.props); i++) {
            if (props_.props[i].prop_label == "gate_buoy") {
                if ((sqrt(pow(props_.props[i].vector.x - obstacle.vector.x, 2) + pow(props_.props[i].vector.y - obstacle.vector.y, 2))) <= 1) {
                    ROS_INFO_STREAM("gate close to buoy");
                    return true;
                }
            }
        }
        return false;
    }

    void setDestination(double x, double y) {

    }

private:

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber prop_map_;
    ros::Subscriber global_pos_;
    ros::Subscriber task_goal_position_;
    ros::Publisher waypoint_;

    nav_msgs::Odometry current_pos_;
    prop_mapper::PropArray props_;
    task_master::TaskGoalPosition goal_pos_;

    double start_pos_x;
    double start_pos_y;
    double end_pos_x;
    double end_pos_y;
    double obstacle_x;
    double obstacle_y;
    bool avoid_left;
    double safety_dist;

    void propMapCallback(const prop_mapper::PropArray msg) {
        props_ = msg;
    }

    void globalPositionCallback(const nav_msgs::Odometry msg) {
        current_pos_ = msg;
        double heading = getGazeboHeading(current_pos_.pose.pose.orientation);
        ROS_INFO_STREAM("Heading: " << heading);
        // check to see if any props are in the path of the boat.
        prop_mapper::Prop obstacle = findObstacle(); // checks for obstacle within 5 meters, returns prop if it exists
        if (obstacle.prop_label != "no obstacle") { // if obstacle, avoid it
            avoidObstacle(obstacle);
        }
        else {
            ROS_DEBUG("No obstacles currently found.");
        }
    }

    void goalPositionCallback(const task_master::TaskGoalPosition msg) {
        goal_pos_ = msg;
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_avoidance_node");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
        ros::console::notifyLoggerLevelsChanged();

    ObjectAvoidance object_avoidance;

    object_avoidance.spin();

    return 0;
}