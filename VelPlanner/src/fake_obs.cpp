#include <stdio.h>
#include <iostream>
#include <Eigen/Eigen>

#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <obj_state_msgs/ObjectsStates.h>

typedef std::vector<Eigen::Vector3d> vecVec3;
using namespace std;
using namespace Eigen;

ros::Publisher obj_vis_pub, obj_state_pub;

void obj_state_pb(const vecVec3 &ps, const vecVec3 &vs, const vecVec3 &ss, const ros::Time &ts, const string &world_frame_id)
{
    obj_state_msgs::ObjectsStates states;

    states.header.frame_id = world_frame_id;
    states.header.stamp = ts; // ros::Time::now(); // // time_list.back();

    for (uint i = 0; i < ps.size(); i++)
    {
        obj_state_msgs::State state;
        state.position.x = ps[i](0);
        state.position.y = ps[i](1);
        state.position.z = ps[i](2);
        state.velocity.x = vs[i](0);
        state.velocity.y = vs[i](1);
        state.velocity.z = vs[i](2);
        state.size.x = ss[i](0);
        state.size.y = ss[i](1);
        state.size.z = ss[i](2);
        state.acceleration.x = 0;
        state.acceleration.y = 0;
        state.acceleration.z = 0;
        states.states.push_back(state);
    }
    obj_state_pub.publish(states);
}
void dyn_pb(const vecVec3 &ps, const vecVec3 &vs, const vecVec3 &ss, const ros::Time &ts, const string &world_frame_id)
{
    visualization_msgs::MarkerArray dyn;
    double life_t = 0.1;
    int id = 0;
    for (uint i = 0; i < ps.size(); i++)
    {
        visualization_msgs::Marker dynobs;
        dynobs.header.frame_id = world_frame_id;
        dynobs.header.stamp = ros::Time::now();
        dynobs.type = visualization_msgs::Marker::CUBE;
        dynobs.pose.position.x = ps[i](0);
        dynobs.pose.position.y = ps[i](1);
        dynobs.pose.position.z = ps[i](2);
        dynobs.id = id++;
        dynobs.scale.x = ss[i](0);
        dynobs.scale.y = ss[i](1);
        dynobs.scale.z = ss[i](2);

        dynobs.color.a = 0.4;
        dynobs.color.r = 0.0;
        dynobs.color.g = 0.9;
        dynobs.color.b = 0.1;
        dynobs.pose.orientation.x = 0;
        dynobs.pose.orientation.y = 0;
        dynobs.pose.orientation.z = 0;
        dynobs.pose.orientation.w = 1.0;

        visualization_msgs::Marker dynv;
        geometry_msgs::Point p1, p2;
        dynv.header.frame_id = world_frame_id;
        dynv.header.stamp = dynobs.header.stamp;
        dynv.type = visualization_msgs::Marker::ARROW;
        dynv.id = id++;
        p1.x = dynobs.pose.position.x;
        p1.y = dynobs.pose.position.y;
        p1.z = dynobs.pose.position.z;
        p2.x = p1.x + vs[i](0);
        p2.y = p1.y + vs[i](1);
        p2.z = p1.z + vs[i](2);
        dynv.points.push_back(p1);
        dynv.points.push_back(p2);
        dynv.pose.orientation.w = 1;
        dynv.scale.x = 0.2; // diameter of arrow shaft
        dynv.scale.y = 0.4; // diameter of arrow head
        dynv.scale.z = 0.6;
        dynv.color.a = 1; // transparency
        dynv.color.r = 0.8;
        dynv.color.g = 0.1;
        dynv.color.b = 0.1;

        visualization_msgs::Marker dynid;
        dynid.header.frame_id = world_frame_id;
        dynid.header.stamp = ros::Time::now();
        dynid.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        dynid.pose.position.x = ps[i](0);
        dynid.pose.position.y = ps[i](1);
        dynid.pose.position.z = ps[i](2) + 1;
        dynid.id = id++;
        dynid.pose.orientation.w = 1;
        dynid.scale.z = 0.5;
        dynid.text = to_string(i);
        dynid.color.a = 1;
        dynid.color.r = 1.0;
        dynobs.lifetime = ros::Duration(life_t);
        dynv.lifetime = dynobs.lifetime;
        dynid.lifetime = dynobs.lifetime;
        // dynbbox.lifetime = rospy.Duration(life_t)
        dyn.markers.push_back(dynobs);
        dyn.markers.push_back(dynv);
        // dyn.markers.push_back(dynid);
        // dyn.markers.append(dynbbox)
    }
    obj_vis_pub.publish(dyn);
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "fake_obs_publisher");
    // Create a handle to this process node.
    ros::NodeHandle nh("~");
    string world_frame = "map";
    int obs_num = 10;
    double obs_max_speed = 2.0;
    int freq = 30;
    ros::Rate loop_rate(freq);
    ros::Time last_t;
    vector<double> gbbox_o, gbbox_l, size_min, size_max;
    nh.getParam("WorldFrameName", world_frame);
    nh.getParam("GlobalBox_min", gbbox_o);
    nh.getParam("GlobalBox_size", gbbox_l);
    nh.getParam("DynObsSize_min", size_min);
    nh.getParam("DynObsSize_max", size_max);
    nh.getParam("DynObsSpeed_max", obs_max_speed);
    nh.getParam("DynObsNum", obs_num);
    

    obj_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/dyn", 3);
    obj_state_pub = nh.advertise<obj_state_msgs::ObjectsStates>("/obj_states", 3);
    vecVec3 pos_list, vel_list, size_list;
    // initialize
    srand((unsigned)time(NULL));
    for (int i = 0; i < obs_num; i++)
    {
        Vector3d pos, vel, size;

        pos(0) = gbbox_o[0] + rand() % (int(gbbox_l[0]) - 6) + 3.0;
        pos(1) = gbbox_o[1] + rand() % (int(gbbox_l[1]) - 6) + 3.0;
        pos(2) = gbbox_o[2] + rand() % int(gbbox_l[2]);

        size(0) = (double)(rand() % 101) / 101 * (size_max[0] - size_min[0]) + size_min[0];
        size(1) = (double)(rand() % 101) / 101 * (size_max[1] - size_min[1]) + size_min[1];
        size(2) = (double)(rand() % 101) / 101 * (size_max[2] - size_min[2]) + size_min[2];

        vel << (double)(rand() % 101) / 101 * obs_max_speed, (double)(rand() % 101) / 101 * obs_max_speed, (double)(rand() % 101) / 101 * obs_max_speed;
        pos_list.emplace_back(pos);
        vel_list.emplace_back(vel);
        size_list.emplace_back(size);
    }
    last_t = ros::Time::now();
    
    while (nh.ok()) // main loop
    {
        // forward the obstacles' states
        double t_gap = (ros::Time::now() - last_t).toSec();
        for (int i = 0; i < obs_num; i++)
        {
            if (pos_list[i](0) < gbbox_o[0] + 1 ||
                pos_list[i](1) < gbbox_o[1] + 1 ||
                pos_list[i](2) < gbbox_o[2] ||
                pos_list[i](0) > (gbbox_o[0] + gbbox_l[0] - 1) ||
                pos_list[i](1) > (gbbox_o[1] + gbbox_l[1] - 1) ||
                pos_list[i](2) > (gbbox_o[2] + gbbox_l[2]))
            {
                vel_list[i] *= -1;  //reverse the speed if the obstacle hit the global boundary
                // cout<<"pos outside the global box: "<<pos_list[i].transpose()<<endl;
                // cout<<"global min and max: "<<gbbox_o[0] + 1<<" "<<gbbox_o[1] + 1<<" "<<gbbox_o[2]<<" "
                // <<(gbbox_o[0] + gbbox_l[0] - 1)<<" "<<(gbbox_o[1] + gbbox_l[1] - 1)<<" "<<(gbbox_o[2] + gbbox_l[2])<<endl;
            }
            pos_list[i] += t_gap * vel_list[i];
        }
        last_t = ros::Time::now();

        obj_state_pb(pos_list, vel_list, size_list, last_t,world_frame);
        dyn_pb(pos_list, vel_list, size_list, last_t,world_frame);
        ros::spinOnce();
        loop_rate.sleep();
    }
}