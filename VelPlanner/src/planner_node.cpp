#include <stdio.h>
#include <iostream>
#include <Eigen/Eigen>
#include <math.h>
#include <ros/ros.h>
#include <ros_missn.h>
#include <planner.hpp>
#include <Tools.hpp>
#include <time.h>
#include <tf/transform_broadcaster.h>

using namespace std;
using namespace Eigen;

double sign(double x)
{
    return (x > 0) - (x < 0);
}

void pubTF(const Vector3d &pos, const double yaw, const string world_fm)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(pos(0), pos(1), pos(2)));
  tf::Quaternion q;
  q.setRPY(0, 0, yaw);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), world_fm, "fake_drone"));
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "traj_planner");
    // Create a handle to this process node.
    ros::NodeHandle nh("~");

    Vector3d ct_pos, ct_vel, ct_acc, home_pos;
    ct_pos.setZero();
    ct_vel.setZero();
    ct_acc.setZero();
    ct_pos(2) = 1.0; // just for rviz sim
    Vector3d p_d, v_d, a_d, p_d_yaw;
    a_d.setZero();
    MatrixXd sp_pos, sp_vel, sp_acc;
    // double last_path_t = 0;

    // Eigen::Vector3d end_state = Eigen::Vector3d::Zero(3);
    vector<double> goalp, gbbox_o, gbbox_l, timecosts;
    Matrix<double, 3, 5> camera_vertex, camera_vertex_b, camera_vertex_bv;

    double d2r = 3.14159265 / 180;
    double cam_depth = 10.0;
    double cam_depth_v = 2.0;
    double h_fov = 87; // in degree
    double v_fov = 58;
    double dis_goal = 8.0, tem_dis_goal;
    // double sfck_t;
    bool ifMove, if_rand;
    int CtrlFreq;
    bool if_debug;
    // bool path_replan;
    bool rc_goal;
    double gap;
    double singlestep_time;
    bool if_reach = false, if_time_out = false;
    bool last_if_reach = false;
    int return_home = 0;
    double min_dist2dynobs;
    Eigen::Vector2d desire_psi(.0, .0);
    string control_mode = "pos_vel_acc_yaw_c";
    string world_frame = "map";
    nh.getParam("goal", goalp);
    nh.getParam("plan_horizon", dis_goal);
    nh.getParam("ifMove", ifMove);
    nh.getParam("cam_depth", cam_depth);
    nh.getParam("h_fov", h_fov);
    nh.getParam("v_fov", v_fov);
    nh.getParam("if_RandomGoal", if_rand);
    nh.getParam("GlobalBox_min", gbbox_o);
    nh.getParam("GlobalBox_size", gbbox_l);
    nh.getParam("CtrlFreq", CtrlFreq);
    nh.getParam("if_debug", if_debug);

    nh.getParam("ControlMode", control_mode);
    nh.getParam("WorldFrameName", world_frame);

    nh.getParam("ReturnHome", return_home);
    nh.getParam("UseRcGuide", rc_goal);

    ros::Rate loop_rate(CtrlFreq);
    camera_vertex_b.col(0) << 0, 0, 0;
    camera_vertex_b.col(1) << cam_depth, tan(h_fov / 2 * d2r) * cam_depth, tan(v_fov / 2 * d2r) * cam_depth;
    camera_vertex_b.col(2) << cam_depth, -tan(h_fov / 2 * d2r) * cam_depth, tan(v_fov / 2 * d2r) * cam_depth;
    camera_vertex_b.col(3) << cam_depth, -tan(h_fov / 2 * d2r) * cam_depth, -tan(v_fov / 2 * d2r) * cam_depth;
    camera_vertex_b.col(4) << cam_depth, tan(h_fov / 2 * d2r) * cam_depth, -tan(v_fov / 2 * d2r) * cam_depth;

    camera_vertex_bv.col(0) << 0, 0, 0;
    camera_vertex_bv.col(1) << cam_depth_v, tan(h_fov / 2 * d2r) * cam_depth_v, tan(v_fov / 2 * d2r) * cam_depth_v;
    camera_vertex_bv.col(2) << cam_depth_v, -tan(h_fov / 2 * d2r) * cam_depth_v, tan(v_fov / 2 * d2r) * cam_depth_v;
    camera_vertex_bv.col(3) << cam_depth_v, -tan(h_fov / 2 * d2r) * cam_depth_v, -tan(v_fov / 2 * d2r) * cam_depth_v;
    camera_vertex_bv.col(4) << cam_depth_v, tan(h_fov / 2 * d2r) * cam_depth_v, -tan(v_fov / 2 * d2r) * cam_depth_v;
    VelPlanner planner(camera_vertex_b);
    // dis_goal = dis_goal_ini-0.5;
    Eigen::Vector3d g_goal = {goalp[0], goalp[1], goalp[2]};
    Eigen::Vector3d goal = g_goal;
    Eigen::Vector3d local_goal = {0, 0, 0};
    Eigen::Vector3d initial_goal = g_goal;
    vector<vector<Eigen::Vector3d>> search_start_end;

    bool if_initial = true;
    bool if_end = false;
    bool if_safe = true;

    // dyn_pass_time = 0.01;

    double timee = 0;
    ros::Time traj_last_t = ros::Time::now();
    chrono::high_resolution_clock::time_point last_traj_tic = chrono::high_resolution_clock::now();
    int rand_num = -1, rand_num_tmp;
    planner.config.loadParameters(nh);
    States state;
    cout << "Traj node initialized!" << endl;
    RosClass RosMsg(&nh, CtrlFreq);
    RosMsg.world_frame = world_frame;
    // do
    // {
    //     state = RosMsg.get_state();
    //     ct_pos = state.P_E;
    //     ros::Duration(0.1).sleep();
    // } while ((ct_pos.norm() < 1e-3) || (ct_pos.norm() > 1e3) || isnan(ct_pos(0)) || isnan(ct_pos(1)) || isnan(ct_pos(2)));
    // cout << "UAV message received!" << ct_pos << endl;
    state = RosMsg.get_state();
    planner.last_check_pos = ct_pos;
    home_pos = ct_pos;
    state.P_E = ct_pos; // just for rviz sim
    // RosMsg.dynobs_pointer->ball_number = 0;

    // ------- start the safety check and re-planning loop--------------------------------------------------------------

    srand((unsigned)time(NULL));
    while (nh.ok()) // main loop
    {
        // if (!planner.inGlobalBound(ct_pos))
        // {
        //     ROS_ERROR_STREAM("Agent's pos is outside the global box!");
        //     // cout << ct_pos.transpose() << endl;
        //     exit(5);
        // }
        if ((return_home > 0 && if_end) || (if_rand && (if_end || (rand_num < 0)))) // choose goal randomly at the global bounding box boundary
        {
            if (if_rand)
            {
                if (rand_num < 0)
                    rand_num = rand() % 4;
                else
                {
                    do
                        rand_num_tmp = rand() % 4;
                    while (rand_num_tmp == rand_num);
                    rand_num = rand_num_tmp;
                }
                if (rand_num == 0)
                {
                    g_goal(0) = gbbox_o[0] + gbbox_l[0] - 1.0;
                    g_goal(1) = gbbox_o[1] + rand() % int(gbbox_l[1]) + 1.0;
                }
                else if (rand_num == 1)
                {
                    g_goal(0) = gbbox_o[0] + rand() % int(gbbox_l[0]) + 1.0;
                    g_goal(1) = gbbox_o[1] + gbbox_l[1] - 1.0;
                }
                else if (rand_num == 2)
                {
                    g_goal(0) = gbbox_o[0] + 1.0;
                    g_goal(1) = gbbox_o[1] + rand() % int(gbbox_l[1]) + 1.0;
                }
                else
                {
                    g_goal(0) = gbbox_o[0] + rand() % int(gbbox_l[0]) + 1.0;
                    g_goal(1) = gbbox_o[1] + 1.0;
                }
                g_goal(2) = 1.5;
                goal = g_goal;
            }
            else if (return_home > 1 && if_end)
            {
                if ((home_pos - ct_pos).norm() > 1)
                    g_goal = home_pos;
                else
                    g_goal = initial_goal;
                goal = g_goal;
                // cout << "one goal: " << g_goal << endl;
                return_home--;
            }

            if_end = false;
            if_initial = true;
            if_safe = true;
            if_reach = false;
            if_time_out = false;
            planner.has_traj = false;

            RosMsg.dynobs_pointer->clear_dyn();
            cout << "Next goal: " << g_goal.transpose() << endl;
            // Vector2d v2 = (g_goal - state.P_E).head(2);
            // Vector2d v1;
            // v1 << 1.0, 0.0;
            // double desire_yaw = acos(v1.dot(v2) / (v1.norm() * v2.norm()));
            // if (v2(1) < 0)
            // {
            //     desire_yaw = -desire_yaw;
            // }
            // ct_pos = state.P_E;
            // double yaw_rate = abs(desire_yaw - state.Euler(2)) > 3.14159 ? -sign(desire_yaw - state.Euler(2)) * 0.6 : sign(desire_yaw - state.Euler(2)) * 0.6;
            // ros::Duration(0.5).sleep();
            // do
            // {
            //     state = RosMsg.step(state.Euler(2) + yaw_rate * 0.5, yaw_rate, ct_pos, Vector3d::Zero(3), Vector3d::Zero(3), control_mode);
            //     ros::Duration(0.05).sleep();
            // } while (abs(state.Euler(2) - desire_yaw) > 0.3);
            // ros::Duration(0.5).sleep();
        }
        // goal is choosen, planner begin
        chrono::high_resolution_clock::time_point t1 = chrono::high_resolution_clock::now();
        if (RosMsg.dyn_update)
        {

            // RosMsg.set_cod_update(false);
            // cout << "dyn obs update! " << endl;
            if (!RosMsg.dyn_update && RosMsg.dynobs_pointer->dyn_number > 0 &&
                ros::Time::now().toSec() - RosMsg.dynobs_pointer->time_stamp > 0.1)
            {
                RosMsg.dynobs_pointer->clear_dyn();
                cout << "dyn time out! " << endl;
            }

            min_dist2dynobs = 1e6;
            for (int bi = 0; bi < RosMsg.dynobs_pointer->dyn_number; bi++)

            {
                double t_gap_ball = ros::Time::now().toSec() - RosMsg.dynobs_pointer->time_stamp;
                Vector3d tmp_dist = (state.P_E - RosMsg.dynobs_pointer->centers[bi] - t_gap_ball * RosMsg.dynobs_pointer->vels[bi]);
                if (((tmp_dist.cwiseAbs().array() - RosMsg.dynobs_pointer->obs_sizes[bi].array()*0.5) < 0).all())
                {
                    ROS_ERROR_STREAM("Agent collides with obstacles!");
                    cout<<state.P_E.transpose()<<" half size: "<<RosMsg.dynobs_pointer->obs_sizes[bi].transpose()*0.5 <<" dist vec: "<<tmp_dist.transpose()<<endl;
                }
                // if (tmp_dist < min_dist2dynobs)
                // {
                //     min_dist2dynobs = tmp_dist;
                //     if ()
                //     if (if_debug)
                //         cout << "min distance from objects to drone:" << min_dist2dynobs << endl
                //              << state.P_E << endl
                //              << RosMsg.dynobs_pointer->centers[bi] + t_gap_ball * RosMsg.dynobs_pointer->vels[bi];
                // }
            }

            if (if_initial || !ifMove)
            {
                ct_pos = state.P_E;
                ct_vel.setZero();
                ct_acc.setZero();
            }
            else
            {
                ct_pos = p_d;
                ct_vel = v_d;
                ct_acc = a_d;
            }

            state = RosMsg.get_state();

            state.P_E = ct_pos; // just for rviz sim
            // Eigen::AngleAxisd sp_rotation_vector = Eigen::AngleAxisd(desire_psi(0), (ct_acc + Vector3d(0, 0, planner.config.gravAcc)).normalized());
            Eigen::Quaterniond quaternion = Tools::calcQuatFromAccYaw(ct_acc, goal - ct_pos);
            state.Rota = quaternion.matrix();
            state.Rota_EB = state.Rota.transpose();

            planner.drone_state = state;
            RosMsg.dyn_update = false;
            RosMsg.set_pcl_update(false);

            last_if_reach = if_reach;
            if (if_debug)
                cout << "track goal dist: " << (state.P_E - p_d).norm() << endl;
            // cout<<"if initial: "<<if_initial<<endl;
            // camera_vertex = (state.Rota * camera_vertex_b).array().colwise() + state.P_E.array();
            double dis2goal = (g_goal - ct_pos).norm();
            // get a projected local goal
            if (dis2goal > dis_goal)
            {
                goal = ct_pos + (g_goal - ct_pos) / dis2goal * dis_goal;
                if_reach = false;
                tem_dis_goal = dis_goal;
            }
            else
            {
                goal = g_goal;
                if_reach = true;
                tem_dis_goal = dis2goal;
            }

            if (!if_initial)
            {
                if_safe = planner.check_vel_safe(ct_pos, ct_vel, RosMsg.dynobs_pointer);
            }
            // if it is the first time to plan, or the current velocity is not safe
            if_time_out = (ros::Time::now() - traj_last_t).toSec() > 1.0;
            if (!if_safe)
            {

                chrono::high_resolution_clock::time_point tic = chrono::high_resolution_clock::now();
                Eigen::Vector3d des_vel = planner.plan_vel(ct_pos, ct_vel, RosMsg.dynobs_pointer);
                planner.gen_traj(ct_pos, ct_vel, ct_acc, goal, des_vel);
                double compTime = chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
                cout << "Traj re-planned! time cost (ms)： " << compTime << endl;
                traj_last_t = ros::Time::now();
                gap = chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - last_traj_tic).count() * 1.0e-6 - timee;
                last_traj_tic = chrono::high_resolution_clock::now();
                if (if_initial)
                    timee = 0; // 1 / CtrlFreq / 4;
                else
                    timee = gap;
                if (if_initial)
                    if_initial = false;
            }
            else if (if_initial || if_time_out)
            {
                Eigen::Vector3d des_vel = planner.config.velMax * (goal - ct_pos).normalized();
                planner.gen_traj(ct_pos, ct_vel, ct_acc, goal, des_vel);
                ROS_INFO_STREAM("\033[33m Traj timeout, use velocity to goal. \033[0m");
                traj_last_t = ros::Time::now();
                gap = chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - last_traj_tic).count() * 1.0e-6 - timee;
                last_traj_tic = chrono::high_resolution_clock::now();
                if (if_initial)
                    timee = 0; // 1 / CtrlFreq / 4;
                else
                    timee = gap;
                if (if_initial)
                    if_initial = false;
            }

            singlestep_time = chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - t1).count() * 1.0e-3;
            // cout << "Total time cost (ms): " << singlestep_time << endl;
            timecosts.push_back(singlestep_time);
            if (timecosts.size() > 1000)
            {
                cout << "Average Total time cost (ms): " << std::accumulate(std::begin(timecosts), std::end(timecosts), 0.0) / timecosts.size() << endl;
                timecosts.clear();
            }
            //  <<ros::Time::now().toSec()<<endl<<t1<<endl<<traj_last_t<<endl<<RosMsg.dynobs_pointer->time_stamp<<endl;
            // RosMsg.pub_fovlist (sp_pos, sp_vel, sp_acc,camera_vertex_bv, planner.yaw_plan);
        }

        else // if pcl has not been updated
        {

            gap = chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - last_traj_tic).count() * 1.0e-6;
            timee = gap; // + 1 / CtrlFreq / 4;
            // cout<<"control sample time gap (no pcl updated): "<<gap<<endl;
            state = RosMsg.get_state();

            state.P_E = ct_pos; // just for rviz sim
            Eigen::AngleAxisd sp_rotation_vector = Eigen::AngleAxisd(desire_psi(0), (ct_acc + Vector3d(0, 0, planner.config.gravAcc)).normalized());
            state.Rota = sp_rotation_vector.matrix();
            state.Rota_EB = state.Rota.transpose();

            planner.drone_state = state;
            // cout << "dyn obs not update, try to refresh" << endl;
        }
        // cout << "if_has_traj: "<<planner.has_traj<<endl;
        if (!planner.has_traj)
        {
            loop_rate.sleep();
            continue;
        }

        if (timee < planner.traj.duration - 0.01)
        {
            planner.get_desire(timee, p_d, v_d, a_d, desire_psi(0));
            // cout << "desire pos vel and acc: \n"
            //      << p_d.transpose() << endl
            //      << v_d.transpose() << endl
            //      << a_d.transpose() << endl;
        }
        else
        {
            planner.get_desire(planner.traj.duration, p_d, v_d, a_d, desire_psi(0));
            cout << "goal reached: " << g_goal.transpose() << endl;
            if_end = true;
        }
        pubTF(p_d,desire_psi(0),world_frame);
        if (ifMove)
        {
            state = RosMsg.step(desire_psi(0), desire_psi(1), p_d, v_d, a_d, control_mode);
        }
        if (ifMove)
            planner.get_traj_samples(sp_pos, sp_vel, sp_acc, timee);
        else
            planner.get_traj_samples(sp_pos, sp_vel, sp_acc, 0.0);
        RosMsg.pub_traj(sp_pos, sp_vel, a_d, desire_psi(0));
        // RosMsg.pub_fovshape(camera_vertex);

        loop_rate.sleep();
        if (if_end && !if_rand && return_home <= 1 && !rc_goal)
        {
            break;
        }
    }

    exit(0);
}
