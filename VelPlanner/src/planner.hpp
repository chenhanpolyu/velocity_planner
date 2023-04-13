#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include <ros_communicate.h>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <Eigen/Eigen>
#include <stdlib.h>
#include <Tools.hpp>
#include <vector>
#include <map>

using namespace std;
using namespace Eigen;
using namespace Tools;
// using namespace min_jerk;
IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

struct Config
{
    string odomFrame;

    // Params
    double scaleSI;
    // double mapHeight;
    Vector3d polyhedronBox, global_min, global_size;
    MatrixXd global_bound;
    double rho;
    double totalT;
    int qdIntervals;
    double horizHalfLen;
    double vertHalfLen;
    double safeMargin;
    double velMax;
    double thrustAccMin;
    double thrustAccMax;
    double horizontalAccMax;
    double bodyRateMax;
    double gravAcc;
    Vector4d penaltyPVTB;
    bool useC2Diffeo;
    bool if_debug;
    double optRelTol;
    double yaw_replan_t;
    double trajVizWidth;
    Vector3d trajVizRGB;
    string routeStoragePath;
    string ellipsoidPath;
    Vector4d ellipsoidVizRGBA;
    int max_yaw_range_n = 10;
    double yaw_w = 0.4;
    double yaw_gap_max = 0.55;
    double horizon = 8.0;
    double yaw_reso = 0.175;
    double delta_t_yaw = 0.5;
    double yaw_heading_t = 0.5;
    double dyn_dis_forcheck = 4.0;
    bool applyUnknownAwareYaw = true;
    bool yawplan = false;
    double weight_dis = 100;
    double jerMax = 32;
    inline void loadParameters(ros::NodeHandle nh_priv)
    // {       cout << "mk3" << endl;
    {
        vector<double> vecGlobal_min, vecGlobal_size;

        nh_priv.getParam("GlobalBox_min", vecGlobal_min);
        nh_priv.getParam("GlobalBox_size", vecGlobal_size);
        global_min << vecGlobal_min[0], vecGlobal_min[1], vecGlobal_min[2];
        global_size << vecGlobal_size[0], vecGlobal_size[1], vecGlobal_size[2];

        nh_priv.getParam("SafeMargin", safeMargin);
        nh_priv.getParam("VelMax", velMax);
        nh_priv.getParam("JerkMax", jerMax);
        nh_priv.getParam("VelPlanDisWeight", weight_dis);

        nh_priv.getParam("ThrustAccMax", thrustAccMax);
        nh_priv.getParam("GravAcc", gravAcc);

        nh_priv.getParam("if_debug", if_debug);
        nh_priv.getParam("search/horizon", horizon);
        nh_priv.getParam("SafeCheckRange", dyn_dis_forcheck);

        global_bound.resize(3, 2);
        global_bound.col(0) = global_min.array() + safeMargin;
        global_bound.col(1) = (global_min + global_size).array() - safeMargin;
        horizontalAccMax = sqrt(thrustAccMax * thrustAccMax - gravAcc * gravAcc);
    }
};

struct Segment
{
    double duration, duration_l, duration_s;
    Vector3d durs, temp_jk;
    Vector3d start_p, start_v, start_a, end_p, end_v, end_a;
    // Matrix3d start, end;
    Matrix<double, 3, 5> A;

    Vector3d getAcc(const double t)
    {
        return 6 * t * A.col(3) + 2 * A.col(2) + 12 * A.col(4) * pow(t, 2);
    }
    Vector3d getVel(const double t)
    {
        return 3 * A.col(3) * t * t + 2 * A.col(2) * t + A.col(1) + 4 * A.col(4) * pow(t, 3);
    }
    Vector3d getPos(const double t)
    {
        return A.col(0) + A.col(1) * t + A.col(2) * t * t + A.col(3) * t * t * t + A.col(4) * pow(t, 4);
    }

    void Generate_MinTEndV(const Vector3d &pos0, const Vector3d &vel0, const Vector3d &acc0, const Vector3d &endvel,
                           const double max_acc, const double max_jerk, const double min_t)
    {
        // cout<<"pos0: "<<pos0.transpose()<<endl;
        A.col(4).setZero();
        start_p = pos0;
        start_v = vel0;
        start_a = acc0;
        end_v = endvel;

        A.col(0) = pos0;
        A.col(1) = vel0;
        A.col(2) = 0.5 * acc0;
        durs = 2 * (endvel - vel0).cwiseAbs().array() / (max_acc + acc0.array());
        duration = max(min_t, durs.maxCoeff());
        // double duration_ori = duration;
        A.col(3) = (endvel - vel0 - acc0 * duration) / (3 * duration * duration);

        if ((A.col(3).cwiseAbs().array() * 6 > (max_jerk)).any())
        {
            duration_l = duration + 2;
            duration_s = duration;
            Vector3d aOverb = (endvel - vel0).array() / (-acc0.array());
            while ((duration_l - duration_s) > 1e-3)
            {
                duration = (duration_l + duration_s) / 2;
                // A.col(3) = (endvel - vel0 - acc0*duration) / (3*duration*duration);
                temp_jk = (endvel - vel0 - acc0 * duration) / (3 * duration * duration);
                bool flag = false;
                for (int m = 0; m < 3; m++)
                {
                    if (abs(temp_jk(m)) > max_jerk / 6)
                    {
                        flag = true;
                        if (aOverb(m) > 0) // a and b are of the same sign
                        {
                            duration_s = duration;
                            break;
                        }

                        else
                        {
                            if (duration < abs(aOverb(m)))
                            {
                                duration_s = duration;
                                duration_l = abs(aOverb(m));
                                break;
                            }
                            else
                            {
                                duration_l = duration;
                                duration_s = abs(aOverb(m));
                                break;
                            }
                        }
                    }
                }
                if (!flag)
                {
                    // duration_l = duration;
                    break;
                }
            }
            duration = (duration_l + duration_s) / 2;
            A.col(3) = (endvel - vel0 - acc0 * duration) / (3 * duration * duration);
        }
        end_p = getPos(duration);
        end_a = getAcc(duration);
        cout << "A: \n"
             << A.format(CleanFmt) << endl;
        cout << "max jerk: " << (A.col(3) * 6).cwiseAbs().maxCoeff() << endl;
        // cout<<"first seg end vel:\n"<< getVel(duration).transpose() <<"\n"<<endvel.transpose()<<endl;
        // cout<<"A0: "<<A.col(0)<<endl;
        // cout<<"A1: "<<A.col(1)<<endl;
        // cout<<"A2: "<<A.col(2)<<endl;
        // cout<<"A3: "<<A.col(3)<<endl;
    }
    void Generate_MinTEndP(const Vector3d &pos0, const Vector3d &vel0, const Vector3d &acc0, const Vector3d &endpos,
                           const float max_vel, const float max_acc, const float max_jerk, const float min_t)
    {
        A.col(4).setZero();
        start_p = pos0;
        start_v = vel0;
        start_a = acc0;
        end_p = endpos;

        A.col(0) = pos0;
        A.col(1) = vel0;
        A.col(2) = 0.5 * acc0;

        // duration_s = 0;
        duration_l = (endpos - pos0).norm() / (max_vel * 0.25);
        double amax, vmax, jmax;
        // while ((duration_l - duration_s) > 1e-3)
        for (duration = min_t; duration < duration_l; duration += 0.05)
        {
            // duration = (duration_l + duration_s) / 2;
            A.col(3) = ((endpos - A.col(0)) - A.col(1) * duration - A.col(2) * pow(duration, 2)) / pow(duration, 3);
            jmax = (6 * A.col(3)).cwiseAbs().maxCoeff();
            amax = (6 * duration * A.col(3) + 2 * A.col(2)).cwiseAbs().maxCoeff();
            Vector3d v_gap = 3 * duration * duration * A.col(3) + 2 * A.col(2) * duration; // end_vel - vel0
            Vector3d b4ac = (4 * A.col(2).array() * A.col(2).array() - 12 * A.col(3).array() * A.col(1).array());
            Vector3d vmax_vec = (A.col(1).array() - 4 * A.col(2).array() * A.col(2).array() / (12 * A.col(3).array()));
            for (int m = 0; m < 3; m++)
            {
                if (b4ac(m) < 0)
                {
                    vmax_vec(m) = 0;
                }
            }
            vmax = vmax_vec.cwiseAbs().maxCoeff();
            vmax = max(vmax, (v_gap + vel0).cwiseAbs().maxCoeff());
            if (jmax < max_jerk && amax < max_acc) //&& vmax < max_vel
            {
                // duration_l = duration;
                break;
            }
            // else
            // {
            //     duration_s = duration;
            // }
            cout << "vmax: " << vmax << " amax: " << amax << " jmax: " << jmax << " dur: " << duration << endl;
        }
        cout << "vmax: " << vmax << " amax: " << amax << " jmax: " << jmax << " dur: " << duration << endl;
        // duration = (duration_l + duration_s) / 2;
        A.col(3) = ((endpos - A.col(0)) - A.col(1) * duration - A.col(2) * pow(duration, 2)) / pow(duration, 3);
        end_v = getVel(duration);
        end_a = getAcc(duration);
    }

    void Generate_MinTEndPV(const Vector3d &pos0, const Vector3d &vel0, const Vector3d &acc0, const Vector3d &endpos, const Vector3d &endvel,
                            const float max_vel, const float max_acc, const float max_jerk, const float min_t)
    {
        A.col(4).setZero();
        start_p = pos0;
        start_v = vel0;
        start_a = acc0;
        end_p = endpos;
        end_v = endvel;

        A.col(0) = pos0;
        A.col(1) = vel0;
        A.col(2) = 0.5 * acc0;

        duration = (endpos - pos0).norm() / max_vel * 2;

        Matrix<double, 3, 2> res, AB;
        Matrix<double, 2, 2> T;

        T << pow(duration, 3), 3 * pow(duration, 2), pow(duration, 4), 4 * pow(duration, 3);
        AB.col(0) = endpos - A.col(0) - A.col(1) * duration - A.col(2) * pow(duration, 2);
        AB.col(1) = endvel - A.col(1) - 2 * A.col(2) * duration;
        res = AB * T.inverse();
        A.col(3) = res.col(0);
        A.col(4) = res.col(1);
        end_a = getAcc(duration);

        cout << "A: \n"
             << A.format(CleanFmt) << endl;
        cout << "seg end vel:\n"
             << getVel(duration).transpose() << "\n"
             << endvel.transpose() << endl;
    }
};

struct Trajectory
{
    vector<Segment> segs;
    double amax, vmax, jmax;
    Vector3d init_pos, init_vel, init_acc;
    Vector3d end_pos, end_vel, end_acc;
    double duration = 0;
    int seg_num = 0;
    Vector3d getAcc(const double t)
    {
        int idx;
        double seg_t;
        getSegT(t, idx, seg_t);
        return segs[idx].getAcc(seg_t);
    }
    Vector3d getVel(const double t)
    {
        int idx;
        double seg_t;
        getSegT(t, idx, seg_t);
        return segs[idx].getVel(seg_t);
    }
    Vector3d getPos(const double t)
    {
        int idx;
        double seg_t;
        getSegT(t, idx, seg_t);
        return segs[idx].getPos(seg_t);
    }

    Matrix3d getPVA(const double t)
    {
        Matrix3d res;
        int idx;
        double seg_t;
        getSegT(t, idx, seg_t);
        // cout<<"get seg_id and t: "<<idx<<" "<<seg_t<<" t on traj: "<<t<<endl;
        res.col(0) = segs[idx].getPos(seg_t);
        res.col(1) = segs[idx].getVel(seg_t);
        res.col(2) = segs[idx].getAcc(seg_t);
        return res;
    }

    void getSegT(double t, int &idx, double &seg_t)
    {
        t = min(t, duration);
        seg_t = t;
        idx = 0;
        while (1)
        {
            if (seg_t - segs[idx].duration < 0 || idx >= seg_num)
                break;
            seg_t -= segs[idx].duration;
            idx++;
        }
    }
    void setKDLimit(const float max_vel, const float max_acc, const float max_jerk)
    {
        amax = max_acc;
        vmax = max_vel;
        jmax = max_jerk;
    }

    void setTrajInitState(const Vector3d &pos, const Vector3d &vel, const Vector3d &acc)
    {
        init_pos = pos;
        init_vel = vel;
        init_acc = acc;
        end_pos = pos;
        end_vel = vel;
        end_acc = acc;
        // cout<<"traj end pos: "<<end_pos.transpose()<<endl;
    }

    void pushSegEndV(const Vector3d &seg_endvel, const double min_t = 0.02)
    {
        Segment seg;

        seg.Generate_MinTEndV(end_pos, end_vel, end_acc, seg_endvel, amax, jmax, min_t);
        segs.push_back(seg);
        end_pos = seg.end_p;
        end_vel = seg.end_v;
        end_acc = seg.end_a;
        duration += seg.duration;
        seg_num++;
    }

    void pushSegEndP(const Vector3d &seg_endpos, const double min_t = 0.02)
    {
        Segment seg;
        seg.Generate_MinTEndP(end_pos, end_vel, end_acc, seg_endpos, vmax, amax, jmax, min_t);
        segs.push_back(seg);
        end_pos = seg.end_p;
        end_vel = seg.end_v;
        end_acc = seg.end_a;
        duration += seg.duration;
        seg_num++;
    }

    void pushSegEndPV(const Vector3d &seg_endpos, const Vector3d &seg_endvel, const double min_t = 0.02)
    {
        Segment seg;
        seg.Generate_MinTEndPV(end_pos, end_vel, end_acc, seg_endpos, seg_endvel, vmax, amax, jmax, min_t);
        segs.push_back(seg);
        end_pos = seg.end_p;
        end_vel = seg.end_v;
        end_acc = seg.end_a;
        duration += seg.duration;
        seg_num++;
    }
    void clear()
    {
        segs.clear();
        duration = 0;
        seg_num = 0;
    }
};

class VelPlanner
{
private:
    Matrix<double, 3, 5> camera_vertex_b;
    ros::NodeHandle nh_;

    double fx = 300, fy = 300, cx = 320, cy = 180;
    // double yaw_heading_t = 0.5;

    inline double cal_yaw_with_2dVec(Vector2d p)
    {
        Vector2d v1(1.0, 0.0);
        double psi = acos(v1.dot(p) / (p.norm()));
        if (p(1) < 0)
        {
            psi = -psi;
        }
        return psi;
    }

public:
    //         MatrixXd waypoints;    // pva * time
    VelPlanner(Matrix<double, 3, 5> camera_vertex_bb)
    {
        camera_vertex_b = camera_vertex_bb;
        Rota_bc << 0, 0, 1,
            -1, 0, 0,
            0, -1, 0; // confirm this！
    }
    States drone_state;
    Config config;
    vec_Vec3f *obs_pointer;
    dynobs_tmp *dynobs_pointer;
    vector<MatrixXd> pyramids;
    // vec_E<Polyhedron3D> decompPolys;

    bool last_dyn_check_fail = false;
    double Max_traj_vel;
    Vector3d last_check_pos, dyn_check_fail_pos;
    Matrix3d Rota_bc;
    bool has_traj = false;
    Matrix<double, 3, 4> pyramid_base;
    // bool static_safe = true; dynamic_safe = true;
    ros::Time last_check_time;
    Trajectory traj;
    // bool check_traj_safe(const MatrixXd &cd_c, const VectorXd &cd_r, const double start_t);
    bool inGlobalBound(const Vector3d &pos)
    {
        return (pos(0) > config.global_bound(0, 0) &&
                pos(0) < config.global_bound(0, 1) &&
                pos(1) > config.global_bound(1, 0) &&
                pos(1) < config.global_bound(1, 1) &&
                pos(2) > config.global_bound(2, 0) &&
                pos(2) < config.global_bound(2, 1));
    }
    Vector3d World2Image(const Vector3d &pos) // pos is in world frame
    {
        Vector3d pos_cam = Rota_bc * drone_state.Rota_EB * (pos - drone_state.P_E);
        double depth = pos_cam.norm();
        return Vector3d(pos_cam(0) * fx / depth, pos_cam(1) * fy / depth, depth);
    }

    Vector3d Image2World(const Vector3d &img_coord)
    {
        return drone_state.Rota * (Rota_bc.transpose() * Vector3d(img_coord(2) * img_coord(0) / fx, img_coord(2) * img_coord(1) / fy, img_coord(2))) + drone_state.P_E;
    }
    void getPyramids(const Vector3d &ct_pos, dynobs_tmp *dynobs)
    {
        pyramids.clear();
        for (uint i = 0; i < dynobs->centers.size(); i++)
        {
            Matrix<double, 3, 5> pyramid;
            pyramid.col(0) = ct_pos;
            Vector3d box_min_inflate = (dynobs->centers[i] - dynobs->obs_sizes[i] * 0.5).array() - config.safeMargin;
            Vector3d box_max_inflate = (dynobs->centers[i] + dynobs->obs_sizes[i] * 0.5).array() + config.safeMargin;
            vector<Vector3d> eightVs = {box_min_inflate,
                                        Vector3d(box_min_inflate(0), box_min_inflate(1), box_max_inflate(2)),
                                        Vector3d(box_min_inflate(0), box_max_inflate(1), box_min_inflate(2)),
                                        Vector3d(box_max_inflate(0), box_min_inflate(1), box_min_inflate(2)),
                                        Vector3d(box_max_inflate(0), box_max_inflate(1), box_min_inflate(2)),
                                        Vector3d(box_max_inflate(0), box_min_inflate(1), box_max_inflate(2)),
                                        Vector3d(box_min_inflate(0), box_max_inflate(1), box_max_inflate(2)), box_max_inflate};
            // vector<Vector3d> eightVs_img;
            for (auto &vt : eightVs)
            {
                vt = World2Image(vt);
            }
            Rect obb = computeOBB(eightVs);
            uint j = 0;
            for (auto p : obb.P)
            {
                pyramid.col(++j) = Image2World(Vector3d(p(0), p(1), eightVs[0](2)));
            }
            pyramids.emplace_back(pyramid);
        }
    }
    void gen_traj(const Vector3d &pos, const Vector3d &vel, const Vector3d &acc, const Vector3d &end_pos, const Vector3d &plan_vel)
    {
        // cout<<"traj init pos: "<<pos.transpose()<<endl;
        traj.clear();
        traj.setKDLimit(config.velMax, config.horizontalAccMax, config.jerMax);
        traj.setTrajInitState(pos, vel, acc);
        if ((end_pos - pos).norm() > 1.0)
        {
            traj.pushSegEndV(plan_vel);
            traj.pushSegEndV((end_pos - traj.end_pos).normalized() * config.velMax); // transition seg. it is hard to meet all kino-constraints if plan a long 3-rd order poly traj seg.
        }
        traj.pushSegEndPV(end_pos, Vector3d::Zero());
        this->has_traj = true;
        cout << "Traj mid vel: " << plan_vel.transpose() << " and end_pos: " << end_pos.transpose() << endl;
        cout << "Traj duration: " << traj.segs[0].duration << " " << traj.segs[1].duration << " " << traj.segs[2].duration << endl;
    }

    Vector3d plan_vel(const Vector3d &ct_pos, const Vector3d &ct_vel, dynobs_tmp *dynobs)
    {

        getPyramids(ct_pos, dynobs);
        vector<Matrix<double, 3, 4>> vel_candidates;
        map<int, Vector3d> vel_scores; // arange the elements by increasing key value
        for (uint i = 0; i < pyramids.size(); i++)
        {
            if (isRayInsec3DBox(ct_pos, ct_pos + ct_vel - dynobs->vels[i], dynobs->centers[i] - dynobs->obs_sizes[i] * 0.5, dynobs->centers[i] + dynobs->obs_sizes[i] * 0.5))
            {
                vel_candidates.emplace_back(getFootOnPyramid(pyramids[i], ct_pos + ct_vel - dynobs->vels[i]).colwise() - (ct_pos - dynobs->vels[i]));
                // get the actual velocity, not relative vel.
            }
        }

        for (auto vel4 : vel_candidates)
        {
            for (uint c = 0; c < 4; c++)
            {
                vel_scores[cal_vel_score(ct_pos, ct_vel, vel4.col(c), dynobs)] = vel4.col(c);
            }
        }

        if (vel_scores.rbegin()->first == 0)
        {
            // try random velocities if no velocity candidates valuable
            ROS_INFO_STREAM("\033[32m Try random vels, candidates from pyramids are all invalid! \033[0m");
            for (uint m = 0; m < 100; m++)
            {
                Vector3d rand_vel = MatrixXd::Random(3, 1) * config.velMax;
                int score = cal_vel_score(ct_pos, ct_vel, rand_vel, dynobs);
                if (score)
                    vel_scores[score] = rand_vel;
            }
        }
        Vector3d res = vel_scores.rbegin()->second;
        cout << "choosed vel: " << res.transpose() << " score: " << vel_scores.rbegin()->first << endl;
        cout << "worst vel: " << vel_scores.begin()->second.transpose() << " score: " << vel_scores.begin()->first << endl;
        return res; // the velocity with the highest score
    }

    int cal_vel_score(const Vector3d &ct_pos, const Vector3d &ct_vel, const Vector3d &vel, const dynobs_tmp *dynobs)
    {
        int score = 0;
        // vel(2) / vel.head(2).norm() < -0.5 ||
        if ((vel.cwiseAbs().array() > config.velMax).any() || (ct_pos(2) <= config.global_bound(2, 0) && vel(2) < 0) || (ct_pos(2) >= config.global_bound(2, 1) && vel(2) > 0))
            return score;
        for (uint i = 0; i < dynobs->centers.size(); i++)
        {
            if ((dynobs->centers[i] - ct_pos).norm() < config.dyn_dis_forcheck &&
                !isRayInsec3DBox(ct_pos, ct_pos + vel - dynobs->vels[i], dynobs->centers[i] - dynobs->obs_sizes[i] * 0.5,
                                 dynobs->centers[i] + dynobs->obs_sizes[i] * 0.5))
            {
                score += static_cast<int>(1000 * config.weight_dis * 1 / pow((dynobs->centers[i] - ct_pos).norm(), 2));
            }
        }
        score += static_cast<int>(1000 * (2 * config.velMax - (vel - ct_vel).norm()));
        return score;
    }

    bool check_vel_safe(const Vector3d &ct_pos, const Vector3d &vel, dynobs_tmp *dynobs)
    {
        for (uint i = 0; i < dynobs->centers.size(); i++)
        {
            if ((ct_pos - dynobs->centers[i]).norm() < config.dyn_dis_forcheck && isRayInsec3DBox(ct_pos, ct_pos + vel - dynobs->vels[i], dynobs->centers[i] - dynobs->obs_sizes[i] * 0.5,
                                                                                                  dynobs->centers[i] + dynobs->obs_sizes[i] * 0.5))
                return false;
        }
        return true;
    }

    // check the vehicle safety by calculating relative velocity towards each obstacle
    void get_desire(double timee, Vector3d &p_d, Vector3d &v_d, Vector3d &a_d, double &yaw_d)
    {
        Matrix3d PVA = traj.getPVA(timee);
        p_d = PVA.col(0);
        v_d = PVA.col(1);
        a_d = PVA.col(2);
        yaw_d = cal_yaw_with_2dVec((traj.end_pos - drone_state.P_E).head(2));
    }

    void get_traj_samples(MatrixXd &sp_pos, MatrixXd &sp_vel, MatrixXd &sp_acc, double start_t)
    {
        double delta_t = 0.05;
        int num = (traj.duration - start_t) / delta_t; // future 2 secs
        num = max(num + 1, 1);
        sp_pos.resize(num, 3);
        sp_vel.resize(num, 3);
        sp_acc.resize(num, 3);
        for (int i = 0; i < num - 1; i++)
        {
            Matrix3d PVA = traj.getPVA(start_t + i * delta_t);
            sp_pos.row(i) = PVA.col(0).transpose();
            sp_vel.row(i) = PVA.col(1).transpose();
            sp_acc.row(i) = PVA.col(2).transpose();
        }

        sp_pos.row(num - 1) = traj.end_pos.transpose();
        sp_vel.row(num - 1) = traj.end_vel.transpose();
        sp_acc.row(num - 1) = traj.end_acc.transpose();
    }
};

#endif