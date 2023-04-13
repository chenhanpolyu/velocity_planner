#ifndef TOOLS_HPP
#define TOOLS_HPP
#include <iostream>
#include <math.h>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <numeric>
using namespace std;
using namespace Eigen;

namespace Tools
{
struct States
{
    // raw states
    Vector3d A_Eraw, Rate_Braw;
    Vector3d Rate_Eraw; //////////

    // for ctrl
    Vector3d P_E, V_E, A_E;

    Matrix3d Rota, Rota_EB;
    Quaterniond Quat;
    Vector3d Euler;

    Vector3d Rate_E, Rate_B;
};


inline double clip(double x, double minv, double maxv)
{
    if (x <= minv)
    {
        return minv;
    }
    else if (x >= maxv)
    {
        return maxv;
    }
    else
    {
        return x;
    }
}

// Quaterniond normalizeQ(Quaterniond q)
// {
//     double norm = sqrt(q.x() * q.x() + q.y() * q.y() + q.z() * q.z() + q.w() * q.w());
//     q.x() = q.x() / norm;
//     q.y() = q.y() / norm;
//     q.z() = q.z() / norm;
//     q.w() = q.w() / norm;

//     return q;
// }

inline Quaterniond Euler2Quaternion(Vector3d euler)
{
    double cr = std::cos(euler(0) * 0.5);
    double sr = std::sin(euler(0) * 0.5);
    double cp = std::cos(euler(1) * 0.5);
    double sp = std::sin(euler(1) * 0.5);
    double cy = std::cos(euler(2) * 0.5);
    double sy = std::sin(euler(2) * 0.5);

    Quaterniond q;
    q.w() = cr * cp * cy + sr * sp * sy;
    q.x() = sr * cp * cy - cr * sp * sy;
    q.y() = cr * sp * cy + sr * cp * sy;
    q.z() = cr * cp * sy - sr * sp * cy;

    return q.normalized();
}

inline Quaterniond Rota2Quaternion(Matrix3d Rota)
{
    Quaterniond q;
    q.w() = std::sqrt(Rota.trace() + 1) / 2;
    q.x() = (Rota(2, 1) - Rota(1, 2)) / 4.0 / q.w();
    q.y() = (Rota(0, 2) - Rota(2, 0)) / 4.0 / q.w();
    q.z() = (Rota(1, 0) - Rota(0, 1)) / 4.0 / q.w();

    return q.normalized();
}

// convert quaternion to euler angles (with theta singularity)
inline Vector3d Quaternion2Euler(Quaterniond q)
{
    Vector3d euler;

    // roll
    euler(0) = std::atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y() * q.y()));

    // pitch
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        euler(1) = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        euler(1) = std::asin(sinp);

    // yaw
    euler(2) = std::atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));

    return euler;
}

// convert quaternion to rotation matrix
inline Matrix3d Quaternion2Rota(Quaterniond q)
{
    Matrix3d rota;
    double r, i, j, k;
    r = q.w();
    i = q.x();
    j = q.y();
    k = q.z();

    // convert to rota
    rota(0, 0) = 1 - 2 * (j * j + k * k);
    rota(0, 1) = 2 * (i * j - k * r);
    rota(0, 2) = 2 * (i * k + j * r);
    //
    rota(1, 0) = 2 * (i * j + k * r);
    rota(1, 1) = 1 - 2 * (i * i + k * k);
    rota(1, 2) = 2 * (j * k - i * r);
    //
    rota(2, 0) = 2 * (i * k - j * r);
    rota(2, 1) = 2 * (j * k + i * r);
    rota(2, 2) = 1 - 2 * (i * i + j * j);

    return rota;
}

// convert rotation matrix to euler angles
inline Vector3d Rota2Euler(Matrix3d Rota)
{
    Quaterniond q = Rota2Quaternion(Rota);
    Vector3d euler = Quaternion2Euler(q);
    return euler;
}

inline bool isInFOV(const Matrix<double, 3, 5> &camera_vertex, const Vector3d &ct_center)
{
    return ((ct_center - camera_vertex.col(0)).dot((camera_vertex.col(0) - camera_vertex.col(1)).cross(camera_vertex.col(0) - camera_vertex.col(2))) > 0 &&
            (ct_center - camera_vertex.col(0)).dot((camera_vertex.col(0) - camera_vertex.col(2)).cross(camera_vertex.col(0) - camera_vertex.col(3))) > 0 &&
            (ct_center - camera_vertex.col(0)).dot((camera_vertex.col(0) - camera_vertex.col(3)).cross(camera_vertex.col(0) - camera_vertex.col(4))) > 0 &&
            (ct_center - camera_vertex.col(0)).dot((camera_vertex.col(0) - camera_vertex.col(4)).cross(camera_vertex.col(0) - camera_vertex.col(1))) > 0 &&
            (ct_center - camera_vertex.col(1)).dot((camera_vertex.col(1) - camera_vertex.col(4)).cross(camera_vertex.col(1) - camera_vertex.col(2))) > 0);
};

inline bool isRayInsec3DBox(const Vector3d &p1, const Vector3d &p2, const Vector3d &box_min, const Vector3d &box_max)
// p1 is the ray origin, p2 notes the direction
{
    Vector3d inv_dir = 1.0f / (p2 - p1).array();
    Vector3d tMin = (box_min - p1).array() * inv_dir.array();
    Vector3d tMax = (box_max - p1).array() * inv_dir.array();
    Vector3d t1(min(tMin[0], tMax[0]), min(tMin[1], tMax[1]), min(tMin[2], tMax[2]));
    Vector3d t2(max(tMin[0], tMax[0]), max(tMin[1], tMax[1]), max(tMin[2], tMax[2]));
    float tNear = max(max(t1[0], t1[1]), t1[2]);
    float tFar = min(min(t2[0], t2[1]), t2[2]);

    return tNear <= tFar && tFar > 0; // tNear < tFar for intersection
}

// inline bool isRayInsec3DBox(const Vector3f& ray_origin, const Vector3f& p2, const Vector3f& aabb_min, const Vector3f& aabb_max)
// {
//     // 将AABB转换为以原点为中心的单位立方体
//     Vector3f ray_direction = p2 - ray_origin;
//     Vector3f aabb_center = (aabb_max + aabb_min) / 2.0f;
//     Vector3f aabb_extent = (aabb_max - aabb_min) / 2.0f;
//     Matrix4f aabb_transform = Matrix4f::Identity();
//     aabb_transform.block<3, 3>(0, 0) = aabb_transform.block<3, 3>(0, 0).diagonal().asDiagonal() * aabb_extent.asDiagonal().inverse();
//     aabb_transform.block<3, 1>(0, 3) = -aabb_center;
    
//     // 计算射线与单位立方体之间的交点
//     Matrix4f ray_transform = Matrix4f::Identity();
//     ray_transform.block<3, 1>(0, 3) = ray_origin;
//     ray_transform.block<3, 3>(0, 0) = Quaternionf::FromTwoVectors(Vector3f::UnitZ(), ray_direction).toRotationMatrix();
//     Vector3f p = aabb_transform.inverse() * ray_transform * Vector4f::Zero();
//     Vector3f d = aabb_transform.inverse().linear() * ray_transform.linear() * Vector3f::UnitZ();
//     float tmin = -p.z() / d.z();
    
//     // 判断交点是否在单位立方体内
//     if (tmin < 0.0f)
//         return false;
//     Vector3f hit_point = p + tmin * d;
//     return (hit_point.x() >= -1.0f && hit_point.x() <= 1.0f &&
//             hit_point.y() >= -1.0f && hit_point.y() <= 1.0f &&
//             hit_point.z() >= -1.0f && hit_point.z() <= 1.0f);
// }

inline Vector3d getFootOnPlane(const Vector3d &p1, const Vector3d &p2, const Vector3d &p3, const Vector3d &p)
// p1 p2 p3 define the plane, p is outside the plane, and this function returns the foot of p on thie plane
{
    Matrix3d A;
    Vector3d B;
    Matrix3d P;
    P << p1, p2, p3;
    Vector3d ABC = -P.inverse() * MatrixXd::Constant(3, 1, 1.0);
    A.row(0) = (p2 - p1).transpose();
    A.row(1) = (p3 - p1).transpose();
    A.row(2) = ABC.transpose();
    B << p.transpose() * (p2 - p1), p.transpose() * (p3 - p1), -1;
    return A.inverse() * B;
}

inline MatrixXd getFootOnPyramid(const MatrixXd &pyramid, const Vector3d &p)
{
    Matrix<double, 3, 4> res;
    res.col(0) = getFootOnPlane(pyramid.col(0), pyramid.col(1), pyramid.col(2), p);
    res.col(1) = getFootOnPlane(pyramid.col(0), pyramid.col(2), pyramid.col(3), p);
    res.col(2) = getFootOnPlane(pyramid.col(0), pyramid.col(3), pyramid.col(4), p);
    res.col(3) = getFootOnPlane(pyramid.col(0), pyramid.col(4), pyramid.col(1), p);
    return res;
}

// 定义OBB结构体
struct Rect
{
    Vector3d center;                          // 中心点
    Vector2d axes[2];                         // 坐标轴
    double extents[2];                        // 长宽
    vector<Vector2d> P = vector<Vector2d>(4); // four vertexes
};

// 计算OBB
inline Rect computeOBB(const vector<Vector3d> &points)
{
    Rect obb;

    // 计算中心点
    Eigen::Map<const Eigen::MatrixXd> matrix(points[0].data(), 3, points.size());
    obb.center = matrix.rowwise().mean();

    // 计算协方差矩阵
    Matrix2d covariance = Matrix2d::Zero();
    for (const auto &point : points)
    {
        Vector2d deviation = point.head(2) - obb.center.head(2);
        covariance += deviation * deviation.transpose();
    }

    // 计算特征值和特征向量
    SelfAdjointEigenSolver<Matrix2d> eigenSolver(covariance);
    Vector2d eigenValues = eigenSolver.eigenvalues();
    Matrix2d eigenVectors = eigenSolver.eigenvectors();

    // 设置OBB的坐标轴和长宽
    obb.axes[0] = eigenVectors.col(0);
    obb.axes[1] = eigenVectors.col(1);
    obb.extents[0] = 2.0 * sqrt(eigenValues[0]);
    obb.extents[1] = 2.0 * sqrt(eigenValues[1]);

    obb.P[0] = obb.center.head(2) - 0.5 * obb.extents[0] * obb.axes[0] - 0.5 * obb.extents[1] * obb.axes[1];
    obb.P[1] = obb.center.head(2) + 0.5 * obb.extents[0] * obb.axes[0] - 0.5 * obb.extents[1] * obb.axes[1];
    obb.P[2] = obb.center.head(2) + 0.5 * obb.extents[0] * obb.axes[0] + 0.5 * obb.extents[1] * obb.axes[1];
    obb.P[3] = obb.center.head(2) - 0.5 * obb.extents[0] * obb.axes[0] + 0.5 * obb.extents[1] * obb.axes[1];

    return obb;
}

inline Eigen::Quaterniond calcQuatFromAccYaw(const Vector3d &acc, const Vector3d &direction)
{
    Eigen::Vector3d gravity_world(0, 0, 9.81); // 重力在世界坐标系下的表示
    Eigen::Vector3d bz_e = (acc + gravity_world).normalized();
    Eigen::Vector3d bx_e(direction(0), direction(1), -(direction(0) * bz_e(0) + direction(1)*bz_e(1))/bz_e(2));
    bx_e.normalize();
    Eigen::Vector3d by_e = bz_e.cross(bx_e);
    Matrix3d axis_e;
    axis_e.col(0) = bx_e; axis_e.col(1) = by_e; axis_e.col(2) = bz_e;
    Matrix3d rota = axis_e * Matrix3d::Identity().inverse();
    Eigen::Vector3d acc_e = rota * Eigen::Vector3d(.0, .0, (acc + gravity_world).norm());
    // cout<<"ori acc: "<<acc.transpose()<<"\nrecover acc: "<< (acc_e - gravity_world).transpose()<<endl;
    return Eigen::Quaterniond(rota).normalized();
}

}

#endif