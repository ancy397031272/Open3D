//
// Created by rvbust on 12/21/22.
//

#include "RansacModel.h"

namespace open3d {
namespace geometry {
void VectorToEigenMatrix(const std::vector<Eigen::Vector3d> &pc,
                         Eigen::Matrix3Xd &new_pc) {
    const size_t num = pc.size();
    new_pc.setZero(3, num);

#pragma omp parallel
    for (size_t i = 0; i < num; i++) {
        new_pc.col(i) = pc[i];
    }
}

double CalcPoint2LineDistance(const Eigen::Vector3d &query,
                              const Eigen::Vector3d &point1,
                              const Eigen::Vector3d &point2) {
    const Eigen::Vector3d a = query - point1;
    const Eigen::Vector3d b = query - point2;
    const Eigen::Vector3d c = point2 - point1;

    return a.cross(b).norm() / c.norm();
}

bool PlaneEstimator::MinimalFit(const open3d::geometry::PointCloud &pc,
                                Model &model) const {
    if (!MinimalCheck(pc.points_.size())) {
        return false;
    }

    const auto &points = pc.points_;

    const Eigen::Vector3d e0 = points[1] - points[0];
    const Eigen::Vector3d e1 = points[2] - points[0];
    Eigen::Vector3d abc = e0.cross(e1);
    const double norm = abc.norm();
    // if the three points are co-linear, return invalid plane
    if (norm < EPS) {
        return false;
    }
    abc /= abc.norm();
    const double d = -abc.dot(points[0]);
    model.parameters_(0) = abc(0);
    model.parameters_(1) = abc(1);
    model.parameters_(2) = abc(2);
    model.parameters_(3) = d;

    return true;
}

bool PlaneEstimator::GeneralFit(const open3d::geometry::PointCloud &pc,
                                Model &model) const {
    /// https://www.ilikebigbits.com/2015_03_04_plane_from_points.html
    const size_t num = pc.points_.size();
    if (!MinimalCheck(num)) {
        return false;
    }

    const auto &points = pc.points_;
    Eigen::Vector3d mean(0, 0, 0);
    for (auto &p : points) {
        mean += p;
    }
    mean /= double(num);

    double xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
#pragma omp parallel for reduction(+ : xx, xy, xz, yy, yz, zz)
    for (size_t i = 0; i < num; ++i) {
        const Eigen::Vector3d residual = points[i] - mean;
        xx += residual(0) * residual(0);
        xy += residual(0) * residual(1);
        xz += residual(0) * residual(2);
        yy += residual(1) * residual(1);
        yz += residual(1) * residual(2);
        zz += residual(2) * residual(2);
    }

    const double det_x = yy * zz - yz * yz;
    const double det_y = xx * zz - xz * xz;
    const double det_z = xx * yy - xy * xy;

    Eigen::Vector3d abc;
    if (det_x > det_y && det_x > det_z) {
        abc = Eigen::Vector3d(det_x, xz * yz - xy * zz, xy * yz - xz * yy);
    } else if (det_y > det_z) {
        abc = Eigen::Vector3d(xz * yz - xy * zz, det_y, xy * xz - yz * xx);
    } else {
        abc = Eigen::Vector3d(xy * yz - xz * yy, xy * xz - yz * xx, det_z);
    }

    const double norm = abc.norm();
    if (norm < EPS) {
        return false;
    }

    abc /= norm;
    const double d = -abc.dot(mean);
    model.parameters_ = Eigen::Vector4d(abc(0), abc(1), abc(2), d);

    return true;
}

double PlaneEstimator::CalcPointToModelDistance(const Eigen::Vector3d &query,
                                                const Model &model) const {
    const Eigen::Vector4d p(query(0), query(1), query(2), 1);
    return std::abs(model.parameters_.transpose() * p) /
           model.parameters_.head<3>().norm();
}

bool SphereEstimator::ValidationCheck(
        const open3d::geometry::PointCloud &pc) const {
    PlaneEstimator fit;
    Plane plane;
    const bool ret = fit.MinimalFit(pc, plane);
    if (!ret) {
        return false;
    }
    return fit.CalcPointToModelDistance(pc.points_[3], plane) >= EPS;
}
bool SphereEstimator::MinimalFit(const open3d::geometry::PointCloud &pc,
                                 Model &model) const {
    const auto &points = pc.points_;
    if (!MinimalCheck(points.size()) || !ValidationCheck(pc)) {
        return false;
    }

    Eigen::Matrix4d det_mat;
    det_mat.setOnes(4, 4);
    for (size_t i = 0; i < 4; i++) {
        det_mat(i, 0) = points[i](0);
        det_mat(i, 1) = points[i](1);
        det_mat(i, 2) = points[i](2);
    }
    const double M11 = det_mat.determinant();

    for (size_t i = 0; i < 4; i++) {
        det_mat(i, 0) = points[i].transpose() * points[i];
        det_mat(i, 1) = points[i](1);
        det_mat(i, 2) = points[i](2);
    }
    const double M12 = det_mat.determinant();

    for (size_t i = 0; i < 4; i++) {
        det_mat(i, 0) = points[i].transpose() * points[i];
        det_mat(i, 1) = points[i](0);
        det_mat(i, 2) = points[i](2);
    }
    const double M13 = det_mat.determinant();

    for (size_t i = 0; i < 4; i++) {
        det_mat(i, 0) = points[i].transpose() * points[i];
        det_mat(i, 1) = points[i](0);
        det_mat(i, 2) = points[i](1);
    }
    const double M14 = det_mat.determinant();

    for (size_t i = 0; i < 4; i++) {
        det_mat(i, 0) = points[i].transpose() * points[i];
        det_mat(i, 1) = points[i](0);
        det_mat(i, 2) = points[i](1);
        det_mat(i, 3) = points[i](2);
    }
    const double M15 = det_mat.determinant();

    const Eigen::Vector3d center(0.5 * (M12 / M11), -0.5 * (M13 / M11),
                                 0.5 * (M14 / M11));
    const double radius = std::sqrt(center.transpose() * center - (M15 / M11));
    model.parameters_(0) = center(0);
    model.parameters_(1) = center(1);
    model.parameters_(2) = center(2);
    model.parameters_(3) = radius;

    return true;
}

bool SphereEstimator::GeneralFit(const open3d::geometry::PointCloud &pc,
                                 Model &model) const {
    const size_t num = pc.points_.size();
    if (!MinimalCheck(num)) {
        return false;
    }

    const auto &o3d_points = pc.points_;
    Eigen::Matrix3Xd points;
    VectorToEigenMatrix(o3d_points, points);

    Eigen::Matrix<double, Eigen::Dynamic, 4> A;
    A.setOnes(num, 4);
    A.col(0) = points.row(0).transpose() * 2;
    A.col(1) = points.row(1).transpose() * 2;
    A.col(2) = points.row(2).transpose() * 2;

    Eigen::VectorXd b =
            (points.row(0).array().pow(2) + points.row(1).array().pow(2) +
             points.row(2).array().pow(2))
                    .matrix();

    // TODO: dangerous when b is very large, which need large memory to
    // compute v. should be improved.
    const Eigen::Vector4d w =
            A.bdcSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(b);
    const double radius = sqrt(w(0) * w(0) + w(1) * w(1) + w(2) * w(2) + w(3));
    model.parameters_(0) = w(0);
    model.parameters_(1) = w(1);
    model.parameters_(2) = w(2);
    model.parameters_(3) = radius;

    return true;
}

double SphereEstimator::CalcPointToModelDistance(const Eigen::Vector3d &query,
                                                 const Model &model) const {
    const Eigen::Vector3d center = model.parameters_.head<3>();
    const double radius = model.parameters_(3);
    const double d = (query - center).norm();

    if (d <= radius) {
        return radius - d;
    } else {
        return d - radius;
    }
}

bool CylinderEstimator::MinimalFit(const open3d::geometry::PointCloud &pc,
                                   Model &model) const {
    if (!pc.HasNormals()) {
        open3d::utility::LogError("Cylinder estimation requires normals.");
    }

    if (!MinimalCheck(pc.points_.size())) {
        return false;
    }

    const auto &points = pc.points_;
    const auto &normals = pc.normals_;
    if (fabs(points[0](0) - points[1](0) <=
                     std::numeric_limits<double>::epsilon() &&
             fabs(points[0](1) - points[1](1)) <=
                     std::numeric_limits<float>::epsilon() &&
             fabs(points[0](2) - points[1](2)) <=
                     std::numeric_limits<float>::epsilon())) {
        return false;
    }

    const Eigen::Vector4d p1(points[0](0), points[0](1), points[0](2), 0);
    const Eigen::Vector4d p2(points[1](0), points[1](1), points[1](2), 0);

    const Eigen::Vector4d n1(normals[0](0), normals[0](1), normals[0](2), 0);
    const Eigen::Vector4d n2(normals[1](0), normals[1](1), normals[1](2), 0);
    const Eigen::Vector4d w = n1 + p1 - p2;

    const double a = n1.dot(n1);
    const double b = n1.dot(n2);
    const double c = n2.dot(n2);
    const double d = n1.dot(w);
    const double e = n2.dot(w);
    const double denominator = a * c - b * b;
    double sc, tc;

    if (denominator < 1e-8)  // The lines are almost parallel
    {
        sc = 0;
        tc = (b > c ? d / b : e / c);  // Use the largest denominator
    } else {
        sc = (b * e - c * d) / denominator;
        tc = (a * e - b * d) / denominator;
    }

    const Eigen::Vector4d line_pt = p1 + n1 + sc * n1;
    Eigen::Vector4d line_dir = p2 + tc * n2 - line_pt;
    line_dir.normalize();

    model.parameters_[0] = line_pt[0];
    model.parameters_[1] = line_pt[1];
    model.parameters_[2] = line_pt[2];
    model.parameters_[3] = line_dir[0];
    model.parameters_[4] = line_dir[1];
    model.parameters_[5] = line_dir[2];
    // cylinder radius
    model.parameters_[6] = CalcPoint2LineDistance(points[0], line_pt.head<3>(),
                                                  line_dir.head<3>());

    return true;
}  // namespace ransac

bool CylinderEstimator::GeneralFit(const open3d::geometry::PointCloud &pc,
                                   Model &model) const {
    if (!pc.HasNormals()) {
        open3d::utility::LogError("Cylinder estimation requires normals.");
    }
    if (!MinimalCheck(pc.points_.size())) {
        return false;
    }
    // todo Not implemented
    return false;
}

double CylinderEstimator::CalcPointToModelDistance(const Eigen::Vector3d &query,
                                                   const Model &model) const {
    const Eigen::Matrix<double, 7, 1> w = model.parameters_;
    const Eigen::Vector3d n(w(3), w(4), w(5));
    const Eigen::Vector3d center(w(0), w(1), w(2));

    const Eigen::Vector3d ref(w(0) + w(3), w(1) + w(4), w(2) + w(5));
    double d = CalcPoint2LineDistance(query, center, ref);

    return abs(d - w(6));
}

Eigen::Matrix4d CalcAxisTransform(const Eigen::Vector3d &x_head,
                                  const Eigen::Vector3d &origin,
                                  const Eigen::Vector3d &ref) {
    const Eigen::Vector3d x_axis = (x_head - origin) / (x_head - origin).norm();
    const Eigen::Vector3d tmp_axis = (ref - origin) / (ref - origin).norm();

    Eigen::Vector3d z_axis = x_axis.cross(tmp_axis);
    if (z_axis.dot(Eigen::Vector3d(0, 0, 1)) > 0) {
        z_axis /= z_axis.norm();
    } else {
        z_axis /= -z_axis.norm();
    }

    Eigen::Vector3d y_axis = z_axis.cross(x_axis);
    y_axis /= y_axis.norm();

    Eigen::Matrix4d transform;
    transform << x_axis(0), y_axis(0), z_axis(0), origin(0), x_axis(1),
            y_axis(1), z_axis(1), origin(1), x_axis(2), y_axis(2), z_axis(2),
            origin(2), 0, 0, 0, 1;

    return transform;
}

bool Circle3DEstimator::MinimalFit(const open3d::geometry::PointCloud &pc,
                                   Model &model) const {
    if (!MinimalCheck(pc.points_.size())) {
        return false;
    }

    const auto point1 = pc.points_[0];
    const auto point2 = pc.points_[1];
    const auto point3 = pc.points_[2];

    const Eigen::Matrix4d transform = CalcAxisTransform(point1, point2, point3);
    const Eigen::Matrix4d transform_inv = transform.inverse();

    const Eigen::Vector3d p1 = (transform_inv.block(0, 0, 3, 3) * point1 +
                                transform_inv.block(0, 3, 3, 1))
                                       .transpose();
    const Eigen::Vector3d p2 = (transform_inv.block(0, 0, 3, 3) * point2 +
                                transform_inv.block(0, 3, 3, 1))
                                       .transpose();
    const Eigen::Vector3d p3 = (transform_inv.block(0, 0, 3, 3) * point3 +
                                transform_inv.block(0, 3, 3, 1))
                                       .transpose();

    // Colinear check
    Eigen::Matrix3d half_determinant;
    half_determinant << p1(0), p2(0), p3(0), p1(1), p2(1), p3(1), 1, 1, 1;

    const double flag = half_determinant.determinant();

    if (flag == 0) {
        open3d::utility::LogWarning("The three points are colinear");
        return false;
    } else {
        Eigen::Matrix3d a;
        Eigen::Matrix3d b;
        Eigen::Matrix3d c;
        Eigen::Matrix3d d;

        a << p1(0), p1(1), 1, p2(0), p2(1), 1, p3(0), p3(1), 1;

        b << p1(0) * p1(0) + p1(1) * p1(1), p1(1), 1,
                p2(0) * p2(0) + p2(1) * p2(1), p2(1), 1,
                p3(0) * p3(0) + p3(1) * p3(1), p3(1), 1;

        c << p1(0) * p1(0) + p1(1) * p1(1), p1(0), 1,
                p2(0) * p2(0) + p2(1) * p2(1), p2(0), 1,
                p3(0) * p3(0) + p3(1) * p3(1), p3(0), 1;

        d << p1(0) * p1(0) + p1(1) * p1(1), p1(0), p1(1),
                p2(0) * p2(0) + p2(1) * p2(1), p2(0), p2(1),
                p3(0) * p3(0) + p3(1) * p3(1), p3(0), p3(1);

        const double A = a.determinant();
        const double B = -b.determinant();
        const double C = c.determinant();
        const double D = -d.determinant();

        double x = -B / (2 * A);
        double y = -C / (2 * A);
        double z = 0;
        const double r = sqrt((B * B + C * C - 4 * A * D) / (4 * A * A));

        Eigen::Vector3d point_plane(x, y, z);
        point_plane = (transform.block(0, 0, 3, 3) * point_plane +
                       transform.block(0, 3, 3, 1))
                              .transpose();

        const double nx = transform(0, 2);
        const double ny = transform(1, 2);
        const double nz = transform(2, 2);

        model.parameters_ << point_plane(0), point_plane(1), point_plane(2), nx,
                ny, nz, r;
    }

    return true;
}
//

Eigen::MatrixX3d Transform(const Eigen::MatrixX3d &points,
                           const Eigen::Matrix4d &transform) {
    const int size = points.rows();
    Eigen::MatrixX3d dst;
    dst.setZero(size, 3);

#pragma omp parallel for
    for (int i = 0; i < size; i++) {
        Eigen::RowVector3d point = points.row(i);
        Eigen::Vector4d new_point =
                transform * Eigen::Vector4d(point(0), point(1), point(2), 1.0);
        dst.row(i) = (new_point.head<3>() / new_point(3)).transpose();
    }

    return dst;
}

bool Circle3DEstimator::GeneralFit(const open3d::geometry::PointCloud &pc,
                                   Model &model) const {
    const size_t num = pc.points_.size();
    if (!MinimalCheck(num)) {
        return false;
    }

    PlaneEstimator plane_estimator;
    Plane plane;
    plane_estimator.GeneralFit(pc, plane);
    const Eigen::VectorXd w = plane.parameters_;

    Eigen::Matrix3Xd points;
    VectorToEigenMatrix(pc.points_, points);

    Eigen::VectorXd d;
    d.setZero(num);
    d.setConstant(w(3));

    Eigen::VectorXd pts_x = points.row(0);
    Eigen::VectorXd pts_y = points.row(1);
    Eigen::VectorXd pts_z = points.row(2);

    Eigen::VectorXd p_x = ((w(1) * w(1) + w(2) * w(2)) * pts_x -
                           w(0) * (w(1) * pts_y + w(2) * pts_z + d)) /
                          (w(0) * w(0) + w(1) * w(1) + w(2) * w(2));
    Eigen::VectorXd p_y = ((w(0) * w(0) + w(2) * w(2)) * pts_y -
                           w(1) * (w(0) * pts_x + w(2) * pts_z + d)) /
                          (w(0) * w(0) + w(1) * w(1) + w(2) * w(2));
    Eigen::VectorXd p_z = ((w(0) * w(0) + w(1) * w(1)) * pts_z -
                           w(2) * (w(0) * pts_x + w(1) * pts_y + d)) /
                          (w(0) * w(0) + w(1) * w(1) + w(2) * w(2));

    Eigen::MatrixX3d plane_pts;
    plane_pts.setZero(num, 3);
    plane_pts.col(0) = p_x;
    plane_pts.col(1) = p_y;
    plane_pts.col(2) = p_z;

    // Build a coordinate system on the plane and get transformation to point
    // cloud coordinate system
    Eigen::RowVector3d p1 = plane_pts.row(0);
    Eigen::RowVector3d p2 = plane_pts.row(num - 1);
    Eigen::RowVector3d p3 = plane_pts.row(1);

    Eigen::Matrix4d transform = CalcAxisTransform(p1, p2, p3);

    Eigen::Matrix4d transform_inv = transform.inverse();

    plane_pts = Transform(plane_pts, transform_inv);

    Eigen::MatrixX3d A;
    A.setZero(num, 3);
    A.setOnes();
    A.col(0) = plane_pts.col(0);
    A.col(1) = plane_pts.col(1);

    Eigen::VectorXd plane_pts_x = plane_pts.col(0);
    Eigen::VectorXd plane_pts_y = plane_pts.col(1);

    Eigen::VectorXd b = plane_pts_x.array().pow(2) + plane_pts_y.array().pow(2);

    Eigen::Vector3d cw =
            A.bdcSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(b);

    double cx = cw(0) / 2;
    double cy = cw(1) / 2;
    double r = sqrt(cw(2) + cx * cx + cy * cy);

    Eigen::RowVector3d circle_center(cx, cy, 0);
    circle_center = Transform(circle_center, transform);

    model.parameters_(0) = circle_center(0);
    model.parameters_(1) = circle_center(1);
    model.parameters_(2) = circle_center(2);
    model.parameters_(3) = w(0);
    model.parameters_(4) = w(1);
    model.parameters_(5) = w(2);
    model.parameters_(6) = r;

    return true;
}

double Circle3DEstimator::CalcPointToModelDistance(const Eigen::Vector3d &query,
                                                   const Model &model) const {
    const Eigen::Vector3d n(model.parameters_(3), model.parameters_(4),
                            model.parameters_(5));
    const Eigen::Vector3d center(model.parameters_(0), model.parameters_(1),
                                 model.parameters_(2));
    const double radius = model.parameters_(6);
    double distance;

    const Eigen::Vector3d ref = query - center;
    const double l = ref.norm();
    const Eigen::Vector3d ref_ = n.cross(ref);

    if (l == 0) {
        distance = radius;
    } else if (ref_.norm() != 0) {
        Eigen::Vector3d d = ref_.cross(n);
        d = d / d.norm();
        const Eigen::Vector3d p_in_circle = center + d * radius;

        distance = (query - p_in_circle).norm();
    } else {
        distance = sqrt(l * l + radius * radius);
    }

    return distance;
}



}  // namespace geometry
}  // namespace open3d