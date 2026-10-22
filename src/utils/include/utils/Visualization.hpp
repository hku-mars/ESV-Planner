#ifndef VIS_HPP
#define VIS_HPP

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <iostream>
#include <unordered_map>
#include <pcl_conversions/pcl_conversions.h>
#include <utils/quickhull.hpp>
#include <utils/geo_utils.hpp>
#include <utils/trajectory.hpp>
#include <string>
#include <utils/se3_state.hpp>
#include <tf2/LinearMath/Quaternion.h>

#define TRAJ_ORDER 5

using namespace Eigen;
using namespace std;
namespace vis {

struct BALL {
    Eigen::Vector3d center;
    double radius;
    BALL(const Eigen::Vector3d &c, double r) : center(c), radius(r) {};
    BALL() {};
};

struct ELLIPSOID {
    Eigen::Vector3d c;
    double rx, ry, rz;
    Eigen::Matrix3d R;
    ELLIPSOID(const Eigen::Vector3d &center, const Eigen::Vector3d &r, const Eigen::Matrix3d &rot) : c(center), rx(r.x()), ry(r.y()), rz(r.z()), R(rot) {};
    ELLIPSOID() {};
};

struct Point3D {
    double x, y, z;
    bool operator<(const Point3D &other) const {
        if (x != other.x)
            return x < other.x;
        if (y != other.y)
            return y < other.y;
        return z < other.z;
    }
};

struct Edge {
    Point3D p1, p2;
    Edge(const Point3D &a, const Point3D &b) {
        p1 = a < b ? a : b;
        p2 = a < b ? b : a;
    }
    bool operator<(const Edge &other) const {
        if (!(p1 < other.p1) && !(other.p1 < p1))
            return p2 < other.p2;
        return p1 < other.p1;
    }
};

using PublisherMap = std::unordered_map<std::string, ros::Publisher>;

enum Color { white, red, green, blue, light_blue, yellow, chartreuse, black, gray, orange, purple, pink, steelblue, lightyellow, lightred, darkred };

class Visualization {
private:
    ros::NodeHandle nh_;
    PublisherMap publisher_map_;
    inline void setMarkerColor(visualization_msgs::Marker &marker, Color color = blue, double a = 1) {
        marker.color.a = a;
        switch (color) {
        case white:
            marker.color.r = 1;
            marker.color.g = 1;
            marker.color.b = 1;
            break;
        case red:
            marker.color.r = 1;
            marker.color.g = 0;
            marker.color.b = 0;
            break;
        case green:
            marker.color.r = 0;
            marker.color.g = 1;
            marker.color.b = 0;
            break;
        case blue:
            marker.color.r = 0;
            marker.color.g = 0;
            marker.color.b = 1;
            break;
        case light_blue:
            marker.color.r = 0.4;
            marker.color.g = 0.6;
            marker.color.b = 1;
            break;
        case yellow:
            marker.color.r = 1;
            marker.color.g = 1;
            marker.color.b = 0;
            break;
        case chartreuse:
            marker.color.r = 0.5;
            marker.color.g = 1;
            marker.color.b = 0;
            break;
        case black:
            marker.color.r = 0;
            marker.color.g = 0;
            marker.color.b = 0;
            break;
        case gray:
            marker.color.r = 0.5;
            marker.color.g = 0.5;
            marker.color.b = 0.5;
            break;
        case orange:
            marker.color.r = 1;
            marker.color.g = 0.5;
            marker.color.b = 0;
            break;
        case purple:
            marker.color.r = 0.5;
            marker.color.g = 0;
            marker.color.b = 1;
            break;
        case pink:
            marker.color.r = 1;
            marker.color.g = 0;
            marker.color.b = 0.6;
            break;
        case steelblue:
            marker.color.r = 0.4;
            marker.color.g = 0.7;
            marker.color.b = 1;
            break;
        case lightyellow:
            marker.color.r = 0.9490196;
            marker.color.g = 0.8862745;
            marker.color.b = 0.5664062;
            break;
        case lightred:
            marker.color.r = 0.9490196;
            marker.color.g = 0.6549019;
            marker.color.b = 0.5019607;
            break;
        case darkred:
            marker.color.r = 0.8470588;
            marker.color.g = 0.4784313;
            marker.color.b = 0.2901960;
            break;
        }
    }

    inline void setMarkerColor(visualization_msgs::Marker &marker, double a, double r, double g, double b) {
        marker.color.a = a;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
    }

    inline void setMarkerScale(visualization_msgs::Marker &marker, const double &x, const double &y, const double &z) {
        marker.scale.x = x;
        marker.scale.y = y;
        marker.scale.z = z;
    }

    inline void setMarkerPose(visualization_msgs::Marker &marker, const double &x, const double &y, const double &z) {
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation.w = 1;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
    }

    template <class ROTATION>
    inline void setMarkerPose(visualization_msgs::Marker &marker, const double &x, const double &y, const double &z, const ROTATION &R) {
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        Eigen::Quaterniond r(R);
        marker.pose.orientation.w = r.w();
        marker.pose.orientation.x = r.x();
        marker.pose.orientation.y = r.y();
        marker.pose.orientation.z = r.z();
    }

public:
    Visualization() = delete;
    Visualization(ros::NodeHandle &nh) : nh_(nh) {}
    typedef std::shared_ptr<Visualization> Ptr;

    template <class TOPIC>
    inline void pubFloat64(const TOPIC &topic, const double msg) {
        auto got = publisher_map_.find(topic);
        if (got == publisher_map_.end()) {
            ros::Publisher pub = nh_.advertise<std_msgs::Float64>(topic, 10000);
            publisher_map_[topic] = pub;
        }
        std_msgs::Float64 datamsg;
        datamsg.data = msg;
        publisher_map_[topic].publish(datamsg);
    }

    template <class CENTER, class TOPIC>
    inline void visABall(const CENTER &c, const double &r, const TOPIC &topic, const Color color = blue, const double a = 1, const bool keep = false) {
        auto got = publisher_map_.find(topic);
        if (got == publisher_map_.end()) {
            ros::Publisher pub = nh_.advertise<visualization_msgs::Marker>(topic, 10000);
            publisher_map_[topic] = pub;
        }
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        setMarkerColor(marker, color, a);
        setMarkerScale(marker, 2 * r, 2 * r, 2 * r);
        setMarkerPose(marker, c[0], c[1], c[2]);
        marker.header.stamp = ros::Time::now();
        if (keep) {
            static int i = 0;
            marker.id = i;
            i++;
        }
        publisher_map_[topic].publish(marker);
    }

    template <class CENTER, class TOPIC>
    inline void visABallWithId(const CENTER &c, const double &r, const TOPIC &topic, const Color color = blue, const double a = 1, const int id = 1) {
        auto got = publisher_map_.find(topic);
        if (got == publisher_map_.end()) {
            ros::Publisher pub = nh_.advertise<visualization_msgs::Marker>(topic, 10000);
            publisher_map_[topic] = pub;
        }
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        setMarkerColor(marker, color, a);
        setMarkerScale(marker, 2 * r, 2 * r, 2 * r);
        setMarkerPose(marker, c[0], c[1], c[2]);
        marker.header.stamp = ros::Time::now();
        marker.id = id;
        publisher_map_[topic].publish(marker);
    }

    template <class PC, class TOPIC>
    inline void visPointcloudByVector(const PC &pc, const TOPIC &topic) {
        auto got = publisher_map_.find(topic);
        if (got == publisher_map_.end()) {
            // Snapshot cloud: retain the latest result for late subscribers
            // (including Foxglove) without queuing obsolete optimization groups.
            ros::Publisher pub = nh_.advertise<sensor_msgs::PointCloud2>(topic, 1, true);
            publisher_map_[topic] = pub;
        }
        pcl::PointCloud<pcl::PointXYZ> point_cloud;
        sensor_msgs::PointCloud2 point_cloud_msg;
        point_cloud.reserve(pc.size());
        for (const auto &pt : pc) {
            point_cloud.points.emplace_back(pt[0], pt[1], pt[2]);
        }
        pcl::toROSMsg(point_cloud, point_cloud_msg);
        point_cloud_msg.header.frame_id = "map";
        point_cloud_msg.header.stamp = ros::Time::now();
        publisher_map_[topic].publish(point_cloud_msg);
    }

    template <class PC, class TOPIC>
    inline void visPointcloudXYZI(const PC &pc, const TOPIC &topic) {
        auto got = publisher_map_.find(topic);
        if (got == publisher_map_.end()) {
            ros::Publisher pub = nh_.advertise<sensor_msgs::PointCloud2>(topic, 10000);
            publisher_map_[topic] = pub;
        }
        sensor_msgs::PointCloud2 point_cloud_msg;
        pcl::toROSMsg(pc, point_cloud_msg);
        point_cloud_msg.header.frame_id = "map";
        point_cloud_msg.header.stamp = ros::Time::now();
        publisher_map_[topic].publish(point_cloud_msg);
    }

    template <class PATH, class TOPIC>
    inline void visPath(const PATH &path, const TOPIC &topic) {
        auto got = publisher_map_.find(topic);
        if (got == publisher_map_.end()) {
            ros::Publisher pub = nh_.advertise<nav_msgs::Path>(topic, 10000);
            publisher_map_[topic] = pub;
        }
        nav_msgs::Path path_msg;
        geometry_msgs::PoseStamped tmpPose;
        tmpPose.header.frame_id = "map";
        for (const auto &pt : path) {
            tmpPose.pose.position.x = pt[0];
            tmpPose.pose.position.y = pt[1];
            tmpPose.pose.position.z = pt[2];
            path_msg.poses.push_back(tmpPose);
        }
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = ros::Time::now();
        publisher_map_[topic].publish(path_msg);
    }

    template <class BALLS, class TOPIC>
    inline void visBalls(const BALLS &balls, const TOPIC &topic, bool keep = false, const Color color = blue, const double a = 0.2) {
        auto got = publisher_map_.find(topic);
        if (got == publisher_map_.end()) {
            ros::Publisher pub = nh_.advertise<visualization_msgs::MarkerArray>(topic, 10000);
            publisher_map_[topic] = pub;
        }
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        if (!keep) {
            marker.id = 0;
        } else {
            static int id = 0;
            marker.id = id;
            id++;
        }
        setMarkerColor(marker, color, a);
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.reserve(balls.size() + 1);
        marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(marker);
        marker.action = visualization_msgs::Marker::ADD;
        for (const auto &ball : balls) {
            setMarkerPose(marker, ball.center[0], ball.center[1], ball.center[2]);
            auto d = 2 * ball.radius;
            setMarkerScale(marker, d, d, d);
            marker_array.markers.push_back(marker);
            marker.id++;
        }
        publisher_map_[topic].publish(marker_array);
    }

    template <class TOPIC>
    inline void visBalls(const Eigen::Matrix3Xd &balls, const TOPIC &topic, bool keep = false, const Color color = blue, const double a = 0.2, const double scale = 1) {
        auto got = publisher_map_.find(topic);
        if (got == publisher_map_.end()) {
            ros::Publisher pub = nh_.advertise<visualization_msgs::MarkerArray>(topic, 10000);
            publisher_map_[topic] = pub;
        }
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        if (!keep) {
            marker.id = 0;
        } else {
            static int id = 0;
            marker.id = id;
            id++;
        }
        setMarkerColor(marker, color, a);
        visualization_msgs::MarkerArray marker_array;
        int size = balls.cols();
        marker_array.markers.reserve(size + 1);
        marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(marker);
        marker.action = visualization_msgs::Marker::ADD;
        for (int i = 0; i < size; i++) {
            setMarkerPose(marker, balls.col(i)[0], balls.col(i)[1], balls.col(i)[2]);
            setMarkerScale(marker, scale, scale, scale);
            marker_array.markers.push_back(marker);
            marker.id++;
        }

        publisher_map_[topic].publish(marker_array);
    }

    template <class ELLIPSOIDS, class TOPIC>
    inline void visEllipsoids(const ELLIPSOIDS &ellipsoids, const TOPIC &topic, const Color color = blue, const double a = 0.8) {
        auto got = publisher_map_.find(topic);
        if (got == publisher_map_.end()) {
            ros::Publisher pub = nh_.advertise<visualization_msgs::MarkerArray>(topic, 10000);
            publisher_map_[topic] = pub;
        }
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.id = 0;
        setMarkerColor(marker, color, a);
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.reserve(ellipsoids.size() + 1);
        marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(marker);
        marker.action = visualization_msgs::Marker::ADD;
        for (const auto &e : ellipsoids) {
            setMarkerPose(marker, e.c[0], e.c[1], e.c[2], e.R);  // 导入xyz位置，姿态默认水平
            setMarkerScale(marker, 2 * e.rx, 2 * e.ry, 2 * e.rz);
            marker_array.markers.push_back(marker);
            marker.id++;
        }
        publisher_map_[topic].publish(marker_array);
    }

    template <class PAIRLINE, class TOPIC>
    inline void visPairline(const PAIRLINE &pairline, const TOPIC &topic, const Color &color = green, double scale = 0.1) {
        auto got = publisher_map_.find(topic);
        if (got == publisher_map_.end()) {
            ros::Publisher pub = nh_.advertise<visualization_msgs::Marker>(topic, 10000);
            publisher_map_[topic] = pub;
        }
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        setMarkerPose(marker, 0, 0, 0);
        setMarkerColor(marker, color, 1);
        setMarkerScale(marker, scale, scale, scale);
        marker.points.resize(2 * pairline.size());
        for (size_t i = 0; i < pairline.size(); ++i) {
            marker.points[2 * i + 0].x = pairline[i].first[0];
            marker.points[2 * i + 0].y = pairline[i].first[1];
            marker.points[2 * i + 0].z = pairline[i].first[2];
            marker.points[2 * i + 1].x = pairline[i].second[0];
            marker.points[2 * i + 1].y = pairline[i].second[1];
            marker.points[2 * i + 1].z = pairline[i].second[2];
        }
        publisher_map_[topic].publish(marker);
    }

    template <class ARROWS, class TOPIC>
    inline void visArrows(const ARROWS &arrows, const TOPIC &topic, const Color &color) {
        auto got = publisher_map_.find(topic);
        if (got == publisher_map_.end()) {
            ros::Publisher pub = nh_.advertise<visualization_msgs::MarkerArray>(topic, 10000);
            publisher_map_[topic] = pub;
        }
        visualization_msgs::Marker clear_previous_msg;
        clear_previous_msg.action = visualization_msgs::Marker::DELETEALL;
        visualization_msgs::Marker arrow_msg;
        arrow_msg.type = visualization_msgs::Marker::ARROW;
        arrow_msg.action = visualization_msgs::Marker::ADD;
        arrow_msg.header.frame_id = "map";
        arrow_msg.id = 0;
        arrow_msg.points.resize(2);
        setMarkerPose(arrow_msg, 0, 0, 0);
        setMarkerScale(arrow_msg, 0.4, 0.7, 0);
        setMarkerColor(arrow_msg, color, 0.7);
        visualization_msgs::MarkerArray arrow_list_msg;
        arrow_list_msg.markers.reserve(1 + arrows.size());
        arrow_list_msg.markers.push_back(clear_previous_msg);
        for (const auto &arrow : arrows) {
            arrow_msg.points[0].x = arrow.first[0];
            arrow_msg.points[0].y = arrow.first[1];
            arrow_msg.points[0].z = arrow.first[2];
            arrow_msg.points[1].x = arrow.second[0];
            arrow_msg.points[1].y = arrow.second[1];
            arrow_msg.points[1].z = arrow.second[2];
            arrow_list_msg.markers.push_back(arrow_msg);
            arrow_msg.id += 1;
        }
        publisher_map_[topic].publish(arrow_list_msg);
    }

    template <class TOPIC>
    inline void visVector(const Eigen::Vector3d &vec_pos, Eigen::Vector3d vec_dir, const TOPIC &topic, const Color &color, const int &id, bool clear_old = false) {
        auto got = publisher_map_.find(topic);
        if (got == publisher_map_.end()) {
            ros::Publisher pub = nh_.advertise<visualization_msgs::Marker>(topic, 10000);
            publisher_map_[topic] = pub;
        }

        if (clear_old) {
            visualization_msgs::Marker clc;
            clc.header.frame_id = "map";
            clc.type = visualization_msgs::Marker::DELETEALL;
            publisher_map_[topic].publish(clc);
        }

        visualization_msgs::Marker vec;
        vec.type = visualization_msgs::Marker::ARROW;
        vec.action = visualization_msgs::Marker::ADD;
        vec.header.frame_id = "map";
        vec.id = id;

        vec.scale.x = 1.0;
        vec.scale.y = 0.2;
        vec.scale.z = 0.2;
        vec.color.a = 1;
        vec.color.r = 1.0;
        vec.color.g = 1.0;
        vec.color.b = 0.0;
        vec.pose.position.x = vec_pos(0);
        vec.pose.position.y = vec_pos(1);
        vec.pose.position.z = vec_pos(2);
        vec_dir.normalize();
        Eigen::Vector3d final_pose = 0.5 * (Eigen::Vector3d(1, 0, 0) + vec_dir);
        // final_pose.normalize();
        Eigen::AngleAxisd t_V(M_PI, final_pose);
        Eigen::Quaterniond q(t_V);
        q.normalize();
        vec.pose.orientation.w = q.w();
        vec.pose.orientation.x = q.x();
        vec.pose.orientation.y = q.y();
        vec.pose.orientation.z = q.z();
        publisher_map_[topic].publish(vec);
    }

    template <class TOPIC>
    inline void visEdge(
        const TOPIC &edge_topic, const Eigen::MatrixX3d &start, const Eigen::MatrixX3d &end, const Eigen::Vector3d &offset = Eigen::Vector3d::Zero(),
        const Eigen::Matrix3d &rot = Eigen::Matrix3d::Identity(), const int tid = 11542, const double a = 1.0) {
        assert(start.rows() == end.rows() && "start point size != end point size");
        auto got = publisher_map_.find(edge_topic);
        if (got == publisher_map_.end()) {
            ros::Publisher pub = nh_.advertise<visualization_msgs::Marker>(edge_topic, 10000);
            publisher_map_[edge_topic] = pub;
        }
        visualization_msgs::Marker edgeMarker;
        edgeMarker.header.stamp = ros::Time::now();
        edgeMarker.header.frame_id = "map";
        edgeMarker.pose.orientation.w = 1.00;
        edgeMarker.action = visualization_msgs::Marker::ADD;
        edgeMarker.type = visualization_msgs::Marker::LINE_LIST;
        edgeMarker.ns = "edge";
        edgeMarker.color.r = 0.00;
        edgeMarker.color.g = 1.00;
        edgeMarker.color.b = 0.00;
        edgeMarker.color.a = a;
        edgeMarker.scale.x = 0.2;
        edgeMarker.scale.y = 0.2;
        edgeMarker.scale.z = 0.2;
        // id++;
        edgeMarker.id = tid;
        geometry_msgs::Point point;
        int size = start.rows();
        Eigen::Vector3d rpt(0, 0, 0);
        for (int i = 0; i < start.rows(); i++) {
            rpt = rot * start.row(i).transpose() + offset;
            point.x = rpt(0);
            point.y = rpt(1);
            point.z = rpt(2);
            edgeMarker.points.push_back(point);
            rpt = rot * end.row(i).transpose() + offset;
            point.x = rpt(0);
            point.y = rpt(1);
            point.z = rpt(2);
            edgeMarker.points.push_back(point);
        }
        publisher_map_[edge_topic].publish(edgeMarker);
    }

    template <class TOPIC>
    inline void visEdge(
        const TOPIC &edge_topic, const Eigen::Vector3d &start, const Eigen::Vector3d &end, const Eigen::Vector3d &offset = Eigen::Vector3d::Zero(),
        const Eigen::Matrix3d &rot = Eigen::Matrix3d::Identity()) {
        auto got = publisher_map_.find(edge_topic);
        if (got == publisher_map_.end()) {
            ros::Publisher pub = nh_.advertise<visualization_msgs::Marker>(edge_topic, 10000);
            publisher_map_[edge_topic] = pub;
        }
        visualization_msgs::Marker edgeMarker;
        edgeMarker.header.stamp = ros::Time::now();
        edgeMarker.header.frame_id = "map";
        edgeMarker.pose.orientation.w = 1.00;
        edgeMarker.action = visualization_msgs::Marker::ADD;
        edgeMarker.type = visualization_msgs::Marker::LINE_LIST;
        edgeMarker.ns = "edge";
        edgeMarker.color.r = 1.00;
        edgeMarker.color.g = 0.00;
        edgeMarker.color.b = 0.00;
        edgeMarker.color.a = 1.00;
        edgeMarker.scale.x = 0.00;
        edgeMarker.scale.y = 0.09;
        edgeMarker.scale.z = 0.09;
        static int id = 0;
        id++;
        edgeMarker.id = id;
        geometry_msgs::Point point;
        Eigen::Vector3d rpt;
        rpt = rot * start + offset;
        point.x = rpt(0);
        point.y = rpt(1);
        point.z = rpt(2);
        edgeMarker.points.push_back(point);
        rpt = rot * end + offset;
        point.x = rpt(0);
        point.y = rpt(1);
        point.z = rpt(2);
        edgeMarker.points.push_back(point);

        publisher_map_[edge_topic].publish(edgeMarker);
    }

    template <class TOPIC1, class TOPIC2>
    inline void visMesh(const TOPIC1 &meth_topic, const TOPIC2 &edge_topic, const Eigen::MatrixXd &U, const Eigen::MatrixXi &G, double s = 0.1, const Color color = steelblue, const double a = 0.02) {
        auto got = publisher_map_.find(meth_topic);
        if (got == publisher_map_.end()) {
            ros::Publisher pub1 = nh_.advertise<visualization_msgs::Marker>(meth_topic, 10000);
            publisher_map_[meth_topic] = pub1;
        }
        auto got2 = publisher_map_.find(edge_topic);
        if (got2 == publisher_map_.end()) {
            ros::Publisher pub2 = nh_.advertise<visualization_msgs::Marker>(edge_topic, 10000);
            publisher_map_[edge_topic] = pub2;
        }

        visualization_msgs::Marker meshMarker, edgeMarker;

        meshMarker.header.stamp = ros::Time::now();
        meshMarker.header.frame_id = "map";
        meshMarker.pose.orientation.w = 1.00;
        meshMarker.action = visualization_msgs::Marker::ADD;
        meshMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
        meshMarker.ns = "mesh";
        meshMarker.color.r = 0.00;
        meshMarker.color.g = 0.00;
        meshMarker.color.b = 1.00;
        meshMarker.color.a = a;

        setMarkerColor(meshMarker, color, a);

        meshMarker.scale.x = 1.00;
        meshMarker.scale.y = 1.00;
        meshMarker.scale.z = 1.00;

        edgeMarker = meshMarker;
        edgeMarker.type = visualization_msgs::Marker::LINE_LIST;
        edgeMarker.ns = "edge";
        edgeMarker.color.r = 0.00;
        edgeMarker.color.g = 1.00;
        edgeMarker.color.b = 0.00;
        edgeMarker.color.a = 1.00;
        edgeMarker.scale.x = 0.005 * s;
        edgeMarker.scale.y = 0.005 * s;
        edgeMarker.scale.z = 0.005 * s;

        geometry_msgs::Point point;

        int faces = G.rows();
        for (int i = 0; i < faces; i++) {
            for (int j = 0; j < 3; j++) {
                point.x = U.row(G.row(i)[j])[0];
                point.y = U.row(G.row(i)[j])[1];
                point.z = U.row(G.row(i)[j])[2];
                meshMarker.points.push_back(point);

                edgeMarker.points.push_back(point);
                point.x = U.row(G.row(i)[(j + 1) % 3])[0];
                point.y = U.row(G.row(i)[(j + 1) % 3])[1];
                point.z = U.row(G.row(i)[(j + 1) % 3])[2];
                edgeMarker.points.push_back(point);
            }
        }
        publisher_map_[edge_topic].publish(edgeMarker);
        publisher_map_[meth_topic].publish(meshMarker);
    }

    template <class TOPIC1, class TOPIC2>
    inline void visPolytope(
        const Eigen::Matrix3Xd &mesh, const TOPIC1 &meth_topic, const TOPIC2 &edge_topic, bool keep = false, double s = 0.1, const int tid = 114513, const Color color = green, const double a = 1.0,
        bool del = false) {
        auto got = publisher_map_.find(meth_topic);
        if (got == publisher_map_.end()) {
            ros::Publisher pub1 = nh_.advertise<visualization_msgs::Marker>(meth_topic, 10000);
            publisher_map_[meth_topic] = pub1;
            double wait_time = 0.0;
            while (pub1.getNumSubscribers() < 1) {
                ros::Duration(0.1).sleep();
                wait_time += 0.1;
                if (wait_time > 0.5) {
                    std::cout << "[Livz] 话题 " << meth_topic << "似乎没有订阅者,请检查rviz配置。" << "[Livz] Looks like Topic " << edge_topic
                              << "is not subscribed by any subscriber. Check rviz config.";
                    break;
                }
            }
        }
        auto got2 = publisher_map_.find(edge_topic);
        if (got2 == publisher_map_.end()) {
            ros::Publisher pub2 = nh_.advertise<visualization_msgs::Marker>(edge_topic, 10000);
            publisher_map_[edge_topic] = pub2;
            double wait_time = 0.0;
            while (pub2.getNumSubscribers() < 1) {
                ros::Duration(0.1).sleep();
                wait_time += 0.1;
                if (wait_time > 0.5) {
                    std::cout << "[Livz] 话题 " << meth_topic << "似乎没有订阅者,请检查rviz配置。" << "[Livz] Looks like Topic " << edge_topic
                              << "is not subscribed by any subscriber. Check rviz config.";
                    break;
                }
            }
        }

        visualization_msgs::Marker meshMarker, edgeMarker;
        std::string se3("SE3path");
        if (meth_topic == se3) {
            meshMarker.lifetime = ros::Duration(2.0);
            edgeMarker.lifetime = ros::Duration(2.0);
        }
        if (!keep) {
            meshMarker.id = tid;
        } else {
            static int id = 700;
            meshMarker.id = id;
            id++;
        }

        if (del == true) {
            meshMarker.action = visualization_msgs::Marker::DELETEALL;
            edgeMarker = meshMarker;
            publisher_map_[meth_topic].publish(meshMarker);
            publisher_map_[edge_topic].publish(edgeMarker);
            return;
        } else {
            meshMarker.action = visualization_msgs::Marker::ADD;
        }
        meshMarker.header.stamp = ros::Time::now();
        meshMarker.header.frame_id = "map";
        meshMarker.pose.orientation.w = 1.00;
        meshMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
        meshMarker.ns = "mesh";

        setMarkerColor(meshMarker, blue, a);
        setMarkerScale(meshMarker, 1.00, 1.00, 1.00);

        edgeMarker = meshMarker;
        edgeMarker.type = visualization_msgs::Marker::LINE_LIST;
        edgeMarker.ns = "edge";
        setMarkerColor(edgeMarker, lightyellow, a);
        setMarkerScale(edgeMarker, 0.005 * s, 0.005 * s, 0.005 * s);

        meshMarker.id = tid;
        geometry_msgs::Point point;

        int ptnum = mesh.cols();

        for (int i = 0; i < ptnum; i++) {
            point.x = mesh(0, i);
            point.y = mesh(1, i);
            point.z = mesh(2, i);
            meshMarker.points.push_back(point);
        }

        for (int i = 0; i < ptnum / 3; i++) {
            for (int j = 0; j < 3; j++) {
                point.x = mesh(0, 3 * i + j);
                point.y = mesh(1, 3 * i + j);
                point.z = mesh(2, 3 * i + j);
                edgeMarker.points.push_back(point);
                point.x = mesh(0, 3 * i + (j + 1) % 3);
                point.y = mesh(1, 3 * i + (j + 1) % 3);
                point.z = mesh(2, 3 * i + (j + 1) % 3);
                edgeMarker.points.push_back(point);
            }
        }
        // ROS_WARN_STREAM(" pub ");
        publisher_map_[edge_topic].publish(edgeMarker);
        publisher_map_[meth_topic].publish(meshMarker);
    }

    template <class TOPIC1, class TOPIC2>
    inline void visPolytopeOffset(
        const Eigen::Matrix3Xd &mesh, const TOPIC1 &meth_topic, const TOPIC2 &edge_topic, bool keep = false, double s = 0.1, const int tid = 14513, const Color color = green, const double a = 1.0,
        bool del = false) {
        auto got = publisher_map_.find(meth_topic);
        if (got == publisher_map_.end()) {
            ros::Publisher pub1 = nh_.advertise<visualization_msgs::Marker>(meth_topic, 10000);
            publisher_map_[meth_topic] = pub1;
            double wait_time = 0.0;
            while (pub1.getNumSubscribers() < 1) {
                ros::Duration(0.1).sleep();
                wait_time += 0.1;
                if (wait_time > 0.5) {
                    std::cout << "[Livz] 话题 " << meth_topic << "似乎没有订阅者,请检查rviz配置。" << "[Livz] Looks like Topic " << edge_topic
                              << "is not subscribed by any subscriber. Check rviz config.";
                    break;
                }
            }
        }
        auto got2 = publisher_map_.find(edge_topic);
        if (got2 == publisher_map_.end()) {
            ros::Publisher pub2 = nh_.advertise<visualization_msgs::Marker>(edge_topic, 10000);
            publisher_map_[edge_topic] = pub2;
            double wait_time = 0.0;
            while (pub2.getNumSubscribers() < 1) {
                ros::Duration(0.1).sleep();
                wait_time += 0.1;
                if (wait_time > 0.5) {
                    std::cout << "[Livz] 话题 " << meth_topic << "似乎没有订阅者,请检查rviz配置。" << "[Livz] Looks like Topic " << edge_topic
                              << "is not subscribed by any subscriber. Check rviz config.";
                    break;
                }
            }
        }

        visualization_msgs::Marker meshMarker, edgeMarker;

        meshMarker.id = tid;

        if (del == true) {
            meshMarker.action = visualization_msgs::Marker::DELETEALL;
            edgeMarker = meshMarker;
            publisher_map_[meth_topic].publish(meshMarker);
            publisher_map_[edge_topic].publish(edgeMarker);
            return;
        } else {
            meshMarker.action = visualization_msgs::Marker::ADD;
        }
        meshMarker.header.stamp = ros::Time::now();
        meshMarker.header.frame_id = "map";
        meshMarker.pose.orientation.w = 1.00;
        meshMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
        meshMarker.ns = "mesh";

        setMarkerColor(meshMarker, lightred, a);

        meshMarker.scale.x = 1.00;
        meshMarker.scale.y = 1.00;
        meshMarker.scale.z = 1.00;

        edgeMarker = meshMarker;
        edgeMarker.type = visualization_msgs::Marker::LINE_LIST;
        edgeMarker.ns = "edge";
        edgeMarker.color.r = 0.00;
        edgeMarker.color.g = 1.00;
        edgeMarker.color.b = 0.00;
        edgeMarker.color.a = a;
        setMarkerColor(edgeMarker, lightyellow, a);
        edgeMarker.scale.x = 0.005 * s;
        edgeMarker.scale.y = 0.005 * s;
        edgeMarker.scale.z = 0.005 * s;

        meshMarker.id = tid;
        geometry_msgs::Point point;

        int ptnum = mesh.cols();  // ptnum/3

        for (int i = 0; i < ptnum; i++) {
            point.x = mesh(0, i);
            point.y = mesh(1, i);
            // point.z = mesh(2, i) + 3.0;
            point.z = mesh(2, i) + 0.07;
            meshMarker.points.push_back(point);
        }

        for (int i = 0; i < ptnum / 3; i++)  // mesh.cols() 12
        {
            for (int j = 0; j < 3; j++) {
                point.x = mesh(0, 3 * i + j);
                point.y = mesh(1, 3 * i + j);
                // point.z = mesh(2, 3 * i + j) + 3.0;
                point.z = mesh(2, 3 * i + j);
                edgeMarker.points.push_back(point);
                point.x = mesh(0, 3 * i + (j + 1) % 3);
                point.y = mesh(1, 3 * i + (j + 1) % 3);
                // point.z = mesh(2, 3 * i + (j + 1) % 3) + 3.0;
                point.z = mesh(2, 3 * i + j);
                edgeMarker.points.push_back(point);
            }
        }

        publisher_map_[edge_topic].publish(edgeMarker);
        publisher_map_[meth_topic].publish(meshMarker);
    }

    template <class TOPIC1, class TOPIC2>
    inline void visPolytopeColli(
        const Eigen::Matrix3Xd &mesh, const TOPIC1 &meth_topic, const TOPIC2 &edge_topic, bool keep = false, double s = 0.1, const int tid = 14513, const Color color = green, const double a = 1.0,
        bool del = false) {
        auto got = publisher_map_.find(meth_topic);
        if (got == publisher_map_.end()) {
            ros::Publisher pub1 = nh_.advertise<visualization_msgs::Marker>(meth_topic, 10000);
            publisher_map_[meth_topic] = pub1;
            // ROS_WARN_STREAM("mesh pub init");
            double wait_time = 0.0;
            while (pub1.getNumSubscribers() < 1) {
                ros::Duration(0.1).sleep();
                wait_time += 0.1;
                if (wait_time > 0.5) {
                    std::cout << "[Livz] 话题 " << meth_topic << "似乎没有订阅者,请检查rviz配置。" << "[Livz] Looks like Topic " << edge_topic
                              << "is not subscribed by any subscriber. Check rviz config.";
                    break;
                }
            }
        }

        visualization_msgs::Marker meshMarker, edgeMarker;
        static int id = 700;
        meshMarker.id = id;
        id++;

        if (del == true) {
            meshMarker.action = visualization_msgs::Marker::DELETEALL;
            // edgeMarker=meshMarker;
            publisher_map_[meth_topic].publish(meshMarker);
            // publisher_map_[edge_topic].publish(edgeMarker);
            return;
        } else {
            meshMarker.action = visualization_msgs::Marker::ADD;
        }
        meshMarker.header.stamp = ros::Time::now();
        meshMarker.header.frame_id = "map";
        meshMarker.pose.orientation.w = 1.00;
        meshMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
        meshMarker.ns = "mesh";

        setMarkerColor(meshMarker, pink, 0.1);

        meshMarker.scale.x = 1.00;
        meshMarker.scale.y = 1.00;
        meshMarker.scale.z = 1.00;

        meshMarker.id = id;  // 保留id增长
        geometry_msgs::Point point;

        int ptnum = mesh.cols();  // ptnum/3

        for (int i = 0; i < ptnum; i++) {
            point.x = mesh(0, i);
            point.y = mesh(1, i);
            point.z = mesh(2, i) + 6.0;
            meshMarker.points.push_back(point);
        }

        for (int i = 0; i < ptnum / 3; i++)  // mesh.cols() 12
        {
            for (int j = 0; j < 3; j++) {
                point.x = mesh(0, 3 * i + j);
                point.y = mesh(1, 3 * i + j);
                point.z = mesh(2, 3 * i + j) + 3.0;
                // edgeMarker.points.push_back(point);
                point.x = mesh(0, 3 * i + (j + 1) % 3);
                point.y = mesh(1, 3 * i + (j + 1) % 3);
                point.z = mesh(2, 3 * i + (j + 1) % 3) + 6.0;
                // edgeMarker.points.push_back(point);
            }
        }
        // ROS_WARN_STREAM(" pub ");
        // publisher_map_[edge_topic].publish(edgeMarker);
        publisher_map_[meth_topic].publish(meshMarker);
    }
    inline void visTrajectoryPiece(
        const Eigen::Matrix3d &Rt, const Eigen::Vector3d &trans, const std::string &topic = "/trajectory_piece_visualization", bool clear_previous = true, double alpha = 0.5,
        const Color color = lightyellow,
        int num_inner_layers = 5  // New parameter to control number of inner layers
    ) {
        auto got = publisher_map_.find(topic);
        if (got == publisher_map_.end()) {
            ros::Publisher pub = nh_.advertise<visualization_msgs::MarkerArray>(topic, 10000);
            publisher_map_[topic] = pub;
            ros::Duration(0.1).sleep();
        }

        visualization_msgs::MarkerArray marker_array;

        // Clear previous markers if requested
        if (clear_previous) {
            visualization_msgs::Marker delete_marker;
            delete_marker.action = visualization_msgs::Marker::DELETEALL;
            marker_array.markers.push_back(delete_marker);
        }
        // Base points of the shape with interpolated corner points
        std::vector<std::pair<double, double>> base_points = {
            {0.0875, -0.6},
            // Add intermediate points around corners
            {0.0875, -0.59},  // Extra point near corner
            {0.0875, 0.59},   // Extra point near corner
            {0.0875, 0.6},
            {0.0875, 0.6},   // Duplicate corner point
            {-0.0025, 0.6},  // Extra point near corner
            {-0.0125, 0.6},
            {-0.0125, 0.6},   // Duplicate corner point
            {-0.0125, 0.55},  // Extra point near corner
            {-0.0125, 0.25},
            {-0.0125, 0.25},  // Duplicate corner point
            {-0.4325, 0.25},  // Extra point near corner
            {-0.4425, 0.25},
            {-0.4425, 0.25},   // Duplicate corner point
            {-0.4425, -0.24},  // Extra point near corner
            {-0.4425, -0.25},
            {-0.4425, -0.25},  // Duplicate corner point
            {-0.0225, -0.25},  // Extra point near corner
            {-0.0125, -0.25},
            {-0.0125, -0.25},  // Duplicate corner point
            {-0.0125, -0.59},  // Extra point near corner
            {-0.0125, -0.6},
            {-0.0125, -0.6},  // Duplicate corner point
            {0.0775, -0.6},   // Extra point near corner
            {0.0875, -0.6}};

        static int marker_id = 0;

        // Calculate the center point of the shape
        Eigen::Vector2d center = Eigen::Vector2d::Zero();
        for (const auto &bp : base_points) {
            center += Eigen::Vector2d(bp.first, bp.second);
        }
        center /= base_points.size();

        // Create multiple layers of outlines
        for (int layer = 0; layer <= num_inner_layers; ++layer) {
            double scale_factor = 0.8 - (double)layer / num_inner_layers;
            double layer_alpha = alpha * (1.0 - 0.3 * layer / num_inner_layers);  // Slightly decrease alpha for inner layers

            visualization_msgs::Marker path_marker;
            path_marker.header.frame_id = "map";
            path_marker.header.stamp = ros::Time::now();
            path_marker.type = visualization_msgs::Marker::LINE_STRIP;
            path_marker.action = visualization_msgs::Marker::ADD;
            path_marker.id = marker_id++;
            path_marker.ns = "trajectory_piece_outline_layer_" + std::to_string(layer);

            // Set the color with varying alpha
            setMarkerColor(path_marker, color, layer_alpha);

            // Set line thickness (decrease for inner layers)
            path_marker.scale.x = 0.1 * (1.0 - 0.5 * layer / num_inner_layers);
            path_marker.scale.y = 0.0;
            path_marker.scale.z = 0.0;

            path_marker.pose.orientation.w = 1.0;

            // Transform and add points with scaling
            for (const auto &bp : base_points) {
                // Scale point relative to center
                Eigen::Vector2d original_point(bp.first, bp.second);
                Eigen::Vector2d scaled_point = center + (original_point - center) * scale_factor;

                Eigen::Vector3d pt(scaled_point.x(), scaled_point.y(), 0.0);
                Eigen::Vector3d shift(0.0, 0.0, 0.0);
                Eigen::Vector3d transformed_pt = Rt * pt + trans + shift;

                geometry_msgs::Point point;
                point.x = transformed_pt(0);
                point.y = transformed_pt(1);
                point.z = transformed_pt(2);
                path_marker.points.push_back(point);
            }

            // Close the loop
            if (!path_marker.points.empty()) {
                path_marker.points.push_back(path_marker.points[0]);
            }

            marker_array.markers.push_back(path_marker);
        }

        // Reset marker ID periodically to prevent overflow
        // if (marker_id > 10000) marker_id = 0;

        // Publish markers
        publisher_map_[topic].publish(marker_array);
    }

    template <class TOPIC>
    inline void visR3Path(const TOPIC &topic, const vector<Vector3d> &path, int path_id = 114514) {
        auto got = publisher_map_.find(topic);
        if (got == publisher_map_.end()) {
            ros::Publisher pub1 = nh_.advertise<visualization_msgs::Marker>(topic, 10000);
            publisher_map_[topic] = pub1;
            double wait_time = 0.0;
            while (pub1.getNumSubscribers() < 1) {
                ros::Duration(0.1).sleep();
                wait_time += 0.1;
                if (wait_time > 0.5) {
                    std::cout << "[Livz] 话题 " << topic << "似乎没有订阅者,请检查rviz配置。" << "[Livz] Looks like Topic " << topic << "is not subscribed by any subscriber. Check rviz config.";
                    break;
                }
            }
        }

        visualization_msgs::Marker sphere, line_strip;
        sphere.lifetime = ros::Duration(4.0);
        line_strip.lifetime = ros::Duration(4.0);

        sphere.header.frame_id = line_strip.header.frame_id = "map";
        sphere.header.stamp = line_strip.header.stamp = ros::Time::now();

        sphere.type = visualization_msgs::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;

        sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
        sphere.id = path_id;
        line_strip.id = path_id + 1000;

        sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        sphere.color.r = line_strip.color.r = 0.4;
        sphere.color.g = line_strip.color.g = 1.0;
        sphere.color.b = line_strip.color.b = 0.4;
        sphere.color.a = line_strip.color.a = 0.8;
        sphere.scale.x = line_strip.scale.x = 0.3;
        sphere.scale.y = line_strip.scale.y = 0.3;
        sphere.scale.z = line_strip.scale.z = 0.3;

        geometry_msgs::Point pt;
        Eigen::Vector3d ptv;
        for (size_t i = 0; i < path.size(); i++) {
            ptv = path[i];
            pt.x = ptv(0);
            pt.y = ptv(1);
            pt.z = 0.0;
            sphere.points.push_back(pt);
            line_strip.points.push_back(pt);
        }
        publisher_map_[topic].publish(sphere);
        publisher_map_[topic].publish(line_strip);
    }

    template <class TOPIC>
    inline void visSE3Path(const TOPIC &topic, const Eigen::Matrix3Xd &mesh, const vector<SE3State> &se3_path, const double a = 0.1, int se3_path_id = 1918) {
        if (se3_path.size() == 0) {
            return;
        }
        auto got = publisher_map_.find(topic);
        if (got == publisher_map_.end()) {
            ros::Publisher pub1 = nh_.advertise<visualization_msgs::Marker>(topic, 10000);
            publisher_map_[topic] = pub1;
            double wait_time = 0.0;
            while (pub1.getNumSubscribers() < 1) {
                ros::Duration(0.1).sleep();
                wait_time += 0.1;
                if (wait_time > 0.5) {
                    std::cout << "[Livz] 话题 " << topic << "似乎没有订阅者,请检查rviz配置。" << "[Livz] Looks like Topic " << topic << "is not subscribed by any subscriber. Check rviz config.";
                    break;
                }
            }
        }

        Eigen::Vector3d pos;
        Eigen::Matrix3d rotate;
        Eigen::Matrix3Xd debugmesh_var;
        bool keep = true;
        visPolytope(debugmesh_var, topic, "SE3edge", keep, 0.1, 311, steelblue, a, true);
        for (int i = 0; i < se3_path.size(); i++) {
            pos = se3_path[i].position;
            rotate = se3_path[i].getRotMatrix();
            debugmesh_var = (rotate * mesh).colwise() + pos;
            if (i == se3_path.size() - 1) {
                keep = false;
            }
            visPolytope(debugmesh_var, topic, "SE3edge", keep, 0.1, 10 + i, steelblue, a, false);
        }
    }

    template <class TOPIC>
    inline void visSE3Vec(const TOPIC &topic, const vector<Eigen::Vector3d> &points, const vector<Eigen::Vector3d> &acc_list, int se3_path_id = 191018) {
        if (acc_list.size() == 0) {
            return;
        }
        auto got = publisher_map_.find(topic);
        if (got == publisher_map_.end()) {
            ros::Publisher pub1 = nh_.advertise<visualization_msgs::Marker>(topic, 10000);
            publisher_map_[topic] = pub1;
        }

        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = "map";
        line_strip.header.stamp = ros::Time::now();
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.id = se3_path_id;

        line_strip.color.r = 1;
        line_strip.color.g = 0;
        line_strip.color.b = 0;
        line_strip.color.a = 1;
        line_strip.scale.x = 0.05 / 2;

        geometry_msgs::Point pt;
        for (double i = 0; i < points.size(); i++) {
            Eigen::Vector3d dur_p = points[i];
            pt.x = dur_p(0);
            pt.y = dur_p(1);
            pt.z = dur_p(2);
            line_strip.points.push_back(pt);
        }
        publisher_map_[topic].publish(line_strip);

        const double alpha = 0.1;
        for (double i = 0; i < points.size(); i++) {
            Eigen::Vector3d arr_p = points[i];
            Eigen::Vector3d arr_a = acc_list[i];

            visualization_msgs::Marker acc_dir;
            acc_dir.header.frame_id = "map";
            acc_dir.id = se3_path_id + 1 + i;
            acc_dir.type = visualization_msgs::Marker::ARROW;
            //   acc_dir.scale.x = arr_p.norm();
            acc_dir.scale.x = 1.0;
            acc_dir.scale.y = 0.2;
            acc_dir.scale.z = 0.2;
            acc_dir.color.a = 1;
            acc_dir.color.r = 0.2;
            acc_dir.color.g = 0.2;
            acc_dir.color.b = 1.0;
            acc_dir.pose.position.x = arr_p(0);
            acc_dir.pose.position.y = arr_p(1);
            acc_dir.pose.position.z = arr_p(2);

            arr_a.normalize();
            Eigen::Vector3d final_pose = 0.5 * (Eigen::Vector3d(1, 0, 0) + arr_a);

            Eigen::AngleAxisd t_V(M_PI, final_pose);
            Eigen::Quaterniond q(t_V);
            q.normalize();
            acc_dir.pose.orientation.w = q.w();
            acc_dir.pose.orientation.x = q.x();
            acc_dir.pose.orientation.y = q.y();
            acc_dir.pose.orientation.z = q.z();
            publisher_map_[topic].publish(acc_dir);
        }
    }

    template <class TOPIC>
    inline void visTraj(const TOPIC &topic, Trajectory<TRAJ_ORDER> traj, int traj_id = 15526015, bool ontop = false, double lifetime = 2) {
        auto got = publisher_map_.find(topic);
        if (got == publisher_map_.end()) {
            ros::Publisher pub1 = nh_.advertise<visualization_msgs::Marker>(topic, 10000);
            publisher_map_[topic] = pub1;
            double wait_time = 0.0;
            while (pub1.getNumSubscribers() < 1) {
                ros::Duration(0.1).sleep();
                wait_time += 0.1;
                if (wait_time > 0.5) {
                    std::cout << "[Livz] 话题 " << topic << "似乎没有订阅者,请检查rviz配置。" << "[Livz] Looks like Topic " << topic << "is not subscribed by any subscriber. Check rviz config.";
                    break;
                }
            }
        }
        visualization_msgs::Marker traj_vis;
        std::string st("step_traj");
        if (topic == st) {
            traj_vis.lifetime = ros::Duration(0.5);
        }
        traj_vis.header.stamp = ros::Time::now();
        traj_vis.header.frame_id = "map";
        traj_vis.id = traj_id;
        traj_vis.type = visualization_msgs::Marker::LINE_STRIP;
        traj_vis.scale.x = 0.05 / 2;
        traj_vis.pose.orientation.x = 0.0;
        traj_vis.pose.orientation.y = 0.0;
        traj_vis.pose.orientation.z = 0.0;
        traj_vis.pose.orientation.w = 1.0;

        if (lifetime > 15) {
            traj_vis.lifetime = ros::Duration(0);
        } else {
            traj_vis.lifetime = ros::Duration(lifetime);
        }

        // traj_vis.color.a = 1.0;
        // traj_vis.color.r = 1.0;
        // traj_vis.color.g = 0.0;
        // traj_vis.color.b = 0.0;
        // setMarkerColor(traj_vis, vis::Color::red, 1.0);
        // Automatically choose color based on traj_id
        static const std::vector<Color> colors = {Color::red,       Color::green,      Color::blue,       Color::yellow,      Color::orange,   Color::purple, Color::pink,
                                                  Color::steelblue, Color::chartreuse, Color::light_blue, Color::lightyellow, Color::lightred, Color::darkred};
        Color auto_color = colors[traj_id % colors.size()];
        setMarkerColor(traj_vis, auto_color, 1.0);

        geometry_msgs::Point pt;
        Vector3d pos;
        double t_duration = traj.getTotalDuration();
        for (double t = 0; t < t_duration; t += 0.05) {
            pos = traj.getPos(t);
            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = 0;
            if (ontop) {
                pt.z = 8;
            }
            traj_vis.points.push_back(pt);
        }
        publisher_map_[topic].publish(traj_vis);
    }

    template <class TOPIC>
    inline void visMultipleTraj(const TOPIC &topic, const std::vector<Trajectory<TRAJ_ORDER>> &trajs, bool ontop = false, const Color color = lightyellow) {
        // Ensure publisher exists or create a new one
        if (publisher_map_.find(topic) == publisher_map_.end()) {
            // Latched snapshot of the finished result, like the aabbpoints
            // cloud. A plan publishes these once and never again, so a viewer
            // that connects afterwards -- Foxglove over the network, or RViz
            // started late -- saw an empty panel with the old volatile
            // publisher. Latching replays the last array on subscribe, and
            // makes the wait-for-subscribers stall below unnecessary.
            ros::Publisher pub = nh_.advertise<visualization_msgs::MarkerArray>(topic, 1, true);
            publisher_map_[topic] = pub;
        }

        visualization_msgs::MarkerArray marker_array;

        // Replace the complete set: RViz retains old IDs even when the next
        // array is empty or contains fewer trajectories.
        visualization_msgs::Marker clear_marker;
        clear_marker.header.frame_id = "map";
        clear_marker.header.stamp = ros::Time::now();
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);

        for (size_t i = 0; i < trajs.size(); ++i) {
            visualization_msgs::Marker traj_vis;
            traj_vis.header.stamp = ros::Time::now();
            traj_vis.header.frame_id = "map";
            traj_vis.id = i;
            traj_vis.type = visualization_msgs::Marker::LINE_STRIP;
            traj_vis.scale.x = 0.05 / 2;
            traj_vis.pose.orientation.w = 1.0;
            // traj_vis.lifetime = ros::Duration(20);

            // Automatically choose color based on traj index
            // static const std::vector<Color> colors = {Color::red,       Color::green,      Color::blue,       Color::yellow,      Color::orange,   Color::purple, Color::pink,
            //                                           Color::steelblue, Color::chartreuse, Color::light_blue, Color::lightyellow, Color::lightred, Color::darkred};
            // Color auto_color = colors[i % colors.size()];

            setMarkerColor(traj_vis, color, 1.0);

            double t_duration = trajs[i].getTotalDuration();
            for (double t = 0; t < t_duration; t += 0.05) {
                Vector3d pos = trajs[i].getPos(t);
                geometry_msgs::Point pt;
                pt.x = pos(0);
                pt.y = pos(1);
                pt.z = ontop ? 8 : 0;
                traj_vis.points.push_back(pt);
            }

            marker_array.markers.push_back(traj_vis);
        }

        publisher_map_[topic].publish(marker_array);
    }

    // discretized fast check node visualization
    template <class TOPIC>
    inline void visPathNode(const TOPIC &topic, const std::vector<Eigen::Vector3d> &nodes, const std::vector<std::vector<bool>> &conditions, double _resolution, double lifetime = 3.0) {
        // Publisher setup remains the same until marker creation
        auto got = publisher_map_.find(topic);
        if (got == publisher_map_.end()) {
            ros::Publisher pub1 = nh_.advertise<visualization_msgs::MarkerArray>(topic, 10000);
            publisher_map_[topic] = pub1;
            double wait_time = 0.0;
            while (pub1.getNumSubscribers() < 1) {
                ros::Duration(0.1).sleep();
                wait_time += 0.1;
                if (wait_time > 0.5) {
                    std::cout << "[Livz] Topic " << topic << " has no subscribers. Check rviz config.\n";
                    break;
                }
            }
        }

        // Clear previous markers
        visualization_msgs::MarkerArray clear_markers;
        visualization_msgs::Marker clear_marker;
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        clear_markers.markers.push_back(clear_marker);
        publisher_map_[topic].publish(clear_markers);
        ros::Duration(0.1).sleep();  // Give time for clearing

        visualization_msgs::MarkerArray marker_array;

        // Setup node marker
        visualization_msgs::Marker node_vis;
        node_vis.header.frame_id = "map";
        node_vis.header.stamp = ros::Time::now();
        node_vis.ns = "demo_node/expanded_nodes";
        node_vis.type = visualization_msgs::Marker::CUBE_LIST;
        node_vis.action = visualization_msgs::Marker::ADD;
        node_vis.id = 0;
        node_vis.pose.orientation.w = 1.0;
        node_vis.scale.x = node_vis.scale.y = node_vis.scale.z = _resolution / 2;
        node_vis.lifetime = ros::Duration(lifetime);

        // Setup arrow template
        visualization_msgs::Marker arrow_vis;
        arrow_vis.header.frame_id = "map";
        arrow_vis.header.stamp = ros::Time::now();
        arrow_vis.ns = "demo_node/yaw_arrows";
        arrow_vis.type = visualization_msgs::Marker::ARROW;
        arrow_vis.action = visualization_msgs::Marker::ADD;
        arrow_vis.scale.x = _resolution * 0.8;
        arrow_vis.scale.y = arrow_vis.scale.z = _resolution * 0.2;
        arrow_vis.color.b = 1.0;
        arrow_vis.color.a = 1.0;
        arrow_vis.lifetime = ros::Duration(lifetime);

        // Process nodes
        size_t node_idx = 0;
        size_t path_start = 0;

        for (const auto &path_conditions : conditions) {
            size_t path_length = path_conditions.size() + 1;  // +1 for start node

            // Process start node of path
            if (path_start < nodes.size()) {
                const auto &start_node = nodes[path_start];

                // Add cube for start node
                geometry_msgs::Point pt;
                pt.x = start_node(0);
                pt.y = start_node(1);
                node_vis.points.push_back(pt);

                // Start node is always green
                std_msgs::ColorRGBA color;
                color.g = 1.0;
                color.a = 0.5;
                node_vis.colors.push_back(color);

                // Add arrow for start node
                visualization_msgs::Marker start_arrow = arrow_vis;
                start_arrow.id = node_idx++;
                start_arrow.pose.position.x = start_node(0);
                start_arrow.pose.position.y = start_node(1);
                tf2::Quaternion q;
                q.setRPY(0, 0, start_node(2));
                start_arrow.pose.orientation.x = q.x();
                start_arrow.pose.orientation.y = q.y();
                start_arrow.pose.orientation.z = q.z();
                start_arrow.pose.orientation.w = q.w();
                marker_array.markers.push_back(start_arrow);
            }

            // Process remaining nodes in path
            for (size_t i = 0; i < path_conditions.size(); i++) {
                size_t current_node = path_start + i + 1;
                if (current_node >= nodes.size())
                    break;

                const auto &node = nodes[current_node];

                // Add cube
                geometry_msgs::Point pt;
                pt.x = node(0);
                pt.y = node(1);
                node_vis.points.push_back(pt);

                // Set color based on condition
                std_msgs::ColorRGBA color;
                color.a = 0.5;
                if (path_conditions[i]) {
                    color.g = 1.0;
                } else {
                    color.r = 1.0;
                }
                node_vis.colors.push_back(color);

                // Add arrow
                visualization_msgs::Marker arrow = arrow_vis;
                arrow.id = node_idx++;
                arrow.pose.position.x = node(0);
                arrow.pose.position.y = node(1);
                tf2::Quaternion q;
                q.setRPY(0, 0, node(2));
                arrow.pose.orientation.x = q.x();
                arrow.pose.orientation.y = q.y();
                arrow.pose.orientation.z = q.z();
                arrow.pose.orientation.w = q.w();
                marker_array.markers.push_back(arrow);
            }

            path_start += path_length;
        }

        marker_array.markers.push_back(node_vis);
        publisher_map_[topic].publish(marker_array);
    }

    // visulize only the trajectory optimization waypoint in rviz
    template <class TOPIC>
    inline void visualizeOptX(const TOPIC &topic, const Eigen::Matrix3Xd &positions, const Eigen::VectorXd &times) {
        auto got = publisher_map_.find(topic);
        if (got == publisher_map_.end()) {
            ros::Publisher pub1 = nh_.advertise<visualization_msgs::MarkerArray>(topic, 10000);
            publisher_map_[topic] = pub1;
            double wait_time = 0.0;
            while (pub1.getNumSubscribers() < 1) {
                ros::Duration(0.1).sleep();
                wait_time += 0.1;
                if (wait_time > 0.5) {
                    std::cout << "[Livz] 话题 " << topic << "似乎没有订阅者,请检查rviz配置。" << "[Livz] Looks like Topic " << topic << "is not subscribed by any subscriber. Check rviz config.";
                    break;
                }
            }
        }
        visualization_msgs::MarkerArray marker_array;

        double max_time = times.sum();

        for (int i = 0; i < positions.cols(); ++i) {
            // Sphere marker for position
            visualization_msgs::Marker sphere_marker;
            sphere_marker.header.frame_id = "map";
            sphere_marker.header.stamp = ros::Time::now();
            sphere_marker.ns = "optimized_trajectory_position";
            sphere_marker.id = i;
            sphere_marker.type = visualization_msgs::Marker::SPHERE;
            sphere_marker.action = visualization_msgs::Marker::ADD;

            sphere_marker.pose.position.x = positions(0, i);
            sphere_marker.pose.position.y = positions(1, i);
            sphere_marker.pose.position.z = 0;  // Set to ground level or adjust as needed

            sphere_marker.pose.orientation.w = 1.0;

            sphere_marker.scale.x = sphere_marker.scale.y = sphere_marker.scale.z = 0.1;

            double time_ratio = std::accumulate(times.data(), times.data() + i + 1, 0.0) / max_time;
            sphere_marker.color.r = 1 - time_ratio;
            sphere_marker.color.g = time_ratio;
            sphere_marker.color.b = 0.0;
            sphere_marker.color.a = 1.0;

            sphere_marker.lifetime = ros::Duration();

            marker_array.markers.push_back(sphere_marker);

            // Arrow marker for direction (yaw)
            visualization_msgs::Marker arrow_marker;
            arrow_marker.header.frame_id = "map";
            arrow_marker.header.stamp = ros::Time::now();
            arrow_marker.ns = "optimized_trajectory_direction";
            arrow_marker.id = i;
            arrow_marker.type = visualization_msgs::Marker::ARROW;
            arrow_marker.action = visualization_msgs::Marker::ADD;

            arrow_marker.pose.position = sphere_marker.pose.position;

            // Convert yaw to quaternion
            double yaw = positions(2, i);  // Yaw is now in the third row
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            arrow_marker.pose.orientation.x = q.x();
            arrow_marker.pose.orientation.y = q.y();
            arrow_marker.pose.orientation.z = q.z();
            arrow_marker.pose.orientation.w = q.w();

            arrow_marker.scale.x = 0.2;   // Length of the arrow
            arrow_marker.scale.y = 0.02;  // Width of the arrow
            arrow_marker.scale.z = 0.02;  // Height of the arrow

            arrow_marker.color.r = 0.0;
            arrow_marker.color.g = 0.0;
            arrow_marker.color.b = 1.0;
            arrow_marker.color.a = 1.0;

            arrow_marker.lifetime = ros::Duration();

            marker_array.markers.push_back(arrow_marker);
        }

        publisher_map_[topic].publish(marker_array);
    }

    template <class TOPIC>
    inline void visWaypoints2D(const std::vector<Eigen::Vector2d> &waypoints, const TOPIC &topic, const Color color = blue, const double scale = 0.2) {
        auto got = publisher_map_.find(topic);
        if (got == publisher_map_.end()) {
            ros::Publisher pub = nh_.advertise<visualization_msgs::MarkerArray>(topic, 10000);
            publisher_map_[topic] = pub;
        }

        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.action = visualization_msgs::Marker::ADD;

        // First marker to clear previous markers
        marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(marker);
        marker.action = visualization_msgs::Marker::ADD;

        // Waypoint spheres
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.ns = "waypoint_spheres";
        setMarkerColor(marker, color);
        setMarkerScale(marker, scale, scale, scale);

        for (const auto &wp : waypoints) {
            geometry_msgs::Point pt;
            pt.x = wp.x();
            pt.y = wp.y();
            pt.z = 0.0;
            marker.points.push_back(pt);
        }
        marker_array.markers.push_back(marker);

        // Text markers for coordinates
        visualization_msgs::Marker text_marker;
        text_marker = marker;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.ns = "waypoint_labels";
        setMarkerColor(text_marker, white);
        setMarkerScale(text_marker, 0, 0, scale);

        for (size_t i = 0; i < waypoints.size(); ++i) {
            text_marker.id = i + 1;
            text_marker.pose.position.x = waypoints[i].x();
            text_marker.pose.position.y = waypoints[i].y();
            text_marker.pose.position.z = scale;  // Place text above the sphere

            std::stringstream ss;
            ss << std::fixed << std::setprecision(1) << "(" << waypoints[i].x() << ", " << waypoints[i].y() << ")";
            text_marker.text = ss.str();

            marker_array.markers.push_back(text_marker);
        }

        publisher_map_[topic].publish(marker_array);
    }

    template <int D>
    inline void visualizeTrajectoryStates(const Trajectory<D> &traj, double time_interval = 0.5, bool show_offset = true, double scale = 1.0, const Color color = lightyellow) {
        // Set up publisher for MarkerArray
        const std::string static_traj_topic = "static_trajectory_markers";
        auto got = publisher_map_.find(static_traj_topic);
        if (got == publisher_map_.end()) {
            ros::Publisher pub = nh_.advertise<visualization_msgs::MarkerArray>(static_traj_topic, 10000);
            publisher_map_[static_traj_topic] = pub;
            ros::Duration(0.1).sleep();
        }

        const double total_duration = traj.getTotalDuration();
        if (total_duration <= 0)
            return;

        visualization_msgs::MarkerArray marker_array;

        // Add deletion marker
        visualization_msgs::Marker delete_marker;
        delete_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(delete_marker);

        // // Base points of the shape with interpolated corner points
        // std::vector<std::pair<double, double>> base_points = {
        //     {0.0875, -0.6},
        //     // Add intermediate points around corners
        //     {0.0875, -0.59},  // Extra point near corner
        //     {0.0875, 0.59},   // Extra point near corner
        //     {0.0875, 0.6},
        //     {0.0875, 0.6},   // Duplicate corner point
        //     {-0.0025, 0.6},  // Extra point near corner
        //     {-0.0125, 0.6},
        //     {-0.0125, 0.6},   // Duplicate corner point
        //     {-0.0125, 0.55},  // Extra point near corner
        //     {-0.0125, 0.25},
        //     {-0.0125, 0.25},  // Duplicate corner point
        //     {-0.4325, 0.25},  // Extra point near corner
        //     {-0.4425, 0.25},
        //     {-0.4425, 0.25},   // Duplicate corner point
        //     {-0.4425, -0.24},  // Extra point near corner
        //     {-0.4425, -0.25},
        //     {-0.4425, -0.25},  // Duplicate corner point
        //     {-0.0225, -0.25},  // Extra point near corner
        //     {-0.0125, -0.25},
        //     {-0.0125, -0.25},  // Duplicate corner point
        //     {-0.0125, -0.59},  // Extra point near corner
        //     {-0.0125, -0.6},
        //     {-0.0125, -0.6},  // Duplicate corner point
        //     {0.0775, -0.6},   // Extra point near corner
        //     {0.0875, -0.6}};

        std::vector<std::pair<double, double>> base_points = {{0.2875, -0.6},   {0.2875, -0.59},  {0.2875, 0.59},   {0.2875, 0.6},   {0.2875, 0.6},   {0.1975, 0.6},   {0.1875, 0.6},
                                                              {0.1875, 0.6},    {0.1875, 0.55},   {0.1875, 0.25},   {0.1875, 0.25},  {-0.3325, 0.25}, {-0.3425, 0.25}, {-0.3425, 0.25},
                                                              {-0.3425, -0.24}, {-0.3425, -0.25}, {-0.3425, -0.25}, {0.1775, -0.25}, {0.1875, -0.25}, {0.1875, -0.25}, {0.1875, -0.59},
                                                              {0.1875, -0.6},   {0.1875, -0.6},   {0.2775, -0.6},   {0.2875, -0.6}};

        // Calculate number of states to show
        int num_states = static_cast<int>(total_duration / time_interval) + 1;

        for (int i = 0; i < num_states; i++) {
            double current_time = std::min(i * time_interval, total_duration);
            Eigen::Vector3d trans = traj.getPos(current_time);
            double total_yaw = trans(2);
            Eigen::Matrix3d Rt = Eigen::AngleAxisd(total_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            trans(2) = 0.0;

            // Create path marker
            visualization_msgs::Marker path_marker;
            path_marker.header.frame_id = "map";
            path_marker.header.stamp = ros::Time::now();
            path_marker.type = visualization_msgs::Marker::LINE_STRIP;
            path_marker.action = visualization_msgs::Marker::ADD;
            path_marker.id = i;
            path_marker.ns = "robot_outline";
            setMarkerColor(path_marker, color, 1);
            setMarkerScale(path_marker, 0.02 * scale, 0.0, 0.0);

            // Set orientation
            path_marker.pose.orientation.w = 1.0;
            path_marker.pose.orientation.x = 0.0;
            path_marker.pose.orientation.y = 0.0;
            path_marker.pose.orientation.z = 0.0;

            // Transform and add points
            for (const auto &bp : base_points) {
                Eigen::Vector3d pt(bp.first, bp.second, 0.0);
                Eigen::Vector3d transformed_pt = Rt * pt + trans;

                geometry_msgs::Point point;
                point.x = transformed_pt(0);
                point.y = transformed_pt(1);
                point.z = transformed_pt(2);
                path_marker.points.push_back(point);
            }

            // Add first point again to close the loop
            Eigen::Vector3d pt(base_points[0].first, base_points[0].second, 0.0);
            Eigen::Vector3d transformed_pt = Rt * pt + trans;
            geometry_msgs::Point point;
            point.x = transformed_pt(0);
            point.y = transformed_pt(1);
            point.z = transformed_pt(2);
            path_marker.points.push_back(point);

            marker_array.markers.push_back(path_marker);
        }

        // Publish markers
        publisher_map_[static_traj_topic].publish(marker_array);
    }

    // Helper function to clear visualization
    inline void clearTrajectoryStates() {
        const std::string static_traj_topic = "static_trajectory_markers";
        if (publisher_map_.find(static_traj_topic) != publisher_map_.end()) {
            visualization_msgs::MarkerArray marker_array;
            visualization_msgs::Marker delete_marker;
            delete_marker.action = visualization_msgs::Marker::DELETEALL;
            marker_array.markers.push_back(delete_marker);
            publisher_map_[static_traj_topic].publish(marker_array);
        }
    }

    // Debug function to print triangle connections
    inline void debugMeshTriangles(const Eigen::Matrix3Xd &mesh) {
        std::cout << "Analyzing mesh triangles:\n";

        for (int i = 0; i < mesh.cols(); i += 3) {
            if (i + 2 < mesh.cols()) {
                std::cout << "Triangle " << i / 3 << ":\n";
                std::cout << "  Vertex 1: (" << mesh(0, i) << ", " << mesh(1, i) << ", " << mesh(2, i) << ")\n";
                std::cout << "  Vertex 2: (" << mesh(0, i + 1) << ", " << mesh(1, i + 1) << ", " << mesh(2, i + 1) << ")\n";
                std::cout << "  Vertex 3: (" << mesh(0, i + 2) << ", " << mesh(1, i + 2) << ", " << mesh(2, i + 2) << ")\n\n";
            }
        }
    }

};  // namespace vis
}  // namespace vis
#endif
