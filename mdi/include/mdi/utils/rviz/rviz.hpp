#ifndef _MULTI_DRONE_INSPECTION_RVIZ_HPP_
#define _MULTI_DRONE_INSPECTION_RVIZ_HPP_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cassert>
#include <chrono>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <string>
#include <string_view>

#include "mdi/utils/time.hpp"
#include "visualization_msgs/MarkerArray.h"

namespace mdi::utils::rviz {

struct RGBA {
    float r, g, b = 0.f, a = 1.f;
};

struct Scale {
    float x = 1.f, y = 1.f, z = 1.f;
};

struct Header {
    unsigned long long id = 0;
    std::string ns = "";
    std::string frame_id = "map";
};

enum class MarkerType {
    SPHERE,
    ARROW,
    CUBE,
};

struct visualization_marker_msg_gen {
    RGBA color{0, 1, 0, 1};
    Scale scale{0.1, 0.1, 0.1};
    Header header{};
    bool auto_incrementing_id = true;
    bool use_namespace_id_suffix = true;

    // visualization_marker_msg_gen() = delete;
    auto delete_all_markers_msg() -> visualization_msgs::Marker {
        // need to make a copy of the marker msg object to not modify the existing marker.
        auto delete_all_markers_msg = visualization_msgs::Marker{};
        msg.ns = header.ns;
        delete_all_markers_msg.action = visualization_msgs::Marker::DELETEALL;
        return delete_all_markers_msg;
    }

    //    private:
    virtual ~visualization_marker_msg_gen() {}

   protected:
    visualization_msgs::Marker msg{};
    unsigned long long id = header.id;
    auto set_fields() -> void {
        msg.ns = header.ns;
        msg.header.frame_id = header.frame_id;
        msg.id = get_id();
        msg.color.r = color.r;
        msg.color.g = color.g;
        msg.color.b = color.b;
        msg.color.a = color.a;
        msg.scale.x = scale.x;
        msg.scale.y = scale.y;
        msg.scale.z = scale.z;
    }
    auto get_id() -> decltype(id) { return auto_incrementing_id ? ++id : id; }
    auto append_epoch_suffix_if_enabled(const std::string& ns) -> std::string {
        const auto namespace_id_suffix = mdi::utils::time::get_seconds_since_epoch();
        return ns + (use_namespace_id_suffix ? "/" + std::to_string(namespace_id_suffix) : "");
    }
};

struct arrow_msg_gen : public visualization_marker_msg_gen {
    class Builder;

    struct Arrow {
        Eigen::Vector3f start, end;
    };

    using Arrows = std::vector<Arrow>;

    static auto builder() -> Builder;

    arrow_msg_gen(const std::string& ns = "arrow") {
        msg.points.resize(2);
        msg.type = visualization_msgs::Marker::ARROW;
        header.ns = append_epoch_suffix_if_enabled(ns);
    }

    auto operator()(Arrow arrow, ros::Time timestamp = ros::Time::now(), ros::Duration lifetime = ros::Duration(0))
        -> visualization_msgs::Marker {
        set_fields();
        msg.header.stamp = timestamp;
        msg.lifetime = lifetime;
        geometry_msgs::Pose pose{};
        pose.orientation.w = 1.0f;
        msg.pose = pose;

        msg.points[0] = vec3_to_geometry_msg_point_(arrow.start);
        msg.points[1] = vec3_to_geometry_msg_point_(arrow.end);

        return msg;
    }

    auto operator()(std::vector<Arrow> arrows, ros::Time timestamp = ros::Time::now(),
                    ros::Duration lifetime = ros::Duration(0)) -> visualization_msgs::MarkerArray {
        auto markerarray = visualization_msgs::MarkerArray();
        std::transform(arrows.cbegin(), arrows.cend(), markerarray.markers.begin(),
                       [this](const auto& arrow) { return this->operator()(arrow); });

        return markerarray;
    }

   private:
    auto vec3_to_geometry_msg_point_(const Eigen::Vector3f& v) -> geometry_msgs::Point {
        auto p = geometry_msgs::Point{};
        p.x = v.x();
        p.y = v.y();
        p.z = v.z();
        return p;
    }
};

class arrow_msg_gen::Builder {
   public:
    Builder& color() { return *this; }
    Builder& arrow_length(const float length) {
        gen.scale.z = length;
        return *this;
    }
    Builder& arrow_width(const float width) {
        gen.scale.x = width;
        return *this;
    }

    Builder& arrow_head_width(const float width) {
        gen.scale.y = width;
        return *this;
    }

    Builder& color(const RGBA& color) {
        gen.color = color;
        return *this;
    }
    Builder& color(float red, float green, float blue, float alpha) {
        gen.color.r = red;
        gen.color.g = green;
        gen.color.b = blue;
        gen.color.a = alpha;
        return *this;
    }

    auto build() -> arrow_msg_gen { return std::move(gen); }

   private:
    arrow_msg_gen gen{};
};

auto arrow_msg_gen::builder() -> Builder { return Builder(); }

struct sphere_msg_gen : public visualization_marker_msg_gen {
    sphere_msg_gen(const std::string& ns = "sphere") {
        msg.type = visualization_msgs::Marker::SPHERE;
        header.ns = append_epoch_suffix_if_enabled(ns);
    }

    auto operator()(geometry_msgs::Pose pose, ros::Time timestamp = ros::Time::now(),
                    ros::Duration lifetime = ros::Duration(0)) -> visualization_msgs::Marker {
        set_fields();
        msg.header.stamp = timestamp;
        msg.lifetime = lifetime;
        msg.pose = pose;

        return msg;
    }

    auto operator()(const Eigen::Vector3f v, ros::Time timestamp = ros::Time::now(),
                    ros::Duration lifetime = ros::Duration(0)) -> visualization_msgs::Marker {
        // set_fields();
        // msg.header.stamp = timestamp;
        // msg.lifetime = lifetime;
        auto pose = geometry_msgs::Pose();
        pose.position.x = v.x();
        pose.position.y = v.y();
        pose.position.z = v.z();
        return this->operator()(pose, timestamp, lifetime);
    }
};

struct cube_msg_gen : public visualization_marker_msg_gen {
    cube_msg_gen(float x, float y, float z, const std::string& ns = "cube") {
        msg.type = visualization_msgs::Marker::CUBE;
        header.ns = append_epoch_suffix_if_enabled(ns);
        scale.x = x;
        scale.y = y;
        scale.z = z;
    }

    cube_msg_gen(float r, const std::string& ns = "cube") {
        msg.type = visualization_msgs::Marker::CUBE;
        header.ns = append_epoch_suffix_if_enabled(ns);
        scale.x = r;
        scale.y = r;
        scale.z = r;
    }

    auto operator()(geometry_msgs::Pose pose, ros::Time timestamp = ros::Time::now(),
                    ros::Duration lifetime = ros::Duration(0)) -> visualization_msgs::Marker {
        set_fields();
        msg.header.stamp = timestamp;
        msg.lifetime = lifetime;
        msg.pose = pose;

        return msg;
    }

    auto operator()(const Eigen::Vector3f v, ros::Time timestamp = ros::Time::now(),
                    ros::Duration lifetime = ros::Duration(0)) -> visualization_msgs::Marker {
        // set_fields();
        // msg.header.stamp = timestamp;
        // msg.lifetime = lifetime;
        auto pose = geometry_msgs::Pose();
        pose.position.x = v.x();
        pose.position.y = v.y();
        pose.position.z = v.z();
        return this->operator()(pose, timestamp, lifetime);
    }
};

struct text_msg_gen : public visualization_marker_msg_gen {
    text_msg_gen(float text_height = 0.5f, const std::string& ns = "text") {
        msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        header.ns = append_epoch_suffix_if_enabled(ns);
        scale.z = text_height;
    }
    auto operator()(std::string_view text, geometry_msgs::Pose pose, ros::Time timestamp = ros::Time::now(),
                    ros::Duration lifetime = ros::Duration(0)) -> visualization_msgs::Marker {
        set_fields();
        msg.header.stamp = timestamp;
        msg.lifetime = lifetime;

        msg.pose = pose;
        msg.text = text;

        return msg;
    }

    auto operator()(std::string_view text, const Eigen::Vector3f v, ros::Time timestamp = ros::Time::now(),
                    ros::Duration lifetime = ros::Duration(0)) -> visualization_msgs::Marker {
        // set_fields();
        // msg.header.stamp = timestamp;
        // msg.lifetime = lifetime;
        auto pose = geometry_msgs::Pose();
        pose.position.x = v.x();
        pose.position.y = v.y();
        pose.position.z = v.z();
        return this->operator()(text, pose, timestamp, lifetime);
    }
};

}  // namespace mdi::utils::rviz

#endif  // _MULTI_DRONE_INSPECTION_RVIZ_HPP_
