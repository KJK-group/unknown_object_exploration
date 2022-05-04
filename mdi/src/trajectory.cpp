#include "mdi/trajectory.hpp"

namespace mdi {
Trajectory::Trajectory(ros::NodeHandle& nh, ros::Rate& rate, std::vector<Eigen::Vector3f> path, float marker_scale)
    : seq_marker(0), rate(rate) {
    pub_visualisation = nh.advertise<visualization_msgs::MarkerArray>("/mdi/visualisation", utils::DEFAULT_QUEUE_SIZE);

    // treat difference between each point in path as a vector
    vector<Eigen::Vector3f> vectors;
    std::adjacent_difference(path.begin(), path.end(), std::back_inserter(vectors));
    vectors.erase(vectors.begin());

    std::cout << "VECTORS: " << std::endl;
    for (auto& v : vectors) {
        std::cout << v << "\n" << std::endl;
    }

    // find angle betweeen each ordered pair of vectors
    vector<float> angles;
    std::transform(vectors.begin(), vectors.end() - 1, vectors.begin() + 1, std::back_inserter(angles),
                   [](auto v1, auto v2) { return (v1.dot(v2)) / (v1.norm() * v2.norm()); });

    std::cout << "ANGLES:" << std::endl;
    for (auto& a : angles) {
        std::cout << a << std::endl;
    }

    // find splitting indices for each section of the path
    // splitting one path point before the point where a!=180
    // splitting where the angle returns to a==180 after having been a!=180
    vector<int> splits = {0};
    auto straight = false;
    for (int i = 0; i < angles.size(); i++) {
        auto angle_before = i > 0 ? angles[i - 1] : 0;
        auto angle_current = angles[i];
        auto angle_after = i < angles.size() ? angles[i + 1] : 0;
        std::cout << "angle_before: " << angle_before << std::endl;
        std::cout << "angle_current: " << angle_current << std::endl;
        std::cout << "angle_before: " << angle_after << std::endl;
        if (std::abs(angle_current - 1) < std::numeric_limits<float>::epsilon()) {
            if (std::abs(angle_before - 1) > std::numeric_limits<float>::epsilon() ||
                std::abs(angle_after - 1) > std::numeric_limits<float>::epsilon()) {
                std::cout << "added idx: " << i << std::endl;
                splits.push_back(i + 1);
            }
        }
        std::cout << std::endl;
        // if (diff <= std::numeric_limits<float>::epsilon() && ! straight) {
        //     straight = true;
        // } else {
        //     if (straight) {
        //         splits.push_back(i);
        //     }
        //     straight = false;
        // }
    }
    // splits.push_back(angles.size());
    splits.push_back(path.size() - 1);

    std::cout << "SPLITS:" << std::endl;
    for (auto& s : splits) {
        std::cout << s << std::endl;
    }

    // uses each pair of splitting indices to slice the path into sections,
    // including points at starting and ending indices
    // vector<vector<Eigen::Vector3f>> sections;
    std::transform(splits.begin(), splits.end() - 1, splits.begin() + 1, std::back_inserter(sections),
                   [&](auto s1, auto s2) {
                       vector<Eigen::Vector3f> section;
                       for (s1; s1 <= s2; s1++) {
                           section.push_back(path[s1]);
                       }
                       return section;
                   });

    std::cout << "SECTIONS" << std::endl;
    for (auto& section : sections) {
        std::cout << "section" << std::endl;
        for (auto& s : section) {
            std::cout << s << std::endl;
        }
    }

    // create list of splines
    std::transform(sections.begin(), sections.end(), std::back_inserter(splines),
                   [](auto section) { return BezierSpline(section); });

    // generates distance LUT
    auto a = 0.f;
    for (auto& spline : splines) {
        auto length = spline.get_length();
        a += length;
        distance_lut.push_back(a);
    }

    visualise(marker_scale);
}

auto Trajectory::visualise(float scale) -> void {
    visualization_msgs::MarkerArray ma;
    for (int s = 0; s < splines.size(); s++) {
        float r = 255, g = 255, b = 255;
        std::tie(r, g, b) = utils::hsb_to_rgb((float)s / (float)splines.size() * 360.f, 80, 80);
        r /= 255;
        g /= 255;
        b /= 255;

        std::cout << "r: " << r << ", g: " << g << ", b: " << b << std::endl;

        visualization_msgs::Marker m;
        m.header.frame_id = utils::FRAME_WORLD;
        m.type = visualization_msgs::Marker::SPHERE;

        m.color.a = 1;
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        for (float d = 0; splines[s].get_length() - d > 0; d += 0.01) {
            m.header.seq = seq_marker++;
            m.header.stamp = ros::Time::now();
            m.id = seq_marker++;

            m.scale.x = scale * 0.5;
            m.scale.y = scale * 0.5;
            m.scale.z = scale * 0.5;

            auto point = splines[s].get_point_at_distance(d);
            m.pose.position.x = point.x();
            m.pose.position.y = point.y();
            m.pose.position.z = point.z();
            ma.markers.push_back(m);
        }
        for (int p = 0; p < sections[s].size(); p++) {
            m.header.seq = seq_marker++;
            m.header.stamp = ros::Time::now();
            m.id = seq_marker++;

            if (p == 0 || p == sections[s].size() - 1) {
                m.color.r = 0.8;
                m.color.g = 0.8;
                m.color.b = 0.8;
            } else {
                m.color.r = r;
                m.color.g = g;
                m.color.b = b;
            }
            m.scale.x = scale;
            m.scale.y = scale;
            m.scale.z = scale;

            auto point = sections[s][p];
            m.pose.position.x = point.x();
            m.pose.position.y = point.y();
            m.pose.position.z = point.z();
            ma.markers.push_back(m);
        }
    }

    ros::spinOnce();
    rate.sleep();
    ros::Duration(2).sleep();
    pub_visualisation.publish(ma);
    ros::spinOnce();
    rate.sleep();
}

auto Trajectory::get_length() -> float { return distance_lut.back(); }

auto Trajectory::get_point_at_distance(float distance) -> Eigen::Vector3f {
    assert(distance >= 0);
    int spline_idx = 0;
    for (spline_idx = 0; spline_idx < distance_lut.size(); spline_idx++) {
        if (distance < distance_lut[spline_idx]) {
            break;
        }
    }

    auto s = splines[spline_idx];
    auto distance_in_spline = s.get_length() - (distance - distance_lut[spline_idx - 1]);
    return s.get_point_at_distance(distance_in_spline);
}
}  // namespace mdi