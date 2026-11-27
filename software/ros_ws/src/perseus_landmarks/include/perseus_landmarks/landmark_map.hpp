#pragma once

/// @file landmark_map.hpp
/// @brief The landmark map: known tags, named destinations and search points, all in the
///        site frame. Loaded from YAML (see config/landmarks.yaml for the format).

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "perseus_landmarks/pose2d.hpp"

namespace perseus_landmarks
{
    /// @brief A tag at a known place.
    struct TagLandmark
    {
        int id{0};
        /// @brief Edge of the tag's black square, metres.
        double size_m{0.16};
        /// @brief Tag centre in the site frame, metres.
        double x{0.0};
        double y{0.0};
        double z{0.0};
        /// @brief Direction the tag FACES (its outward normal) in the site frame, radians.
        ///        A tag on a wall facing east has yaw 0; facing north, pi/2.
        double yaw{0.0};
    };

    /// @brief A named place to drive to.
    struct Destination
    {
        std::string name;
        Pose2D pose;
        /// @brief A tag at the destination, used to re-localise on arrival.
        std::optional<int> tag;
    };

    /// @brief Where to look when a tag is not in view.
    struct SearchPoint
    {
        double x{0.0};
        double y{0.0};
        /// @brief "start": relative to the robot's pose when the search began (x forward,
        ///        y left). "site": in the site frame (needs a fix).
        std::string frame{"start"};
    };

    /// @brief See the file comment.
    struct LandmarkMap
    {
        std::string site_frame{"site"};
        std::string family{"36h11"};
        double default_tag_size_m{0.16};
        std::map<int, TagLandmark> tags;
        std::vector<Destination> destinations;
        std::vector<SearchPoint> search_points;

        /// @brief Size to use for a tag id: its own if known, else the default.
        double size_of(int id) const;
    };

    /// @brief Loads a landmark map. Throws std::runtime_error with a message naming the
    ///        offending entry on any malformed or duplicate entry.
    LandmarkMap load_landmark_map(const std::string& path);

    /// @brief Parses a landmark map from YAML text (load_landmark_map's worker; testable).
    LandmarkMap parse_landmark_map(const std::string& yaml_text);

}
