#include "perseus_landmarks/landmark_map.hpp"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>

namespace perseus_landmarks
{
    double LandmarkMap::size_of(int id) const
    {
        const auto found = tags.find(id);
        return found != tags.end() ? found->second.size_m : default_tag_size_m;
    }

    namespace
    {
        std::vector<double> numbers(const YAML::Node& node, std::size_t count, const std::string& what)
        {
            if (!node || !node.IsSequence() || node.size() != count)
            {
                throw std::runtime_error(what + ": expected a list of " + std::to_string(count) + " numbers");
            }
            std::vector<double> out;
            for (const auto& item : node)
            {
                out.push_back(item.as<double>());
            }
            return out;
        }
    }

    LandmarkMap parse_landmark_map(const std::string& yaml_text)
    {
        LandmarkMap map;
        YAML::Node root;
        try
        {
            root = YAML::Load(yaml_text);
        }
        catch (const YAML::Exception& error)
        {
            throw std::runtime_error(std::string("landmark map is not valid YAML: ") + error.what());
        }
        if (!root || !root.IsMap())
        {
            throw std::runtime_error("landmark map must be a YAML mapping");
        }
        try
        {
            if (root["site_frame"])
            {
                map.site_frame = root["site_frame"].as<std::string>();
            }
            if (root["tag_family"])
            {
                map.family = root["tag_family"].as<std::string>();
            }
            if (root["default_tag_size_m"])
            {
                map.default_tag_size_m = root["default_tag_size_m"].as<double>();
            }
            if (root["tags"])
            {
                for (const auto& node : root["tags"])
                {
                    TagLandmark tag;
                    tag.id = node["id"].as<int>();
                    const std::string what = "tag " + std::to_string(tag.id);
                    tag.size_m = node["size_m"] ? node["size_m"].as<double>() : map.default_tag_size_m;
                    // pose: [x, y, z, yaw_deg] -- yaw in degrees: easier to measure and read.
                    const auto pose = numbers(node["pose"], 4, what + " pose [x, y, z, yaw_deg]");
                    tag.x = pose[0];
                    tag.y = pose[1];
                    tag.z = pose[2];
                    tag.yaw = wrap_angle(pose[3] * M_PI / 180.0);
                    if (tag.size_m <= 0.0)
                    {
                        throw std::runtime_error(what + ": size_m must be > 0");
                    }
                    if (!map.tags.emplace(tag.id, tag).second)
                    {
                        throw std::runtime_error(what + " is listed twice");
                    }
                }
            }
            std::set<std::string> names;
            if (root["destinations"])
            {
                for (const auto& node : root["destinations"])
                {
                    Destination destination;
                    destination.name = node["name"].as<std::string>();
                    const auto pose = numbers(node["pose"], 3, "destination '" + destination.name + "' pose [x, y, yaw_deg]");
                    destination.pose = {pose[0], pose[1], wrap_angle(pose[2] * M_PI / 180.0)};
                    if (node["tag"])
                    {
                        destination.tag = node["tag"].as<int>();
                        if (map.tags.find(*destination.tag) == map.tags.end())
                        {
                            throw std::runtime_error("destination '" + destination.name + "' refers to unknown tag " +
                                                     std::to_string(*destination.tag));
                        }
                    }
                    if (!names.insert(destination.name).second)
                    {
                        throw std::runtime_error("destination '" + destination.name + "' is listed twice");
                    }
                    map.destinations.push_back(destination);
                }
            }
            if (root["search_points"])
            {
                for (const auto& node : root["search_points"])
                {
                    SearchPoint point;
                    const auto xy = numbers(node["xy"], 2, "search point xy");
                    point.x = xy[0];
                    point.y = xy[1];
                    if (node["frame"])
                    {
                        point.frame = node["frame"].as<std::string>();
                    }
                    if (point.frame != "start" && point.frame != "site")
                    {
                        throw std::runtime_error("search point frame must be 'start' or 'site', got '" + point.frame + "'");
                    }
                    map.search_points.push_back(point);
                }
            }
        }
        catch (const YAML::Exception& error)
        {
            throw std::runtime_error(std::string("landmark map: ") + error.what());
        }
        return map;
    }

    LandmarkMap load_landmark_map(const std::string& path)
    {
        std::ifstream file(path);
        if (!file)
        {
            throw std::runtime_error("cannot open landmark map '" + path + "'");
        }
        std::stringstream text;
        text << file.rdbuf();
        return parse_landmark_map(text.str());
    }

}
