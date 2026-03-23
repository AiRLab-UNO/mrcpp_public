//
// Created by airlab on 7/18/25.
//

#ifndef CROW_SERVER_SERVERREQUEST_H
#define CROW_SERVER_SERVERREQUEST_H
#include "../bt_headers.h"
class ServerRequest: public SyncActionNode
{
public:
    ServerRequest(const std::string& name, const NodeConfig& config, const json& data)
            : SyncActionNode(name, config), data_(data) {}

    static PortsList providedPorts()
    {
        return {
                OutputPort<int>("num_robots"),
                OutputPort<double>("robot_radius"),
                OutputPort<std::string>("json_str")
        };
    }

    NodeStatus tick() override
    {
        convertObstacles(); // Convert obstacles to Cartesian coordinates
        setOutput("json_str", data_.dump());
        setOutput("num_robots", data_["num_robots"].get<int>());
        double robot_radius = data_["robot_radius"].get<double>();
        setOutput("robot_radius", robot_radius);
        return NodeStatus::SUCCESS;
    }
private:
    json data_;

    std::pair<double, double> getOrigin(const json& data)
    {
        double ref_lat, ref_lon;
        ref_lat = ref_lon = std::numeric_limits<double>::max();
        for(const auto& coord : data["polygon"])
        {
            double lat = coord[0];
            double lon = coord[1];
            ref_lat = std::min(ref_lat, lat);
            ref_lon = std::min(ref_lon, lon);
        }

        return {ref_lat, ref_lon};
    }

    void convertObstacles()
    {
        if(!data_.contains("obstacles") || data_["obstacles"].empty())
        {
            data_["obstacles"] = json::array();
            return;
        }

        // Check if obstacles are already in flat format [type_id, cx, cy, w, h, ...]
        // (first element of first obstacle is a number, not a string key)
        auto& first_obs = data_["obstacles"][0];
        if(first_obs.is_array() && !first_obs.empty() && first_obs[0].is_number())
        {
            // Already in flat format — skip conversion
            DEBUG("[ServerRequest]: Obstacles already in flat format, skipping conversion");
            return;
        }

        auto origin = getOrigin(data_);

        json flat_obstacles = json::array();
        double robot_radius = data_["robot_radius"].get<double>();

        for (auto& obstacle : data_["obstacles"])
        {
            if (obstacle["type"] == "circle")
            {
                auto center = obstacle["center"];
                mrcpp::gps_point p(center[0], center[1]);
                p.toCartesian(origin.first, origin.second);
                double obs_radius = obstacle["radius"];
                obs_radius +=  robot_radius; // Add robot radius to obstacle radius
                flat_obstacles.push_back({1, p.x, p.y, obs_radius, obs_radius});
            }
            else if(obstacle["type"] == "rectangle")
            {
                auto bounds = obstacle["bounds"];
                auto top_left = bounds[0];
                auto bottom_right = bounds[1];
                mrcpp::gps_point p1(top_left[0], top_left[1]);
                mrcpp::gps_point p2(bottom_right[0], bottom_right[1]);
                p1.toCartesian(origin.first, origin.second);
                p2.toCartesian(origin.first, origin.second);
                double width = (p2.x - p1.x) / 2.0 +  robot_radius; // Add robot radius to width
                double height = (p2.y - p1.y) / 2.0 + robot_radius; // Add robot radius to height
                double center_x = (p1.x + p2.x) / 2.0;
                double center_y = (p1.y + p2.y) / 2.0;

                flat_obstacles.push_back({2, center_x, center_y, width, height});
            }
            else if(obstacle["type"] == "polygon")
            {
                std::vector<double> obs_points_vec;
                wykobi::polygon<double, 2> obs_polygon;
                obs_points_vec.push_back(3);  // Type ID for polygon
                for (const auto& point : obstacle["points"])
                {
                    mrcpp::gps_point p(point[0], point[1]);
                    p.toCartesian(origin.first, origin.second);
                    obs_polygon.push_back(wykobi::make_point(p.x, p.y));
                }
                auto offset_obs = mrcpp::avoidance::offset_boundary_polygon(obs_polygon, 2*robot_radius);
                if(wykobi::area(offset_obs) < wykobi::area(obs_polygon))
                {
                    offset_obs = mrcpp::avoidance::offset_boundary_polygon(obs_polygon, -2*robot_radius);
                }

                for(const auto& point : offset_obs)
                {
                    obs_points_vec.push_back(point.x);
                    obs_points_vec.push_back(point.y);
                }
                flat_obstacles.push_back(obs_points_vec);
            }
        }

        data_["obstacles"] = flat_obstacles;

        DEBUG("[ServerRequest]: \n" << data_["obstacles"].dump(2));
    }
};

#endif //CROW_SERVER_SERVERREQUEST_H
