//
// Created by airlab on 7/18/25.
//

#ifndef CROW_SERVER_OBSTACLEAVOIDANCE_H
#define CROW_SERVER_OBSTACLEAVOIDANCE_H
#include "../bt_headers.h"
class ObstacleAvoidance : public SyncActionNode
{
public:
    ObstacleAvoidance(const std::string& name, const NodeConfig& config)
            : SyncActionNode(name, config){}

    static PortsList providedPorts()
    {
        return {
                InputPort<SharedQueue<wykobi::segment<double, 2>>>("line_segments"),
                InputPort<std::string>("json_str"),
                InputPort<double>("robot_radius"),
                InputPort<double>("interpolate_distance", 0.5, "Interpolation point spacing along obstacle edges"),
                OutputPort<SharedQueuePolygon>("obstacle_list")
        };
    }

    NodeStatus tick() override
    {
        std::string json_str;
        if (!getInput("json_str", json_str))
        {
            throw RuntimeError("error reading port [json_str]: ");
        }

        if(!getInput("robot_radius", robot_radius_))
        {
            throw RuntimeError("error reading port [robot_radius]: ");
        }

        if(!getInput("interpolate_distance", interpolate_distance_))
        {
            throw RuntimeError("error reading port [interpolate_distance]: ");
        }

        json data = json::parse(json_str);
        std::vector<wykobi::polygon<double, 2>> obstacle_list;
        for (const auto& obstacle : data["obstacles"]) {

            int type = obstacle[0].get<int>();
            switch (type) {
                case 1: // Circular obstacle
                    parse_circular_obstacle(obstacle, obstacle_list);
                    break;
                case 2: // Rectangular obstacle
                    parse_rectangular_obstacle(obstacle, obstacle_list);
                    break;
                case 3: // Polygonal obstacle
                    parse_polygonal_obstacle(obstacle, obstacle_list);
                    break;
                default:
                    std::cerr << "Unknown obstacle type: " << type << std::endl;
            }
        }

        auto line_segments = std::make_shared<std::deque<wykobi::segment<double, 2>>>();
        if (!getInput("line_segments", line_segments))
        {
            throw RuntimeError("error reading port [line_segments]: ");
        }
        std::vector<wykobi::segment<double, 2>> _segments;
        for (const auto& segment : *line_segments)
        {
            _segments.push_back(segment);
        }
        // obstacle_list already contains robot_radius-expanded polygons from
        // ServerRequest::convertObstacles() — clip swaths directly against them.
        mrcpp::avoidance::split_swath_lines(_segments, obstacle_list, _segments);

        // overwrite the line segments with the processed ones
        line_segments->clear();
        for (const auto& pt : _segments)
        {
            line_segments->push_back(pt);
        }
        // Output the pre-expanded polygons for PathPlanner's vis-graph.
        auto obstacle_list_ptr = std::make_shared<std::deque<wykobi::polygon<double, 2>>>();
        for (const auto& obs : obstacle_list)
        {
            obstacle_list_ptr->push_back(obs);
        }

        setOutput<SharedQueuePolygon>("obstacle_list", obstacle_list_ptr);

        DEBUG("[ObstacleAvoidance]: swaths size: " << line_segments->size());
        return NodeStatus::SUCCESS;
    }
private:
    double robot_radius_;
    double interpolate_distance_ = 0.5;
    void parse_circular_obstacle(const json& obstacle,  std::vector<wykobi::polygon<double, 2>>& obstacle_list)
    {
        double cx = obstacle[1].get<double>();
        double cy = obstacle[2].get<double>();
        double width = obstacle[3].get<double>();
        double height = obstacle[4].get<double>();

        double radius = sqrt(width * width + height * height); // Calculate the radius from width and height
        int n = static_cast<int>(std::ceil((radius / 3.0) * 16));
        n = std::max(n, 8); // Ensure a reasonable minimum number of segments
        double initial_angle = M_PI_4;
        double angle_step = 2 * M_PI / n;
        wykobi::polygon<double, 2> obs_points;
        for (int i = 0; i < n; ++i) {
            double angle = initial_angle + i * angle_step;
            double x = cx + width * cos(angle);
            double y = cy + height * sin(angle);
            auto obs_point = wykobi::make_point(x, y);
            obs_points.push_back(obs_point);
        }
        // Close the polygon if not already closed
        if(wykobi::distance(obs_points.front(), obs_points.back()) > 1e-6) {
            obs_points.push_back(obs_points.front());
        }

        obstacle_list.push_back(obs_points);
    }
    void parse_rectangular_obstacle(
            const nlohmann::json& obstacle,
            std::vector<wykobi::polygon<double, 2>>& obstacle_list)
    {
        using Point = wykobi::point2d<double>;
        using Polygon = wykobi::polygon<double, 2>;

        const double spacing = robot_radius_; // Robot radius or interpolation spacing

        double cx = obstacle[1].get<double>();
        double cy = obstacle[2].get<double>();
        double half_w = obstacle[3].get<double>();
        double half_h = obstacle[4].get<double>();


        // Define rectangle corners in clockwise order
        Point corners[4] = {
                wykobi::make_point(cx - half_w, cy + half_h),  // top-left
                wykobi::make_point(cx + half_w, cy + half_h),  // top-right
                wykobi::make_point(cx + half_w, cy - half_h),  // bottom-right
                wykobi::make_point(cx - half_w, cy - half_h)   // bottom-left
        };

        Polygon dense_polygon;

        for (int i = 0; i < 4; ++i) {
            Point start = corners[i];
            Point end = corners[(i + 1) % 4];
            // Interpolate points between start and end
            interpolate(start, end, interpolate_distance_, dense_polygon);
        }

        // Add final corner to close the polygon
        dense_polygon.push_back(dense_polygon.front());

        obstacle_list.push_back(dense_polygon);
    }

    void parse_polygonal_obstacle(const json& obstacle,  std::vector<wykobi::polygon<double, 2>>& obstacle_list) {
        // convert obstacle to std::vector<double>
//        DEBUG("[ObstacleAvoidance]: parsing polygonal obstacle: " << obstacle.dump());
        std::vector<double> obs_points_vec;
        for (size_t i = 1; i < obstacle.size(); ++i) {
            obs_points_vec.push_back(obstacle[i].get<double>());
        }
        wykobi::polygon<double, 2> obs_points;
        for (size_t i = 0; i < obs_points_vec.size(); i += 2)
        {
            auto obs_point = wykobi::make_point(obs_points_vec[i], obs_points_vec[i + 1]);
            obs_points.push_back(obs_point);
        }
        wykobi::polygon<double, 2> interpolated_points;
        for (int i = 0; i < obs_points.size(); ++i) {
            int j = (i + 1) % obs_points.size();
            const auto& start = obs_points[i];
            const auto& end = obs_points[j];
            // Interpolate points between start and end
            interpolate(start, end, interpolate_distance_, interpolated_points);
        }

        DEBUG("[ObstacleAvoidance]: polygonal obstacle points size: " << obs_points.size());
        obstacle_list.push_back(interpolated_points);
    }

    void interpolate(const wykobi::point2d<double>& start,
                     const wykobi::point2d<double>& end,
                     double spacing,
                     wykobi::polygon<double, 2>& dense_polygon)
    {
        double dx = end.x - start.x;
        double dy = end.y - start.y;
        double length = std::sqrt(dx * dx + dy * dy);

        int num_segments = std::max(1, static_cast<int>(std::ceil(length / spacing)));

        for (int j = 0; j < num_segments; ++j) {
            double t = static_cast<double>(j) / num_segments;
            double x = start.x + t * dx;
            double y = start.y + t * dy;
            dense_polygon.push_back(wykobi::make_point(x, y));
        }
    }

};
#endif //CROW_SERVER_OBSTACLEAVOIDANCE_H