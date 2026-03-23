//
// Created by airlab on 7/18/25.
//

#ifndef CROW_SERVER_SWATHGENERATOR_H
#define CROW_SERVER_SWATHGENERATOR_H
#include "../bt_headers.h"
class SwathGenerator : public SyncActionNode
{
public:
    SwathGenerator(const std::string& name, const NodeConfig& config)
            : SyncActionNode(name, config) {}

    static PortsList providedPorts()
    {
        return {
                InputPort<double>("robot_radius"),
                InputPort<std::string>("orientation_strategy", "mar", "Sweep orientation: mar, angle_search, pca, min_width"),
                InputPort<SharedQueue<wykobi::point2d<double>>>("roi_poly"),
                OutputPort<SharedQueue<wykobi::segment<double, 2>>>("line_segments")
        };
    }

    NodeStatus tick() override
    {

        SharedQueue<wykobi::point2d<double>> roi_poly = std::make_shared<std::deque<wykobi::point2d<double>>>();
        if (!getInput("roi_poly", roi_poly))
        {
            throw RuntimeError("error reading port [polygon]: ");
        }

        double robot_radius;
        if (!getInput("robot_radius", robot_radius))
        {
            throw RuntimeError("error reading port [robot_radius]: ");
        }

        std::string orientation_strategy = "mar";
        getInput("orientation_strategy", orientation_strategy);

        std::vector<wykobi::point2d<double>> polygon;
        wykobi::polygon<double, 2> bounding_polygon;
        for (const auto& pt : *roi_poly)
        {
            polygon.push_back(pt);
            bounding_polygon.push_back(pt);
        }
        DEBUG("[SwathGenerator]: orientation_strategy=" << orientation_strategy);
        auto swath = mrcpp::swathlines::generateSwathLines(polygon, robot_radius, orientation_strategy);

        mrcpp::swathlines::clipSwathLinesToPolygon(polygon, swath);
        //sort the swath lines based on their starting points
        std::sort(swath.begin(), swath.end(), [](const wykobi::segment<double, 2>& seg1, const wykobi::segment<double, 2>& seg2) {
            const auto& a = seg1[0];
            const auto& b = seg2[0];
            double dx = a.x - b.x;
            double dy = a.y - b.y;
            if(std::abs(dy) > std::abs(dx)) {
                return (a.y < b.y) || (a.y == b.y && a.x < b.x);
            }
            return (a.x < b.x) || (a.x == b.x && a.y < b.y);
        });


        DEBUG("[SwathGenerator]: swaths size: " << swath.size());
        auto line_segments = std::make_shared<std::deque<wykobi::segment<double, 2>>>();
        for (const auto& pt : swath)
        {
            line_segments->push_back(pt);
        }
        setOutput<SharedQueue<wykobi::segment<double, 2>>>("line_segments", line_segments);
        return NodeStatus::SUCCESS;
    }

};
#endif //CROW_SERVER_SWATHGENERATOR_H
