//
// Created by airlab on 7/18/25.
//

#ifndef CROW_SERVER_SWATHGENERATORDISJOINT_H
#define CROW_SERVER_SWATHGENERATORDISJOINT_H
#include "../bt_headers.h"
class SwathGeneratorDisjoint : public SyncActionNode
{
public:
    SwathGeneratorDisjoint(const std::string& name, const NodeConfig& config)
            : SyncActionNode(name, config) {}

    static PortsList providedPorts()
    {
        return {
                InputPort<double>("robot_radius"),
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
        std::vector<wykobi::point2d<double>> polygon;
        for (const auto& pt : *roi_poly)
        {
            polygon.push_back(pt);
        }

        auto swath = mrcpp::swathlines::generateSwathLines(polygon, robot_radius);
        mrcpp::swathlines::clipSwathLinesToPolygon(polygon, swath);


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

#endif //CROW_SERVER_SWATHGENERATORDISJOINT_H
