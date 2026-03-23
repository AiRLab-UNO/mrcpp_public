//
// Created by airlab on 7/18/25.
//

#ifndef CROW_SERVER_PATHACCUMULATOR_H
#define CROW_SERVER_PATHACCUMULATOR_H
#include "../bt_headers.h"
class PathAccumulator : public SyncActionNode
{
public:
    PathAccumulator(const std::string& name, const NodeConfig& config, SharedQueuePolygon accumulated_paths)
            : SyncActionNode(name, config), accumulated_paths_(accumulated_paths) {}

    static PortsList providedPorts()
    {
        return {
                InputPort<SharedQueue<wykobi::point2d<double>>>("path")
        };
    }

    NodeStatus tick() override
    {
        SharedQueue<wykobi::point2d<double>> path = std::make_shared<std::deque<wykobi::point2d<double>>>();
        if (!getInput("path", path))
        {
            throw RuntimeError("error reading port [path]: ");
        }
        if (path->empty())
        {
            DEBUG("[PathAccumulator]: No path to accumulate.");
            return NodeStatus::FAILURE;
        }
        wykobi::polygon<double, 2> path_polygon;
        for (const auto& point : *path)
        {
            path_polygon.push_back(point);
        }
        accumulated_paths_->push_back(path_polygon);

        return NodeStatus::SUCCESS;
    }
private:
    SharedQueuePolygon accumulated_paths_;
};

#endif //CROW_SERVER_PATHACCUMULATOR_H
