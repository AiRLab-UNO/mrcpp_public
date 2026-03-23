//
// Created by airlab on 7/18/25.
//

#ifndef CROW_SERVER_DATALOADER_H
#define CROW_SERVER_DATALOADER_H
#include "../bt_headers.h"

class DataLoader : public SyncActionNode
{
public:
    DataLoader(const std::string& name, const NodeConfig& config)
            : SyncActionNode(name, config) {}

    static PortsList providedPorts()
    {
        return {
                InputPort<std::string>("json_str"),
                OutputPort<SharedQueue<wykobi::point2d<double>>>("roi_poly")
        };
    }

    NodeStatus tick() override
    {
        std::string json_str;
        if (!getInput("json_str", json_str))
        {
            throw RuntimeError("error reading port [json_str]: ");
        }

        json data = json::parse(json_str);
        std::vector<wykobi::point2d<double>> polygon;
        for (const auto& point : data["polygon"])
        {
            if (point.size() != 2)
            {
                throw RuntimeError("Invalid point format: " + point.dump());
            }
            auto p = wykobi::make_point(point[0].get<double>(), point[1].get<double>());
            polygon.push_back(p);
        }
        DEBUG("[DataLoader]: ROI points: " << polygon.size());
        SharedQueue<wykobi::point2d<double>> roi_poly = std::make_shared<std::deque<wykobi::point2d<double>>>();
        for (const auto& pt : polygon)
        {
            roi_poly->push_back(pt);
        }
        setOutput<SharedQueue<wykobi::point2d<double>>>("roi_poly", roi_poly);
        return NodeStatus::SUCCESS;
    }
};

#endif //CROW_SERVER_DATALOADER_H
