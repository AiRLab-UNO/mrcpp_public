//
// Created by airlab on 7/18/25.
//

#ifndef CROW_SERVER_CONVERTTOGPSPATH_H
#define CROW_SERVER_CONVERTTOGPSPATH_H
#include "../bt_headers.h"

class ConvertToGpsPath : public SyncActionNode
{
public:
    ConvertToGpsPath(const std::string& name, const NodeConfig& config)
            : SyncActionNode(name, config) {}

    static PortsList providedPorts()
    {
        return {
                InputPort<wykobi::point2d<double>>("origin"),
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

        wykobi::point2d<double> origin;
        if (!getInput("origin", origin))
        {
            throw RuntimeError("error reading port [origin]: ");
        }

        DEBUG("[ConvertToGpsPath] origin:= " << origin.x << ", " << origin.y);


        for (auto& point : *path)
        {
            mrcpp::gps_point p(point);
            p.toGps(origin.x, origin.y); // Convert to GPS coordinates
            point.y = p.longitude();
            point.x = p.latitude();
//            DEBUG("[ConvertToGpsPath]: gps points: " << point.x << ", " << point.y);

        }

        return NodeStatus::SUCCESS;
    }
};

#endif //CROW_SERVER_CONVERTTOGPSPATH_H
