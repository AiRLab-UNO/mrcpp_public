//
// Created by airlab on 7/18/25.
//

#ifndef CROW_SERVER_GPSDECODER_H
#define CROW_SERVER_GPSDECODER_H
#include "../bt_headers.h"
class GpsDecoder : public SyncActionNode
{
public:
    GpsDecoder(const std::string& name, const NodeConfig& config)
            : SyncActionNode(name, config) {}

    static PortsList providedPorts()
    {
        return {
                OutputPort<wykobi::point2d<double>>("origin"),
                InputPort<SharedQueue<wykobi::point2d<double>>>("roi_poly")

        };
    }

    NodeStatus tick() override
    {
        SharedQueue<wykobi::point2d<double>> roi_poly = std::make_shared<std::deque<wykobi::point2d<double>>>();
        if (!getInput("roi_poly", roi_poly))
        {
            throw RuntimeError("error reading port [polygon]: ");
        }

        double lat_ref, lon_ref;
        lat_ref = lon_ref = std::numeric_limits<double>::max();
        for (const auto& point : *roi_poly){
            double lat = point.x;
            double lng = point.y;
            lat_ref = std::min(lat_ref, lat);
            lon_ref = std::min(lon_ref, lng);
        }
        for (auto& point : *roi_poly)
        {
            mrcpp::gps_point p(point.x, point.y);
            p.toCartesian(lat_ref, lon_ref);
            point.x = p.x;
            point.y = p.y;
        }
        DEBUG("[GpsDecoder]: ROI points: " << roi_poly->size());
        setOutput<wykobi::point2d<double>>("origin", wykobi::make_point(lat_ref, lon_ref));
        return NodeStatus::SUCCESS;
    }
};

#endif //CROW_SERVER_GPSDECODER_H
