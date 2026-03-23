//
// Created by airlab on 7/18/25.
//

#pragma once
#include "../bt_headers.h"
class OffsetPolygon : public SyncActionNode
{
public:
    OffsetPolygon(const std::string& name, const NodeConfig& config)
            : SyncActionNode(name, config) {}

    static PortsList providedPorts()
    {
        return {
                InputPort<double>("robot_radius"),
                InputPort<double>("headland_scale", 1.0, "Scale factor for headland buffer (0=none, 1=default)"),
                InputPort<SharedQueue<wykobi::point2d<double>>>("roi_poly"),
                OutputPort<SharedQueue<wykobi::point2d<double>>>("dilated_poly")
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

        double headland_scale = 1.0;
        getInput("headland_scale", headland_scale);

        double offset = robot_radius * headland_scale;
        DEBUG("[OffsetPolygon]: robot_radius=" << robot_radius << " headland_scale=" << headland_scale << " offset=" << offset);

        // If headland_scale is 0, skip offset entirely — use original polygon
        if(std::abs(offset) < 1e-10)
        {
            auto dilated_poly = std::make_shared<std::deque<wykobi::point2d<double>>>();
            std::copy(roi_poly->begin(), roi_poly->end(), std::back_inserter(*dilated_poly));
            setOutput<SharedQueue<wykobi::point2d<double>>>("dilated_poly", dilated_poly);
            return NodeStatus::SUCCESS;
        }

        auto offset_polygon = mrcpp::avoidance::offset_boundary_polygon(*roi_poly, offset);
//        if(wykobi::area(toPolygon(*roi_poly)) < wykobi::area(toPolygon(offset_polygon)))
//        {
//            offset_polygon = mrcpp::avoidance::offset_boundary_polygon(*roi_poly, -offset);
//        }
        auto outer_polygon = toPolygon(*roi_poly);
        auto inner_polygon = toPolygon(offset_polygon);
        if(!isPolygonInside(inner_polygon, outer_polygon))
        {
            std::cout << "[OffsetPolygon]: Inner polygon is not inside the outer polygon. Adjusting offset." << std::endl;
            offset_polygon = mrcpp::avoidance::offset_boundary_polygon(*roi_poly, -offset);
        }


        auto dilated_poly = std::make_shared<std::deque<wykobi::point2d<double>>>();
        std::copy(roi_poly->begin(), roi_poly->end(), std::back_inserter(*dilated_poly));
        roi_poly->clear();
        std::copy(offset_polygon.begin(), offset_polygon.end(), std::back_inserter(*roi_poly));
        setOutput<SharedQueue<wykobi::point2d<double>>>("dilated_poly", dilated_poly);
        return NodeStatus::SUCCESS;
    }

protected:
    template<class T>
    wykobi::polygon<double, 2> toPolygon(const T& points)
    {
        wykobi::polygon<double, 2> poly;
        for (const auto& point : points)
        {
            poly.push_back(wykobi::make_point(point.x, point.y));
        }
        return poly;
    }

    bool isPolygonInside(const wykobi::polygon<double, 2>& inner, const wykobi::polygon<double, 2>& outer) {
        if (inner.size() < 3 || outer.size() < 3) return false;

        for (const auto& point : inner) {
            if (!pointInPolygonRayCast(point, outer)) {
                return false; // If any point of the inner polygon is outside the outer polygon
            }
        }
        return true; // All points of the inner polygon are inside the outer polygon
    }


    // Method 1: Ray casting algorithm
    bool pointInPolygonRayCast(const wykobi::point2d<double>& point, const wykobi::polygon<double, 2>& polygon) {
        if (polygon.size() < 3) return false;

        bool inside = false;
        double x = point.x, y = point.y;

        for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
            double xi = polygon[i].x, yi = polygon[i].y;
            double xj = polygon[j].x, yj = polygon[j].y;

            if (((yi > y) != (yj > y)) &&
                (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) {
                inside = !inside;
            }
        }
        return inside;
    }

};
