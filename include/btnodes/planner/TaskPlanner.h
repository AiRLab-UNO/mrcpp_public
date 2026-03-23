//
// Created by airlab on 7/18/25.
//

#ifndef CROW_SERVER_TASKPLANNER_H
#define CROW_SERVER_TASKPLANNER_H
#include "../bt_headers.h"
#include "mrcpp/utils/GpsPoint.h"
class TaskPlanner : public SyncActionNode
{
public:
    TaskPlanner(const std::string& name, const NodeConfig& config)
            : SyncActionNode(name, config) {}

    static PortsList providedPorts()
    {
        return {
                InputPort<SharedQueue<wykobi::segment<double, 2>>>("line_segments"),
                InputPort<int>("num_robots"),
                InputPort<wykobi::point2d<double>>("origin"),
                InputPort<wykobi::point2d<double>>("start_point_latlon"),
                OutputPort<SharedQueue<ASSIGNMENT>>("task_map")
        };
    }

    NodeStatus tick() override
    {
        auto line_segments = std::make_shared<std::deque<wykobi::segment<double, 2>>>();
        if (!getInput("line_segments", line_segments))
        {
            throw RuntimeError("error reading port [line_segments]: ");
        }


        std::vector<wykobi::segment<double, 2>> swath;
        for (const auto& segment : *line_segments)
        {
            swath.push_back(segment);
        }
        int num_robots;
        if (!getInput("num_robots", num_robots))
        {
            throw RuntimeError("error reading port [num_robots]: ");
        }

        // Get origin and start_point_latlon for depot location
        wykobi::point2d<double> origin;
        if (!getInput("origin", origin))
        {
            throw RuntimeError("error reading port [origin]: ");
        }

        wykobi::point2d<double> start_point_latlon;
        if (!getInput("start_point_latlon", start_point_latlon))
        {
            throw RuntimeError("error reading port [start_point_latlon]: ");
        }

        // Convert start_point from latlon to metric using the origin
        mrcpp::gps_point depot_gps(start_point_latlon.x, start_point_latlon.y);
        depot_gps.toCartesian(origin.x, origin.y);
        wykobi::point2d<double> depot_metric = wykobi::make_point(depot_gps.x, depot_gps.y);
        DEBUG("[TaskPlanner] Depot (metric): " << depot_metric.x << ", " << depot_metric.y);

        // solve MTSP with LKH3 library
        LKH::mTSP mtsp;
        // compute cost matrix with depot at start_point
        auto computeCostMatrix = [](const std::vector<wykobi::segment<double, 2>>& sweepLines,
                                    const wykobi::point2d<double>& depot){
            int n = sweepLines.size() + 1; // +1 for the depot
            const long COST_SCALE = 10000;
            std::vector<std::vector<uint64_t>> costMatrix(n, std::vector<uint64_t>(n, std::numeric_limits<uint64_t>::max()));
            // Depot self-distance
            costMatrix[0][0] = 0;
            // Depot to each sweep line: closest point on segment to depot
            for (size_t i = 1; i < n; ++i) {
                auto closest = wykobi::closest_point_on_segment_from_point(sweepLines[i - 1], depot);
                const double dist = wykobi::distance(depot, closest);
                costMatrix[0][i] = costMatrix[i][0] = COST_SCALE * dist;
            }
            // Inter-segment distances
            for (size_t i = 1; i < n; ++i) {
                costMatrix[i][i] = 0;
                for (size_t j = i + 1; j < n; ++j) {
                    const double distance = wykobi::distance(sweepLines[i - 1], sweepLines[j - 1]);
                    costMatrix[i][j] = costMatrix[j][i] = COST_SCALE * distance;
                }
            }
            return costMatrix;
        };

        auto cost = computeCostMatrix(swath, depot_metric);
        mtsp.setProblem(cost, num_robots);
        auto task_map = std::make_shared<std::deque<ASSIGNMENT>>();

        if (mtsp.solve()) {
            std::cout << "Solved!" << std::endl;
            std::map<int, std::deque<int>> result;
            auto tours = mtsp.getTours();
            for(int i = 0; i < tours.size(); ++i)
            {
                for(const auto& segment_id : tours[i])
                {
                    result[i + 1].push_back(segment_id - 2); // robot_id starts from 1
                }
            }

            for (const auto& [key, value] : result) {
                task_map->emplace_back(key, value);
            }
            setOutput<SharedQueue<ASSIGNMENT>>("task_map", task_map);
            DEBUG("[TaskPlanner]: task_map size: " << task_map->size());
            return NodeStatus::SUCCESS;
        }
        std::cout << "Failed to solve!" << std::endl;

        return NodeStatus::FAILURE;
    }
};

#endif //CROW_SERVER_TASKPLANNER_H
