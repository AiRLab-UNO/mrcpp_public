//
// Created by airlab on 7/18/25.
//

#ifndef CROW_SERVER_ROUTEGENERATOR_H
#define CROW_SERVER_ROUTEGENERATOR_H
#include "../bt_headers.h"

class RouteGenerator : public SyncActionNode
{
public:
    RouteGenerator(const std::string& name, const NodeConfig& config, const RoutePlannerPtr& planner)
            : SyncActionNode(name, config), planner_(planner) {}

    static PortsList providedPorts()
    {
        return {
                InputPort<SharedQueue<wykobi::segment<double, 2>>>("line_segments"),
                InputPort<SharedQueue<std::pair<int, int>>>("task_map")
        };
    }

    NodeStatus tick() override
    {
        auto line_segments = std::make_shared<std::deque<wykobi::segment<double, 2>>>();
        if (!getInput("line_segments", line_segments))
        {
            throw RuntimeError("error reading port [line_segments]: ");
        }

        auto task_map = std::make_shared<std::deque<std::pair<int, int>>>();
        if (!getInput("task_map", task_map))
        {
            throw RuntimeError("error reading port [task_map]: ");
        }

        std::unordered_map<int, int> solution;
        for (const auto& [key, value] : *task_map)
        {
            solution[key] = value;
//            DEBUG("[VizResults]: Task " << key << " assigned to robot " << value);
        }
        std::unordered_map<int, std::vector<int>> task_assignment;
        for (int i = 0; i < line_segments->size(); ++i) {
            task_assignment[solution[i]].push_back(i);
        }

        std::vector<wykobi::segment<double, 2>> swath;
        for (const auto& segment : *line_segments)
        {
            swath.push_back(segment);
        }
        DEBUG("[RouteGenerator]: swaths size: " << swath.size());


        for (auto& [robot_id, line_index_list] : task_assignment) {
            std::vector<wykobi::segment<double, 2>> local_swath;
            for(auto& line_index: line_index_list)
            {
                local_swath.push_back(swath[line_index]);
            }
            addEdges(local_swath);
        }
        return NodeStatus::SUCCESS;
    }
private:
    RoutePlannerPtr planner_;
protected:
    void addEdges(const std::vector<wykobi::segment<double, 2>>& local_swath)
    {
        int sweep_dir = 0;
        int index = 0;
        double cost = 0.0;
        for(auto& line : local_swath)
        {
            cost = wykobi::distance(line[0], line[1]);
            planner_->addEdge(line[0], line[1], cost);
            if(index > 0)
            {
                if(sweep_dir % 2 == 0) {
                    // last point
                    wykobi::point2d<double> end_point = local_swath[index-1][0];
                    // Add start point
                    wykobi::point2d<double> start_point = line[0];
                    cost = wykobi::distance(end_point, start_point);
                    planner_->addEdge(end_point, start_point, cost);

                } else {

                    wykobi::point2d<double> start_point = line[1];
                    auto edge1 = wykobi::make_segment(local_swath[index-1][1], start_point);
                    auto edge2 = wykobi::make_segment(local_swath[index-1][0], start_point);
                    for(auto& edge : {edge1, edge2}) {
                        cost = FIXED_COST * wykobi::distance(edge[0], edge[1]);
                        planner_->addEdge(edge[0], edge[1], cost);
                    }
                }
            }

            sweep_dir = 1 - sweep_dir; // Alternate direction for each line
            index++;
        }
    }
};

#endif //CROW_SERVER_ROUTEGENERATOR_H
