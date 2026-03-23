// Created by redwan on 7/12/25.

#ifndef CROW_SERVER_ROUTEPLANNER_H
#define CROW_SERVER_ROUTEPLANNER_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include "wykobi/wykobi.hpp"
#include <queue>
#include <stack>
#include <limits>
#include <algorithm>
#include <memory>

// Hash for wykobi::point2d<double>
const double FIXED_COST = 1.0e6;
namespace std {
    template <>
    struct hash<wykobi::point2d<double>> {
        size_t operator()(const wykobi::point2d<double>& p) const {
            std::hash<long long> hasher;
            long long x = static_cast<long long>(p.x * 1e6);
            long long y = static_cast<long long>(p.y * 1e6);
            return hasher((x << 32) ^ y);
        }
    };

    template <>
    struct equal_to<wykobi::point2d<double>> {
        bool operator()(const wykobi::point2d<double>& a, const wykobi::point2d<double>& b) const {
            return std::abs(a.x - b.x) < 1e-6 && std::abs(a.y - b.y) < 1e-6;
        }
    };
}

namespace mrcpp {
    using point = wykobi::point2d<double>;

    class RoutePlanner {
        using node = std::pair<double, point>;

        // Custom comparator for min-heap
        struct CompareNode {
            bool operator()(const node& a, const node& b) const {
                return a.first > b.first; // Min-heap
            }
        };

    public:
        RoutePlanner() = default;

        void setBoundaryPolygon(const wykobi::polygon<double, 2>& boundary_polygon) {
            boundary_polygon_ = boundary_polygon;
            local_planner_ = std::make_shared<RoutePlanner>();
//            for(int i = 0; i < boundary_polygon.size(); ++i) {
//                int j = (i + 1) % boundary_polygon.size();
//                wykobi::segment<double, 2> edge = wykobi::make_segment(boundary_polygon[i], boundary_polygon[j]);
//                double cost = wykobi::distance(edge[0], edge[1]);
//                local_planner_->addEdge(edge[0], edge[1], cost);
//            }
            std::cout << "[RoutePlanner]: swath size: " << boundary_polygon.size();
        }

        void planRoute(const std::vector<wykobi::segment<double, 2>>& swath) {
           std::vector<wykobi::point2d<double>> top_points, bottom_points;
           for(const auto& segment : swath) {
               top_points.push_back(segment[0]);
               bottom_points.push_back(segment[1]);
           }
           std::reverse(bottom_points.begin(), bottom_points.end());
           // add all top points and bottom points to the local graph
           for(const auto& points: {top_points, bottom_points}) {
               for(int i = 1; i < points.size(); ++i) {
                   double cost = wykobi::distance(points[i-1], points[i]);
                   local_planner_->addEdge(points[i-1], points[i], cost);
               }
           }
           auto top_front = top_points.front();
           auto bottom_front = bottom_points.front();
           auto top_back = top_points.back();
           auto bottom_back = bottom_points.back();

          // add edges between top and bottom points
           local_planner_->addEdge(top_front, bottom_front, wykobi::distance(top_front, bottom_front));
           local_planner_->addEdge(top_back, bottom_back, wykobi::distance(top_back, bottom_back));
           std::cout << "[RoutePlanner]: swath size: " << swath.size();

        }

        void generateSweep(const std::vector<wykobi::segment<double, 2>>& swath)
        {
            int sweep_dir = 0;
            int index = 0;
            for (auto& line : swath) {
                double cost = wykobi::distance(line[0], line[1]);

                if(index > 0)
                {
                    if(sweep_dir % 2 == 0) {
                        // last point
                        wykobi::point2d<double> end_point = swath[index-1][0];
                        // Add start point
                        wykobi::point2d<double> start_point = line[0];
                        cost = wykobi::distance(end_point, start_point);
                        addEdge(end_point, start_point, cost);

                    } else if(boundary_polygon_.size() > 0) {

                        wykobi::point2d<double> start_point = line[1];
                        auto edge1 = wykobi::make_segment(swath[index-1][1], start_point);
                        auto edge2 = wykobi::make_segment(swath[index-1][0], start_point);
                        for(auto& edge : {edge1, edge2}) {
                            if(isValidEdge(edge, boundary_polygon_)) {
                                cost = FIXED_COST * wykobi::distance(edge[0], edge[1]);
                                addEdge(edge[0], edge[1], cost);
                            }
                            else{
                                findValidEdge(edge);
                            }
                        }

                    }
                }
                else
                {
                    addEdge(line[0], line[1], cost);
                }
                sweep_dir = 1 - sweep_dir; // Alternate direction for each line
                index++;
            }
        }



        void findValidEdge(const wykobi::segment<double, 2>& edge)
        {
            double cost;
            auto close_pt1 = closest_point_on_polygon(edge[0], boundary_polygon_);
            auto close_pt2 = closest_point_on_polygon(edge[1], boundary_polygon_);
            cost = FIXED_COST * wykobi::distance(edge[0], close_pt1);
            wykobi::segment<double, 2> edge_c1 = wykobi::make_segment(edge[0], close_pt1);
            wykobi::segment<double, 2> edge_c2 = wykobi::make_segment(edge[1], close_pt2);
            if(isValidEdge(edge_c1, boundary_polygon_))
                addEdge(edge[0], close_pt1, cost);
            cost = FIXED_COST * wykobi::distance(edge[1], close_pt2);
            if(isValidEdge(edge_c2, boundary_polygon_))
                addEdge(edge[1], close_pt2, cost);
        }

        wykobi::point2d<double> closest_point_on_polygon(const wykobi::point2d<double>& point,
                                                                const wykobi::polygon<double, 2>& polygon)
        {
            wykobi::point2d<double> closest_point(polygon[0]);
            double min_distance = wykobi::distance(point, closest_point);

            for (auto& vertex : polygon)
            {
                double distance = wykobi::distance(point, vertex);
                if (distance < min_distance)
                {
                    min_distance = distance;
                    closest_point = vertex;
                }
            }
            return closest_point;
        }



        bool isExistingEdge(const point& src, const point& dest) const {
            auto it = graph_.find(src);
            if (it != graph_.end()) {
                for (const auto& edge : it->second) {
                    if (edge.first == dest) {
                        return true;
                    }
                }
            }
            return false;
        }

        void addEdge(point src, point dest, double weight) {
            graph_[src].emplace_back(dest, weight);
            graph_[dest].emplace_back(src, weight); // Undirected
        }

        void removeEdge(point src, point dest) {
            auto& neighbors = graph_[src];
            neighbors.erase(std::remove_if(neighbors.begin(), neighbors.end(),
                                            [&dest](const std::pair<point, double>& edge) {
                                                return edge.first == dest;
                                            }), neighbors.end());
            graph_[dest].erase(std::remove_if(graph_[dest].begin(), graph_[dest].end(),
                                              [&src](const std::pair<point, double>& edge) {
                                                  return edge.first == src;
                                              }), graph_[dest].end());
        }

        std::vector<point> shortestPath(point src, point dest) {
            return Djikstra(src, dest);
        }

        std::vector<point> boundaryPath(point src, point dest) {
            return local_planner_->Djikstra(src, dest);
        }

        bool isValidEdge(const wykobi::segment<double, 2>& line) const
        {
            return isValidEdge(line, boundary_polygon_);
        }

    private:
        std::unordered_map<point, std::vector<std::pair<point, double>>> graph_;
        wykobi::polygon<double, 2> boundary_polygon_;
        std::shared_ptr<RoutePlanner> local_planner_;
    protected:
        std::vector<point> Djikstra(point src, point dest) {
            std::unordered_map<point, point> parent;
            std::unordered_map<point, double> cost;

            for (const auto& [node, _] : graph_) {
                cost[node] = std::numeric_limits<double>::infinity();
            }

            cost[src] = 0.0;
            std::priority_queue<node, std::vector<node>, CompareNode> queue;
            queue.push({0.0, src});

            while (!queue.empty()) {
                auto [current_cost, u] = queue.top();
                queue.pop();

                if (std::abs(u.x - dest.x) < 1e-6 && std::abs(u.y - dest.y) < 1e-6)
                    break;

                for (auto& [neighbor, weight] : graph_[u]) {
                    double new_cost = cost[u] + weight;
                    if (new_cost < cost[neighbor]) {
                        cost[neighbor] = new_cost;
                        parent[neighbor] = u;
                        queue.push({new_cost, neighbor});
                    }
                }
            }

            std::vector<point> path;
            if (parent.find(dest) == parent.end() && !(std::abs(src.x - dest.x) < 1e-6 && std::abs(src.y - dest.y) < 1e-6)) {
                std::cout << "No path found!" << std::endl;
                return {};
            }

            for (point p = dest; !(std::abs(p.x - src.x) < 1e-6 && std::abs(p.y - src.y) < 1e-6); p = parent[p]) {
                path.push_back(p);
            }
            path.push_back(src);
            std::reverse(path.begin(), path.end());

            for (const auto& p : path) {
                std::cout << p.x << ", " << p.y << std::endl;
            }

            return path;
        }
        std::vector<point> DFS(point src, point dest) {
            std::unordered_map<point, point> parent;
            std::unordered_set<point> visited;
            std::stack<point> stack;

            stack.push(src);

            while (!stack.empty()) {
                point current = stack.top();
                stack.pop();

                if (visited.find(current) != visited.end()) {
                    continue;
                }

                visited.insert(current);

                if (wykobi::is_equal(current, dest)) {
                    break;
                }

                for (auto& [neighbor, weight] : graph_[current]) {
                    if (visited.find(neighbor) == visited.end()) {
                        parent[neighbor] = current;
                        stack.push(neighbor);
                    }
                }
            }

            std::vector<point> path;
            if (parent.find(dest) == parent.end() && !wykobi::is_equal(src, dest)) {
                std::cout << "No path found!" << std::endl;
                return {};
            }

            point p = dest;
            while (!wykobi::is_equal(p, src)) {
                path.push_back(p);
                p = parent[p];
            }
            path.push_back(src);
            std::reverse(path.begin(), path.end());

            return path;
        }

        bool isValidEdge(const wykobi::segment<double, 2>& line,
                         const wykobi::polygon<double, 2>& collision_polygon) const
        {
            std::vector<wykobi::point2d<double>> intersection_list;
            for(int i = 0; i < collision_polygon.size(); ++i) {
                std::size_t j = (i + 1) % collision_polygon.size();
                wykobi::segment<double, 2> edge = wykobi::make_segment(collision_polygon[i], collision_polygon[j]);
                auto segment_list = std::vector<wykobi::segment<double, 2>>{edge, line};
                wykobi::algorithm::naive_group_intersections<wykobi::segment<double,2>>
                        (
                                segment_list.begin(),segment_list.end(),std::back_inserter(intersection_list)
                        );
            }
            return intersection_list.size() < 2; // If there are less than 2 intersection points, the edge is valid
        }
    };


} // namespace mrcpp
typedef std::shared_ptr<mrcpp::RoutePlanner> RoutePlannerPtr;
#endif // CROW_SERVER_ROUTEPLANNER_H
