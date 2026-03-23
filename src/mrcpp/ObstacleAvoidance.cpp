//
// Created by redwan on 7/12/25.
//
//
// Created by airlab on 7/10/25.
//
#include <optional>
#include "wykobi/wykobi.hpp"
#include "wykobi/wykobi_algorithm.hpp"
#include "mrcpp/RoutePlanner.h"

namespace mrcpp::avoidance{
    inline wykobi::point2d<double> closest_point_on_polygon(const wykobi::point2d<double>& point,
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
    /**
     * Splits swath lines by obstacles using ray casting algorithm.
     * @param swath_lines The input swath lines to be split.
     * @param obstacles The list of polygons representing obstacles.
     * @param split_lines The mrcpp_with_boxes vector to store the split lines.
     */
    inline void split_swath_lines(
            const std::vector<wykobi::segment<double, 2>>& swath_lines,
            const std::vector<wykobi::polygon<double, 2>>& obstacles,
            RoutePlannerPtr route_planner
    )
    {

        for(auto & line : swath_lines) {

            std::vector<wykobi::point2d<double>> segment_points{line[0]};
            for(const auto& collision_polygon : obstacles)
            {
                std::vector<wykobi::point2d<double>> intersection_list;
                auto local_planner(std::make_shared<RoutePlanner>());
                for(int i = 0; i < collision_polygon.size(); ++i) {
                    std::size_t j = (i + 1) % collision_polygon.size();
                    wykobi::segment<double, 2> edge = wykobi::make_segment(collision_polygon[i], collision_polygon[j]);
                    auto segment_list = std::vector<wykobi::segment<double, 2>>{edge, line};
                    wykobi::algorithm::naive_group_intersections<wykobi::segment<double,2>>
                            (
                                    segment_list.begin(),segment_list.end(),std::back_inserter(intersection_list)
                            );
                    double cost = wykobi::distance(edge[0], edge[1]);
                    local_planner->addEdge(edge[0], edge[1], cost);
                }

                if(intersection_list.empty()) {
                    continue; // no intersection with this polygon
                }



                for(auto &pt : intersection_list) {
                    auto closet_pt = closest_point_on_polygon(pt, collision_polygon);
                    auto cost = wykobi::distance(pt, closet_pt);
                    local_planner->addEdge(pt, closet_pt, cost);
                }

                for(int i = 0; i < intersection_list.size(); i+=2) {
                    std::size_t j = (i + 1) % intersection_list.size();
                    auto route = local_planner->shortestPath(intersection_list[i], intersection_list[j]);
                    for(int j = 0; j < route.size() - 1; ++j) {
                        auto segment = wykobi::make_segment(route[j], route[j + 1]);
                        auto cost = wykobi::distance(route[j], route[j + 1]);
                        route_planner->addEdge(route[j], route[j + 1], cost);
                    }
                }

                route_planner->removeEdge(line[0], line[1]); // remove line from graph
                // sort intersection points
                std::sort(intersection_list.begin(), intersection_list.end(), [](const wykobi::point2d<double>& a, const wykobi::point2d<double>& b) {
                    double dx = a.x - b.x;
                    double dy = a.y - b.y;
                    if(std::abs(dy) > std::abs(dx)) {
                        return (a.y < b.y) || (a.y == b.y && a.x < b.x);
                    }
                    return (a.x < b.x) || (a.x == b.x && a.y < b.y);
                });
                std::copy(intersection_list.begin(), intersection_list.end(), std::back_inserter(segment_points));
            }

            segment_points.push_back(line[1]);
            for(size_t i = 0; i < segment_points.size(); i += 2) {
                auto segment = wykobi::make_segment(segment_points[i], segment_points[i + 1]);
                auto cost = wykobi::distance(segment_points[i], segment_points[i + 1]);
                route_planner->addEdge(segment_points[i], segment_points[i + 1], cost);
            }

        }

    }

    // Ray-casting point-in-polygon test.
    inline bool point_in_obstacle(const wykobi::point2d<double>& pt,
                                  const wykobi::polygon<double, 2>& poly)
    {
        int cnt = 0;
        const size_t n = poly.size();
        for(size_t i = 0; i < n; ++i) {
            const auto& p1 = poly[i];
            const auto& p2 = poly[(i + 1) % n];
            double y1 = p1.y, y2 = p2.y;
            if((y1 <= pt.y && pt.y < y2) || (y2 <= pt.y && pt.y < y1)) {
                double xi = p1.x + (pt.y - y1) * (p2.x - p1.x) / (y2 - y1);
                if(pt.x < xi) ++cnt;
            }
        }
        return (cnt % 2) == 1;
    }

    inline void split_swath_lines(
            const std::vector<wykobi::segment<double, 2>>& swath_lines,
            const std::vector<wykobi::polygon<double, 2>>& obstacles,
            std::vector<wykobi::segment<double, 2>>& split_lines
    )
    {
        std::vector<wykobi::segment<double, 2>> result_lines;
        for(auto & line : swath_lines) {

            // Collect ALL intersection points from ALL obstacles in one pass.
            std::vector<wykobi::point2d<double>> all_intersections;
            for(const auto& collision_polygon : obstacles) {
                for(int i = 0; i < (int)collision_polygon.size(); ++i) {
                    std::size_t j = (i + 1) % collision_polygon.size();
                    wykobi::segment<double, 2> edge = wykobi::make_segment(collision_polygon[i], collision_polygon[j]);
                    auto segment_list = std::vector<wykobi::segment<double, 2>>{edge, line};
                    wykobi::algorithm::naive_group_intersections<wykobi::segment<double,2>>(
                            segment_list.begin(), segment_list.end(),
                            std::back_inserter(all_intersections));
                }
            }

            // Sort ALL intersections by distance from line[0] so they follow
            // the swath direction regardless of which axis it travels along.
            const auto& origin = line[0];
            std::sort(all_intersections.begin(), all_intersections.end(),
                [&origin](const wykobi::point2d<double>& a, const wykobi::point2d<double>& b) {
                    double da2 = (a.x - origin.x) * (a.x - origin.x)
                               + (a.y - origin.y) * (a.y - origin.y);
                    double db2 = (b.x - origin.x) * (b.x - origin.x)
                               + (b.y - origin.y) * (b.y - origin.y);
                    return da2 < db2;
                });

            // Remove near-duplicate intersection points (same edge corner hit twice).
            all_intersections.erase(
                std::unique(all_intersections.begin(), all_intersections.end(),
                    [](const wykobi::point2d<double>& a, const wykobi::point2d<double>& b) {
                        return std::abs(a.x - b.x) < 1e-9 && std::abs(a.y - b.y) < 1e-9;
                    }),
                all_intersections.end());

            if(all_intersections.empty()) {
                // No crossings: keep the segment only when its start is outside all obstacles.
                bool inside = false;
                for(const auto& poly : obstacles)
                    if(point_in_obstacle(line[0], poly)) { inside = true; break; }
                if(!inside)
                    result_lines.push_back(line);
                continue;
            }

            // Build ordered boundary-point list along the swath.
            std::vector<wykobi::point2d<double>> pts;
            pts.push_back(line[0]);
            for(const auto& pt : all_intersections)
                pts.push_back(pt);
            pts.push_back(line[1]);

            // Determine whether line[0] starts inside any obstacle.
            // If inside, the first segment (pts[0]→pts[1]) is the inside-obstacle chord
            // and must be skipped; emit starting from pts[1] instead.
            bool first_inside = false;
            for(const auto& poly : obstacles)
                if(point_in_obstacle(line[0], poly)) { first_inside = true; break; }

            const size_t start_idx = first_inside ? 1 : 0;
            for(size_t i = start_idx; i + 1 < pts.size(); i += 2)
                result_lines.push_back(wykobi::make_segment(pts[i], pts[i + 1]));
        }

        split_lines.clear();
        std::copy(result_lines.begin(), result_lines.end(), std::back_inserter(split_lines));

    }

    // Offset a polygon by a given distance using the pointwise Minkowski
    // sum/difference with a disk of radius |offset_distance|.
    //
    // Algorithm (edge-offset intersection / miter join):
    //   1. For each polygon edge, compute a parallel offset line shifted by
    //      offset_distance along the edge's left normal (-dy, dx).
    //   2. For each vertex (junction of two consecutive offset edges), compute
    //      the intersection of the two offset lines. This gives the correct
    //      Minkowski offset vertex for BOTH convex and reflex vertices.
    //
    // The old vertex-normal averaging method fails at reflex vertices because
    // the average of two opposing normals can point outward. The edge-offset
    // intersection method avoids this entirely — it computes the geometric
    // intersection of the two shifted edge lines, which is always correct.
    //
    // Miter capping: at very sharp angles the miter point can be extremely
    // far from the original vertex. We cap the miter distance at
    // 4 * |offset_distance| to prevent degenerate spikes.
    template<class T>
    inline T offset_boundary_polygon(const T& boundary_polygon, double offset_distance)
    {
        T result_polygon;
        size_t n = boundary_polygon.size();
        if (n < 3) return result_polygon;

        // Step 1: Compute offset edges. Each edge (p_i → p_{i+1}) is shifted
        // by offset_distance along its left normal (-dy, dx)/len.
        struct OffsetEdge {
            double ax, ay, bx, by; // shifted edge endpoints
        };
        std::vector<OffsetEdge> offset_edges(n);

        for (size_t i = 0; i < n; ++i)
        {
            size_t j = (i + 1) % n;
            double dx = boundary_polygon[j].x - boundary_polygon[i].x;
            double dy = boundary_polygon[j].y - boundary_polygon[i].y;
            double len = std::sqrt(dx * dx + dy * dy);
            if (len < 1e-20)
            {
                // Degenerate edge — copy endpoint with zero offset
                offset_edges[i] = {boundary_polygon[i].x, boundary_polygon[i].y,
                                   boundary_polygon[j].x, boundary_polygon[j].y};
                continue;
            }
            // Left normal: (-dy, dx) / len
            double nx = -dy / len * offset_distance;
            double ny =  dx / len * offset_distance;

            offset_edges[i] = {
                boundary_polygon[i].x + nx, boundary_polygon[i].y + ny,
                boundary_polygon[j].x + nx, boundary_polygon[j].y + ny
            };
        }

        // Step 2: For each vertex i, intersect offset_edge[prev] and
        // offset_edge[i] to find the new vertex position.
        const double MITER_LIMIT = 4.0 * std::abs(offset_distance);

        for (size_t i = 0; i < n; ++i)
        {
            size_t prev = (i + n - 1) % n;

            // Direction vectors of the two offset edges
            double dx1 = offset_edges[prev].bx - offset_edges[prev].ax;
            double dy1 = offset_edges[prev].by - offset_edges[prev].ay;
            double dx2 = offset_edges[i].bx - offset_edges[i].ax;
            double dy2 = offset_edges[i].by - offset_edges[i].ay;

            double det = dx1 * dy2 - dy1 * dx2;

            double ix, iy;
            if (std::abs(det) < 1e-10)
            {
                // Parallel edges — midpoint of the two offset points
                ix = (offset_edges[prev].bx + offset_edges[i].ax) * 0.5;
                iy = (offset_edges[prev].by + offset_edges[i].ay) * 0.5;
            }
            else
            {
                // Line intersection: solve for parameter t on offset_edges[prev]
                double t = ((offset_edges[i].ax - offset_edges[prev].ax) * dy2
                          - (offset_edges[i].ay - offset_edges[prev].ay) * dx2) / det;
                ix = offset_edges[prev].ax + t * dx1;
                iy = offset_edges[prev].ay + t * dy1;

                // Miter cap: if the intersection is too far from the original
                // vertex, clamp it to prevent degenerate spikes.
                double mdx = ix - boundary_polygon[i].x;
                double mdy = iy - boundary_polygon[i].y;
                double miter_dist = std::sqrt(mdx * mdx + mdy * mdy);
                if (miter_dist > MITER_LIMIT && miter_dist > 1e-10)
                {
                    double scale = MITER_LIMIT / miter_dist;
                    ix = boundary_polygon[i].x + mdx * scale;
                    iy = boundary_polygon[i].y + mdy * scale;
                }
            }

            result_polygon.push_back(wykobi::make_point(ix, iy));
        }

        return result_polygon;
    }
}