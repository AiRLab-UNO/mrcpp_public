//
// Created by airlab on 7/18/25.
//

#ifndef CROW_SERVER_PATHPLANNER_H
#define CROW_SERVER_PATHPLANNER_H
#include "../bt_headers.h"
#include "cppvisgraph/vis_graph.hpp"
//============================ Path Planner Nodes ============================
class PathPlanner : public SyncActionNode
{
public:
    PathPlanner(const std::string& name, const NodeConfig& config)
            : SyncActionNode(name, config){}

    static PortsList providedPorts()
    {
        return {
                InputPort<SharedQueue<wykobi::point2d<double>>>("roi_poly"),
                InputPort<SharedQueue<wykobi::point2d<double>>>("dilated_poly", "Original (pre-offset) ROI polygon used for hard boundary checking"),
                InputPort<SharedQueue<wykobi::segment<double, 2>>>("line_segments"),
                InputPort<ASSIGNMENT>("task"),
                InputPort<SharedQueuePolygon>("obstacle_list"),
                InputPort<std::string>("transition_strategy", "full", "Transition routing: full, direct, dijkstra_only"),
                InputPort<double>("robot_radius", 0.1, "Robot radius for obstacle clearance buffer"),
                OutputPort<SharedQueue<wykobi::point2d<double>>>("path")
        };
    }

    NodeStatus tick() override
    {
        SharedQueue<wykobi::point2d<double>> roi_poly = std::make_shared<std::deque<wykobi::point2d<double>>>();
        if (!getInput("roi_poly", roi_poly))
        {
            throw RuntimeError("error reading port [polygon]: ");
        }

        auto line_segments = std::make_shared<std::deque<wykobi::segment<double, 2>>>();
        if (!getInput("line_segments", line_segments))
        {
            throw RuntimeError("error reading port [line_segments]: ");
        }

        ASSIGNMENT task;
        if (!getInput("task", task))
        {
            throw RuntimeError("error reading port [task_map]: ");
        }

        std::string transition_strategy = "full";
        getInput("transition_strategy", transition_strategy);

        getInput("robot_radius", robot_radius_);

        // PERFORM COLLISION CHECKING (also stores roi_poly as boundary)
        collision_check(roi_poly);

        std::vector<wykobi::segment<double, 2>> swath, new_swath;
        for (const auto& segment_id : task.second)
            swath.push_back(line_segments->at(segment_id));

        std::vector<wykobi::point2d<double>> route;
        int sweep_dir = (swath.size() % 2 == 0) ? 0 : 1;
        for (const auto& segment : swath)
        {
            if(sweep_dir % 2 == 0)
                addEdge(segment[0], segment[1], route);
            else
                addEdge(segment[1], segment[0], route);
            sweep_dir = 1 - sweep_dir;
        }

        // VALIDATION PASS: Ensure all route waypoints are inside the polygon boundary.
        std::vector<wykobi::point2d<double>> validated_route;
        for(const auto& pt : route)
        {
            if(pointInPolygon(pt, roi_boundary_))
            {
                validated_route.push_back(pt);
            }
            else
            {
                auto snapped = snapPointToBoundary(pt, roi_boundary_);
                validated_route.push_back(snapped);
            }
        }

        auto path = std::make_shared<std::deque<wykobi::point2d<double>>>();

        for(size_t i = 0; i < validated_route.size() - 1; ++i)
        {
            auto segment = wykobi::make_segment(validated_route[i], validated_route[i + 1]);

            bool hits_obstacle  = isCollision(segment, obstacles_);
            bool exits_boundary = !segmentInsidePoly(segment, roi_boundary_);

            if(!hits_obstacle && !exits_boundary)
            {
                // Direct transit is safe
                path->push_back(validated_route[i]);
            }
            else if(transition_strategy == "direct")
            {
                // ABLATION: No rerouting — always use direct connection
                path->push_back(validated_route[i]);
            }
            else if(transition_strategy == "dijkstra_only")
            {
                // ABLATION: Use polygon-vertex Dijkstra for ALL collision cases
                // (no visibility graph — routes through ROI boundary vertices)
                auto interior = shortestPathInsidePoly(validated_route[i], validated_route[i + 1]);
                for(size_t k = 0; k + 1 < interior.size(); ++k)
                    path->push_back(interior[k]);
            }
            else // "full" — default behavior
            {
                if(hits_obstacle)
                {
                    // Route around obstacle using the obstacle visibility graph.
                    // Swath endpoints are clipped exactly at the expanded obstacle
                    // boundary, so cppvisgraph may see them as "inside" the polygon
                    // and return no visible vertices. Nudge them just outside first.
                    auto src  = cppvisgraph::Point(validated_route[i].x,     validated_route[i].y);
                    auto dest = cppvisgraph::Point(validated_route[i + 1].x, validated_route[i + 1].y);

                    int src_poly  = graph_->point_in_polygon(src);
                    int dest_poly = graph_->point_in_polygon(dest);
                    if(src_poly  >= 0) src  = graph_->closest_point(src,  src_poly);
                    if(dest_poly >= 0) dest = graph_->closest_point(dest, dest_poly);

                    auto shortest = graph_->shortest_path(src, dest);
                    if(shortest.empty())
                    {
                        // Fallback: no vis-graph path found, use direct connection.
                        path->push_back(validated_route[i]);
                    }
                    else
                    {
                        for(const auto& pt : shortest)
                            path->push_back(wykobi::make_point(pt.x(), pt.y()));
                    }
                }
                else
                {
                    // Transit crosses roi boundary — route through polygon vertices
                    auto interior = shortestPathInsidePoly(validated_route[i], validated_route[i + 1]);
                    for(size_t k = 0; k + 1 < interior.size(); ++k)
                        path->push_back(interior[k]);
                }
            }
        }
        // add the last point
        if(!path->empty() && !wykobi::is_equal(path->back(), validated_route.back()))
        {
            path->push_back(validated_route.back());
        }

        // FINAL VALIDATION PASS: re-check every segment in the assembled path
        // against the ROI boundary. This catches violations introduced by:
        //  - obstacle vis-graph paths that exit through ROI concavities
        //  - numerical edge cases in segmentInsidePoly during the main loop
        //  - Dijkstra fallback paths
        {
            auto final_path = std::make_shared<std::deque<wykobi::point2d<double>>>();
            for(size_t i = 0; i + 1 < path->size(); ++i)
            {
                auto seg = wykobi::make_segment((*path)[i], (*path)[i + 1]);
                if(!segmentInsidePoly(seg, roi_boundary_))
                {
                    // Ensure endpoints are inside before rerouting
                    auto p0 = pointInPolygon((*path)[i], roi_boundary_)
                              ? (*path)[i] : snapPointToBoundary((*path)[i], roi_boundary_);
                    auto p1 = pointInPolygon((*path)[i + 1], roi_boundary_)
                              ? (*path)[i + 1] : snapPointToBoundary((*path)[i + 1], roi_boundary_);
                    auto rerouted = shortestPathInsidePoly(p0, p1);
                    for(size_t k = 0; k + 1 < rerouted.size(); ++k)
                        final_path->push_back(rerouted[k]);
                }
                else
                {
                    final_path->push_back((*path)[i]);
                }
            }
            if(!path->empty())
                final_path->push_back(path->back());
            path = final_path;
        }

        setOutput("path", path);
        DEBUG("[PathPlanner]: robot "<< task.first <<" strategy=" << transition_strategy
              << " task size: " << task.second.size() << "  path size: " << path->size());

        return NodeStatus::SUCCESS;
    }
private:
    std::vector<wykobi::polygon<double, 2>> obstacles_;
    wykobi::polygon<double, 2> roi_boundary_;   // roi boundary for transit containment checks
    std::shared_ptr<cppvisgraph::VisGraph> graph_;
    double robot_radius_ = 0.1;

    // Pre-built ROI visibility graph: adj[i] = {(j, dist), ...} for boundary vertex i.
    // Built once in collision_check and reused by shortestPathInsidePoly every call.
    std::vector<std::vector<std::pair<size_t, double>>> roi_vis_adj_;

protected:
    void addEdge(const wykobi::point2d<double>& start, const wykobi::point2d<double>& dest,
                 std::vector<wykobi::point2d<double>>& route)
    {
        route.push_back(start);
        route.push_back(dest);
    }

    void collision_check(const SharedQueue<wykobi::point2d<double>>& roi_poly)
    {
        if(!obstacles_.empty())
            return;

        // Prefer the original (pre-offset) polygon for boundary checking.
        // OffsetPolygon saves the original polygon to dilated_poly and overwrites
        // roi_poly with the shrunk polygon. The shrunk polygon's vertex-normal offset
        // is incorrect at reflex vertices (pushes vertices outward), so segments that
        // exit the original ROI may appear "inside" the shrunk polygon and bypass
        // the boundary check. Using the original polygon avoids this.
        SharedQueue<wykobi::point2d<double>> boundary_poly =
            std::make_shared<std::deque<wykobi::point2d<double>>>();
        if(!getInput("dilated_poly", boundary_poly) || boundary_poly->empty())
            boundary_poly = roi_poly; // fallback: no offset step in pipeline

        roi_boundary_.clear();
        for(const auto& pt : *boundary_poly)
            roi_boundary_.push_back(pt);

        // populate the obstacles
        auto obstacle_list = std::make_shared<std::deque<wykobi::polygon<double, 2>>>();
        getInput("obstacle_list", obstacle_list);

        DEBUG("obstacle_list size: " << obstacle_list->size());

        // Build visgraph from obstacle polygons ONLY (NOT the roi boundary —
        // cppvisgraph treats all input polygons as obstacles to route around
        // their exterior, which would prevent routing between interior points)
        std::vector<std::vector<cppvisgraph::Point>> viz_polygons;
        for(auto& obs : *obstacle_list)
        {
            // obstacles_ uses the pre-expanded polygon (from ServerRequest) for
            // isCollision edge checks — this is the authoritative boundary.
            obstacles_.push_back(obs);

            // The vis-graph obstacles must be STRICTLY LARGER than the swath-clip
            // boundary. Swath endpoints are clipped exactly at obs boundary, so
            // using obs directly for cppvisgraph leaves endpoints on the polygon edge.
            // polygon_crossing() (ray-cast) is numerically ambiguous for on-edge
            // points, and visible_vertices() rejects edges from boundary points,
            // producing empty visibility → empty shortest_path → no transition path.
            //
            // Fix: expand each obstacle outward by a small epsilon so swath endpoints
            // are strictly OUTSIDE the vis-graph polygon and visible_vertices works.
            const double VIS_EPS = 0.1;
            auto viz_obs = mrcpp::avoidance::offset_boundary_polygon(obs, VIS_EPS);
            if(wykobi::area(viz_obs) < wykobi::area(obs)) // wrong winding — flip
                viz_obs = mrcpp::avoidance::offset_boundary_polygon(obs, -VIS_EPS);

            std::vector<cppvisgraph::Point> obs_points;
            for(const auto& point : viz_obs)
            {
                obs_points.emplace_back(point.x, point.y);
            }
            // Close the polygon if not already closed
            if(wykobi::distance(viz_obs.front(), viz_obs.back()) > 1e-6)
                obs_points.push_back(obs_points.front());
            viz_polygons.push_back(obs_points);
        }

        // Create obstacle-avoidance visibility graph
        graph_ = std::make_shared<cppvisgraph::VisGraph>();
        graph_->build(viz_polygons);

        // Pre-build ROI interior visibility graph for boundary-violation routing
        buildRoiVisGraph();
    }

    /**
     * Pre-builds the ROI interior visibility graph from the boundary polygon vertices.
     * Called once from collision_check; reused by every shortestPathInsidePoly call.
     *
     * For each pair of boundary vertices, an edge is added iff segmentInsidePoly
     * returns true (segment stays entirely within the ROI). This produces a
     * correct vis-graph for non-convex polygons: only "diagonal" chords that do
     * not exit through a concavity are included.
     */
    void buildRoiVisGraph()
    {
        const size_t N = roi_boundary_.size();
        roi_vis_adj_.assign(N, {});

        for(size_t i = 0; i < N; ++i)
        {
            for(size_t j = i + 1; j < N; ++j)
            {
                auto seg = wykobi::make_segment(roi_boundary_[i], roi_boundary_[j]);
                if(segmentInsidePoly(seg, roi_boundary_))
                {
                    double d = wykobi::distance(roi_boundary_[i], roi_boundary_[j]);
                    roi_vis_adj_[i].emplace_back(j, d);
                    roi_vis_adj_[j].emplace_back(i, d);
                }
            }
        }
    }

    /**
     * Finds the shortest path from 'from' to 'to' that stays strictly inside
     * roi_boundary_, using the pre-built ROI visibility graph.
     *
     * The vis-graph (roi_vis_adj_) is built once in collision_check from all pairs
     * of boundary vertices whose connecting segment passes segmentInsidePoly.
     * At query time, only 'from' and 'to' need their connections resolved —
     * O(N) checks each — and then Dijkstra runs on the combined graph.
     */
    std::vector<wykobi::point2d<double>> shortestPathInsidePoly(
            const wykobi::point2d<double>& from,
            const wykobi::point2d<double>& to) const
    {
        using Point = wykobi::point2d<double>;
        using Edge  = std::pair<size_t, double>;

        if(roi_boundary_.size() < 3)
            return {from, to};

        // Node layout: 0..N_POLY-1 = ROI boundary vertices,
        //              N_POLY      = 'from',
        //              N_POLY+1    = 'to'
        const size_t N_POLY   = roi_boundary_.size();
        const size_t FROM_IDX = N_POLY;
        const size_t TO_IDX   = N_POLY + 1;
        const size_t N        = N_POLY + 2;

        // Flat node lookup (boundary vertices + from + to)
        std::vector<Point> nodes;
        nodes.reserve(N);
        for(const auto& v : roi_boundary_) nodes.push_back(v);
        nodes.push_back(from);
        nodes.push_back(to);

        // Copy pre-built boundary vis-graph; add edges for from/to at query time
        std::vector<std::vector<Edge>> adj(N);
        for(size_t i = 0; i < N_POLY; ++i)
            adj[i] = roi_vis_adj_[i];

        auto tryAdd = [&](size_t i, size_t j)
        {
            auto seg = wykobi::make_segment(nodes[i], nodes[j]);
            if(segmentInsidePoly(seg, roi_boundary_))
            {
                double d = wykobi::distance(nodes[i], nodes[j]);
                adj[i].emplace_back(j, d);
                adj[j].emplace_back(i, d);
            }
        };

        for(size_t v = 0; v < N_POLY; ++v)
        {
            tryAdd(FROM_IDX, v);
            tryAdd(TO_IDX,   v);
        }
        tryAdd(FROM_IDX, TO_IDX);

        // Dijkstra from FROM_IDX to TO_IDX
        const double DIST_INF = std::numeric_limits<double>::infinity();
        std::vector<double> dist(N, DIST_INF);
        std::vector<int>    parent(N, -1);
        dist[FROM_IDX] = 0.0;
        using PQNode = std::pair<double, size_t>;
        std::priority_queue<PQNode, std::vector<PQNode>, std::greater<PQNode>> pq;
        pq.push({0.0, FROM_IDX});

        while(!pq.empty())
        {
            auto [d, u] = pq.top(); pq.pop();
            if(d > dist[u]) continue;
            if(u == TO_IDX) break;
            for(const auto& [v, w] : adj[u])
            {
                if(dist[u] + w < dist[v])
                {
                    dist[v] = dist[u] + w;
                    parent[v] = static_cast<int>(u);
                    pq.push({dist[v], v});
                }
            }
        }

        if(dist[TO_IDX] == DIST_INF)
        {
            // Dijkstra found no interior path — walk along boundary edges as fallback.
            // Adjacent boundary vertices are always connected (polygon edges are trivially
            // inside), so a boundary walk is guaranteed to produce a valid interior path.
            DEBUG("[PathPlanner] WARNING: no interior Dijkstra path, falling back to boundary walk");

            // Find nearest boundary vertices to from and to
            size_t near_from = 0, near_to = 0;
            double d_from_min = std::numeric_limits<double>::infinity();
            double d_to_min   = std::numeric_limits<double>::infinity();
            for(size_t v = 0; v < N_POLY; ++v)
            {
                double df = wykobi::distance(from, roi_boundary_[v]);
                double dt = wykobi::distance(to,   roi_boundary_[v]);
                if(df < d_from_min) { d_from_min = df; near_from = v; }
                if(dt < d_to_min)   { d_to_min = dt;   near_to   = v; }
            }

            // Walk boundary in both directions, pick shorter
            auto boundaryWalk = [&](bool forward) -> std::vector<size_t>
            {
                std::vector<size_t> indices;
                size_t cur = near_from;
                for(size_t steps = 0; steps <= N_POLY; ++steps)
                {
                    indices.push_back(cur);
                    if(cur == near_to) break;
                    cur = forward ? (cur + 1) % N_POLY : (cur + N_POLY - 1) % N_POLY;
                }
                return indices;
            };

            auto fwd_walk = boundaryWalk(true);
            auto bwd_walk = boundaryWalk(false);
            const auto& walk = (fwd_walk.size() <= bwd_walk.size()) ? fwd_walk : bwd_walk;

            std::vector<Point> result;
            result.push_back(from);
            for(size_t idx : walk)
                result.push_back(roi_boundary_[idx]);
            result.push_back(to);
            return result;
        }

        std::vector<Point> result;
        for(int cur = static_cast<int>(TO_IDX); cur != -1; cur = parent[cur])
            result.push_back(nodes[cur]);
        std::reverse(result.begin(), result.end());
        return result;
    }

    /**
     * Returns true if 'seg' lies entirely inside 'polygon'.
     *
     * Two-step Wykobi-based check (no sampling, O(n) in polygon edges):
     *  1. Both endpoints must be inside or on the boundary — uses the custom
     *     pointInPolygon which treats on-edge points as inside (distance < EPS).
     *     This handles waypoints clipped exactly to ROI/obstacle edges.
     *  2. No polygon edge may be crossed — wykobi::intersect(segment, edge).
     *     The loop uses (i+1) % size to implicitly close open polygons.
     *     Degenerate zero-length edges (from explicitly-closed polygons where
     *     last vertex duplicates first) are skipped via wykobi::distance check.
     */
    bool segmentInsidePoly(const wykobi::segment<double, 2>& seg,
                           const wykobi::polygon<double, 2>& polygon) const
    {
        if(polygon.size() < 3) return true;

        // 1. Endpoint containment — custom pointInPolygon handles on-edge points
        if(!pointInPolygon(seg[0], polygon) || !pointInPolygon(seg[1], polygon))
            return false;

        // 2. Parametric edge-crossing check — O(n), no sampling.
        //
        //    For test segment A→B and polygon edge C→E:
        //      det = fx*dy - dx*fy
        //      t   = (fx*(cy-ay) - fy*(cx-ax)) / det   [parameter along test segment]
        //      s   = (dx*(cy-ay) - dy*(cx-ax)) / det   [parameter along polygon edge]
        //
        //    Crossing iff: t ∈ (ε, 1-ε)   — interior of test segment (endpoints
        //                                    already verified inside by check above)
        //                  s ∈ [-ε, 1+ε]  — CLOSED on the polygon edge, so crossings
        //                                    at polygon vertices (s≈0 or s≈1) are
        //                                    caught. wykobi::intersect misses these
        //                                    due to open-endpoint semantics.
        const double EPS = 1e-9;
        const double ax = seg[0].x, ay = seg[0].y;
        const double bx = seg[1].x, by = seg[1].y;
        const double dx = bx - ax,  dy = by - ay;

        for(size_t i = 0; i < polygon.size(); ++i)
        {
            const size_t j = (i + 1) % polygon.size();
            const double cx = polygon[i].x, cy = polygon[i].y;
            const double ex = polygon[j].x, ey = polygon[j].y;
            const double fx = ex - cx,      fy = ey - cy;

            // Skip degenerate edges (closed polygon with duplicate last vertex)
            if(fx * fx + fy * fy < 1e-20) continue;

            const double det = fx * dy - dx * fy;
            if(std::abs(det) < EPS) continue; // parallel — no crossing

            const double t = (fx * (cy - ay) - fy * (cx - ax)) / det;
            const double s = (dx * (cy - ay) - dy * (cx - ax)) / det;

            if(t > EPS && t < 1.0 - EPS && s >= -EPS && s <= 1.0 + EPS)
                return false; // segment exits polygon
        }
        return true;
    }

    bool isCollision(const wykobi::segment<double, 2>& segment,
                     const std::vector<wykobi::polygon<double, 2>>& obstacles) const
    {
        // Midpoint of the query segment — detects chords whose endpoints both
        // sit ON an obstacle boundary (split_swath_lines clips exactly there).
        // wykobi::intersect uses open/touching semantics and misses such chords.
        wykobi::point2d<double> mid;
        mid.x = (segment[0].x + segment[1].x) * 0.5;
        mid.y = (segment[0].y + segment[1].y) * 0.5;

        for(const auto& obs : obstacles)
        {
            // Fast path: midpoint inside → segment passes through obstacle interior
            if(pointInPolygon(mid, obs))
                return true;

            // Edge-crossing check: segment enters/exits obstacle at non-vertex points
            for(size_t i = 0; i < obs.size(); ++i)
            {
                size_t j = (i + 1) % obs.size();
                wykobi::segment<double, 2> edge = wykobi::make_segment(obs[i], obs[j]);
                if(wykobi::intersect(segment, edge))
                    return true;
            }
        }
        return false;
    }

    /**
     * Ray-casting algorithm to check if a point is inside a polygon.
     * Returns true if the point is strictly inside (not on boundary).
     */
    bool pointInPolygon(const wykobi::point2d<double>& point,
                        const wykobi::polygon<double, 2>& polygon) const
    {
        if(polygon.size() < 3)
            return true; // Degenerate polygon, assume inside
        
        int intersections = 0;
        const double EPS = 1e-10;

        for(size_t i = 0; i < polygon.size(); ++i)
        {
            size_t j = (i + 1) % polygon.size();
            const auto& p1 = polygon[i];
            const auto& p2 = polygon[j];

            // Skip degenerate edges (closed polygon where last vertex duplicates first)
            double dx = p2.x - p1.x;
            double dy = p2.y - p1.y;
            if(dx * dx + dy * dy < 1e-20) continue;

            // Check if point is on the edge
            double d = pointToSegmentDistance(point, p1, p2);
            if(d < EPS)
                return true; // Point is on boundary, consider inside

            // Ray casting: cast a ray from point to infinity along positive x-axis
            // Count intersections with polygon edges
            double y1 = p1.y;
            double y2 = p2.y;

            if((y1 <= point.y && point.y < y2) || (y2 <= point.y && point.y < y1))
            {
                // Edge crosses the horizontal ray from point
                // Calculate x-intersection
                double x_intersect = p1.x + (point.y - y1) * (p2.x - p1.x) / (y2 - y1);
                if(point.x < x_intersect)
                    intersections++;
            }
        }

        return intersections % 2 == 1;
    }

    /**
     * Calculate the minimum distance from a point to a line segment.
     */
    double pointToSegmentDistance(const wykobi::point2d<double>& point,
                                  const wykobi::point2d<double>& seg_start,
                                  const wykobi::point2d<double>& seg_end) const
    {
        double dx = seg_end.x - seg_start.x;
        double dy = seg_end.y - seg_start.y;
        double t = ((point.x - seg_start.x) * dx + (point.y - seg_start.y) * dy) / (dx * dx + dy * dy);
        t = std::max(0.0, std::min(1.0, t));
        
        double closest_x = seg_start.x + t * dx;
        double closest_y = seg_start.y + t * dy;
        
        double dist_x = point.x - closest_x;
        double dist_y = point.y - closest_y;
        
        return std::sqrt(dist_x * dist_x + dist_y * dist_y);
    }

    /**
     * Snap a point that is outside the polygon to the nearest boundary point.
     * Finds the closest point on any edge of the polygon and returns it with
     * a small inward offset to ensure it's definitively inside.
     */
    wykobi::point2d<double> snapPointToBoundary(
            const wykobi::point2d<double>& point,
            const wykobi::polygon<double, 2>& polygon) const
    {
        if(polygon.size() < 3)
            return point;

        double min_dist = std::numeric_limits<double>::infinity();
        wykobi::point2d<double> closest_point = point;
        size_t closest_edge = 0;

        // Find the closest point on any edge of the polygon
        for(size_t i = 0; i < polygon.size(); ++i)
        {
            size_t j = (i + 1) % polygon.size();
            const auto& p1 = polygon[i];
            const auto& p2 = polygon[j];

            double dx = p2.x - p1.x;
            double dy = p2.y - p1.y;
            double t = ((point.x - p1.x) * dx + (point.y - p1.y) * dy) / (dx * dx + dy * dy);
            t = std::max(0.0, std::min(1.0, t));

            double proj_x = p1.x + t * dx;
            double proj_y = p1.y + t * dy;

            double dist_x = point.x - proj_x;
            double dist_y = point.y - proj_y;
            double dist = std::sqrt(dist_x * dist_x + dist_y * dist_y);

            if(dist < min_dist)
            {
                min_dist = dist;
                closest_point = wykobi::make_point(proj_x, proj_y);
                closest_edge = i;
            }
        }

        // Move the point slightly inward from the boundary
        // Calculate inward direction as the perpendicular towards polygon center
        const double INSET = 0.001; // ~1 meter in lat/lng
        wykobi::point2d<double> polygon_center = polygonCentroid(polygon);
        
        double inward_x = polygon_center.x - closest_point.x;
        double inward_y = polygon_center.y - closest_point.y;
        double inward_len = std::sqrt(inward_x * inward_x + inward_y * inward_y);
        
        if(inward_len > 1e-10)
        {
            inward_x /= inward_len;
            inward_y /= inward_len;
            closest_point.x += inward_x * INSET;
            closest_point.y += inward_y * INSET;
        }

        return closest_point;
    }

    /**
     * Calculate the centroid of a polygon.
     */
    wykobi::point2d<double> polygonCentroid(const wykobi::polygon<double, 2>& polygon) const
    {
        double cx = 0.0, cy = 0.0;
        for(const auto& pt : polygon)
        {
            cx += pt.x;
            cy += pt.y;
        }
        cx /= polygon.size();
        cy /= polygon.size();
        return wykobi::make_point(cx, cy);
    }

};

#endif //CROW_SERVER_PATHPLANNER_H
