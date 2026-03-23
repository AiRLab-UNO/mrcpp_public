//
// Created by airlab on 7/10/25.
//
#include <algorithm>
#include <optional>
#include <vector>
#include <cmath>
#include <string>
#include <numeric>
#include <wykobi/wykobi.hpp>
#include <wykobi/wykobi_algorithm.hpp>
#include "mrcpp/MinRect.h"



namespace mrcpp::swathlines {

    using Point = wykobi::point2d<double>;

    // ----------------------------------------------------------------
    // Helper: given a sweep angle (radians), build swath lines covering
    // the polygon's bounding extent along that direction.
    // ----------------------------------------------------------------
    inline std::vector<wykobi::segment<double, 2>>
    generateSwathLinesAtAngle(const std::vector<Point> &poly,
                              double robot_radius, double angle_rad)
    {
        std::vector<wykobi::segment<double, 2>> swath_lines;

        // Sweep direction (along which swath lines run)
        Point dir = wykobi::make_point(std::cos(angle_rad), std::sin(angle_rad));
        // Perpendicular (direction we step across)
        Point perp = wykobi::make_point(-dir.y, dir.x);

        // Project all polygon vertices onto the sweep direction and perpendicular
        double min_along = std::numeric_limits<double>::max();
        double max_along = std::numeric_limits<double>::lowest();
        double min_perp  = std::numeric_limits<double>::max();
        double max_perp  = std::numeric_limits<double>::lowest();

        for (const auto& pt : poly) {
            double proj_along = pt.x * dir.x + pt.y * dir.y;
            double proj_perp  = pt.x * perp.x + pt.y * perp.y;
            min_along = std::min(min_along, proj_along);
            max_along = std::max(max_along, proj_along);
            min_perp  = std::min(min_perp, proj_perp);
            max_perp  = std::max(max_perp, proj_perp);
        }

        double swath_spacing = 2.0 * robot_radius;
        double width = max_perp - min_perp;
        int num_lines = static_cast<int>(width / swath_spacing) + 1;

        // Extend swath lines well beyond polygon along the sweep direction
        double extend = (max_along - min_along) * 0.1; // 10% margin

        for (int i = 0; i <= num_lines; ++i) {
            double perp_offset = min_perp + i * swath_spacing;

            // Base point at this perpendicular offset
            double base_x = perp.x * perp_offset;
            double base_y = perp.y * perp_offset;

            // Endpoints: extend along sweep direction
            Point P1 = wykobi::make_point(base_x + dir.x * (min_along - extend),
                                          base_y + dir.y * (min_along - extend));
            Point P2 = wykobi::make_point(base_x + dir.x * (max_along + extend),
                                          base_y + dir.y * (max_along + extend));
            swath_lines.push_back(wykobi::make_segment(P1, P2));
        }

        return swath_lines;
    }

    // ----------------------------------------------------------------
    // Strategy: Minimum Area Rectangle (MAR) — original / default
    // ----------------------------------------------------------------
    inline std::vector<wykobi::segment<double, 2>>
    generateSwathLines_MAR(const std::vector<Point> &poly, double robot_radius)
    {
        std::vector<wykobi::segment<double, 2>> swath_lines;

        // 1. Get the minimum area rectangle
        auto minRect = mrcpp::minrect::MinRect::minAreaRect(poly);
        auto &rect = minRect.rect;

        // 2. Determine longer side
        double dist01 = mrcpp::minrect::MinRect::dist(rect[0], rect[1]);
        double dist12 = mrcpp::minrect::MinRect::dist(rect[1], rect[2]);
        int init = (dist01 < dist12) ? 1 : 0;

        Point A = rect[init];
        Point B = rect[(init + 1) % 4];

        // 3. Direction vector along longest side (AB)
        Point dir = B - A;
        double length = std::sqrt(dir.x * dir.x + dir.y * dir.y);
        dir.x /= length;
        dir.y /= length;

        // 4. Perpendicular direction (for shifting swath lines)
        Point perp = wykobi::make_point(-dir.y, dir.x);

        // 5. Calculate swath spacing and height
        double swath_spacing = 2.0 * robot_radius;
        double height = (dist01 < dist12) ? dist12 : dist01;
        int num_lines = static_cast<int>(height / swath_spacing) + 1;

        for (int i = 0; i <= num_lines; ++i) {
            double offset = i * swath_spacing;
            Point shift = wykobi::make_point(perp.x * offset, perp.y * offset);
            Point P1 = wykobi::make_point(A.x + shift.x, A.y + shift.y);
            Point P2 = wykobi::make_point(B.x + shift.x, B.y + shift.y);
            swath_lines.push_back(wykobi::make_segment(P1, P2));
        }

        return swath_lines;
    }

    // ----------------------------------------------------------------
    // Strategy: Brute-force Angle Search (0–180° in 5° steps)
    // Minimizes the total number of swath lines (proxy for turn count).
    // ----------------------------------------------------------------
    inline std::vector<wykobi::segment<double, 2>>
    generateSwathLines_AngleSearch(const std::vector<Point> &poly, double robot_radius)
    {
        double best_angle = 0.0;
        double min_width = std::numeric_limits<double>::max();

        for (int deg = 0; deg < 180; deg += 5) {
            double angle_rad = deg * M_PI / 180.0;
            Point perp = wykobi::make_point(-std::sin(angle_rad), std::cos(angle_rad));

            double min_proj = std::numeric_limits<double>::max();
            double max_proj = std::numeric_limits<double>::lowest();
            for (const auto& pt : poly) {
                double proj = pt.x * perp.x + pt.y * perp.y;
                min_proj = std::min(min_proj, proj);
                max_proj = std::max(max_proj, proj);
            }
            double width = max_proj - min_proj;
            if (width < min_width) {
                min_width = width;
                best_angle = angle_rad;
            }
        }

        return generateSwathLinesAtAngle(poly, robot_radius, best_angle);
    }

    // ----------------------------------------------------------------
    // Strategy: PCA (Principal Component Analysis)
    // Aligns swath lines with the principal axis of the polygon vertices.
    // ----------------------------------------------------------------
    inline std::vector<wykobi::segment<double, 2>>
    generateSwathLines_PCA(const std::vector<Point> &poly, double robot_radius)
    {
        if (poly.size() < 2) return {};

        // Compute centroid
        double cx = 0.0, cy = 0.0;
        for (const auto& pt : poly) {
            cx += pt.x;
            cy += pt.y;
        }
        cx /= poly.size();
        cy /= poly.size();

        // Compute covariance matrix elements
        double cov_xx = 0.0, cov_xy = 0.0, cov_yy = 0.0;
        for (const auto& pt : poly) {
            double dx = pt.x - cx;
            double dy = pt.y - cy;
            cov_xx += dx * dx;
            cov_xy += dx * dy;
            cov_yy += dy * dy;
        }

        // Principal eigenvector via analytic formula for 2x2 symmetric matrix
        // The angle of the principal eigenvector:
        double angle = 0.5 * std::atan2(2.0 * cov_xy, cov_xx - cov_yy);

        return generateSwathLinesAtAngle(poly, robot_radius, angle);
    }

    // ----------------------------------------------------------------
    // Strategy: Minimum Width Rectangle
    // Like MAR but minimizes width (perpendicular extent) instead of area.
    // This directly minimizes the number of swath lines needed.
    // ----------------------------------------------------------------
    inline std::vector<wykobi::segment<double, 2>>
    generateSwathLines_MinWidth(const std::vector<Point> &poly, double robot_radius)
    {
        auto hull = mrcpp::minrect::MinRect::convexHull(poly);
        if (hull.size() < 3) return generateSwathLines_MAR(poly, robot_radius);

        double best_angle = 0.0;
        double min_width = std::numeric_limits<double>::max();

        // Test each edge of the convex hull as a potential sweep direction
        for (size_t i = 0; i < hull.size(); ++i) {
            size_t j = (i + 1) % hull.size();
            double dx = hull[j].x - hull[i].x;
            double dy = hull[j].y - hull[i].y;
            double len = std::sqrt(dx * dx + dy * dy);
            if (len < 1e-12) continue;

            double angle = std::atan2(dy, dx);
            // Perpendicular direction
            Point perp = wykobi::make_point(-std::sin(angle), std::cos(angle));

            // Compute width in this perpendicular direction
            double min_proj = std::numeric_limits<double>::max();
            double max_proj = std::numeric_limits<double>::lowest();
            for (const auto& pt : hull) {
                double proj = pt.x * perp.x + pt.y * perp.y;
                min_proj = std::min(min_proj, proj);
                max_proj = std::max(max_proj, proj);
            }
            double width = max_proj - min_proj;
            if (width < min_width) {
                min_width = width;
                best_angle = angle;
            }
        }

        return generateSwathLinesAtAngle(poly, robot_radius, best_angle);
    }

    // ----------------------------------------------------------------
    // Main entry point — dispatches to the selected strategy
    // ----------------------------------------------------------------
    inline std::vector<wykobi::segment<double, 2>>
    generateSwathLines(const std::vector<Point> &poly, double robot_radius,
                       const std::string& strategy = "mar")
    {
        if (strategy == "angle_search") {
            return generateSwathLines_AngleSearch(poly, robot_radius);
        } else if (strategy == "pca") {
            return generateSwathLines_PCA(poly, robot_radius);
        } else if (strategy == "min_width") {
            return generateSwathLines_MinWidth(poly, robot_radius);
        } else {
            // Default: MAR (minimum area rectangle)
            return generateSwathLines_MAR(poly, robot_radius);
        }
    }

    inline std::vector<wykobi::point2d<double>> segment_to_poly_intersection(const std::vector<wykobi::segment<double, 2>>& boundary_poly,
                                        const wykobi::segment<double, 2>& seg)
    {
        std::vector<wykobi::point2d<double>> intersection_list;
        for (const auto& edge : boundary_poly) {
            std::vector<wykobi::segment<double,2>> segment_list{edge, seg};
            wykobi::algorithm::naive_group_intersections<wykobi::segment<double ,2>>
                    (
                            segment_list.begin(),
                            segment_list.end(),
                            std::back_inserter(intersection_list)
                    );
        }
        std::sort(intersection_list.begin(), intersection_list.end(), [](const wykobi::point2d<double>& a, const wykobi::point2d<double>& b) {
            double dx = a.x - b.x;
            double dy = a.y - b.y;
            if(std::abs(dy) > std::abs(dx)) {
                return (a.y < b.y) || (a.y == b.y && a.x < b.x);
            }
            return (a.x < b.x) || (a.x == b.x && a.y < b.y);
        });
        return intersection_list;
    }

    // Clip all swath lines to the convex polygon
    inline void clipSwathLinesToPolygon(
            const std::vector<wykobi::point2d<double>>& poly,
            std::vector<wykobi::segment<double, 2>>& swath_lines)
    {
        std::vector<wykobi::segment<double, 2>> boundary_poly;
        for (size_t i = 0; i < poly.size(); ++i) {
            auto Pe = poly[i];
            auto Pe1 = poly[(i + 1) % poly.size()];
            boundary_poly.push_back(wykobi::make_segment(Pe, Pe1));
        }

        // find intersections
        std::vector<wykobi::segment<double, 2>> new_swath_lines;
        for (auto& seg : swath_lines) {
            auto intersection_list = segment_to_poly_intersection(boundary_poly, seg);
            for(size_t i = 0; i < intersection_list.size(); i += 2) {
                auto segment = wykobi::make_segment(intersection_list[i], intersection_list[i + 1]);
                new_swath_lines.push_back(segment);
            }
        }
       swath_lines.clear();
       std::copy(new_swath_lines.begin(), new_swath_lines.end(), std::back_inserter(swath_lines));
    }

}
