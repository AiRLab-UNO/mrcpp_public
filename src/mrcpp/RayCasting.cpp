//
// Created by airlab on 7/10/25.
//
#include <optional>
#include "wykobi/wykobi.hpp"
#include "wykobi/wykobi_algorithm.hpp"

namespace mrcpp::raycasting{
    /**
     * Splits swath lines by obstacles using ray casting algorithm.
     * @param swath_lines The input swath lines to be split.
     * @param obstacles The list of polygons representing obstacles.
     * @param split_lines The mrcpp_with_boxes vector to store the split lines.
     */
    inline void split_swath_lines(
            const std::vector<wykobi::segment<double, 2>>& swath_lines,
            const std::vector<wykobi::polygon<double, 2>>& obstacles,
            std::vector<wykobi::segment<double, 2>>& split_lines
    )
    {
        std::vector<wykobi::segment<double, 2>> result_lines;
        for(auto & line : swath_lines) {

            std::vector<wykobi::point2d<double>> segment_points{line[0]};
            for(const auto& collision_polygon : obstacles)
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
                result_lines.push_back(segment);
            }
        }

        split_lines.clear();
        std::copy(result_lines.begin(), result_lines.end(), std::back_inserter(split_lines));

    }




















}