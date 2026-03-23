//
// Created by airlab on 7/10/25.
//

#ifndef CROW_SERVER_MINRECT_H
#define CROW_SERVER_MINRECT_H
#include <vector>
#include <algorithm>
#include <wykobi/wykobi.hpp>
// Rotating Calipers algorithm for minimum area rectangle
namespace mrcpp::minrect{
    using Point = wykobi::point2d<double>;
    struct MinAreaState
    {
        size_t bottom;
        size_t left;
        double height;
        double width;
        double base_a;
        double base_b;
        double area;
    };

    struct MinAreaRect
    {
        double width;
        double height;
        double area;
        double angle_width;
        double angle_height;
        Point center;
        Point corner;
        Point vector_width;
        Point vector_height;
        std::vector<Point> rect;

    };
    class MinRect
    {

    public:
        //the distance of point to the line, which describes by start and end
        static double _distance(const Point &start, const Point &end, const Point &point);

        static inline double cross(const Point &O, const Point &A, const Point &B)
        {
            return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
        }

        static std::vector<Point> convexHull(const std::vector<Point>& p);

        static inline double area(const Point &a, const Point &b, const Point &c)
        {
            return fabs((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
        }

        static inline double dist(const Point &a, const Point &b)
        {
            return hypot(a.x - b.x, a.y - b.y);
        }

        static double diameter(const std::vector<Point> &p);

        static MinAreaRect minAreaRect(const std::vector<Point> &p);
    };
}


#endif //CROW_SERVER_MINRECT_H
