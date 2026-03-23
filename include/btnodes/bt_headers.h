//
// Created by airlab on 7/18/25.
//

#ifndef CROW_SERVER_BT_HEADERS_H
#define CROW_SERVER_BT_HEADERS_H
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/decorators/loop_node.h"
#include "behaviortree_cpp/json_export.h"
#include <sstream>
#include <iomanip>
#include "mrcpp/utils/GpsPoint.h"
#include "../../src/mrcpp/SwathGenerator.cpp"
#include "../../src/mrcpp/ObstacleAvoidance.cpp"
#include "../../src/mrcpp/RayCasting.cpp"
#include "mtsp/mTSP.h"
//#include "mrcpp/utils/matplotlibcpp.h"
#include "mrcpp/RoutePlanner.h"


#define DEBUG(x) std::cerr << x << std::endl;

using namespace BT;
using json = nlohmann::json;

BT_JSON_CONVERTER(wykobi::point2d<double>, point)
{
    add_field("x", &point.x);
    add_field("y", &point.y);
}

typedef std::pair<int, std::deque<int>> ASSIGNMENT;
typedef std::shared_ptr<std::deque<wykobi::polygon<double, 2>>> SharedQueuePolygon;

namespace BT {
    template<>
    inline wykobi::point2d<double> convertFromString(StringView str) {
        double x, y;
        auto parts = splitString(str, ';');
        x     = convertFromString<double>(parts[0]);
        y     = convertFromString<double>(parts[1]);
        return wykobi::make_point(x, y);
    }

    template<>
    inline wykobi::segment<double, 2> convertFromString(StringView str) {

        auto parts = splitString(str, ';');
        auto x     = convertFromString<wykobi::point2d<double>>(parts[0]);
        auto y     = convertFromString<wykobi::point2d<double>>(parts[1]);
        return wykobi::make_segment(x, y);
    }

    template<>
    inline std::pair<int, int> convertFromString(StringView str) {

        auto parts = splitString(str, ';');
        auto x     = convertFromString<int>(parts[0]);
        auto y     = convertFromString<int>(parts[1]);
        return std::make_pair(x, y);
    }

    template<>
    inline ASSIGNMENT convertFromString(StringView str) {

        auto parts = splitString(str, ';');
        auto x     = convertFromString<int>(parts[0]);
        auto y     = convertFromString<std::deque<int>>(parts[1]);
        return std::make_pair(x, y);
    }
}


#endif //CROW_SERVER_BT_HEADERS_H
