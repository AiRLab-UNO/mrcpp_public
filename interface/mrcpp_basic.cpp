#ifdef B0
#undef B0
#endif
#include "behaviortree_cpp/bt_factory.h"
#include "btnodes/bt_nodes.hpp"
#include <sstream>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <exception>
#include <nlohmann/json.hpp>
#include <filesystem>
#include "metric/EnergyCalculator.h"
using json = nlohmann::json;

void save_path(const std::vector<std::pair<double, double>>& path, std::string& outfile){
    // Create an output file stream
    std::ofstream file(outfile);

    // Check if the file opened successfully
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << outfile << std::endl;
        return;
    }

    // Optional: Add a header row
    file << "Latitude,Longitude\n";

    file << std::fixed << std::setprecision(16);

    // Iterate through the path and write coordinates
    for (const auto& point : path) {
        file << point.first << "," << point.second << "\n";
    }

    // Close the file (optional, as it closes automatically when 'file' goes out of scope)
    file.close();

    std::cout << "Path successfully saved to " << outfile << std::endl;
}


void run(const std::string& bt_file_path, const json& data,
         const wykobi::point2d<double>& start_point_latlon,
         const std::string& output_dir)
{
    SharedQueuePolygon results = std::make_shared<std::deque<wykobi::polygon<double, 2>>>();
    BT::BehaviorTreeFactory factory;

    // Register all custom BT nodes
    factory.registerNodeType<FileReader>("FileReader");
    factory.registerNodeType<ServerRequest>("ServerRequest", data);
    factory.registerNodeType<OffsetPolygon>("OffsetPolygon");
    factory.registerNodeType<GpsDecoder>("GpsDecoder");
    factory.registerNodeType<DataLoader>("DataLoader");
    factory.registerNodeType<SwathGenerator>("SwathGenerator");
    factory.registerNodeType<ObstacleAvoidance>("ObstacleAvoidance");
//        factory.registerNodeType<PolygonalObstacleAvoidance>("PolygonalObstacleAvoidance");
    factory.registerNodeType<TaskPlanner>("TaskPlanner");
    factory.registerNodeType<PathPlanner>("PathPlanner");
    factory.registerNodeType<LoopNode<ASSIGNMENT>>("LoopAssignment");
    factory.registerNodeType<ConvertToGpsPath>("ConvertToGpsPath");
    factory.registerNodeType<SwathGeneratorDisjoint>("SwathGeneratorDisjoint");
    factory.registerNodeType<PathAccumulator>("PathAccumulator", results);
//        factory.registerNodeType<VizResults>("VizResults", results);

    std::cout << "Creating behavior tree from file: " << bt_file_path << std::endl;
    auto tree = factory.createTreeFromFile(bt_file_path);

    // Set start_point_latlon on the blackboard so TaskPlanner can use it
    tree.rootBlackboard()->set("start_point_latlon", start_point_latlon);

    // Set orientation_strategy from JSON if provided
    if (data.contains("orientation_strategy")) {
        std::string orient = data["orientation_strategy"].get<std::string>();
        tree.rootBlackboard()->set("orientation_strategy", orient);
        std::cout << "Orientation strategy: " << orient << std::endl;
    }

    // Set headland_scale so that offset = 2 * robot_radius (one swath width)
    tree.rootBlackboard()->set("headland_scale", 2.0);
    std::cout << "Headland scale: 2.0" << std::endl;

    std::cout << "Executing behavior tree..." << std::endl;
    auto status = tree.tickWhileRunning();
    std::cout << "Behavior tree execution completed with status: " << static_cast<int>(status) << std::endl;

    battery_model_t bm{
        5,
        4,
        0.99876,
        -0.0020,
        -5.2484e-05,
        1.2230e-07
    };

    best_speed_model_t sm{
            0.041546,
            0.041122,
            0.00053292
    };

    energy_calculator_config_t ec_config{
        bm,
        sm,
        3.2,
        0.07,
        2,
        0.19,
        4,
        2
    };

    EnergyCalculator ec(ec_config);

    int index = 0;
    for (const auto& polygon : *results) {

        std::vector<std::pair<double, double>> path;
        // Prepend start_point (latlon)
        path.push_back({ start_point_latlon.x, start_point_latlon.y });
        for (const auto& point : polygon) {
            path.push_back({ point.x, point.y });
        }
        // Append start_point (latlon)
        path.push_back({ start_point_latlon.x, start_point_latlon.y });

        DEBUG("[EnergyConsumption] Robot " << index << " path consumes " << ec.calculate_path_energy_consumption(path) << " Joules" );
        index++;

        std::string outfile = output_dir + "path_" + std::to_string(index) + ".csv";
        save_path(path, outfile);
    }


}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <experiment_directory>" << std::endl;
        std::cerr << "  The directory should contain:" << std::endl;
        std::cerr << "    - start_point.csv  (lat, lon)" << std::endl;
        std::cerr << "    - mrcpp_basic.xml  (behavior tree)" << std::endl;
        std::cerr << "    - request.json     (problem definition)" << std::endl;
        return 1;
    }

    std::string dir = argv[1];
    // Normalize trailing separator
    if (!dir.empty() && dir.back() != '/' && dir.back() != '\\') {
        dir += '/';
    }

    std::string bt_file = dir + "mrcpp_basic.xml";
    std::string json_file = dir + "request.json";
    std::string start_file = dir + "start_point.csv";

    // Parse start_point.csv (format: lat, lon)
    std::ifstream sf(start_file);
    if (!sf.is_open()) {
        std::cerr << "Error: Cannot open start_point file: " << start_file << std::endl;
        return 1;
    }
    double start_lat, start_lon;
    char comma;
    if (!(sf >> start_lat >> comma >> start_lon)) {
        std::cerr << "Error: Failed to parse start_point.csv (expected: lat, lon)" << std::endl;
        return 1;
    }
    sf.close();
    wykobi::point2d<double> start_point_latlon = wykobi::make_point(start_lat, start_lon);
    std::cout << "Start point (latlon): " << start_lat << ", " << start_lon << std::endl;

    // Parse JSON file
    std::ifstream f(json_file);
    if (!f.is_open()) {
        std::cerr << "Error: Cannot open JSON file: " << json_file << std::endl;
        return 1;
    }

    json data;
    try {
        data = json::parse(f);
    } catch (const json::parse_error& e) {
        std::cerr << "JSON parse error: " << e.what() << std::endl;
        return 1;
    }
    f.close();

    // Create mrcpp_results subdirectory inside the experiment directory
    std::string results_dir = dir + "mrcpp_results/";
    std::filesystem::create_directories(results_dir);

    run(bt_file, data, start_point_latlon, results_dir);

    return 0;
}
