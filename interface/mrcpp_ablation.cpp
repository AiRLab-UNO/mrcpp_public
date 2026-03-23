#ifdef B0
#undef B0
#endif
#include "behaviortree_cpp/bt_factory.h"
#include "btnodes/bt_nodes.hpp"
#include <sstream>
#include <fstream>
#include <iostream>
#include <exception>
#include <chrono>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <filesystem>
#include "metric/EnergyCalculator.h"
using json = nlohmann::json;
using PATH = std::vector<std::pair<double, double>>;

// ============================================================
// Geodesy helpers (from energy_estimator.cpp)
// ============================================================
static inline double deg2rad(double d) { return d * M_PI / 180.0; }

static PATH latlon_to_local_xy_m(const PATH& latlon_deg)
{
    PATH out;
    if (latlon_deg.empty()) return out;

    const double R = 6378137.0; // WGS84 equatorial radius [m]
    const double lat0 = deg2rad(latlon_deg.front().first);
    const double lon0 = deg2rad(latlon_deg.front().second);

    out.reserve(latlon_deg.size());
    for (const auto& ll : latlon_deg) {
        const double lat = deg2rad(ll.first);
        const double lon = deg2rad(ll.second);
        const double dlat = lat - lat0;
        const double dlon = lon - lon0;
        const double x = R * dlon * std::cos((lat + lat0) * 0.5); // East
        const double y = R * dlat;                                  // North
        out.emplace_back(x, y);
    }
    return out;
}

static double path_length_m(const PATH& p) {
    if (p.size() < 2) return 0.0;
    double L = 0.0;
    for (size_t i = 1; i < p.size(); ++i) {
        double dx = p[i].first  - p[i-1].first;
        double dy = p[i].second - p[i-1].second;
        L += std::sqrt(dx*dx + dy*dy);
    }
    return L;
}

// ============================================================
// Ablation configuration
// ============================================================
struct AblationConfig {
    std::string orientation_strategy = "mar";      // Study 1
    double      headland_scale       = 1.0;        // Study 2
    std::string transition_strategy  = "full";     // Study 3
};

// ============================================================
// Result structure
// ============================================================
struct AblationResult {
    std::string data_file;
    std::string orientation_strategy;
    double      headland_scale;
    std::string transition_strategy;
    int         num_robots;
    double      total_energy;           // with depot round-trips (E_t)
    double      total_energy_no_depot;  // coverage path only (bar{E}_t)
    double      total_path_len;
    double      max_path_len;
    int         total_waypoints;
    double      compute_time_ms;
    bool        success;
};

// ============================================================
// Detect whether JSON data has GPS coordinates
// GPS: latitude in [-90, 90], longitude in [-180, 180]
// Cartesian: arbitrary coordinates (typically 0–100+)
// ============================================================
bool isGpsData(const json& data) {
    if (!data.contains("polygon") || data["polygon"].empty())
        return false;
    double first_x = data["polygon"][0][0].get<double>();
    double first_y = data["polygon"][0][1].get<double>();
    // GPS latitude is [-90, 90], Cartesian coords are typically > 90
    return (std::abs(first_x) <= 90.0 && std::abs(first_y) <= 180.0);
}

// ============================================================
// Run a single ablation experiment
// ============================================================
AblationResult runAblation(
    const std::string& bt_file,
    const json& input_data,
    const AblationConfig& config,
    const std::string& data_file_name,
    const energy_calculator_config_t& ec_config,
    const wykobi::point2d<double>& start_point_latlon,
    bool is_gps)
{
    AblationResult result;
    result.data_file = data_file_name;
    result.orientation_strategy = config.orientation_strategy;
    result.headland_scale = config.headland_scale;
    result.transition_strategy = config.transition_strategy;
    result.success = false;

    // Prepare JSON data with required fields
    json data = input_data;
    if (!data.contains("num_robots"))
        data["num_robots"] = 3;
    if (!data.contains("robot_radius"))
        data["robot_radius"] = 1.5;
    if (!data.contains("obstacles"))
        data["obstacles"] = json::array();

    result.num_robots = data["num_robots"].get<int>();

    // Handle island_flyzone obstacle format: obstacles is array of polygon point arrays
    // Convert [[pt1, pt2, ...]] to [[3, x1, y1, x2, y2, ...]]
    if (data.contains("obstacles") && !data["obstacles"].empty()) {
        auto& obs = data["obstacles"];
        if (obs[0].is_array() && !obs[0].empty() && obs[0][0].is_array()) {
            // It's an array of polygon point arrays like [[[x,y],[x,y],...]]
            json converted = json::array();
            for (const auto& poly_points : obs) {
                std::vector<double> flat_obs;
                flat_obs.push_back(3); // polygon type
                for (const auto& pt : poly_points) {
                    flat_obs.push_back(pt[0].get<double>());
                    flat_obs.push_back(pt[1].get<double>());
                }
                converted.push_back(flat_obs);
            }
            data["obstacles"] = converted;
        }
    }

    try {
        SharedQueuePolygon results = std::make_shared<std::deque<wykobi::polygon<double, 2>>>();
        BT::BehaviorTreeFactory factory;

        // Register all custom BT nodes
        factory.registerNodeType<ServerRequest>("ServerRequest", data);
        factory.registerNodeType<OffsetPolygon>("OffsetPolygon");
        factory.registerNodeType<GpsDecoder>("GpsDecoder");
        factory.registerNodeType<DataLoader>("DataLoader");
        factory.registerNodeType<SwathGenerator>("SwathGenerator");
        factory.registerNodeType<ObstacleAvoidance>("ObstacleAvoidance");
        factory.registerNodeType<TaskPlanner>("TaskPlanner");
        factory.registerNodeType<PathPlanner>("PathPlanner");
        factory.registerNodeType<LoopNode<ASSIGNMENT>>("LoopAssignment");
        factory.registerNodeType<ConvertToGpsPath>("ConvertToGpsPath");
        factory.registerNodeType<SwathGeneratorDisjoint>("SwathGeneratorDisjoint");
        factory.registerNodeType<PathAccumulator>("PathAccumulator", results);

        auto tree = factory.createTreeFromFile(bt_file);

        // Set ablation parameters on the blackboard
        auto blackboard = tree.rootBlackboard();
        blackboard->set("orientation_strategy", config.orientation_strategy);
        blackboard->set("headland_scale", config.headland_scale);
        blackboard->set("transition_strategy", config.transition_strategy);
        blackboard->set("start_point_latlon", start_point_latlon);

        // Time the execution
        auto t_start = std::chrono::high_resolution_clock::now();
        auto status = tree.tickWhileRunning();
        auto t_end = std::chrono::high_resolution_clock::now();

        result.compute_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

        if (status != BT::NodeStatus::SUCCESS) {
            std::cerr << "[ABLATION] FAILED: " << data_file_name
                      << " orient=" << config.orientation_strategy
                      << " headland=" << config.headland_scale
                      << " transit=" << config.transition_strategy << std::endl;
            return result;
        }

        // Compute energy and path metrics
        EnergyCalculator ec(ec_config);
        result.total_energy = 0.0;
        result.total_energy_no_depot = 0.0;
        result.total_path_len = 0.0;
        result.max_path_len = 0.0;
        result.total_waypoints = 0;

        for (const auto& polygon : *results) {
            // Path with depot round-trips
            PATH raw_path_depot;
            raw_path_depot.push_back({start_point_latlon.x, start_point_latlon.y});
            for (const auto& point : polygon) {
                raw_path_depot.push_back({point.x, point.y});
            }
            raw_path_depot.push_back({start_point_latlon.x, start_point_latlon.y});

            // Path without depot (coverage only)
            PATH raw_path_no_depot;
            for (const auto& point : polygon) {
                raw_path_no_depot.push_back({point.x, point.y});
            }

            PATH path_m = is_gps ? latlon_to_local_xy_m(raw_path_depot) : raw_path_depot;
            PATH path_m_no_depot = is_gps ? latlon_to_local_xy_m(raw_path_no_depot) : raw_path_no_depot;

            result.total_energy += ec.calculate_path_energy_consumption(path_m);
            result.total_energy_no_depot += ec.calculate_path_energy_consumption(path_m_no_depot);

            double len = path_length_m(path_m);
            result.total_path_len += len;
            result.max_path_len = std::max(result.max_path_len, len);
            result.total_waypoints += static_cast<int>(polygon.size());
        }

        result.success = true;

    } catch (const std::exception& e) {
        std::cerr << "[ABLATION] EXCEPTION: " << e.what()
                  << " (" << data_file_name
                  << " orient=" << config.orientation_strategy
                  << " headland=" << config.headland_scale
                  << " transit=" << config.transition_strategy << ")" << std::endl;
    }

    return result;
}

// ============================================================
// Main: run all ablation configurations
// ============================================================
int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <experiment_directory>" << std::endl;
        std::cerr << "  The directory should contain:" << std::endl;
        std::cerr << "    - start_point.csv  (lat, lon)" << std::endl;
        std::cerr << "    - request.json     (problem definition)" << std::endl;
        return 1;
    }

    std::string dir = argv[1];
    // Normalize trailing separator
    if (!dir.empty() && dir.back() != '/' && dir.back() != '\\') {
        dir += '/';
    }

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

    // Determine BT config based on data type (GPS vs Cartesian)
    // Resolve config path by walking up from experiment directory until config files are found.
    std::filesystem::path exp_path = std::filesystem::canonical(dir);
    std::filesystem::path search = exp_path;
    std::filesystem::path project_root;

    while (!search.empty()) {
        auto cfg_gps = search / "config" / "mrcpp_bt_ablation.xml";
        if (std::filesystem::exists(cfg_gps)) {
            project_root = search;
            break;
        }
        auto parent = search.parent_path();
        if (parent == search) {
            break;
        }
        search = parent;
    }

    if (project_root.empty()) {
        std::cerr << "Error: could not locate project root with config/mrcpp_bt_ablation.xml from: " << exp_path << std::endl;
        return 1;
    }

    std::string config_gps       = (project_root / "config" / "mrcpp_bt_ablation.xml").string();
    std::string config_cartesian = (project_root / "config" / "mrcpp_bt_ablation_cartesian.xml").string();

    bool gps = isGpsData(data);
    std::string bt_config = gps ? config_gps : config_cartesian;
    if (!gps && !std::filesystem::exists(bt_config)) {
        std::cerr << "Error: cartesian BT config not found: " << bt_config << std::endl;
        return 1;
    }

    std::cerr << "Data type: " << (gps ? "GPS" : "Cartesian") << std::endl;
    std::cerr << "Project root: " << project_root << std::endl;
    std::cerr << "BT config: " << bt_config << std::endl;

    // Extract display name from experiment directory
    std::string display_name = std::filesystem::path(dir).parent_path().filename().string();

    // Energy calculator config (same as mrcpp_basic.cpp)
    battery_model_t bm{5, 4, 0.99876, -0.0020, -5.2484e-05, 1.2230e-07};
    best_speed_model_t sm{0.041546, 0.041122, 0.00053292};
    energy_calculator_config_t ec_config{bm, sm, 3.2, 0.07, 2, 0.19, 4, 2};

    // Default headland_scale = 2.0 so offset = 2 * robot_radius = one swath width
    double default_headland_scale = 2.0;
    std::cerr << "Default headland_scale: " << default_headland_scale << std::endl;

    // ============================================================
    // Study 1: Sweep Orientation
    // ============================================================
    std::vector<std::string> orientations = {"mar", "angle_search", "pca", "min_width"};

    // ============================================================
    // Study 2: Headland Buffer Scale
    // ============================================================
    std::vector<double> headland_scales = {0.0, 0.5, 1.0, 1.5, 2.0};

    // ============================================================
    // Study 3: Transition Strategy
    // ============================================================
    std::vector<std::string> transitions = {"full", "direct", "dijkstra_only"};

    // Create mrcpp_ablation subdirectory inside the experiment directory
    std::string results_dir = dir + "mrcpp_ablation/";
    std::filesystem::create_directories(results_dir);

    // Collect all results
    std::vector<AblationResult> all_results;

    // CSV header
    std::string csv_header = "study,data_file,orientation,headland_scale,transition,"
                             "num_robots,Et_Wh,Et_bar_Wh,total_path_len_m,"
                             "max_path_len_m,total_waypoints,compute_time_ms,success";

    std::cout << csv_header << std::endl;

    auto formatResult = [](const std::string& study, const AblationResult& r) -> std::string {
        std::ostringstream oss;
        oss << study << ","
            << r.data_file << ","
            << r.orientation_strategy << ","
            << r.headland_scale << ","
            << r.transition_strategy << ","
            << r.num_robots << ","
            << std::fixed << std::setprecision(4) << r.total_energy / 3600.0 << ","
            << r.total_energy_no_depot / 3600.0 << ","
            << r.total_path_len << ","
            << r.max_path_len << ","
            << r.total_waypoints << ","
            << r.compute_time_ms << ","
            << (r.success ? "1" : "0");
        return oss.str();
    };

    auto saveStudyCSV = [&](const std::string& filename,
                            const std::string& study_name,
                            const std::vector<AblationResult>& study_results) {
        std::string filepath = results_dir + filename;
        std::ofstream out(filepath);
        if (!out.is_open()) {
            std::cerr << "Error: Cannot open output file: " << filepath << std::endl;
            return;
        }
        out << csv_header << "\n";
        for (const auto& r : study_results) {
            out << formatResult(study_name, r) << "\n";
        }
        out.close();
        std::cerr << "  Saved: " << filepath << std::endl;
    };

    // ============================================================
    // Run Study 1: Sweep Orientation (vary orientation, fix others)
    // ============================================================
    std::cerr << "\n=== STUDY 1: Sweep Orientation ===" << std::endl;
    {
        std::vector<AblationResult> study_results;
        for (const auto& orient : orientations) {
            AblationConfig cfg;
            cfg.orientation_strategy = orient;
            cfg.headland_scale = default_headland_scale;
            cfg.transition_strategy = "full"; // default

            std::cerr << "  Running: " << display_name << " orient=" << orient << std::endl;
            auto result = runAblation(bt_config, data, cfg, display_name, ec_config, start_point_latlon, gps);
            std::cout << formatResult("orientation", result) << std::endl;
            study_results.push_back(result);
            all_results.push_back(result);
        }
        saveStudyCSV("ablation_orientation.csv", "orientation", study_results);
    }

    // ============================================================
    // Run Study 2: Headland Buffer (vary headland_scale, fix others)
    // ============================================================
    std::cerr << "\n=== STUDY 2: Headland Buffer ===" << std::endl;
    {
        std::vector<AblationResult> study_results;
        for (double scale : headland_scales) {
            AblationConfig cfg;
            cfg.orientation_strategy = "mar";  // default
            cfg.headland_scale = scale;
            cfg.transition_strategy = "full";  // default

            std::cerr << "  Running: " << display_name << " headland=" << scale << std::endl;
            auto result = runAblation(bt_config, data, cfg, display_name, ec_config, start_point_latlon, gps);
            std::cout << formatResult("headland", result) << std::endl;
            study_results.push_back(result);
            all_results.push_back(result);
        }
        saveStudyCSV("ablation_headland.csv", "headland", study_results);
    }

    // ============================================================
    // Run Study 3: Transition Strategy (vary transition, fix others)
    // ============================================================
    std::cerr << "\n=== STUDY 3: Transition Strategy ===" << std::endl;
    {
        std::vector<AblationResult> study_results;
        for (const auto& transit : transitions) {
            AblationConfig cfg;
            cfg.orientation_strategy = "mar";  // default
            cfg.headland_scale = default_headland_scale;
            cfg.transition_strategy = transit;

            std::cerr << "  Running: " << display_name << " transit=" << transit << std::endl;
            auto result = runAblation(bt_config, data, cfg, display_name, ec_config, start_point_latlon, gps);
            std::cout << formatResult("transition", result) << std::endl;
            study_results.push_back(result);
            all_results.push_back(result);
        }
        saveStudyCSV("ablation_transition.csv", "transition", study_results);
    }

    // ============================================================
    // Summary
    // ============================================================
    int total = all_results.size();
    int success = 0;
    for (const auto& r : all_results)
        if (r.success) success++;

    std::cerr << "\n=== ABLATION COMPLETE ===" << std::endl;
    std::cerr << "Total experiments: " << total << std::endl;
    std::cerr << "Successful: " << success << "/" << total << std::endl;

    return 0;
}
