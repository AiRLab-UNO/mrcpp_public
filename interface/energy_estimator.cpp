// // Created by airlab on 7/30/25.
// //
#include <iostream>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>
#include <iomanip>
#include <algorithm>
#include <cmath>

#include "metric/rapidcsv.h"
#include "metric/EnergyCalculator.h"

#define DEBUG(x) std::cout << x << std::endl
namespace fs = std::filesystem;

using PATH = std::vector<std::pair<double, double>>;

// ----------------- geodesy helpers -----------------
static inline double deg2rad(double d) { return d * M_PI / 180.0; }

// Convert a (lat, lon) path [degrees] to local ENU meters using the first point as the origin.
// x ≈ East, y ≈ North
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
        // Equirectangular projection around origin; accurate for local paths
        const double dlat = lat - lat0;
        const double dlon = lon - lon0;
        const double x = R * dlon * std::cos((lat + lat0) * 0.5); // East
        const double y = R * dlat;                                 // North
        out.emplace_back(x, y);
    }
    return out;
}

// ----------------- distance helpers -----------------
static double euclid(const std::pair<double,double>& a,
                     const std::pair<double,double>& b) {
    const double dx = a.first  - b.first;
    const double dy = a.second - b.second;
    return std::sqrt(dx*dx + dy*dy);
}

static double path_length_m(const PATH& p) {
    if (p.size() < 2) return 0.0;
    double L = 0.0;
    for (size_t i = 1; i < p.size(); ++i) L += euclid(p[i-1], p[i]);
    return L;
}

// ----------------- main computation -----------------
void compute(const std::vector<PATH>& results_meters, const std::vector<std::string>& filenames)
{
    battery_model_t bm{ 5, 4, 0.99876, -0.0020, -5.2484e-05, 1.2230e-07 };
    best_speed_model_t sm{ 0.041546, 0.041122, 0.00053292 };

    energy_calculator_config_t ec_config{
        bm,
        sm,
        3.2,   // drone_mass [kg]
        0.07,  // drone_area [m^2]
        2.0,   // average_acceleration [m/s^2]
        0.19,  // propeller_radius [m]
        4,     // number_of_propellers
        2.0    // allowed_path_deviation [m]
    };

    EnergyCalculator ec(ec_config);

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "\nFile                         "
                 "Len[m]     Energy[J]   Energy[Wh]   kJ/km\n";
    std::cout << "---------------------------------------------------------------\n";

    double path_cost_sum_J = 0.0;
    double max_path_cost_J = 0.0;
    double total_len_m     = 0.0;
    std::string max_file;

    for (size_t i = 0; i < results_meters.size(); ++i) {
        const auto& path_m = results_meters[i];
        const auto& name   = filenames[i];

        const double L      = path_length_m(path_m);
        const double E_J    = ec.calculate_path_energy_consumption(path_m);
        const double E_Wh   = E_J / 3600.0;
        const double kJ_per_km = (L > 0.0) ? (E_J / 1000.0) / (L / 1000.0) : 0.0;

        path_cost_sum_J += E_J;
        total_len_m     += L;
        if (E_J > max_path_cost_J) { max_path_cost_J = E_J; max_file = name; }

        std::cout << std::left  << std::setw(28) << name << " "
                  << std::right << std::setw(9)  << L        << "  "
                  << std::setw(11) << E_J        << "  "
                  << std::setw(11) << E_Wh       << "  "
                  << std::setw(6)  << kJ_per_km  << "\n";
        // DEBUG("[EnergyConsumption] Robot " << name << " path consumes " << E_J << " Joules");
    }

    std::cout << "---------------------------------------------------------------\n";
    std::cout << "path_cost_sum (fleet energy): " << path_cost_sum_J
              << " J  (" << path_cost_sum_J/3600.0 << " Wh)\n";
    std::cout << "max_path_cost  (worst UAV):   " << max_path_cost_J
              << " J  (" << max_path_cost_J/3600.0 << " Wh)";
    if (!max_file.empty()) std::cout << "  in " << max_file;
    std::cout << "\nTOTAL length: " << total_len_m << " m\n\n";
}

// ----------------- CLI -----------------
int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::cerr << "Usage: " << (argc>0?argv[0]:"energy_from_csv")
        << " <directory_with_csv_paths> [--latlon]\n"
        << "  CSV columns must be either:\n"
        << "    meters: X, Y (default), or\n"
        << "    degrees: latitude, longitude  (use --latlon)\n";
        return 1;
    }

    std::string directory_path = argv[1];
    bool input_is_latlon = false;
    for (int i = 2; i < argc; ++i) {
        std::string flag = argv[i];
        if (flag == "--latlon") input_is_latlon = true;
        else {
            std::cerr << "Unknown flag: " << flag << "\n";
            return 1;
        }
    }

    if (!fs::exists(directory_path) || !fs::is_directory(directory_path)) {
        std::cerr << "Invalid directory path: " << directory_path << "\n";
        return 1;
    }

    std::vector<PATH> paths_meters;
    std::vector<std::string> filenames;
    std::vector<fs::path> file_list;

    // Collect CSV files
    for (const auto& entry : fs::directory_iterator(directory_path)) {
        if (entry.is_regular_file() && entry.path().extension() == ".csv") {
            file_list.push_back(entry.path());
        }
    }
    if (file_list.empty()) {
        std::cerr << "No *.csv files in: " << directory_path << "\n";
        return 1;
    }

    // Sort for consistent order
    std::sort(file_list.begin(), file_list.end());

    // Read each CSV (col0, col1)
    for (const auto& csv_path : file_list) {
        try {
            rapidcsv::Document doc(csv_path.string());
            std::vector<double> C0 = doc.GetColumn<double>("Latitude");
            std::vector<double> C1 = doc.GetColumn<double>("Longitude");
            if (C0.size() != C1.size() || C0.empty()) {
                std::cerr << "Skipping bad CSV: " << csv_path.filename().string() << "\n";
                continue;
            }

            PATH raw_pts;
            raw_pts.reserve(C0.size());
            for (size_t i = 0; i < C0.size(); ++i) {
                if (input_is_latlon) {
                    // Interpret as (lat, lon) in degrees for now; convert later
                    raw_pts.emplace_back(C0[i], C1[i]);
                } else {
                    // Already meters (X, Y)
                    raw_pts.emplace_back(C0[i], C1[i]);
                }
            }

            PATH path_m =
                input_is_latlon ? latlon_to_local_xy_m(raw_pts) : std::move(raw_pts);

            filenames.push_back(csv_path.filename().string());
            paths_meters.push_back(std::move(path_m));
        } catch (const std::exception& e) {
            try{
                rapidcsv::Document doc(csv_path.string(), rapidcsv::LabelParams(-1, -1));
                std::vector<double> C0 = doc.GetColumn<double>(0);
                std::vector<double> C1 = doc.GetColumn<double>(1);
                if (C0.size() != C1.size() || C0.empty()) {
                    std::cerr << "Skipping bad CSV: " << csv_path.filename().string() << "\n";
                    continue;
                }

                PATH raw_pts;
                raw_pts.reserve(C0.size());
                for (size_t i = 0; i < C0.size(); ++i) {
                    if (input_is_latlon) {
                        // Interpret as (lat, lon) in degrees for now; convert later
                        raw_pts.emplace_back(C0[i], C1[i]);
                    } else {
                        // Already meters (X, Y)
                        raw_pts.emplace_back(C0[i], C1[i]);
                    }
                }

                PATH path_m =
                    input_is_latlon ? latlon_to_local_xy_m(raw_pts) : std::move(raw_pts);

                filenames.push_back(csv_path.filename().string());
                paths_meters.push_back(std::move(path_m));
            }
            catch(const std::exception& e){
                std::cerr << "Error reading " << csv_path << ": " << e.what() << "\n";
            }
        }
    }

    if (paths_meters.empty()) {
        std::cerr << "No valid paths parsed in: " << directory_path << "\n";
        return 1;
    }

    compute(paths_meters, filenames);
    return 0;
}
//
// Energy-from-CSV with optional lat/lon conversion
// Computes both E_o (fast waypoint estimator via EnergyCalculator)
// and E_t (trajectory-based estimator: spline + forward/backward time param)
// Created by airlab on 7/30/25 (extended with E_t integration)
//
// Energy from CSV: E_o (EnergyCalculator) + E_t (paper-exact trajectory-based)
// - Reads all *.csv in a directory
// - CSV columns: (X,Y) in meters OR (lat,lon) in degrees with --latlon
// - E_o via EnergyCalculator
// - E_t via arclength spline + forward/backward (rest-to-rest) with speed <= v_r, accel <= a_max
// - Power rule: P(v)=P_h for v < v_r, else P_r ; add positive ΔKE; no regeneration.
