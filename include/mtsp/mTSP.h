//
// Created by redwan on 5/3/25.
//

#ifndef AREACOVERAGE_MTSP_H
#define AREACOVERAGE_MTSP_H
#include <cstdint>
#include <algorithm>
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <sstream>
#include <deque>
#include <unordered_map>
#include <regex>
extern "C" {
#include "LKH/LKH.h"
#include "LKH/Genetic.h"
#include "LKH/BIT.h"
}

namespace LKH {

    class mTSP {
    public:
        mTSP() = default;
        void setProblem(const std::vector<std::vector<uint64_t>>& cost, int num_salesmen);
        bool solve();
        std::unordered_map<int, int> getSolution() const;
        std::vector<std::deque<int>> getTours() const {
            return tours;
        }

    private:
        const std::string file_atsp = "amtsp.atsp";
        const std::string file_param = "amtsp.par";
        std::unordered_map<int, int> solution;
        std::vector<std::deque<int>> tours;

    protected:
        int solveMTSPWithLKH3(const char* input_file);
        std::vector<std::deque<int>> parseSolution(const std::string &output_file) const;
        int getNumNodes() const;
        std::vector<std::vector<uint64_t>> cost_matrix_;

    };

} // LKH

#endif //AREACOVERAGE_MTSP_H
