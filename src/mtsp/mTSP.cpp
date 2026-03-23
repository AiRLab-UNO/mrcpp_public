//
// Created by redwan on 5/3/25.
//

#include "mtsp/mTSP.h"

namespace LKH {
    void mTSP::setProblem(const std::vector<std::vector<uint64_t>> &cost, int num_salesmen) {
        std::stringstream ss, params;
        ss << "NAME : amtsp" << '\n';
        ss << "TYPE : ATSP" << '\n';
        ss << "DIMENSION : " << cost.size() << '\n';
        ss << "EDGE_WEIGHT_TYPE : EXPLICIT" << '\n';
        ss << "EDGE_WEIGHT_FORMAT : FULL_MATRIX" << '\n';
        ss << "EDGE_WEIGHT_SECTION" << '\n';
        for (const auto& row : cost) {
            for (const auto& val : row) {
                ss << val << " ";
            }
            ss << '\n';
        }


        int min_size = cost.size() / (num_salesmen );
//        std::cout << ss.str() << std::endl;
        params << "SPECIAL" << '\n';
        params << "PROBLEM_FILE = " << file_atsp << '\n';
        params << "SALESMEN = " << num_salesmen << '\n';
        params << "MTSP_OBJECTIVE = MINSUM" << '\n';
        params << "MTSP_MIN_SIZE = " << min_size << '\n';
        params << "RUNS = 4" << '\n';
        params << "TRACE_LEVEL = 0" << '\n';
        params << "TOUR_FILE = amtsp.tour" << '\n';
        params << "MTSP_SOLUTION_FILE = amtsp.tour" << '\n';

        // Write the cost matrix to a file
        std::ofstream out(file_atsp);
        if (out.is_open()) {
            out << ss.str();
            out.close();
        } else {
            std::cerr << "Error writing to file: " << file_atsp << std::endl;
        }
        // Write the parameters to a file
        std::ofstream out_param(file_param);
        if (out_param.is_open()) {
            out_param << params.str();
            out_param.close();
        } else {
            std::cerr << "Error writing to file: " << file_param << std::endl;
        }

    }

    bool mTSP::solve() {
        if( solveMTSPWithLKH3(file_param.c_str()) == 0)
        {
            std::cout << "Solved successfully! reading = " << TourFileName << std::endl;
            solution.clear();
            tours = parseSolution(std::string(TourFileName));
            for(int i = 0; i < tours.size(); ++i)
            {
                auto tour = tours[i];
                std::cout << "Tour " << i << ": ";
                for (const auto& node : tour) {
                    std::cout << node << " ";
                    solution[node] = i;
                }
                std::cout << std::endl;
            }
        }
        else
        {
            std::cout << "Failed to solve!" << std::endl;
            return false;
        }
        return true;
    }

    int mTSP::solveMTSPWithLKH3(const char* input_file)
    {
        GainType Cost, OldOptimum;
        double Time, LastTime = GetTime();
        Node* N;
        int i;

        /* Read the specification of the problem */
        // if (argc >= 2)
        //     ParameterFileName = argv[1];
        ParameterFileName = const_cast<char*>(input_file);
        ReadParameters();
        MaxMatrixDimension = 20000;
        MergeWithTour = Recombination == IPT ? MergeWithTourIPT : MergeWithTourGPX2;
        ReadProblem();

        if (SubproblemSize > 0) {
            if (DelaunayPartitioning)
                SolveDelaunaySubproblems();
            else if (KarpPartitioning)
                SolveKarpSubproblems();
            else if (KCenterPartitioning)
                SolveKCenterSubproblems();
            else if (KMeansPartitioning)
                SolveKMeansSubproblems();
            else if (RohePartitioning)
                SolveRoheSubproblems();
            else if (MoorePartitioning || SierpinskiPartitioning)
                SolveSFCSubproblems();
            else
                SolveTourSegmentSubproblems();
            printff("SubproblemSize > 0 \n");
            return EXIT_SUCCESS;
        }
        AllocateStructures();
        if (ProblemType == TSPTW) TSPTW_Reduce();
        if (ProblemType == VRPB || ProblemType == VRPBTW) VRPB_Reduce();
        if (ProblemType == PDPTW) PDPTW_Reduce();
        CreateCandidateSet();
        InitializeStatistics();

        if (Norm != 0 || Penalty) {
            Norm = 9999;
            BestCost = PLUS_INFINITY;
            BestPenalty = CurrentPenalty = PLUS_INFINITY;
        } else {
            /* The ascent has solved the problem! */
            Optimum = BestCost = (GainType)LowerBound;
            UpdateStatistics(Optimum, GetTime() - LastTime);
            RecordBetterTour();
            RecordBestTour();
            CurrentPenalty = PLUS_INFINITY;
            BestPenalty = CurrentPenalty = Penalty ? Penalty() : 0;
            WriteTour(OutputTourFileName, BestTour, BestCost);
            WriteTour(TourFileName, BestTour, BestCost);
            Runs = 0;
            printf("The ascent has solved the problem \n");
        }

        /* Find a specified number (Runs) of local optima */
        printf("Find a specified number (Runs) of local optima \n");

        for (Run = 1; Run <= Runs; Run++) {

            LastTime = GetTime();
            Cost = FindTour(); /* using the Lin-Kernighan heuristic */
            if (MaxPopulationSize > 1 && !TSPTW_Makespan) {
                /* Genetic algorithm */
                int i;
                for (i = 0; i < PopulationSize; i++) {
                    GainType OldPenalty = CurrentPenalty;
                    GainType OldCost = Cost;
                    Cost = MergeTourWithIndividual(i);
                    if (TraceLevel >= 1 &&
                        (CurrentPenalty < OldPenalty || (CurrentPenalty == OldPenalty && Cost < OldCost))) {
                        if (CurrentPenalty)
                            printff("  Merged with %d: Cost = " GainFormat, i + 1, Cost);
                        else
                            printff(
                                    "  Merged with %d: Cost = " GainFormat "_" GainFormat, i + 1, CurrentPenalty, Cost);
                        if (Optimum != MINUS_INFINITY && Optimum != 0) {
                            if (ProblemType != CCVRP && ProblemType != TRP && ProblemType != MLP &&
                                MTSPObjective != MINMAX && MTSPObjective != MINMAX_SIZE)
                                printff(", Gap = %0.4f%%", 100.0 * (Cost - Optimum) / Optimum);
                            else
                                printff(", Gap = %0.4f%%", 100.0 * (CurrentPenalty - Optimum) / Optimum);
                        }
                        printff("\n");
                    }
                }
                if (!HasFitness(CurrentPenalty, Cost)) {
                    if (PopulationSize < MaxPopulationSize) {
                        AddToPopulation(CurrentPenalty, Cost);
                        if (TraceLevel >= 1) PrintPopulation();
                    } else if (SmallerFitness(CurrentPenalty, Cost, PopulationSize - 1)) {
                        i = ReplacementIndividual(CurrentPenalty, Cost);
                        ReplaceIndividualWithTour(i, CurrentPenalty, Cost);
                        if (TraceLevel >= 1) PrintPopulation();
                    }
                }
            } else if (Run > 1 && !TSPTW_Makespan)
                Cost = MergeTourWithBestTour();
            if (CurrentPenalty < BestPenalty || (CurrentPenalty == BestPenalty && Cost < BestCost)) {
                BestPenalty = CurrentPenalty;
                BestCost = Cost;
                RecordBetterTour();
                RecordBestTour();
                WriteTour(TourFileName, BestTour, BestCost);
            }
            OldOptimum = Optimum;
            if (!Penalty || (MTSPObjective != MINMAX && MTSPObjective != MINMAX_SIZE)) {
                if (CurrentPenalty == 0 && Cost < Optimum) Optimum = Cost;
            } else if (CurrentPenalty < Optimum)
                Optimum = CurrentPenalty;
            if (Optimum < OldOptimum) {
                printff("*** New OPTIMUM = " GainFormat " ***\n\n", Optimum);
                if (FirstNode->InputSuc) {
                    Node* N = FirstNode;
                    while ((N = N->InputSuc = N->Suc) != FirstNode)
                        ;
                }
            }
            Time = fabs(GetTime() - LastTime);
            UpdateStatistics(Cost, Time);
            if (TraceLevel >= 1 && Cost != PLUS_INFINITY) {
                printff("Run %d: ", Run);
                StatusReport(Cost, LastTime, "");
                printff("\n");
            }
            if (StopAtOptimum && MaxPopulationSize >= 1) {
                if (ProblemType != CCVRP && ProblemType != TRP && ProblemType != MLP &&
                    MTSPObjective != MINMAX && MTSPObjective != MINMAX_SIZE ?
                    CurrentPenalty == 0 && Cost == Optimum :
                    CurrentPenalty == Optimum) {
                    Runs = Run;
                    break;
                }
            }
            if (PopulationSize >= 2 &&
                (PopulationSize == MaxPopulationSize || Run >= 2 * MaxPopulationSize) && Run < Runs) {
                Node* N;
                int Parent1, Parent2;
                Parent1 = LinearSelection(PopulationSize, 1.25);
                do
                    Parent2 = LinearSelection(PopulationSize, 1.25);
                while (Parent2 == Parent1);
                ApplyCrossover(Parent1, Parent2);
                N = FirstNode;
                do {
                    if (ProblemType != HCP && ProblemType != HPP) {
                        int d = C(N, N->Suc);
                        AddCandidate(N, N->Suc, d, INT_MAX);
                        AddCandidate(N->Suc, N, d, INT_MAX);
                    }
                    N = N->InitialSuc = N->Suc;
                } while (N != FirstNode);
            }
            SRandom(++Seed);
        }

        PrintStatistics();
        if (Salesmen > 1) {
            if (Dimension == DimensionSaved) {
                for (i = 1; i <= Dimension; i++) {
                    N = &NodeSet[BestTour[i - 1]];
                    (N->Suc = &NodeSet[BestTour[i]])->Pred = N;
                }
            } else {
                for (i = 1; i <= DimensionSaved; i++) {
                    Node* N1 = &NodeSet[BestTour[i - 1]];
                    Node* N2 = &NodeSet[BestTour[i]];
                    Node* M1 = &NodeSet[N1->Id + DimensionSaved];
                    Node* M2 = &NodeSet[N2->Id + DimensionSaved];
                    (M1->Suc = N1)->Pred = M1;
                    (N1->Suc = M2)->Pred = N1;
                    (M2->Suc = N2)->Pred = M2;
                }
            }

            CurrentPenalty = BestPenalty;
            MTSP_Report(BestPenalty, BestCost);
            MTSP_WriteSolution(MTSPSolutionFileName, BestPenalty, BestCost);
            SINTEF_WriteSolution(SINTEFSolutionFileName, BestCost);
        }


        if (ProblemType == ACVRP || ProblemType == BWTSP || ProblemType == CCVRP || ProblemType == CTSP ||
            ProblemType == CVRP || ProblemType == CVRPTW || ProblemType == MLP ||
            ProblemType == M_PDTSP || ProblemType == M1_PDTSP || MTSPObjective != -1 ||
            ProblemType == ONE_PDTSP || ProblemType == OVRP || ProblemType == PDTSP ||
            ProblemType == PDTSPL || ProblemType == PDPTW || ProblemType == RCTVRP ||
            ProblemType == RCTVRPTW || ProblemType == SOP || ProblemType == TRP || ProblemType == TSPTW ||
            ProblemType == VRPB || ProblemType == VRPBTW || ProblemType == VRPPD) {
            printff("Best %s solution:\n", Type);
            CurrentPenalty = BestPenalty;
            SOP_Report(BestCost);
        }
        printff("\n");

        return EXIT_SUCCESS;
    }


    std::vector<std::deque<int>> mTSP::parseSolution(const std::string& filename) const
    {

        std::vector<std::deque<int>> tours;
        std::ifstream file(filename);

        if (!file.is_open()) {
            std::cerr << "Error opening file: " << filename << std::endl;
            return tours;
        }

        std::string line;

        // Skip the first line (total cost)
        std::getline(file, line);

        // Skip the description line
        std::getline(file, line);

        // Read tour information
        while (std::getline(file, line)) {
//        printff("line: %s \n", line.c_str());
            std::deque<int> path;
            // Find the position of "(#" which marks the end of the path
            size_t hashPos = line.find("(#");
            if (hashPos != std::string::npos) {
                // Extract the path part of the string
                std::string pathStr = line.substr(0, hashPos);

                // Use regex to find all numbers in the path
                std::regex numberPattern("\\d+");
                auto numbers_begin = std::sregex_iterator(pathStr.begin(), pathStr.end(), numberPattern);
                auto numbers_end = std::sregex_iterator();

                for (std::sregex_iterator i = numbers_begin; i != numbers_end; ++i) {
                    std::smatch match = *i;
                    path.push_back(std::stoi(match.str()));
                }
            }


            // remove start and end depot
            path.pop_front();
            path.pop_back();

            tours.push_back(path);
        }

        return tours;
    }

    std::unordered_map<int, int> mTSP::getSolution() const {
        return solution;
    }

    int mTSP::getNumNodes() const {
        return DimensionSaved - Salesmen + 1;
    }
} // LKH