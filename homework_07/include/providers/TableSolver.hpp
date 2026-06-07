#pragma once

#include "interfaces/IBallisticSolver.hpp"
#include <string>
#include <vector>

class TableSolver : public IBallisticSolver {
public:
    explicit TableSolver(const std::string& ballisticTablePath = "ballistic_table.txt");

    BallisticsResult solve(const DroneConfig& config, const AmmoParams& ammo) override;

private:
    struct Result {
        double t{};
        double hDist{};
    };
    struct BallisticTable {

        std::vector<double> axisZ0;
        std::vector<double> axisV0;
        std::vector<double> axisM;
        std::vector<double> axisD;
        std::vector<double> axisL;
        std::vector<Result> data;

        bool load(const std::string& path);
        size_t index(int iz, int iv, int im, int id, int il) const;
        const Result& at(int iz, int iv, int im, int id, int il) const;
        Result lookup(double z0, double v0, double m, double d, double l) const;
    };

    struct Interp {
        int lo{};
        double frac{};
    };

    static Result lerp(const Result& a, const Result& b, double t);
    static Interp findInterp(double val, const std::vector<double>& axis);

    BallisticTable table_;
};