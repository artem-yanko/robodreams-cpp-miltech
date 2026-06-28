#include "providers/TableSolver.hpp"
#include "utils/logger.hpp"

#include <algorithm>
#include <fstream>

TableSolver::TableSolver(const std::string& ballisticTablePath) {
    if (!table.load(ballisticTablePath)) {
        ERROR_LOG("Failed to load ballistic table from \"" << ballisticTablePath << "\"");
    }
}

bool TableSolver::BallisticTable::load(const std::string& path) {
    std::ifstream inputFile(path);
    if (!inputFile) {
        ERROR_LOG("Cannot open ballistic table file \"" << path << "\"");
        return false;
    }

    int nZ = 0;
    int nV = 0;
    int nM = 0;
    int nD = 0;
    int nL = 0;

    if (!(inputFile >> nZ >> nV >> nM >> nD >> nL)) {
        ERROR_LOG("Failed to read ballistic table");
        return false;
    }

    axisZ0.resize(nZ);
    axisV0.resize(nV);
    axisM.resize(nM);
    axisD.resize(nD);
    axisL.resize(nL);

    for (double& v : axisZ0) inputFile >> v;
    for (double& v : axisV0) inputFile >> v;
    for (double& v : axisM) inputFile >> v;
    for (double& v : axisD) inputFile >> v;
    for (double& v : axisL) inputFile >> v;

    const std::size_t total =
        static_cast<std::size_t>(nZ) *
        static_cast<std::size_t>(nV) *
        static_cast<std::size_t>(nM) *
        static_cast<std::size_t>(nD) *
        static_cast<std::size_t>(nL);

    data.resize(total);

    for (std::size_t i = 0; i < total; ++i) {
        if (!(inputFile >> data[i].t >> data[i].hDist)) {
            ERROR_LOG("Failed to read ballistic table values");
            return false;
        }
    }

    return true;
}

std::size_t TableSolver::BallisticTable::index(int iz, int iv, int im, int id, int il) const {
    return (((static_cast<std::size_t>(iz) * axisV0.size() + static_cast<std::size_t>(iv))
            * axisM.size() + static_cast<std::size_t>(im))
            * axisD.size() + static_cast<std::size_t>(id))
            * axisL.size() + static_cast<std::size_t>(il);
}

const TableSolver::Result& TableSolver::BallisticTable::at(int iz, int iv, int im, int id, int il) const {
    return data[index(iz, iv, im, id, il)];
}

TableSolver::Result TableSolver::lerp(const TableSolver::Result& a, const TableSolver::Result& b, double t) {
    return {
        a.t + (b.t - a.t) * t,
        a.hDist + (b.hDist - a.hDist) * t
    };
}

TableSolver::Interp TableSolver::findInterp(double val, const std::vector<double>& axis) {
    if (val <= axis.front()) {
        return {0, 0.0};
    }

    if (val >= axis.back()) {
        return {static_cast<int>(axis.size()) - 2, 1.0};
    }

    auto it = std::lower_bound(axis.begin(), axis.end(), val);
    int i = static_cast<int>(it - axis.begin()) - 1;
    if (i < 0) {
        i = 0;
    }

    double frac = (val - axis[i]) / (axis[i + 1] - axis[i]);
    return {i, frac};
}

TableSolver::Result TableSolver::BallisticTable::lookup(double z0, double v0, double m, double d, double l) const {
    TableSolver::Interp iz = TableSolver::findInterp(z0, axisZ0);
    TableSolver::Interp iv = TableSolver::findInterp(v0, axisV0);
    TableSolver::Interp im = TableSolver::findInterp(m, axisM);
    TableSolver::Interp id = TableSolver::findInterp(d, axisD);
    TableSolver::Interp il = TableSolver::findInterp(l, axisL);

    Result v[16];
    for (int a = 0; a < 2; ++a) {
        for (int b = 0; b < 2; ++b) {
            for (int c = 0; c < 2; ++c) {
                for (int e = 0; e < 2; ++e) {
                    const Result& lo = at(iz.lo + a, iv.lo + b, im.lo + c, id.lo + e, il.lo);
                    const Result& hi = at(iz.lo + a, iv.lo + b, im.lo + c, id.lo + e, il.lo + 1);
                    v[a * 8 + b * 4 + c * 2 + e] = TableSolver::lerp(lo, hi, il.frac);
                }
            }
        }
    }

    Result w[8];
    for (int a = 0; a < 2; ++a) {
        for (int b = 0; b < 2; ++b) {
            for (int c = 0; c < 2; ++c) {
                w[a * 4 + b * 2 + c] = TableSolver::lerp(v[a * 8 + b * 4 + c * 2],
                    v[a * 8 + b * 4 + c * 2 + 1], id.frac);
            }
        }
    }

    Result u[4];
    for (int a = 0; a < 2; ++a) {
        for (int b = 0; b < 2; ++b) {
            u[a * 2 + b] = TableSolver::lerp(w[a * 4 + b * 2], w[a * 4 + b * 2 + 1], im.frac);
        }
    }

    Result s[2];
    for (int a = 0; a < 2; ++a) {
        s[a] = TableSolver::lerp(u[a * 2], u[a * 2 + 1], iv.frac);
    }

    return TableSolver::lerp(s[0], s[1], iz.frac);
}

BallisticsResult TableSolver::solve(const DroneConfig& config, const AmmoParams& ammo) {
    Result tableResult = table.lookup(
        config.altitude,
        config.attackSpeed,
        ammo.mass,
        ammo.drag,
        ammo.lift
    );

    BallisticsResult result{};
    result.flightTime = tableResult.t;
    result.horizontalDistance = tableResult.hDist;

    LOG("Table ballistics calculated: flightTime=" << result.flightTime
        << ", horizontalDistance=" << result.horizontalDistance);

    return result;
}
