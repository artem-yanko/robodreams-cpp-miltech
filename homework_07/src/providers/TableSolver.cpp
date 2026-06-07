#include "providers/TableSolver.hpp"
#include "utils/logger.hpp"
#include <fstream>

TableSolver::TableSolver(const std::string& ballisticTablePath) {
    if (!table_.load(ballisticTablePath)) {
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
    for (double& v : axisM)  inputFile >> v;
    for (double& v : axisD)  inputFile >> v;
    for (double& v : axisL)  inputFile >> v;

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