#include <fstream>
#include <iostream>
#include <cstring>

bool readTargets(double targetXInTime[5][60], double targetYInTime[5][60]) {

    std::ifstream targetsFile("targets.txt");
    if (!targetsFile) {
        std::cerr << "ERROR: Не вдалося відкрити файл \"targets.txt\"" << std::endl;
        return false;
    }

    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 60; ++j) {
            if (!(targetsFile >> targetXInTime[i][j])) {
                return false;
            }
        }
    }

    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 60; ++j) {
            if (!(targetsFile >> targetYInTime[i][j])) {
                return false;
            }
        }
    }
    return true;

}

int main() {
    double targetXInTime[5][60] = {{0.0}};
    double targetYInTime[5][60] = {{0.0}};

    if (!readTargets(targetXInTime, targetYInTime)) {
        std::cerr << "ERROR: Невірний формат даних у файлі \"targets.txt\"" << std::endl;
        return 1;
    }


    std::cout << targetXInTime[0][0]  << std::endl;
    std::cout << targetYInTime[0][0]  << std::endl;


    return 0;
}