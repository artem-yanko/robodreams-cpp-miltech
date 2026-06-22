#include <iostream>
#include <fstream>
#include "ballistics.hpp"

int main(int argc, char** argv)
{
  if (argc != 3) {
    std::cerr << "ERROR: missing arguments\nUsage: ballistics_check <input_path> <output_path>\n";
    return 1;
  }
  BallisticsInput input;
  BallisticsResult result;

  // Читання вхідних даних з файлу
  readFile(argv[1], input);

  // Розрахунок баллістики
  if (!calculateBallitsics(input, result)) {
    std::cerr << "ERROR: Не вдалося розрахувати баллістику." << '\n';
    return 1;
  }

  // Запис результатів у файл
  writeFile(argv[2], result);

  return 0;
}