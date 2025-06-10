#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <string>

void printTournamentSelection(int k, int n = 10000)
{
  k = std::max(k, 1);
  std::cout << "k = " << k << " n = " << n << "\n";
  static std::random_device rd{};
  static std::mt19937 gen{ rd() };

  std::uniform_int_distribution getIdx0(0, n - 1);
  auto getIdx{ [&]() { return getIdx0(gen); } };

  auto tournament{ [&]() {
    int best = -1;
    for(int i{ 0 }; i < k; i++) {
      int next{ getIdx() };
      if(best == -1 || next < best) {
        best = next;
      }
    }
    return best;
  } };

  std::map<long, unsigned> histogram{};
  for(auto n{ 10000 }; n; --n)
    ++histogram[tournament()];

  long maxB{ 5 };
  for(const auto [k, v] : histogram) {
    if(v >= 100) {
      maxB = std::max(maxB, k + 2);
    }
  }

  for(const auto [b, v] : histogram) {
    if(b <= maxB)
      std::cout << std::setw(2) << b << ' ' << std::string(v / 200, '*') << '\n';
  }
}

int main()
{
  for(int k{ 1 }; k <= 11; k += 2) {
    printTournamentSelection(k);
    std::cout << "\n";
  }
  for(int k{ 1 }; k <= 50; k += 2) {
    for(int n{ 100 }; n <= 100; n += 20) {
      printTournamentSelection(k, n);
      std::cout << "\n";
    }
  }

  return 0;
}
