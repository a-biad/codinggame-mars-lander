#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <string>

void printWalkAlgorithmHist(double b, int vmin = -10000, int vmax = 10000) {
  b = std::min(b, 0.999);
  std::cout << "b = " << b << " vmin = " << vmin << " vmax = " << vmax << "\n";
  std::random_device rd{};
  std::mt19937 gen{rd()};

  std::bernoulli_distribution flip0{b};
  // Returns true with probability b
  auto flip{[&gen, &flip0]() { return flip0(gen); }};

  std::bernoulli_distribution n0{0.5};
  const int n1[]{-1, 1};
  auto n{
      [&gen, &n0, &n1]() { return n1[n0(gen)]; } // Return -1 or 1
  };

  auto walk{[&flip, &n, &vmin, &vmax]() {
    int v{0};
    do {
      int nr{n()};
      // Check if new value is withing bounds
      if (v + nr <= vmax && v + nr >= vmin)
        v = v + nr;
      else if (v - nr <= vmax && v - nr >= vmin)
        v = v - nr;
    } while (flip());
    return v;
  }};

  std::map<long, unsigned> histogram{};
  for (auto n{10000}; n; --n)
    ++histogram[walk()];

  long minK{0};
  long maxK{0};
  for (const auto [k, v] : histogram) {
    if (v >= 100) {
      minK = std::min(minK, k - 2);
      maxK = std::max(maxK, k + 2);
    }
  }

  for (const auto [k, v] : histogram) {
    if (minK <= k && k <= maxK)
      std::cout << std::setw(2) << k << ' ' << std::string(v / 200, '*')
                << '\n';
  }
}

int main() {
  for (float b{0.05}; b <= 0.95; b += 0.15) {
    printWalkAlgorithmHist(b);
    std::cout << "\n";
  }
  for (float b{0.05}; b <= 0.95; b += 0.15) {
    for (int v{0}; v <= 4; v++) {
      long vmin{0 - v};
      long vmax{4 - v};
      printWalkAlgorithmHist(b, vmin, vmax);
      std::cout << "\n";
    }
  }

  return 0;
}
