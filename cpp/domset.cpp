#include <cstdlib>

#include "../sparsedomsets/cpp/algorithms/c_dominating.hpp"
#include "../sparsedomsets/cpp/readwrite/c_edgelist.hpp"
#include "timer.hpp"

using namespace std;
using namespace sparsedomsets::readwrite;
using namespace sparsedomsets::algorithms;

int main(int argc, char* argv[]) {
  if (argc != 7) {
    printf("Usage: domset <input_path> <radius> <strategy> <seed> <num_threads> <output_path>\n");
    return 1;
  }

  // parse arguments
  auto input_path = argv[1];
  auto radius = atoi(argv[2]);
  auto strategy = atoi(argv[3]);
  auto seed = atoi(argv[4]);
  auto num_threads = atoi(argv[5]);
  auto output_path = argv[6];

  // create timer
  auto tm = Timer();

  // main logic
  tm.start("read_gxt()");
  auto G = read_gxt(input_path);
  tm.stop();

  tm.start("find_dominating_set()");
  auto doms = find_dominating_set(G, strategy, seed, radius, num_threads);
  tm.stop();

  tm.start("write_vertices()");
  write_vertices(doms, output_path, false);
  tm.stop();
}
