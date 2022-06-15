#include <cstdlib>

#include "../sparsedomsets/cpp/algorithms/c_partition.hpp"
#include "../sparsedomsets/cpp/readwrite/c_edgelist.hpp"
#include "timer.hpp"

using namespace std;
using namespace sparsedomsets::readwrite;
using namespace sparsedomsets::algorithms;

int main(int argc, char* argv[]) {
  if (argc != 6) {
    printf("Usage: nbrprt <graph_path> <domset_path> <strategy> <seed> <output_path>\n");
    return 1;
  }

  // parse arguments
  auto graph_path = argv[1];
  auto domset_path = argv[2];
  auto strategy = atoi(argv[3]);
  auto seed = atoi(argv[4]);
  auto output_path = argv[5];

  // create timer
  auto tm = Timer();

  // main logic
  tm.start("read_gxt()");
  auto G = read_gxt(graph_path);
  tm.stop();

  tm.start("read_vertices()");
  auto doms = read_vertices(domset_path);
  tm.stop();

  tm.start("neighborhood_partition()");
  auto parts = neighborhood_partition(G, doms, strategy, seed);
  tm.stop();

  tm.start("write_parts()");
  write_parts(parts, output_path);
  tm.stop();
}
