#include <cstdlib>

#include "../sparsedomsets/cpp/graph/CIntGraph.hpp"
#include "../sparsedomsets/cpp/graph/c_graph_ops.hpp"
#include "../sparsedomsets/cpp/readwrite/c_edgelist.hpp"
#include "timer.hpp"

using namespace std;
using namespace sparsedomsets::readwrite;

int main(int argc, char *argv[]) {
  if (argc != 4) {
    printf("Usage: distance_closure_size <input_path> <radius> <num_threads>\n");
    return 1;
  }

  // parse arguments
  auto input_path = argv[1];
  auto radius = atoi(argv[2]);
  auto num_threads = atoi(argv[3]);

  // create timer
  auto tm = Timer();

  // main logic
  tm.start("read_gxt()");
  auto G = read_gxt(input_path);
  tm.stop();

  tm.start("distance_closure()");
  auto H = distance_closure(G, radius, num_threads);
  tm.stop();

  // output result
  printf("%d,%lld\n", H.number_of_nodes(), H.number_of_edges());
}