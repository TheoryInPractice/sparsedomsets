/**
 * Output: <number of vertices>,<number of dominators>,<congestion sum>
 */
#include <cstdlib>

#include "../sparsedomsets/cpp/graph/c_graph_ops.hpp"
#include "../sparsedomsets/cpp/readwrite/c_edgelist.hpp"

using namespace std;
using namespace sparsedomsets::readwrite;
using namespace sparsedomsets::graph;

int main(int argc, char* argv[]) {
  if (argc != 5) {
    printf("Usage: domset-stats <graph_path> <domset_path> <radius> <num_threads>\n");
    return 1;
  }

  // parse arguments
  auto graph_path = argv[1];
  auto domset_path = argv[2];
  auto radius = atoi(argv[3]);
  auto num_threads = atoi(argv[4]);

  // main logic
  auto G = read_gxt(graph_path);
  auto landmarks = read_vertices(domset_path);
  auto cong_sum = compute_congestion_sum(G, landmarks, radius, num_threads);
  printf("%d,%u,%lld\n", G.number_of_nodes(), landmarks.size(), cong_sum);
}
