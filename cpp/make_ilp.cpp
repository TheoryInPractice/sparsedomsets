#include <cstdlib>
#include <cstring>

#include "../sparsedomsets/cpp/graph/c_graph_ops.hpp"
#include "../sparsedomsets/cpp/readwrite/c_edgelist.hpp"
#include "timer.hpp"

using namespace std;
using namespace sparsedomsets::readwrite;
using namespace sparsedomsets::graph;

int main(int argc, char* argv[]) {
  if (argc != 5 || (strcmp(argv[3], "mds") != 0 && strcmp(argv[3], "mac") != 0)) {
    fprintf(stderr, "usage: make_ilp <input_path> <radius> <problem> <num_threads>\n\n");
    fprintf(stderr, "Outputs a Gurobi model for solving dominating set problems.\n\n");
    fprintf(stderr, "positional arguments:\n");
    fprintf(stderr, "  input_path          path to input GXT file\n");
    fprintf(stderr, "  radius              radius of a dominating set\n");
    fprintf(stderr, "  problem {mds, mac}  problem to solve\n");
    fprintf(stderr, "  num_threads         number of threads\n");
    return 1;
  }

  // parse arguments
  auto input_path = argv[1];
  auto radius = atoi(argv[2]);
  auto problem = argv[3];
  auto num_threads = atoi(argv[4]);

  bool is_mds = strcmp(problem, "mds") == 0;

  // create timer
  auto tm = Timer();

  // main logic
  tm.start("read_gxt()");
  auto G = read_gxt(input_path);
  tm.stop();

  tm.start("compute_all_closed_neighborhood()");
  auto cnbrs = compute_all_closed_neighborhood(G, radius, num_threads);  // multi-threaded
  tm.stop();

  tm.start("output model");

  // output objective
  int n = G.number_of_nodes();
  int chunk_size = 100;
  printf("Minimize\n");
  for (int i = 0; i < n; i += chunk_size) {
    for (int j = i; j < std::min(n, i + chunk_size); ++j) {
      if (is_mds) {
        printf(" + x_%d", j);
      } else {
        printf(" + %d x_%d", cnbrs[j].size(), j);
      }
    }
    printf("\n");
  }

  // output constraints
  printf("Subject To\n");
  for (int i = 0; i < n; ++i) {
    for (auto k : cnbrs[i]) printf(" + x_%d", k);
    printf(" >= 1\n");
  }

  // output variables
  printf("Binary\n");
  for (int i = 0; i < n; i += chunk_size) {
    for (int j = i; j < std::min(n, i + chunk_size); ++j) { printf(" x_%d", j); }
    printf("\n");
  }
  tm.stop();
}
