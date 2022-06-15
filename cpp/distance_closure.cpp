#include "../sparsedomsets/cpp/graph/CIntGraph.hpp"
#include "../sparsedomsets/cpp/graph/c_graph_ops.hpp"
#include "../sparsedomsets/cpp/readwrite/c_edgelist.hpp"
#include <chrono>
#include <cstdlib>

using namespace std;
using namespace sparsedomsets::graph;
using namespace sparsedomsets::readwrite;

int main(int argc, char *argv[]) {
  if (argc != 5) {
    printf("Usage: distance_closure <input_path> <radius> <num_threads> <output_path>\n");
    return 1;
  }
  typedef std::chrono::high_resolution_clock Time;
  typedef std::chrono::duration<float> fsec;
  typedef CIntGraph Graph;

  // parse arguments
  auto input_path = argv[1];
  auto radius = atoi(argv[2]);
  auto num_threads = atoi(argv[3]);
  auto output_path = argv[4];

  auto t0 = Time::now();
  auto G = read_gxt(input_path);
  auto t1 = Time::now();
  fsec fs1 = t1 - t0;
  printf("read_gxt(): %.2f\n", fs1.count());

  auto H = distance_closure(G, radius, num_threads);
  auto t2 = Time::now();
  fsec fs2 = t2 - t1;
  printf("distance_closure(): %.2f\n", fs2.count());

  write_gxt(H, output_path, false);
  auto t3 = Time::now();
  fsec fs3 = t3 - t2;
  printf("write_gxt(): %.2f\n", fs3.count());
}