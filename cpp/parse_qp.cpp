/**
 * Parses a Gurobi solution file for solving the minimum-variance
 * distance-preserving neighborhood partitioning, given a graph and
 * chosen landmarks.
 */
#include <cstdlib>
#include <cstring>

#include <algorithm>

#include "../sparsedomsets/cpp/graph/c_graph_ops.hpp"
#include "../sparsedomsets/cpp/readwrite/c_edgelist.hpp"
#include "timer.hpp"

using namespace std;
using namespace sparsedomsets::readwrite;
using namespace sparsedomsets::graph;

template <typename T>
static inline bool contains(std::unordered_set<T> const& s, T const& x) {
  return s.find(x) != s.end();
}

template <typename T>
static inline bool contains(std::vector<T> const& s, T const& x) {
  return std::find(s.begin(), s.end(), x) != s.end();
}

std::vector<int> read_solution(int n, char const* solution_path) {
  std::vector<int> assign(n, -1);  // -1: unassigned

  FILE* fp;
  if (!(fp = fopen(solution_path, "r"))) { perror("IO Error"); }

  int const BUFFER_SIZE = 256;
  char buf[BUFFER_SIZE];
  fgets(buf, BUFFER_SIZE, fp);  // skip the first line

  int ret, v, u;
  double x;
  char ch;

  for (;;) {
    ret = fscanf(fp, "%c", &ch);
    if (ret == EOF) break;
    if (ret != 1) continue;
    if (ch == 'y') {
      fgets(buf, BUFFER_SIZE, fp);  // ignore this line
      continue;
    }

    ret = fscanf(fp, "_%d_%d %lf\n", &v, &u, &x);
    if (ret == 3) {
      if (x > 0.9) {
        // might be a fractional number but should be more than 0.999
        assign[v] = u;  // v is assigned to dominator u
      }
    } else {
      printf("ret=%d\n", ret);
      perror("invalid content");
    }
  }

  fclose(fp);

  return assign;
}

void output_result(std::vector<int> const& landmarks, std::vector<int>& assign, std::vector<std::vector<int>> const& groups) {
  int n = groups.size();

  // assign landmarks to themselves
  for (auto u : landmarks) assign[u] = u;

  // assign all group members
  for (int i = 0; i < n; ++i) {
    if (groups[i].size() > 1) {
      if (assign[i] < 0) abort();
      for (auto w : groups[i]) {
        if (assign[w] < 0) {
          assign[w] = assign[i];
        } else if (assign[w] != assign[i]) {
          abort();
        }
      }
    }
  }

  // sanity check
  for (int i = 0; i < n; ++i) {
    if (assign[i] < 0) abort();
  }

  // aggregate by landmarks
  std::vector<std::vector<int>> dict(n);
  for (int i = 0; i < n; ++i) dict[assign[i]].push_back(i);

  for (auto u : landmarks) {
    std::sort(dict[u].begin(), dict[u].end());
    printf("%d", u);
    for (auto v : dict[u]) printf(" %d", v);
    printf("\n");
  }
}

int main(int argc, char* argv[]) {
  if (argc != 4) {
    fprintf(stderr, "usage: parse_qp <graph_path> <landmark_path> <solution_path\n\n");
    fprintf(stderr, "Parses a Gurobi solution for solving neighborhood partitoning problems.\n\n");
    fprintf(stderr, "positional arguments:\n");
    fprintf(stderr, "  graph_path          path to input GXT file\n");
    fprintf(stderr, "  landmark_path       path to input landmark file\n");
    fprintf(stderr, "  solution_path       path to Gurobi's solution file\n");
    return 1;
  }

  // parse arguments
  auto graph_path = argv[1];
  auto landmark_path = argv[2];
  auto solution_path = argv[3];

  // create timer
  auto tm = Timer();

  // main logic
  tm.start("read_gxt()");
  auto G = read_gxt(graph_path);
  tm.stop();

  tm.start("read_vertices()");
  auto landmarks = read_vertices(landmark_path);
  tm.stop();

  tm.start("read_solution()");
  auto assign = read_solution(G.number_of_nodes(), solution_path);
  tm.stop();

  tm.start("compact_neighborhood_kernel()");
  auto cnk = compact_neighborhood_kernel(G, landmarks);
  tm.stop();

  tm.start("output_result()");
  output_result(landmarks, assign, cnk.second);
  tm.stop();
}
