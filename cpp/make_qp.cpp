/**
 * Creates a Gurobi model file for solving the minimum-variance
 * distance-preserving neighborhood partitioning, given a graph and
 * chosen landmarks.
 */
#include <cstdlib>
#include <cstring>

#include <algorithm>
#include <queue>

#include "../sparsedomsets/cpp/graph/c_graph_ops.hpp"
#include "../sparsedomsets/cpp/readwrite/c_edgelist.hpp"
#include "timer.hpp"

using namespace std;
using namespace sparsedomsets::readwrite;
using namespace sparsedomsets::graph;

std::vector<std::vector<int>> choices;     // vertex -> reachable landmarks
std::vector<std::vector<int>> candidates;  // landmark -> reachable vertices

template <typename T>
static inline bool contains(std::unordered_set<T> const& s, T const& x) {
  return s.find(x) != s.end();
}

template <typename T>
static inline bool contains(std::vector<T> const& s, T const& x) {
  return std::find(s.begin(), s.end(), x) != s.end();
}

void compute_reachability(CIntDiGraph const& H, std::vector<int> const& landmarks) {
  // Assume H is a directed acyclic graph.
  // Assume each vertex in H except landmarks must have at least two choices.
  auto n = H.number_of_nodes();
  choices.clear();
  choices.resize(n);
  candidates.clear();
  candidates.resize(n);

  std::queue<int> q;
  for (auto u : landmarks) {
    std::unordered_set<int> visited;
    q.push(u);
    while (!q.empty()) {
      auto v = q.front();
      q.pop();

      if (!contains(visited, v)) {
        choices[v].push_back(u);  // v is reachable from u
        candidates[u].push_back(v);
        visited.insert(v);
      }

      for (auto w : H.out_neighbors(v)) q.push(w);
    }
  }
}

void create_model(CIntDiGraph const& H, std::vector<std::vector<int>> const& groups, std::vector<int> const& landmarks) {
  auto n = H.number_of_nodes();
  compute_reachability(H, landmarks);
  std::vector<int> effective_landmarks;  // landmarks that have any children
  for (auto u : landmarks) {
    if (candidates[u].size() >= 2) effective_landmarks.push_back(u);
  }
  for (auto u : effective_landmarks) { std::sort(candidates[u].begin(), candidates[u].end()); }

  // Objective
  printf("Minimize\n");
  std::string obj_padding = "";
  for (auto u: effective_landmarks) {
    printf("%s[ y_%d * y_%d ]", obj_padding.c_str(), u, u);
    obj_padding = " + ";
  }
  printf("\n");

  // Constraints
  printf("Subject To\n");

  // (1) Define y_u.
  for (auto u: effective_landmarks) {
    printf("y_%d", u);
    for (auto x: candidates[u]) {
      auto w = groups[x].size();
      if (w == 1) {
        printf(" - x_%d_%d", x, u);
      } else {
        printf(" - %d x_%d_%d", w, x, u);
      }
    }
    printf(" = %d\n", groups[u].size());  // u's weight
  }

  // (2) Each vertex must map to one landmark.
  for (int i = 0; i < n; ++i) {
    if (choices[i].size() < 2) continue;  // contracted, landmarks, or unreachable from any landmark
    std::string padding = "";
    for (auto u : choices[i]) {
      printf("%sx_%d_%d", padding.c_str(), i, u);
      padding = " + ";
    }
    printf(" = 1\n");
  }

  // (3) There must be an induced path from the landmark in each part.
  for (int i = 0; i < n; ++i) {
    if (choices[i].size() < 2) continue;  // contracted, landmarks, or unreachable from any landmark

    auto& nbrs = H.in_neighbors(i);
    for (auto u : choices[i]) {
      if (contains(nbrs, u)) continue;  // landmark is a direct parent

      printf("x_%d_%d", i, u);
      for (auto w : nbrs) {
        if (contains(choices[w], u)) printf(" - x_%d_%d", w, u);
      }
      printf(" <= 0\n");
    }
  }

  // Variables
  printf("Binary\n");
  for (int i = 0; i < n; ++i) {
    if (choices[i].size() < 2) continue;  // contracted, landmarks, or unreachable from any landmark
    std::string padding = "";
    for (auto u : choices[i]) {
      printf("%sx_%d_%d", padding.c_str(), i, u);
      padding = " ";
    }
    printf("\n");
  }
}

int main(int argc, char* argv[]) {
  if (argc != 3) {
    fprintf(stderr, "usage: make_qp <graph_path> <landmark_path>\n\n");
    fprintf(stderr, "Outputs a Gurobi model for solving neighborhood partitoning problems.\n\n");
    fprintf(stderr, "positional arguments:\n");
    fprintf(stderr, "  graph_path          path to input GXT file\n");
    fprintf(stderr, "  landmark_path       path to input landmark file\n");
    return 1;
  }

  // parse arguments
  auto graph_path = argv[1];
  auto landmark_path = argv[2];

  // create timer
  auto tm = Timer();

  // main logic
  tm.start("read_gxt()");
  auto G = read_gxt(graph_path);
  tm.stop();

  tm.start("read_vertices()");
  auto landmarks = read_vertices(landmark_path);
  tm.stop();

  tm.start("compact_neighborhood_kernel()");
  auto ret = compact_neighborhood_kernel(G, landmarks);
  tm.stop();

  tm.start("output model");
  create_model(ret.first, ret.second, landmarks);
  tm.stop();
}
