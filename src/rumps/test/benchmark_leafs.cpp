#include "../include/rumps/rumps.hpp"
#include "rumps/containers.hpp"
#include "rumps/rmp_leaf.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <benchmark/benchmark.h>

using namespace rumps;

template <template <typename, typename, typename> class ContainerNode = Node, template <typename, typename, size_t> class Container = DynamicNodeContainer>
void buildTree(int N_attractor, int N_repuslor, Root<SE3>& root) {
  auto& position_map = root.addChild<Position3DMap>();

  if (N_attractor > 0) {
    auto& goals = position_map.addChild<ContainerNode<Position3D, Position3D, Container<Position3D, PositionAttractor<Position3D>, 10000>>>();
    for (int i = 0; i < N_attractor; ++i) {
      Eigen::Vector3d c(i / 1000.0, 0, 0);
      goals.template addChild<PositionAttractor<Position3D>>(c);
    }
  }

  if (N_repuslor > 0) {
    auto& obstacles = position_map.addChild<ContainerNode<Position3D, Position3D, Container<Position3D, PositionRepulsor<Position3D>, 10000>>>();

    for (int i = 0; i < N_repuslor; ++i) {
      Eigen::Vector3d c(-i / 1000.0, 0, 0);
      obstacles.template addChild<PositionRepulsor<Position3D>>(c);
    }
  }
}

static void BM_Attractor(benchmark::State& state) {
  SE3::Vec a;
  auto q = SE3::Vec::Zero();
  auto q_dot = SE3::Vec::Zero();

  int N = state.range(0);  // Number of obstacles
  Root<SE3> root;
  buildTree(N, 0, root);
  for (auto _ : state) {
    // Timed part
    a = root.solve(q, q_dot);
  }
  benchmark::DoNotOptimize(a);
}
BENCHMARK(BM_Attractor)->Range(8, 8 << 10)->Unit(benchmark::kMillisecond);

static void BM_Repulsor(benchmark::State& state) {
  SE3::Vec a;
  auto q = SE3::Vec::Zero();
  auto q_dot = SE3::Vec::Zero();

  int N = state.range(0);  // Number of obstacles
  Root<SE3> root;
  buildTree(0, N, root);
  for (auto _ : state) {
    // Timed part
    a = root.solve(q, q_dot);

  }
  benchmark::DoNotOptimize(a);
}
BENCHMARK(BM_Repulsor)->Range(8, 8 << 10)->Unit(benchmark::kMillisecond);

// Benchmark function
static void BM_RepulsorStaticNode(benchmark::State& state) {
  SE3::Vec a;
  auto q = SE3::Vec::Zero();
  auto q_dot = SE3::Vec::Zero();

  int N = state.range(0);  // Number of obstacles
  Root<SE3> root;
  buildTree<Node, StaticNodeContainer>(0, N, root);
  for (auto _ : state) {

    // Timed part
    a = root.solve(q, q_dot);

  }
  benchmark::DoNotOptimize(a);
}
BENCHMARK(BM_RepulsorStaticNode)->Range(8, 8 << 10)->Unit(benchmark::kMillisecond);

static void BM_AttractorParallelNode(benchmark::State& state) {
  SE3::Vec a;
  auto q = SE3::Vec::Zero();
  auto q_dot = SE3::Vec::Zero();

  int N = state.range(0);  // Number of obstacles
  Root<SE3> root;
  buildTree<ParallelNode>(0, N, root);
  for (auto _ : state) {

    // Timed part
    a = root.solve(q, q_dot);

  }
  benchmark::DoNotOptimize(a);
}
BENCHMARK(BM_AttractorParallelNode)->Range(8, 8 << 10)->Unit(benchmark::kMillisecond);

static void BM_AttractorStaticParallelNode(benchmark::State& state) {
  SE3::Vec a;
  auto q = SE3::Vec::Zero();
  auto q_dot = SE3::Vec::Zero();

  int N = state.range(0);  // Number of obstacles
  Root<SE3> root;
  buildTree<ParallelNode, StaticNodeContainer>(0, N, root);
  for (auto _ : state) {
    // Timed part
    a = root.solve(q, q_dot);

  }
  benchmark::DoNotOptimize(a);
}
BENCHMARK(BM_AttractorStaticParallelNode)->Range(8, 8 << 10)->Unit(benchmark::kMillisecond);

static void BM_AttractorPushforward(benchmark::State& state) {
  auto q = SE3::Vec::Zero();
  auto q_dot = SE3::Vec::Zero();

  int N = state.range(0);  // Number of obstacles
  Root<SE3> root;
  buildTree(N, 0, root);
  for (auto _ : state) {
    // Timed part
    root.pushforward(q, q_dot);
  }
}
BENCHMARK(BM_AttractorPushforward)->Range(8, 8 << 10)->Unit(benchmark::kMillisecond);

static void BM_AttractorPullback(benchmark::State& state) {
  SE3::Vec f = SE3::Vec::Zero();
  SE3::MatrixM M = SE3::MatrixM::Zero();

  int N = state.range(0);  // Number of obstacles
  Root<SE3> root;
  buildTree(N, 0, root);
  for (auto _ : state) {
    root.pullback(f, M);

  }
  benchmark::DoNotOptimize(f);
  benchmark::DoNotOptimize(M);
}
BENCHMARK(BM_AttractorPullback)->Range(8, 8 << 10)->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();
