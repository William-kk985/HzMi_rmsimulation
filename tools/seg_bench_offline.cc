// tools/seg_bench_offline.cc —— linefit 单帧耗时的离线基准（不经 ROS/DDS，纯 CPU 计时）
//
// 为什么需要它：判"感知链卡顿"到底是算力还是交付。本项目实测结论：
//   segment() 在 30000 点 / sim 参数下只要 ~1.0 ms（最慢 1.8 ms），而线上 /segmentation/obstacle
//   只有 0.31 Hz、空洞最长 20.8 s -> 说明卡的不是算法，是大点云的订阅交付（QoS）。
//
// 用法（在仓库根目录）：
//   # 1) 从 bag 抽出若干帧点云为简单二进制（[int32 n][float32 x,y,z]*n）：
//   #    python3 -c "..."  见 docs/issues_and_findings.md #25 的复现命令
//   # 2) 编译并运行：
//   VTK=$(ldd install/linefit_ground_segmentation/lib/liblinefit_ground_segmentation.so \
//         | grep -oE "libvtk[A-Za-z0-9]+-9\\.1" | sort -u | sed "s/^lib/-l/")
//   g++ -O2 -fopenmp -o /tmp/segbench tools/seg_bench_offline.cc \
//     -I install/linefit_ground_segmentation/include -I/usr/include/pcl-1.12 \
//     -I/usr/include/eigen3 -I/usr/include/vtk-9.1 \
//     -L install/linefit_ground_segmentation/lib -llinefit_ground_segmentation \
//     -lpcl_common -lpcl_visualization $VTK \
//     -Wl,-rpath,$PWD/install/linefit_ground_segmentation/lib
//   /tmp/segbench frames.bin

#include <ground_segmentation/ground_segmentation.h>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <vector>

int main(int argc, char** argv) {
  GroundSegmentationParams params;
  params.visualize = false;
  params.r_min_square = 0.2 * 0.2;
  params.r_max_square = 50.0 * 50.0;
  params.n_bins = 120;
  params.n_segments = 360;
  params.max_dist_to_line = 0.1;
  params.min_slope = -0.4;
  params.max_slope = 0.4;
  params.n_threads = 4;
  params.max_error_square = 0.05 * 0.05;
  params.long_threshold = 1.0;
  params.max_long_height = 0.1;
  params.max_start_height = 0.5;
  params.sensor_height = 0.275;
  params.line_search_angle = 0.8;

  GroundSegmentation seg(params);
  std::ifstream f(argv[1], std::ios::binary);
  int frame = 0;
  double worst = 0, total = 0;
  while (true) {
    int32_t n = 0;
    f.read(reinterpret_cast<char*>(&n), 4);
    if (!f || n <= 0) break;
    PointCloud cloud;
    cloud.resize(n);
    f.read(reinterpret_cast<char*>(cloud.points.data()), sizeof(float) * 3 * n);
    std::vector<int> labels;
    auto t0 = std::chrono::steady_clock::now();
    seg.segment(cloud, &labels);
    auto t1 = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    int ground = 0;
    for (int l : labels) if (l == 1) ++ground;
    worst = ms > worst ? ms : worst; total += ms; ++frame;
    printf("帧%2d  n=%6d  segment() = %9.2f ms   labels=%6zu  ground=%6d  obstacle=%6d\n",
           frame, n, ms, labels.size(), ground, n - ground);
    fflush(stdout);
  }
  printf("\n共 %d 帧：合计 %.2f s，平均 %.2f ms/帧，最慢 %.2f ms\n", frame, total/1000.0, total/frame, worst);
  return 0;
}
