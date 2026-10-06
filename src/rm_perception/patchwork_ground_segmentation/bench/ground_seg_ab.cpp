// =============================================================================
// ground_seg_ab —— 地面分割 A/B 台架（linefit vs Patchwork++，**同一批点**上跑）
// -----------------------------------------------------------------------------
// 为什么不用"两个节点各回放一遍 bag"：那样两边的输入帧、时序状态、CPU 竞争都不一样，
// 量出来的"一致率"混了回放噪声。本台架把**同一份点集**先后喂给两个库，逐点比对：
//   · 一致率 / 混淆矩阵（both ground / linefit-only / patchwork-only / both obstacle）
//   · **按"离地高度"分桶的障碍保留率**（回答：0.2/0.3 m 台阶、矮墙还留得住吗）
//   · **按"局部法向倾角"分桶的障碍保留率**（回答：10~22° 坡面被判成地面了吗）
//   · 单帧耗时（median/p95/max）
//
// 两个模式：
//   --frames-dir DIR   读 DIR/frame_XXXXXX.bin（Nx3 float32 原始点，雷达系，脚本 tools 从 bag 导出）
//   --synthetic        构造一份 RMUC2026 风格地形（平地 + 10/15/22° 坡 + 0.2/0.3 m 台阶 +
//                      0.15/0.3/0.4 m 薄墙），传感器按 sensor_height 架在平地之上
//
// 参数默认值 = 两侧**各自节点正在用的参数文件**的默认值（linefit: config/segmentation_sim.yaml；
// patchwork: config/ground_segmentation_sim.yaml），可被命令行覆盖 ⇒ 台架数值 = 节点数值。
//
// ⚠ 已知限制（写进 docs）：本台架**不**复现 Patchwork++ 的跨帧自适应（elevation_thr/flatness_thr/
//   sensor_height 是逐帧更新的）。台架为"逐帧独立"口径；节点里是"帧间连续"口径。差别在
//   ~10 帧内消失（自适应收敛），且台架**更保守**（冷启动的初值最不利）。
// =============================================================================

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "ground_segmentation/ground_segmentation.h"
#include "patchwork/patchworkpp.h"

namespace fs = std::filesystem;

// ---------------------------------------------------------------- 参数（两套）
struct LinefitCfg
{
  // 默认 = src/rm_perception/linefit_ground_segementation_ros2/.../config/segmentation_sim.yaml
  double r_min = 0.2, r_max = 50.0, max_dist_to_line = 0.05;
  int n_bins = 120, n_segments = 360, n_threads = 4;
  double min_slope = -0.4, max_slope = 0.4, max_fit_error = 0.05;
  double long_threshold = 1.0, max_long_height = 0.1, max_start_height = 0.5;
  double sensor_height = 0.226, line_search_angle = 0.8;
};

struct AbOptions
{
  std::string frames_dir;
  bool synthetic = false;
  int max_frames = 100000;
  int synthetic_frames = 12;
  double sensor_height = 0.226;
  std::string out_json;
  // patchwork 侧可覆盖的关键参数
  double th_dist = 0.08, min_range = 0.2, max_range = 20.0, uprightness_thr = 0.707;
  int num_iter = 3, num_min_pts = 10, num_zones = 4, num_rings_of_interest = 4;
  bool enable_TGR = true, enable_RVPF = true;
  // 法向估计（倾角分桶用）邻域半径
  bool drop_zeros = false;
  double normal_radius = 0.25;
  int min_neighbors = 8;
};

// ---------------------------------------------------------------- 计时
static double now_ms()
{
  using namespace std::chrono;
  return duration<double, std::milli>(steady_clock::now().time_since_epoch()).count();
}

struct Stats
{
  std::vector<double> v;
  void add(double x) {v.push_back(x);}
  double median() const
  {
    if (v.empty()) {return 0.0;}
    std::vector<double> s = v;
    std::sort(s.begin(), s.end());
    return s[s.size() / 2];
  }
  double pct(double p) const
  {
    if (v.empty()) {return 0.0;}
    std::vector<double> s = v;
    std::sort(s.begin(), s.end());
    size_t i = static_cast<size_t>(p * (s.size() - 1) + 0.5);
    return s[std::min(i, s.size() - 1)];
  }
  double max() const {return v.empty() ? 0.0 : *std::max_element(v.begin(), v.end());}
  double mean() const
  {
    if (v.empty()) {return 0.0;}
    double s = 0; for (double x : v) {s += x;}
    return s / static_cast<double>(v.size());
  }
};

// ---------------------------------------------------------------- 分桶
struct Bucket
{
  const char * name;
  double lo, hi;   // [lo, hi)
  long long total = 0, lf_obs = 0, pw_obs = 0;
};

static std::vector<Bucket> make_height_buckets()
{
  return {
    {"h < -0.30", -1e9, -0.30},
    {"-0.30 <= h < -0.10", -0.30, -0.10},
    {"-0.10 <= h < 0.00", -0.10, 0.00},
    {"0.00 <= h < 0.05", 0.00, 0.05},
    {"0.05 <= h < 0.10", 0.05, 0.10},
    {"0.10 <= h < 0.20", 0.10, 0.20},
    {"0.20 <= h < 0.30", 0.20, 0.30},
    {"0.30 <= h < 0.40", 0.30, 0.40},
    {"0.40 <= h < 1.00", 0.40, 1.00},
    {"h >= 1.00", 1.00, 1e9},
  };
}

// 真实帧的"按高度带的方位覆盖率"：对每个高度带，按 p2l 的 0.0043 rad 分方位 bin，
// 统计"该带里有点的方位 bin 中，有多少至少还留了一个**落在 p2l 高度带内**的障碍点"。
// 这是"这个高度上的特征还会不会出现在 /scan 里"的直接答案（p2l 每 bin 只留最近一个点）。
struct BandCov
{
  const char * name;
  double lo, hi;
  std::set<int> all, lf, pw;
};

static std::vector<BandCov> make_band_cov()
{
  std::vector<BandCov> v(6);
  const double lo[6] = {0.05, 0.10, 0.20, 0.30, 0.40, 1.00};
  const double hi[6] = {0.10, 0.20, 0.30, 0.40, 1.00, 1e9};
  const char * nm[6] = {"0.05<=h<0.10", "0.10<=h<0.20", "0.20<=h<0.30",
                        "0.30<=h<0.40", "0.40<=h<1.00", "h>=1.00"};
  for (int i = 0; i < 6; ++i) {
    v[i].name = nm[i];
    v[i].lo = lo[i];
    v[i].hi = hi[i];
  }
  return v;
}

static std::vector<Bucket> make_tilt_buckets()
{
  return {
    {"tilt 0-5 deg", 0.0, 5.0},
    {"tilt 5-10 deg", 5.0, 10.0},
    {"tilt 10-15 deg", 10.0, 15.0},
    {"tilt 15-22 deg", 15.0, 22.0},
    {"tilt 22-35 deg", 22.0, 35.0},
    {"tilt 35-60 deg", 35.0, 60.0},
    {"tilt 60-90 deg", 60.0, 90.0001},
  };
}

static void tally(std::vector<Bucket> & bs, double x, bool lf_obstacle, bool pw_obstacle)
{
  for (auto & b : bs) {
    if (x >= b.lo && x < b.hi) {
      b.total++;
      if (lf_obstacle) {b.lf_obs++;}
      if (pw_obstacle) {b.pw_obs++;}
      return;
    }
  }
}

// ---------------------------------------------------------------- 局部法向（倾角）
// 3D 网格哈希 + 邻域 PCA。不引入 PCL features 依赖（PCL 1.12 的 NormalEstimation 要额外的
// features/kdtree 组件，而这里只差一个 acos）。
static void estimate_normals(
  const std::vector<Eigen::Vector3f> & pts, double radius, int min_neighbors,
  std::vector<float> * tilt_deg)
{
  const double cell = radius;
  const auto key = [cell](double x, double y, double z) {
      return std::make_tuple(
        static_cast<long long>(std::floor(x / cell)),
        static_cast<long long>(std::floor(y / cell)),
        static_cast<long long>(std::floor(z / cell)));
    };
  std::map<std::tuple<long long, long long, long long>, std::vector<int>> grid;
  for (size_t i = 0; i < pts.size(); ++i) {
    grid[key(pts[i].x(), pts[i].y(), pts[i].z())].push_back(static_cast<int>(i));
  }
  tilt_deg->assign(pts.size(), -1.0f);
  const double r2 = radius * radius;
  std::vector<int> nb;
  for (size_t i = 0; i < pts.size(); ++i) {
    const auto k0 = key(pts[i].x(), pts[i].y(), pts[i].z());
    nb.clear();
    for (long long dx = -1; dx <= 1; ++dx) {
      for (long long dy = -1; dy <= 1; ++dy) {
        for (long long dz = -1; dz <= 1; ++dz) {
          auto it = grid.find(
            std::make_tuple(std::get<0>(k0) + dx, std::get<1>(k0) + dy, std::get<2>(k0) + dz));
          if (it == grid.end()) {continue;}
          for (int j : it->second) {
            if ((pts[j] - pts[i]).squaredNorm() <= r2) {nb.push_back(j);}
          }
        }
      }
    }
    if (static_cast<int>(nb.size()) < min_neighbors) {continue;}
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for (int j : nb) {mean += pts[j];}
    mean /= static_cast<float>(nb.size());
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (int j : nb) {
      const Eigen::Vector3f d = pts[j] - mean;
      cov += d * d.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
    const Eigen::Vector3f n = es.eigenvectors().col(0);   // 最小特征值方向 = 法向
    double cos_t = std::fabs(static_cast<double>(n.z()));
    cos_t = std::min(1.0, std::max(0.0, cos_t));
    (*tilt_deg)[i] = static_cast<float>(std::acos(cos_t) * 180.0 / M_PI);
  }
}

// ---------------------------------------------------------------- 数据读取
static bool read_bin(const std::string & path, std::vector<Eigen::Vector3f> * out)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) {return false;}
  f.seekg(0, std::ios::end);
  const std::streamoff bytes = f.tellg();
  f.seekg(0, std::ios::beg);
  if (bytes <= 0 || bytes % (3 * sizeof(float)) != 0) {return false;}
  out->resize(static_cast<size_t>(bytes / (3 * sizeof(float))));
  f.read(reinterpret_cast<char *>(out->data()), bytes);
  return static_cast<bool>(f);
}

// ---------------------------------------------------------------- 合成地形
// 构造"RMUC2026 风格"的传感器系点云：平地 + 可选坡 + 可选台阶 + 薄墙。
// 全部解析生成 ⇒ **每个点的真值类别已知**（kFlatGround / kRampSurface / ...），
// 于是"坡面被判成地面了吗""台阶/薄墙还留得住吗"可以**按真值类别**回答，不靠猜。
enum Kind {
  kFlatGround = 0,
  kRampSurface = 1,
  kRampTop = 2,
  kTerraceTop = 3,
  kTerraceFace = 4,
  kWall = 5,
  kNumKinds = 6,
};
static const char * kind_name(int k)
{
  switch (k) {
    case kFlatGround: return "平地(真值地面)";
    case kRampSurface: return "坡面(真值地面)";
    case kRampTop: return "坡顶平台(真值地面)";
    case kTerraceTop: return "台阶顶面(真值障碍)";
    case kTerraceFace: return "台阶立面(真值障碍)";
    case kWall: return "薄墙(真值障碍)";
    default: return "?";
  }
}

struct TerrainSpec
{
  const char * name;
  double ramp_deg;         // >0 = 3 m 长的连续坡
  double terrace_height;   // >0 = 带**竖直立面**的台阶（老场景）
  double wall_height;      // >0 = 薄墙
  bool sensor_on_ramp;     // true = 传感器架在坡中部（车正在爬坡）
  // ★ RMUC2026 现场几何的**忠实代理**（数值来自对 RMUC2026.stl 的实测，见 docs §5.2）：
  //   场地 = 地面(z=0) + **0.2/0.3 m 平台**，平台边缘是 **10~22° 倒角**（不是长坡），
  //   外加一个 **−0.2 m 的坑**。chamfer_height > 0 时按 chamfer_deg 建"地面→倒角→平台"。
  double chamfer_height = 0.0;
  double chamfer_deg = 15.0;
};

struct SynthCloud
{
  std::vector<Eigen::Vector3f> pts;
  std::vector<int> kind;
  double sensor_z;   // 传感器在"平地"坐标系里的高度（用于 h = z - (floor_z) 的换算）
  double floor_z;    // 传感器系里平地的 z（= -sensor_z）
};

// 场景坐标系约定：**世界系**里地面 z=0、坡沿 +x 上升；最后整体平移到传感器系
// （传感器在 (0,0,sensor_height_world)）。
static SynthCloud build_terrain(const TerrainSpec & s, double sensor_height)
{
  SynthCloud c;
  const double deg = M_PI / 180.0;
  const double t = std::tan(s.ramp_deg * deg);
  const double ramp_x0 = 1.0, ramp_run = 3.0;
  const double ramp_rise = ramp_run * t;
  // 传感器世界高度：默认 0.226（平地）；上坡场景则抬到坡面之上同一相对高度
  double sensor_world_z = sensor_height;
  double sensor_world_x = 0.0;
  if (s.sensor_on_ramp) {
    sensor_world_x = ramp_x0 + ramp_run / 2.0;
    sensor_world_z = (ramp_run / 2.0) * t + sensor_height;
  }
  auto push = [&](double wx, double wy, double wz, int k) {
      c.pts.emplace_back(
        static_cast<float>(wx - sensor_world_x), static_cast<float>(wy),
        static_cast<float>(wz - sensor_world_z));
      c.kind.push_back(k);
    };

  // 平地
  for (double x = -8.0; x <= 8.0; x += 0.1) {
    for (double y = -8.0; y <= 8.0; y += 0.1) {
      push(x, y, 0.0, kFlatGround);
    }
  }
  if (s.ramp_deg > 0.0) {
    const double slope_len = ramp_run / std::cos(s.ramp_deg * deg);
    for (double u = 0.0; u <= slope_len; u += 0.06) {
      for (double y = -4.0; y <= 4.0; y += 0.06) {
        push(ramp_x0 + u * std::cos(s.ramp_deg * deg), y,
          u * std::sin(s.ramp_deg * deg), kRampSurface);
      }
    }
    for (double x = ramp_x0 + ramp_run; x <= ramp_x0 + ramp_run + 3.0; x += 0.1) {
      for (double y = -4.0; y <= 4.0; y += 0.1) {
        push(x, y, ramp_rise, kRampTop);
      }
    }
  }
  if (s.terrace_height > 0.0) {
    // 台阶：顶面 x∈[-5,-2], y∈[2,6]；含竖直立面（朝 -x，位于 x=-2）
    for (double x = -5.0; x <= -2.0; x += 0.08) {
      for (double y = 2.0; y <= 6.0; y += 0.08) {
        push(x, y, s.terrace_height, kTerraceTop);
      }
    }
    for (double z = 0.0; z <= s.terrace_height; z += 0.02) {
      for (double y = 2.0; y <= 6.0; y += 0.08) {
        push(-2.0, y, z, kTerraceFace);
      }
    }
  }
  if (s.wall_height > 0.0) {
    // 薄墙：y = -3 平面（单层点），0.15/0.30/0.40 m 高
    for (double x = -6.0; x <= 6.0; x += 0.05) {
      for (double z = 0.0; z <= s.wall_height; z += 0.03) {
        push(x, -3.0, z, kWall);
      }
    }
  }
  if (s.chamfer_height != 0.0) {
    // 倒角台阶：x < x0 为地面(世界 z=0)，x0..x0+run 为倒角，之后为平台(world z=h)。
    // h<0 ⇒ 坑（往下的倒角）。倒角面按 0.05 m 沿斜面采样；平台顶面 0.08 m 网格。
    const double h = s.chamfer_height;
    const double ang = s.chamfer_deg * deg;
    const double run = std::fabs(h) / std::tan(ang);
    const double x0 = 1.5;
    const double sgn = (h > 0.0) ? 1.0 : -1.0;
    const double slen = std::sqrt(run * run + h * h);
    for (double u = 0.0; u <= slen; u += 0.05) {
      for (double y = -5.0; y <= 5.0; y += 0.06) {
        push(x0 + u * std::cos(ang), y, sgn * u * std::sin(ang), kRampSurface);
      }
    }
    for (double x = x0 + run; x <= x0 + run + 3.0; x += 0.08) {
      for (double y = -5.0; y <= 5.0; y += 0.08) {
        push(x, y, h, kRampTop);
      }
    }
    for (double z = (h > 0.0 ? 0.0 : h); z <= (h > 0.0 ? h : 0.0); z += 0.02) {
      // 平台立面朝 -x（在 x0+run 处）——倒角场景下这条面几乎不露，保留以备 pit 用
      for (double y = -5.0; y <= 5.0; y += 0.08) {
        push(x0 + run, y, z, kTerraceFace);
      }
    }
  }
  c.sensor_z = sensor_world_z;
  c.floor_z = -sensor_world_z;
  return c;
}

// ---------------------------------------------------------------- 主流程
int main(int argc, char ** argv)
{
  AbOptions o;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string {
        if (i + 1 >= argc) {std::cerr << "缺少参数值: " << a << "\n"; std::exit(2);}
        return argv[++i];
      };
    if (a == "--frames-dir") {o.frames_dir = next();}
    else if (a == "--synthetic") {o.synthetic = true;}
    else if (a == "--max-frames") {o.max_frames = std::stoi(next());}
    else if (a == "--synthetic-frames") {o.synthetic_frames = std::stoi(next());}
    else if (a == "--sensor-height") {o.sensor_height = std::stod(next());}
    else if (a == "--th-dist") {o.th_dist = std::stod(next());}
    else if (a == "--min-range") {o.min_range = std::stod(next());}
    else if (a == "--max-range") {o.max_range = std::stod(next());}
    else if (a == "--uprightness-thr") {o.uprightness_thr = std::stod(next());}
    else if (a == "--num-iter") {o.num_iter = std::stoi(next());}
    else if (a == "--num-min-pts") {o.num_min_pts = std::stoi(next());}
    else if (a == "--no-tgr") {o.enable_TGR = false;}
    else if (a == "--drop-zeros") {o.drop_zeros = true;}
    else if (a == "--normal-radius") {o.normal_radius = std::stod(next());}
    else if (a == "--out") {o.out_json = next();}
    else if (a == "--help" || a == "-h") {
      std::cout <<
        "usage: ground_seg_ab (--frames-dir DIR | --synthetic) [--max-frames N] "
        "[--sensor-height H] [--th-dist D] [--min-range R] [--max-range R] "
        "[--uprightness-thr U] [--num-iter N] [--num-min-pts N] [--no-tgr] "
        "[--normal-radius R] [--out FILE.json]\n";
      return 0;
    } else {std::cerr << "未知参数: " << a << "\n"; return 2;}
  }
  if (o.frames_dir.empty() && !o.synthetic) {
    std::cerr << "必须给 --frames-dir DIR 或 --synthetic\n";
    return 2;
  }

  // ---- 两侧配置 ----
  LinefitCfg lc;
  lc.sensor_height = o.sensor_height;
  GroundSegmentationParams lp;
  lp.visualize = false;
  lp.r_min_square = lc.r_min * lc.r_min;
  lp.r_max_square = lc.r_max * lc.r_max;
  lp.n_bins = lc.n_bins;
  lp.n_segments = lc.n_segments;
  lp.max_dist_to_line = lc.max_dist_to_line;
  lp.min_slope = lc.min_slope;
  lp.max_slope = lc.max_slope;
  lp.max_error_square = lc.max_fit_error * lc.max_fit_error;
  lp.long_threshold = lc.long_threshold;
  lp.max_long_height = lc.max_long_height;
  lp.max_start_height = lc.max_start_height;
  lp.sensor_height = lc.sensor_height;
  lp.line_search_angle = lc.line_search_angle;
  lp.n_threads = lc.n_threads;

  patchwork::Params pp;
  pp.sensor_height = o.sensor_height;
  pp.min_range = o.min_range;
  pp.max_range = o.max_range;
  pp.th_dist = o.th_dist;
  pp.num_iter = o.num_iter;
  pp.num_min_pts = o.num_min_pts;
  pp.num_zones = o.num_zones;
  pp.num_rings_of_interest = o.num_rings_of_interest;
  pp.uprightness_thr = o.uprightness_thr;
  pp.enable_TGR = o.enable_TGR;
  pp.enable_RVPF = o.enable_RVPF;
  pp.enable_RNR = false;   // 无 intensity ⇒ 开了也不生效（见 VENDORING.md §5）
  pp.verbose = false;

  std::cout << "== ground_seg_ab ==\n"
            << "  linefit   : r_min " << lc.r_min << " r_max " << lc.r_max
            << " max_dist_to_line " << lc.max_dist_to_line << " sensor_height " << lc.sensor_height
            << " n_bins " << lc.n_bins << " n_segments " << lc.n_segments << "\n"
            << "  patchwork : min_range " << pp.min_range << " max_range " << pp.max_range
            << " th_dist " << pp.th_dist << " num_iter " << pp.num_iter
            << " num_min_pts " << pp.num_min_pts << " uprightness_thr " << pp.uprightness_thr
            << " TGR " << pp.enable_TGR << " RVPF " << pp.enable_RVPF
            << " sensor_height " << pp.sensor_height << "\n";

  // ---- 帧列表 ----
  struct Frame
  {
    std::string label;
    std::vector<Eigen::Vector3f> pts;
    std::vector<int> kind;   // 仅合成模式有；空 = 未知（真实帧）
    bool fresh_state = false; // true = 本帧换一套全新的分割器（合成模式：每帧是**不同的地形**，
                              // 不能让上一帧的自适应状态泄漏过来；真实帧模式相反，必须复用）
  };
  std::vector<Frame> frames;
  if (o.synthetic) {
    const TerrainSpec specs[] = {
      {"flat-only", 0.0, 0.0, 0.0, false},
      {"ramp-10deg(sensor@bottom)", 10.0, 0.0, 0.0, false},
      {"ramp-15deg(sensor@bottom)", 15.0, 0.0, 0.0, false},
      {"ramp-22deg(sensor@bottom)", 22.0, 0.0, 0.0, false},
      {"ramp-10deg(sensor@mid-ramp)", 10.0, 0.0, 0.0, true},
      {"ramp-15deg(sensor@mid-ramp)", 15.0, 0.0, 0.0, true},
      {"ramp-22deg(sensor@mid-ramp)", 22.0, 0.0, 0.0, true},
      {"terrace-0.20m", 0.0, 0.20, 0.0, false},
      {"terrace-0.30m", 0.0, 0.30, 0.0, false},
      {"wall-0.15m", 0.0, 0.0, 0.15, false},
      {"wall-0.30m", 0.0, 0.0, 0.30, false},
      {"wall-0.40m", 0.0, 0.0, 0.40, false},
      // ★ RMUC2026.stl 实测几何的代理：0.2/0.3 m 平台 + 10/15/22° 倒角 + −0.2 m 坑
      {"stl-terrace0.2-chamfer10", 0.0, 0.0, 0.0, false, 0.20, 10.0},
      {"stl-terrace0.2-chamfer15", 0.0, 0.0, 0.0, false, 0.20, 15.0},
      {"stl-terrace0.2-chamfer22", 0.0, 0.0, 0.0, false, 0.20, 22.0},
      {"stl-terrace0.3-chamfer10", 0.0, 0.0, 0.0, false, 0.30, 10.0},
      {"stl-terrace0.3-chamfer15", 0.0, 0.0, 0.0, false, 0.30, 15.0},
      {"stl-terrace0.3-chamfer22", 0.0, 0.0, 0.0, false, 0.30, 22.0},
      {"stl-pit-0.2-chamfer15", 0.0, 0.0, 0.0, false, -0.20, 15.0},
    };
    const int reps = std::max(1, o.synthetic_frames);
    for (int r = 0; r < reps; ++r) {
      for (const auto & sp : specs) {
        Frame f;
        f.label = sp.name;
        SynthCloud sc = build_terrain(sp, o.sensor_height);
        f.pts = std::move(sc.pts);
        f.kind = std::move(sc.kind);
        f.fresh_state = true;
        frames.push_back(std::move(f));
      }
    }
  } else {
    std::vector<std::string> files;
    for (const auto & e : fs::directory_iterator(o.frames_dir)) {
      if (e.is_regular_file() && e.path().extension() == ".bin") {
        files.push_back(e.path().string());
      }
    }
    std::sort(files.begin(), files.end());
    if (files.empty()) {std::cerr << "目录里没有 .bin: " << o.frames_dir << "\n"; return 1;}
    for (const auto & p : files) {
      Frame f;
      f.label = fs::path(p).stem().string();
      if (!read_bin(p, &f.pts)) {std::cerr << "读不了: " << p << "\n"; return 1;}
      if (o.drop_zeros) {
        // 只对**真实帧**生效：老 bag（插件修复前）里 79% 的点是 (0,0,0) 无回波填充点。
        // 两个分割器都把 r=0 判成障碍（linefit: range_square < r_min_square；patchwork: r <= min_range）
        // ⇒ 它们会同时抬高"一致率"。丢掉它们才能看清真实回波上的一致率。
        std::vector<Eigen::Vector3f> keep;
        keep.reserve(f.pts.size());
        for (const auto & q : f.pts) {
          if (!(q.x() == 0.0f && q.y() == 0.0f && q.z() == 0.0f)) {keep.push_back(q);}
        }
        f.pts.swap(keep);
      }
      frames.push_back(std::move(f));
      if (static_cast<int>(frames.size()) >= o.max_frames) {break;}
    }
  }
  std::cout << "  帧数 = " << frames.size() << "\n";

  // ---- 逐帧跑 ----
  // 真实帧模式：两个分割器**全程复用**（与节点里一样，含 Patchwork++ 的跨帧自适应）。
  // 合成模式：每帧换一套新的（每帧是不同地形，不能互相污染），并单独记一条时间。
  auto linefit = std::make_shared<GroundSegmentation>(lp);
  auto pw = std::make_unique<patchwork::PatchWorkpp>(pp);
  std::unique_ptr<GroundSegmentation> linefit_fresh;
  std::unique_ptr<patchwork::PatchWorkpp> pw_fresh;

  // 合成模式：按(场景, 真值类别)统计
  struct KindStat
  {
    long long n = 0, lf_obs = 0, pw_obs = 0;
    // ★ 下游（pointcloud_to_laserscan）真正关心的不是"障碍点占比"，而是
    //   "**这个方位上还有没有障碍点**" —— p2l 每个角度 bin 只留最近的一个点。
    //   所以按 p2l 的 angle_increment(0.0043 rad) 分方位 bin，统计"该 bin 内至少有一个
    //   **落在 p2l 高度带内**(z_sensor ∈ (-1.0, 0.1)，取自 laserscan_params.yaml)的障碍点"的 bin 数。
    std::set<int> lf_bins, pw_bins;
    std::set<int> all_bins;
  };
  std::vector<std::array<KindStat, kNumKinds>> syn;   // [场景][类别]
  std::vector<std::string> syn_names;
  std::map<std::string, int> syn_index;

  std::vector<BandCov> bc = make_band_cov();
  std::vector<Bucket> hb = make_height_buckets();
  std::vector<Bucket> tb = make_tilt_buckets();
  long long pooled_total = 0, pooled_agree = 0;
  long long both_ground = 0, lf_only = 0, pw_only = 0, both_obs = 0;
  Stats lf_ms, pw_ms, agree_pct, ground_ratio_lf, ground_ratio_pw;
  long long nonzero_pts = 0, degenerate_pts = 0;

  std::vector<std::string> per_frame_lines;

  for (const auto & f : frames) {
    const size_t n = f.pts.size();
    PointCloud cloud;
    cloud.reserve(n);
    for (const auto & p : f.pts) {cloud.emplace_back(p.x(), p.y(), p.z());}

    if (f.fresh_state) {
      linefit_fresh = std::make_unique<GroundSegmentation>(lp);
      pw_fresh = std::make_unique<patchwork::PatchWorkpp>(pp);
    }
    GroundSegmentation & lf = f.fresh_state ? *linefit_fresh : *linefit;
    patchwork::PatchWorkpp & pwref = f.fresh_state ? *pw_fresh : *pw;

    // 合成模式：把本帧挂到一个"场景行"（按 label 聚合）
    int syn_row = -1;
    if (!f.kind.empty()) {
      auto it = syn_index.find(f.label);
      if (it == syn_index.end()) {
        syn_row = static_cast<int>(syn.size());
        syn_index[f.label] = syn_row;
        syn.push_back(std::array<KindStat, kNumKinds>());
        syn_names.push_back(f.label);
      } else {
        syn_row = it->second;
      }
    }

    // --- linefit ---
    std::vector<int> lf_labels;
    const double t0 = now_ms();
    lf.segment(cloud, &lf_labels);
    const double t1 = now_ms();
    lf_ms.add(t1 - t0);

    // --- patchwork ---
    Eigen::MatrixXf mat(static_cast<Eigen::Index>(n), 3);
    for (size_t i = 0; i < n; ++i) {
      mat(static_cast<Eigen::Index>(i), 0) = f.pts[i].x();
      mat(static_cast<Eigen::Index>(i), 1) = f.pts[i].y();
      mat(static_cast<Eigen::Index>(i), 2) = f.pts[i].z();
    }
    const double t2 = now_ms();
    pwref.estimateGround(mat);
    const double t3 = now_ms();
    pw_ms.add(t3 - t2);

    std::vector<char> pw_ground(n, 0);
    const Eigen::VectorXi gi = pwref.getGroundIndices();
    for (Eigen::Index j = 0; j < gi.size(); ++j) {
      const int idx = gi(j);
      if (idx >= 0 && static_cast<size_t>(idx) < n) {pw_ground[static_cast<size_t>(idx)] = 1;}
    }

    // 地面高度基准：两侧都判 ground 且 r<3m 的点的 z 中位数（比硬编码 sensor_height 诚实）
    std::vector<float> gz;
    for (size_t i = 0; i < n; ++i) {
      const double r = std::hypot(f.pts[i].x(), f.pts[i].y());
      if (r < 3.0 && lf_labels[i] == 1 && pw_ground[i] == 1) {gz.push_back(f.pts[i].z());}
    }
    double z_ground = -o.sensor_height;
    if (!gz.empty()) {
      std::sort(gz.begin(), gz.end());
      z_ground = gz[gz.size() / 2];
    }

    // 局部法向倾角
    std::vector<float> tilt;
    estimate_normals(f.pts, o.normal_radius, 8, &tilt);

    long long frame_total = 0, frame_agree = 0;
    long long fg = 0, fo = 0;
    for (size_t i = 0; i < n; ++i) {
      if (f.pts[i].x() == 0.0f && f.pts[i].y() == 0.0f && f.pts[i].z() == 0.0f) {
        ++degenerate_pts;
      } else {++nonzero_pts;}
      const bool lg = lf_labels[i] == 1;
      const bool pg = pw_ground[i] == 1;
      const bool lobs = !lg, pobs = !pg;
      ++frame_total;
      if (lobs == pobs) {++frame_agree;}
      if (lg && pg) {++both_ground; ++fg;} else if (lg && !pg) {++lf_only;} else if (!lg && pg) {
        ++pw_only;
      } else {++both_obs; ++fo;}
      const double h_rel = f.pts[i].z() - z_ground;
      tally(hb, h_rel, lobs, pobs);
      {
        const bool in_band = (f.pts[i].z() > -1.0f && f.pts[i].z() < 0.1f);
        const double az = std::atan2(f.pts[i].y(), f.pts[i].x());
        const int abin = static_cast<int>(std::floor((az + M_PI) / 0.0043));
        for (auto & b : bc) {
          if (h_rel < b.lo || h_rel >= b.hi) {continue;}
          b.all.insert(abin);
          if (in_band) {
            if (lobs) {b.lf.insert(abin);}
            if (pobs) {b.pw.insert(abin);}
          }
          break;
        }
      }
      if (syn_row >= 0) {
        const int k = f.kind[i];
        if (k >= 0 && k < kNumKinds) {
          KindStat & ks = syn[syn_row][k];
          ++ks.n;
          if (lobs) {++ks.lf_obs;}
          if (pobs) {++ks.pw_obs;}
          // p2l 口径的方位覆盖（0.0043 rad/bin，高度带 z_sensor ∈ (-1.0, 0.1)）
          const double az = std::atan2(f.pts[i].y(), f.pts[i].x());
          const int bin = static_cast<int>(std::floor((az + M_PI) / 0.0043));
          ks.all_bins.insert(bin);
          const bool in_band = (f.pts[i].z() > -1.0f && f.pts[i].z() < 0.1f);
          if (in_band) {
            if (lobs) {ks.lf_bins.insert(bin);}
            if (pobs) {ks.pw_bins.insert(bin);}
          }
        }
      }
      // 倾角分桶只统计"低处"的点（< 0.5 m 离地）—— 高处的墙面倾角没有坡面含义
      if (tilt[i] >= 0.0f && (f.pts[i].z() - z_ground) < 0.5) {
        tally(tb, tilt[i], lobs, pobs);
      }
    }
    pooled_total += frame_total;
    pooled_agree += frame_agree;
    agree_pct.add(100.0 * static_cast<double>(frame_agree) / static_cast<double>(frame_total));
    ground_ratio_lf.add(100.0 * static_cast<double>(fg) / static_cast<double>(frame_total));
    ground_ratio_pw.add(100.0 * static_cast<double>(frame_total - fo) /
      static_cast<double>(frame_total));
    {
      std::ostringstream os;
      os << "  " << f.label << ": n=" << frame_total
         << " agree=" << (100.0 * frame_agree / frame_total) << "%"
         << " ground(lf)=" << (100.0 * fg / frame_total) << "%"
         << " ground(pw)=" << (100.0 * (frame_total - fo) / frame_total) << "%"
         << " zg=" << z_ground
         << " lf=" << (t1 - t0) << "ms pw=" << (t3 - t2) << "ms";
      per_frame_lines.push_back(os.str());
    }
  }

  // ---- 输出 ----
  auto print_buckets = [](const char * title, const std::vector<Bucket> & bs) {
      std::printf("\n%s\n", title);
      std::printf("%-22s %10s %14s %14s\n", "bucket", "points", "linefit_obs%", "patchwork_obs%");
      for (const auto & b : bs) {
        if (b.total == 0) {continue;}
        std::printf(
          "%-22s %10lld %13.2f%% %13.2f%%\n", b.name, b.total,
          100.0 * static_cast<double>(b.lf_obs) / static_cast<double>(b.total),
          100.0 * static_cast<double>(b.pw_obs) / static_cast<double>(b.total));
      }
    };

  std::printf("\n== 逐帧 ==\n");
  for (const auto & s : per_frame_lines) {std::printf("%s\n", s.c_str());}

  std::printf("\n== 汇总 ==\n");
  std::printf("  点数(总/非零点/全零点) = %lld / %lld / %lld\n",
    pooled_total, nonzero_pts, degenerate_pts);
  std::printf("  逐点一致率 pooled = %.2f%%   per-frame mean = %.2f%% (min %.2f%%, max %.2f%%)\n",
    100.0 * static_cast<double>(pooled_agree) / static_cast<double>(pooled_total),
    agree_pct.mean(), agree_pct.v.empty() ? 0.0 : *std::min_element(agree_pct.v.begin(),
    agree_pct.v.end()), agree_pct.v.empty() ? 0.0 : *std::max_element(agree_pct.v.begin(),
    agree_pct.v.end()));
  std::printf("  混淆矩阵: both_ground=%lld (%.2f%%)  linefit_only_ground=%lld (%.2f%%)  "
    "patchwork_only_ground=%lld (%.2f%%)  both_obstacle=%lld (%.2f%%)\n",
    both_ground, 100.0 * both_ground / pooled_total,
    lf_only, 100.0 * lf_only / pooled_total,
    pw_only, 100.0 * pw_only / pooled_total,
    both_obs, 100.0 * both_obs / pooled_total);
  std::printf("  地面占比: linefit mean %.2f%%   patchwork mean %.2f%%\n",
    ground_ratio_lf.mean(), ground_ratio_pw.mean());
  std::printf("  单帧耗时: linefit median %.3f / p95 %.3f / max %.3f ms   "
    "patchwork median %.3f / p95 %.3f / max %.3f ms   (patchwork/linefit = %.2fx)\n",
    lf_ms.median(), lf_ms.pct(0.95), lf_ms.max(),
    pw_ms.median(), pw_ms.pct(0.95), pw_ms.max(),
    lf_ms.median() > 0 ? pw_ms.median() / lf_ms.median() : 0.0);
  std::printf("\n== 按【离地高度带】的方位覆盖率（p2l 口径：该方位在该高度带里还有没有障碍点）==\n");
  std::printf("  口径：把每个方位分成 p2l 的 0.0043 rad bin；覆盖率 = 该高度带里有点的 bin 中，\n"
    "        至少还有一个**障碍点且 z_sensor ∈ (-1.0, 0.1)**（= p2l 的高度带）的 bin 占比。\n"
    "  0%% = 这个高度的特征在 /scan 里整个消失；100%% = 每个方位都还留着。\n");
  std::printf("%-16s %10s %14s %14s\n", "高度带(离地)", "bins", "linefit_cov%", "patchwork_cov%");
  for (const auto & b : bc) {
    if (b.all.empty()) {continue;}
    std::printf("%-16s %10zu %13.2f%% %13.2f%%\n", b.name, b.all.size(),
      100.0 * static_cast<double>(b.lf.size()) / static_cast<double>(b.all.size()),
      100.0 * static_cast<double>(b.pw.size()) / static_cast<double>(b.all.size()));
  }
  print_buckets("== 按【离地高度】分桶的障碍保留率 ==", hb);
  print_buckets("== 按【局部法向倾角】分桶的障碍保留率（仅离地<0.5m 的点）==", tb);

  if (!syn.empty()) {
    std::printf("\n== 合成地形：按【真值类别】的障碍判出率（越接近真值越对）==\n");
    std::printf("  真值说明：平地/坡面/坡顶平台 = **应该判地面**（obs%% 越低越好）；"
      "台阶顶面/台阶立面/薄墙 = **应该判障碍**（obs%% 越高越好）\n");
    for (size_t r = 0; r < syn.size(); ++r) {
      std::printf("\n  [%s]\n", syn_names[r].c_str());
      std::printf("    %-24s %8s %14s %14s\n", "真值类别", "points", "linefit_obs%", "patchwork_obs%");
      for (int k = 0; k < kNumKinds; ++k) {
        const auto & ks = syn[r][k];
        if (ks.n == 0) {continue;}
        std::printf("    %-24s %8lld %13.2f%% %13.2f%%\n", kind_name(k), ks.n,
          100.0 * static_cast<double>(ks.lf_obs) / static_cast<double>(ks.n),
          100.0 * static_cast<double>(ks.pw_obs) / static_cast<double>(ks.n));
      }
      // ★ 方位覆盖率：p2l 每个 0.0043 rad 的 bin 只留最近一个障碍点 ⇒ 覆盖率 = "该特征在 /scan 里
      //   还留得下吗"。0% = 整个特征从 /scan 消失；100% = 每个有该特征的方位都还有障碍回波。
      std::printf("    %-24s %8s %14s %14s\n", "[方位覆盖率@p2l口径]", "bins",
        "linefit_cov%", "patchwork_cov%");
      for (int k = 0; k < kNumKinds; ++k) {
        const auto & ks = syn[r][k];
        if (ks.all_bins.empty()) {continue;}
        std::printf("    %-24s %8zu %13.2f%% %13.2f%%\n", kind_name(k), ks.all_bins.size(),
          100.0 * static_cast<double>(ks.lf_bins.size()) / static_cast<double>(ks.all_bins.size()),
          100.0 * static_cast<double>(ks.pw_bins.size()) / static_cast<double>(ks.all_bins.size()));
      }
    }
    std::printf("\n== 合成地形：坡面点的**地面判出率**（= 100%% − obs%%，越高越好）==\n");
    std::printf("  %-30s %8s %12s %12s\n", "场景", "points", "linefit", "patchwork");
    for (size_t r = 0; r < syn.size(); ++r) {
      const auto & ks = syn[r][kRampSurface];
      if (ks.n == 0) {continue;}
      std::printf("  %-30s %8lld %11.2f%% %11.2f%%\n", syn_names[r].c_str(), ks.n,
        100.0 * (1.0 - static_cast<double>(ks.lf_obs) / static_cast<double>(ks.n)),
        100.0 * (1.0 - static_cast<double>(ks.pw_obs) / static_cast<double>(ks.n)));
    }
  }

  if (!o.out_json.empty()) {
    std::ofstream j(o.out_json);
    j << "{\n";
    j << "  \"frames\": " << frames.size() << ",\n";
    j << "  \"points_total\": " << pooled_total << ",\n";
    j << "  \"agreement_pooled_pct\": "
      << 100.0 * static_cast<double>(pooled_agree) / static_cast<double>(pooled_total) << ",\n";
    j << "  \"agreement_per_frame_mean_pct\": " << agree_pct.mean() << ",\n";
    j << "  \"both_ground\": " << both_ground << ",\n";
    j << "  \"linefit_only_ground\": " << lf_only << ",\n";
    j << "  \"patchwork_only_ground\": " << pw_only << ",\n";
    j << "  \"both_obstacle\": " << both_obs << ",\n";
    j << "  \"linefit_ms_median\": " << lf_ms.median() << ",\n";
    j << "  \"linefit_ms_p95\": " << lf_ms.pct(0.95) << ",\n";
    j << "  \"patchwork_ms_median\": " << pw_ms.median() << ",\n";
    j << "  \"patchwork_ms_p95\": " << pw_ms.pct(0.95) << ",\n";
    j << "  \"height_buckets\": [\n";
    for (size_t i = 0; i < hb.size(); ++i) {
      const auto & b = hb[i];
      j << "    {\"name\": \"" << b.name << "\", \"points\": " << b.total
        << ", \"linefit_obs_pct\": "
        << (b.total ? 100.0 * static_cast<double>(b.lf_obs) / static_cast<double>(b.total) : 0.0)
        << ", \"patchwork_obs_pct\": "
        << (b.total ? 100.0 * static_cast<double>(b.pw_obs) / static_cast<double>(b.total) : 0.0)
        << "}" << (i + 1 < hb.size() ? "," : "") << "\n";
    }
    j << "  ],\n  \"tilt_buckets\": [\n";
    for (size_t i = 0; i < tb.size(); ++i) {
      const auto & b = tb[i];
      j << "    {\"name\": \"" << b.name << "\", \"points\": " << b.total
        << ", \"linefit_obs_pct\": "
        << (b.total ? 100.0 * static_cast<double>(b.lf_obs) / static_cast<double>(b.total) : 0.0)
        << ", \"patchwork_obs_pct\": "
        << (b.total ? 100.0 * static_cast<double>(b.pw_obs) / static_cast<double>(b.total) : 0.0)
        << "}" << (i + 1 < tb.size() ? "," : "") << "\n";
    }
    j << "  ],\n  \"synthetic_kind\": [\n";
    for (size_t r = 0; r < syn.size(); ++r) {
      for (int k = 0; k < kNumKinds; ++k) {
        const auto & ks = syn[r][k];
        if (ks.n == 0) {continue;}
        j << "    {\"scenario\": \"" << syn_names[r] << "\", \"kind\": \"" << kind_name(k)
          << "\", \"points\": " << ks.n
          << ", \"linefit_obs_pct\": "
          << 100.0 * static_cast<double>(ks.lf_obs) / static_cast<double>(ks.n)
          << ", \"patchwork_obs_pct\": "
          << 100.0 * static_cast<double>(ks.pw_obs) / static_cast<double>(ks.n) << "},\n";
      }
    }
    j << "    {}\n  ]\n}\n";
    std::printf("\n  JSON 已写: %s\n", o.out_json.c_str());
  }
  return 0;
}
