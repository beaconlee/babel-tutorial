#include "hybrid_a_star.h"
#include "src/common/utils.h"
#include <queue>

namespace beacon
{

namespace
{

struct cmp
{
  bool operator()(std::pair<int, double> a, std::pair<int, double> b)
  {
    return a.second > b.second;
  }
};
} // namespace

void HybridAStar::Init(point_arr_t points)
{
  frame_ = std::make_shared<Frame>();
  astar_ = std::make_shared<AStar>();
  astar_result_ = std::make_shared<AstarResult>();
  // std::shared_ptr<beacon::AstarResult> resutlt =
  // std::make_shared<beacon::AstarResult>();

  std::cout << __FUNCTION__ << "  " << __LINE__ << "\n";
  CalcParameters(points);
  std::cout << __FUNCTION__ << "  " << __LINE__ << "\n";
}

Status HybridAStar::Plan(Eigen::Vector3d start,
                         Eigen::Vector3d goal,
                         std::shared_ptr<Frame> frame)
{
  std::cout << __FUNCTION__ << "   " << __LINE__ << "\n";
  DEBUG_LOG
  DEBUG_LOG
  DEBUG_LOG
  DEBUG_LOG

  LOG(start.x());
  LOG(frame_->para_->xyreso_);
  int sx_reso = round(start.x() / frame_->para_->xyreso_);
  DEBUG_LOG
  int sy_reso = round(start.y() / frame_->para_->xyreso_);
  DEBUG_LOG
  LOG(start.x());
  LOG(frame_->para_->xyreso_);
  int syaw_reso = round(Mod2Pi(start.z() / frame_->para_->yawreso_));
  DEBUG_LOG

  int gx_reso = round(goal.x() / frame_->para_->xyreso_);
  int gy_reso = round(goal.y() / frame_->para_->xyreso_);
  int gyaw_reso = round(Mod2Pi(goal.z() / frame_->para_->yawreso_));
  DEBUG_LOG


  std::shared_ptr<TrajectoryNode> tn_start(new TrajectoryNode(sx_reso,
                                                              sy_reso,
                                                              syaw_reso,
                                                              1,
                                                              {start.x()},
                                                              {start.y()},
                                                              {start.z()},
                                                              {1},
                                                              0.0,
                                                              0.0,
                                                              -1));

  std::shared_ptr<TrajectoryNode> tn_goal(new TrajectoryNode(gx_reso,
                                                             gy_reso,
                                                             gyaw_reso,
                                                             1,
                                                             {goal.x()},
                                                             {goal.y()},
                                                             {goal.z()},
                                                             {1},
                                                             0.0,
                                                             0.0,
                                                             -1));

  Eigen::Vector2d heu_goal{goal.x(), goal.y()};

  DEBUG_LOG
  astar_->Plan(frame_->obs, heu_goal, astar_result_);
  DEBUG_LOG

  CalcMotionSet(frame_);

  DEBUG_LOG

  // 使用 open_set 表示
  std::unordered_map<int, std::shared_ptr<TrajectoryNode>> open_set;
  std::unordered_map<int, std::shared_ptr<TrajectoryNode>> closed_set;

  open_set[CalcIndex(tn_start)] = tn_start;

  std::priority_queue<std::pair<int, double>,
                      std::vector<std::pair<int, double>>,
                      cmp>
      q_priority;

  q_priority.push(
      std::make_pair(CalcIndex(tn_start), CalcHybridCost(tn_start)));

  // 用来存储 rs 的结点
  std::shared_ptr<TrajectoryNode> rs_node{};
  DEBUG_LOG
  while(true)
  {
  }
}

void HybridAStar::CalcMotionSet(std::shared_ptr<Frame> frame)
{
  double step = frame->vc.max_steer_ / N_STEER;
  std::vector<double> steer;
  std::vector<int> direc;

  for(double i = -frame->vc.max_steer_; i <= frame->vc.max_steer_; i += step)
  {
    steer.push_back(i);
  }
  for(size_t i = 0; i < steer.size(); ++i)
  {
    direc.push_back(1);
  }
  for(size_t i = 0; i < steer.size(); ++i)
  {
    direc.push_back(-1);
  }
  for(double i = -frame->vc.max_steer_; i <= frame->vc.max_steer_; i += step)
  {
    steer.push_back(i);
  }

  motion_set_ = std::make_pair(steer, direc);
}

int HybridAStar::CalcIndex(const std::shared_ptr<TrajectoryNode>& node)
{
  int idx = (node->yaw_ - frame_->para_->minyaw_) * frame_->para_->xw_ *
                frame_->para_->yw_ +
            (node->y_coord_ - frame_->para_->miny_) * frame_->para_->xw_ +
            (node->x_coord_ - frame_->para_->minx_);
  return idx;
}

double HybridAStar::CalcHybridCost(const std::shared_ptr<TrajectoryNode>& node)
{
  double cost =
      node->cost_ +
      H_COST * astar_result_->cost_so_far[{node->x_coord_, node->y_coord_}];

  return cost;
}

void HybridAStar::CalcParameters(point_arr_t points)
{
  // std::cout << __FUNCTION__ << "  " << __LINE__ << "\n";
  int minx = round(Min(points[0]) / XY_RESO);
  int miny = round(Min(points[1]) / XY_RESO);
  // std::cout << __FUNCTION__ << "  " << __LINE__ << "\n";
  int maxx = round(Max(points[0]) / XY_RESO);
  int maxy = round(Max(points[1]) / XY_RESO);

  // std::cout << __FUNCTION__ << "  " << __LINE__ << "\n";
  int xw = maxx - minx;
  int yw = maxy - miny;
  // std::cout << __FUNCTION__ << "  " << __LINE__ << "\n";
  int minyaw = round(-M_PI / YAW_RESO) - 1;
  int maxyaw = round(M_PI / YAW_RESO);
  int yaww = maxyaw - minyaw;

  // std::cout << __FUNCTION__ << "  " << __LINE__ << "\n";
  frame_->para_ = std::make_shared<Para>(Para(
      minx, miny, minyaw, maxx, maxy, maxyaw, xw, yw, yaww, XY_RESO, YAW_RESO));

  frame_->obs = std::make_shared<KDTree>(points);
}
} // namespace beacon