#include "hybrid_a_star.h"
#include "src/common/utils.h"
#include "src/common/datastruct/priority_queue.h"
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
  // PriorityQueue<Eigen::Vector2d, double> frontier;
  q_priority.push(
      std::make_pair(CalcIndex(tn_start), CalcHybridCost(tn_start)));

  // 用来存储 rs 的结点
  std::shared_ptr<TrajectoryNode> rs_node{};
  DEBUG_LOG
  while(true)
  {
    if(open_set.empty())
    {
      return Status::ERROR;
    }

    int idx = q_priority.top().first;
    q_priority.pop();
    std::shared_ptr<TrajectoryNode> curr_node = open_set[idx];
    open_set.erase(idx);

    bool update = ReedsSheepPath(curr_node, tn_goal, rs_node);
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

std::shared_ptr<TrajectoryNode> HybridAStar::CalcNextNode(
    std::shared_ptr<TrajectoryNode>& curr_node, int c_id, double u, int d)
{
  // c++ 的取整函数
  // 1. ceil 函数，向上取整
  // 2. floor 函数，向下取整
  // 3. fix 函数，向 0 取整
  // 4. round 函数，四舍五入

  double step = XY_RESO * 2.5;
  int nlist = std::ceil(
      step / MOVE_STEP); // 使用布长除以移动分辨率，得到一个 node 里面点的个数

  std::vector<double> xlist = {curr_node->x_list_.back() +
                               d * MOVE_STEP *
                                   cos(curr_node->yaw_list_.back())};

  std::vector<double> ylist = {curr_node->y_list_.back() +
                               d * MOVE_STEP *
                                   sin(curr_node->yaw_list_.back())};

  std::vector<double> yawlist = {
      Mod2Pi(curr_node->yaw_list_.back() +
             d * MOVE_STEP / frame_->vc.whell_base_ * tan(u))};

  for(size_t idx = 0; idx < nlist - 1; ++idx)
  {
    xlist.push_back(xlist[idx] + d * MOVE_STEP * cos(yawlist[idx]));
    ylist.push_back(ylist[idx] + d * MOVE_STEP * sin(yawlist[idx]));
    yawlist.push_back(
        Mod2Pi(yawlist[idx] + d * MOVE_STEP / frame_->vc.whell_base_ * tan(u)));
  }

  // 通过放缩得到原来的坐标，并进行四舍五入
  int xind = std::round(xlist.back() / XY_RESO);
  int yind = std::round(ylist.back() / XY_RESO);
  int yawind = std::round(yawlist.back() / YAW_RESO);

  if(!IsIndexOk(xind, yind, xlist, ylist, yawlist))
  {
    return {};
  }

  double cost = 0.;
  int direction = 1;

  if(d > 0)
  {
    direction = 1;
    cost += abs(step);
  }
  else
  {
    direction = -1;
    cost += abs(step) * BACKWARD_COST;
  }

  if(direction != curr_node->direction_)
  {
    cost += GEAR_COST;
  }

  cost += STEER_ANGLE_COST * abs(u);
  cost += STEER_CHANGE_COST * abs(curr_node->steer_ - u);
  cost += curr_node->cost_;
  std::vector<int> directions(xlist.size(), direction);

  return std::make_shared<TrajectoryNode>(xind,
                                          yind,
                                          yawind,
                                          direction,
                                          xlist,
                                          ylist,
                                          yawlist,
                                          directions,
                                          u,
                                          cost,
                                          c_id);
}


bool HybridAStar::IsIndexOk(int xind,
                            int yind,
                            const std::vector<double>& xlist,
                            const std::vector<double>& ylist,
                            const std::vector<double>& yawlist)
{
  if(xind <= frame_->para_->minx_ || xind >= frame_->para_->maxx_ ||
     yind <= frame_->para_->miny_ || yind >= frame_->para_->maxy_)
  {
    return false;
  }

  std::vector<double> nodex;
  std::vector<double> nodey;
  std::vector<double> nodeyaw;
  for(size_t idx = 0; idx < xlist.size(); idx += COLLISION_CHECK_STEP)
  {
    nodex.push_back(xlist[idx]);
    nodey.push_back(ylist[idx]);
    nodeyaw.push_back(yawlist[idx]);
  }

  if(IsCollision(nodex, nodey, nodeyaw))
  {
    return false;
  }

  return true;
}


bool HybridAStar::IsCollision(std::vector<double>& x,
                              std::vector<double>& y,
                              std::vector<double>& yaw)
{
  for(size_t idx = 0; idx < x.size(); ++idx)
  {
    int d = 1;
    double dl = (frame_->vc.rf_ - frame_->vc.rb_) / 2.0;
    double r = (frame_->vc.rf_ + frame_->vc.rb_) / 2.0 + d;
    double cx = x[idx] + dl * cos(yaw[idx]);
    double cy = y[idx] + dl * sin(yaw[idx]);
    std::vector<point_t> ids = frame_->obs->NeighborhoodPoints({cx, cy}, r);

    for(const point_t& ob : ids)
    {
      double xo = ob[0] - cx;
      double yo = ob[1] - cy;
      double dx = xo * cos(yaw[idx]) + yo * sin(yaw[idx]);
      double dy = -xo * sin(yaw[idx]) + yo * cos(yaw[idx]);

      if(abs(dx) < r && abs(dy) < frame_->vc.width_ / 2 + d)
      {
        return true;
      }
    }
  }

  return false;
}

bool HybridAStar::ReedsSheepPath(std::shared_ptr<TrajectoryNode> n_curr,
                                 std::shared_ptr<TrajectoryNode> ngoal,
                                 std::shared_ptr<TrajectoryNode>& fpath)
{
  
}
} // namespace beacon