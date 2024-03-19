#include "hybrid_a_star.h"
#include "src/common/utils.h"
#include "src/common/datastruct/priority_queue.h"
#include "src/common/matplotlibcpp.h"


#include <queue>


namespace plt = matplotlibcpp;

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

void HybridAStar::Init(std::vector<std::vector<double>> obs)
{
  frame_ = std::make_shared<Frame>();

  beacon::point_arr_t points;
  for(size_t idx = 0; idx < obs.at(0).size(); ++idx)
  {
    points.push_back({obs.at(0)[idx], obs.at(1)[idx]});
  }

  frame_->obs = std::make_shared<KDTree>(points);

  astar_ = std::make_shared<AStar>();
  astar_result_ = std::make_shared<AstarResult>();
  rs_path_ = std::make_shared<ReedsSheppPath>();
  CalcParameters(obs);
}

Status HybridAStar::Plan(Eigen::Vector3d start,
                         Eigen::Vector3d goal,
                         std::shared_ptr<Frame> frame)
{
  // std::cout << __FUNCTION__ << "   " << __LINE__ << "\n";
  // DEBUG_LOG
  // DEBUG_LOG

  // LOG(start.x());
  // LOG(frame_->para_->xyreso_);
  int sx_reso = round(start.x() / frame_->para_->xyreso_);
  // DEBUG_LOG
  int sy_reso = round(start.y() / frame_->para_->xyreso_);
  // DEBUG_LOG
  // LOG(start.x());
  // LOG(frame_->para_->xyreso_);
  int syaw_reso = round(Mod2Pi(start.z() / frame_->para_->yawreso_));
  // DEBUG_LOG

  int gx_reso = round(goal.x() / frame_->para_->xyreso_);
  int gy_reso = round(goal.y() / frame_->para_->xyreso_);
  int gyaw_reso = round(Mod2Pi(goal.z() / frame_->para_->yawreso_));
  // DEBUG_LOG


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
  // std::cout << "-------------------------------------\n";
  astar_->Plan(vector_obs_, heu_goal, astar_result_);
  // std::cout << "-------------------------------------\n";

  DEBUG_LOG

  for(auto item : astar_result_->cost_so_far)
  {
  }

  CalcMotionSet(frame_);

  // DEBUG_LOG

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
  std::shared_ptr<TrajectoryNode> rs_node;
  DEBUG_LOG
  while(true)
  {
    if(open_set.empty())
    {
      return Status::ERROR;
    }

    DEBUG_LOG
    int idx = q_priority.top().first;
    q_priority.pop();
    std::shared_ptr<TrajectoryNode> curr_node = open_set[idx];
    open_set.erase(idx);
    DEBUG_LOG
    bool update = ReedsSheepPath(curr_node, tn_goal, rs_node);

    if(update)
    {
      break;
    }

    DEBUG_LOG

    for(size_t id = 0; id < motion_set_.first.size(); ++id)
    {
      // DEBUG_LOG
      // std::cout << "current id: " << id << "\n";
      std::shared_ptr<TrajectoryNode> temp_node = CalcNextNode(
          curr_node, idx, motion_set_.first[id], motion_set_.second[id]);
      DEBUG_LOG
      // std::cout << temp_node->x_list_.size() << "\n";
      if(temp_node == nullptr)
      {
        continue;
      }

      plt::plot(temp_node->x_list_, temp_node->y_list_, "-");
      // for(size_t idx = 0; idx < temp_node->x_list_.size(); ++idx)
      // {
      //   std::cout << "(" << temp_node->x_list_[idx] << ","
      //             << temp_node->y_list_[idx] << ")  ";
      // }
      // std::cout << "\n";
      plt::pause(0.001);
      int temp_node_idx = CalcIndex(temp_node);

      if(open_set.find(temp_node_idx) == open_set.end())
      {
        DEBUG_LOG
        open_set[temp_node_idx] = temp_node;
        q_priority.push(
            std::make_pair(temp_node_idx, CalcHybridCost(temp_node)));
      }
      else if(open_set[temp_node_idx]->cost_ > temp_node->cost_)
      {
        DEBUG_LOG
        open_set[temp_node_idx] = temp_node;
      }
    }
  }
  plt::plot(rs_node->x_list_, rs_node->y_list_, "-");
  plt::pause(10);
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
  double cost = node->cost_ +
                H_COST * astar_result_->hmap_[node->x_coord_][node->y_coord_];

  return cost;
}

void HybridAStar::CalcParameters(std::vector<std::vector<double>> obs)
{
  int minx = round(Min(obs[0]) / XY_RESO);
  int miny = round(Min(obs[1]) / XY_RESO);
  int maxx = round(Max(obs[0]) / XY_RESO);
  int maxy = round(Max(obs[1]) / XY_RESO);

  int xw = maxx - minx;
  int yw = maxy - miny;
  int minyaw = round(-M_PI / YAW_RESO) - 1;
  int maxyaw = round(M_PI / YAW_RESO);
  int yaww = maxyaw - minyaw;

  frame_->para_ = std::make_shared<Para>(Para(
      minx, miny, minyaw, maxx, maxy, maxyaw, xw, yw, yaww, XY_RESO, YAW_RESO));

  std::vector<double> ox;
  std::vector<double> oy;
  for(size_t idx = 0; idx < obs[0].size(); ++idx)
  {
    ox.push_back(obs[0][idx] / XY_RESO);
    oy.push_back(obs[1][idx] / XY_RESO);
  }

  vector_obs_ =
      std::vector<std::vector<double>>(xw, std::vector<double>(yw, 0));
  astar_result_->hmap_ =
      std::vector<std::vector<int>>(xw, std::vector<int>(yw, 8888));
  for(size_t x = 0; x < xw; ++x)
  {
    int cur_x = x + minx;
    // vector_obs_
    for(size_t y = 0; y < yw; ++y)
    {
      int cur_y = y + miny;
      for(size_t idx = 0; idx < ox.size(); ++idx)
      {
        if(std::hypot(ox[idx] - cur_x, oy[idx] - cur_y) <= 1.0 / XY_RESO)
        {
          vector_obs_[x][y] = 1;
        }
      }
    }
  }
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
  DEBUG_LOG

  // plt::plot(xlist, ylist, "r");
  // plt::pause(0.01);
  // std::cout << "draw...................\n";
  // 通过放缩得到原来的坐标，并进行四舍五入
  DEBUG_LOG
  int xind = std::round(xlist.back() / XY_RESO);
  DEBUG_LOG

  int yind = std::round(ylist.back() / XY_RESO);
  int yawind = std::round(yawlist.back() / YAW_RESO);
  DEBUG_LOG

  if(!IsIndexOk(xind, yind, xlist, ylist, yawlist))
  {
    DEBUG_LOG
    // std::cout << "IsIndexOK is fasle\n";
    return std::make_shared<TrajectoryNode>();
  }
  DEBUG_LOG

  double cost = 0.;
  int direction = 1;
  DEBUG_LOG

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
  DEBUG_LOG

  if(direction != curr_node->direction_)
  {
    cost += GEAR_COST;
  }
  DEBUG_LOG

  cost += STEER_ANGLE_COST * abs(u);
  cost += STEER_CHANGE_COST * abs(curr_node->steer_ - u);
  cost += curr_node->cost_;
  std::vector<int> directions(xlist.size(), direction);
  DEBUG_LOG

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
    DEBUG_LOG
    // std::cout << "override bound\n";
    return false;
  }

  DEBUG_LOG
  std::vector<double> nodex;
  std::vector<double> nodey;
  std::vector<double> nodeyaw;
  DEBUG_LOG
  for(size_t idx = 0; idx < xlist.size(); idx += COLLISION_CHECK_STEP)
  {
    nodex.push_back(xlist[idx]);
    nodey.push_back(ylist[idx]);
    nodeyaw.push_back(yawlist[idx]);
  }

  DEBUG_LOG
  if(IsCollision(nodex, nodey, nodeyaw))
  {
    // std::cout << "is collision\n";
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

bool HybridAStar::ReedsSheepPath(std::shared_ptr<TrajectoryNode> curr,
                                 std::shared_ptr<TrajectoryNode> goal,
                                 std::shared_ptr<TrajectoryNode>& fpath)
{
  DEBUG_LOG
  if(curr == nullptr || goal == nullptr || fpath == nullptr)
  {
    DEBUG_LOG
    std::cout << "curr is nullptr\n";
  }


  ReedsSheppPath rspath = rs_->AnalysticExpantion(curr, goal, frame_);
  DEBUG_LOG

  if(rspath.x_vec_.empty() || rspath.y_vec_.empty() || rspath.yaw_vec_.empty())
  {
    std::cout << " " << __FUNCTION__ << " " << __LINE__ << "rs failed!\n";
    return false;
  }
  DEBUG_LOG

  std::vector<double> x_list(rspath.x_vec_.begin() + 1,
                             rspath.x_vec_.end() - 1);
  std::vector<double> y_list(rspath.y_vec_.begin() + 1,
                             rspath.y_vec_.end() - 1);
  std::vector<double> yaw_list(rspath.yaw_vec_.begin() + 1,
                               rspath.yaw_vec_.end() - 1);
  std::vector<int> dir_list(rspath.direc_vec_.begin() + 1,
                            rspath.direc_vec_.end() - 1);
  double cost = curr->cost_ + rs_->CalcRspathCost(rspath, frame_);
  DEBUG_LOG

  int node_idx = CalcIndex(curr);
  double fsteer = 0.;
  DEBUG_LOG

  fpath = std::make_shared<TrajectoryNode>(curr->x_coord_,
                                           curr->y_coord_,
                                           curr->yaw_,
                                           curr->direction_,
                                           x_list,
                                           y_list,
                                           yaw_list,
                                           dir_list,
                                           fsteer,
                                           cost,
                                           node_idx);

  return true;
}

} // namespace beacon