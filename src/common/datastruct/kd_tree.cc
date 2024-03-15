#include "kd_tree.h"
#include <algorithm>
#include <cmath>

namespace beacon
{

namespace
{
KDNodePtr NewKDNodePtr()
{
  return std::make_shared<KDNode>();
}
} // namespace

// square euclidean distance
double dist2(const point_t& point_a, const point_t& point_b)
{
  double result = 0.;
  for(size_t index = 0; index < point_a.size(); ++index)
  {
    double dis = point_a.at(index) - point_b.at(index);
    result += std::pow(dis, 2);
  }

  return result;
}

double dist2(const KDNodePtr& node_ptr_a, const KDNodePtr& node_ptr_b)
{
  return dist2(node_ptr_a->point_, node_ptr_b->point_);
}

// euclidean distance
double dist(const point_t& point_a, const point_t& point_b)
{
  return std::sqrt(dist2(point_a, point_b));
}

double dist(const KDNodePtr& node_ptr_a, const KDNodePtr& node_ptr_b)
{
  return std::sqrt(dist2(node_ptr_a, node_ptr_b));
}

struct Comparer
{
  size_t idx_;

  explicit Comparer(size_t idx)
    : idx_(idx)
  {}

  [[nodiscard]] bool ComparerIdex(const point_index_t& point_idx_a,
                                  const point_index_t& point_idx_b) const
  {
    return point_idx_a.first.at(idx_) < point_idx_b.first.at(idx_);
  }
};

inline void SortOnIdx(const point_index_arr_t::iterator& begin,
                      const point_index_arr_t::iterator& end,
                      size_t idx)
{
  Comparer comp(idx);

  // std::nth_element(begin,
  //                  begin + std::distance(begin, end) / 2,
  //                  end,
  //                  std::bind(&Comparer::ComparerIdex,
  //                            comp,
  //                            std::placeholders::_1,
  //                            std::placeholders::_2));
  std::nth_element(begin,
                   begin + std::distance(begin, end) / 2,
                   end,
                   [comp](auto&& PH1, auto&& PH2)
                   {
                     // PH1  PH2  placehoders 占位符
                     return comp.ComparerIdex(std::forward<decltype(PH1)>(PH1),
                                              std::forward<decltype(PH2)>(PH2));
                   });
}


//////////////////////////////////////////////////////////////////////
//////////
//////////
////////// KDTree
//////////
//////////
//////////////////////////////////////////////////////////////////////

KDNodePtr KDTree::MakeTree(const point_index_arr_t::iterator& begin,
                           const point_index_arr_t::iterator& end,
                           const size_t& length,
                           const size_t& level)
{
  if(begin == end)
  {
    return NewKDNodePtr();
  }

  size_t dimen = begin->first.size();

  if(length > 1)
  {
    SortOnIdx(begin, end, level);
  }

  auto mid = begin + (length / 2);

  auto l_begin = begin;
  auto l_end = mid;
  auto r_begin = mid + 1;
  auto r_end = end;

  size_t l_len = length / 2;
  size_t r_len = length - l_len - 1;

  KDNodePtr left;
  if(l_len > 0 && dimen > 0)
  {
    left = MakeTree(l_begin, l_end, l_len, (level + 1) % dimen);
  }
  else
  {
    left = leaf_;
  }

  KDNodePtr right;
  if(r_len > 0 && dimen > 0)
  {
    right = MakeTree(r_begin, r_end, r_len, (level + 1) % dimen);
  }
  else
  {
    right = leaf_;
  }

  return std::make_shared<KDNode>(*mid, left, right);
}


KDTree::KDTree(point_arr_t point_array)
  : leaf_(NewKDNodePtr())
{
  point_index_arr_t point_arr;

  for(size_t idx = 0; idx < point_array.size(); ++idx)
  {
    point_arr.emplace_back(point_array.at(idx), idx);
  }

  auto begin = point_arr.begin();
  auto end = point_arr.end();

  size_t length = point_arr.size();
  size_t level = 0;

  root_ = MakeTree(begin, end, length, level);
}


KDNodePtr KDTree::Nearest(const KDNodePtr& root,
                          const point_t& target_point,
                          const size_t& dimen,
                          const KDNodePtr& best_point,
                          const double& best_dist)
{
  double euclid_dist;
  double dimen_dist;


  if(!bool(*root))
  {
    return std::make_shared<KDNode>();
  }

  point_t root_point(*root);
  size_t dim = root_point.size();


  euclid_dist = dist(root_point, target_point);

  dimen_dist = root_point.at(dimen) - target_point.at(dimen);

  auto new_point = best_point;
  double new_dist = best_dist;

  if(euclid_dist < best_dist)
  {
    new_point = root;
    new_dist = euclid_dist;
  }

  size_t next_dimen = (dimen + 1) % dim;
  KDNodePtr section; // 部分 section
  KDNodePtr other;   // 其它 other

  if(dimen_dist > 0)
  {
    section = root->left_;
    other = root->right_;
  }
  else
  {
    section = root->right_;
    other = root->left_;
  }

  KDNodePtr further =
      Nearest(section, target_point, next_dimen, new_point, new_dist);

  if(!further->point_.empty())
  {
    double euclid_dist = dist(further->point_, target_point);
    if(euclid_dist < new_dist)
    {
      new_dist = euclid_dist;
      new_point = further;
    }
  }

  if(std::abs(dimen_dist) < new_dist)
  {
    further = Nearest(other, target_point, next_dimen, new_point, new_dist);

    if(!further->point_.empty())
    {
      double euclid_dist = dist(further->point_, target_point);
      if(euclid_dist < new_dist)
      {
        new_dist = euclid_dist;
        new_point = further;
      }
    }
  }
  return new_point;
}

KDNodePtr KDTree::Nearest(const point_t& point)
{
  size_t level = 0;
  double root_dist = dist(point_t(*root_), point);
  return Nearest(root_, point, level, root_, root_dist);
}

point_t KDTree::NearestPoint(const point_t& point)
{
  return point_t(*Nearest(point));
}

size_t KDTree::NearestIndex(const point_t& point)
{
  return size_t(*Nearest(point));
}

point_index_t KDTree::NearestPointIndex(const point_t& point)
{
  auto nearest = Nearest(point);
  return {point_t(*nearest), size_t(*nearest)};
}

point_index_arr_t KDTree::Neighborhood(const KDNodePtr& root,
                                       const point_t& point,
                                       const double& radius,
                                       const size_t& level)
{
  double euclid_dist = 0.;
  double dimen_dist = 0.;

  if(!bool(*root))
  {
    return {};
  }

  size_t dimen = point.size();
  euclid_dist = dist(point_t(*root), point);
  dimen_dist = point_t(*root).at(level) - point.at(level);

  point_index_arr_t nbhd{}; // 存储当前遍历的满足最近点的 数组
  point_index_arr_t nbhd_section{}; // 存储 section 下的最近点的 数组
  point_index_arr_t nbhd_other{};   // 存储 other 下的最近点的 数组

  if(euclid_dist < radius)
  {
    nbhd.emplace_back(*root);
  }

  KDNodePtr section;
  KDNodePtr other;

  if(dimen_dist > 0)
  {
    section = root->left_;
    other = root->right_;
  }
  else
  {
    section = root->right_;
    other = root->left_;
  }

  nbhd_section = Neighborhood(section, point, radius, (level + 1) % dimen);
  nbhd.insert(std::end(nbhd), std::begin(nbhd_section), std::end(nbhd_section));

  if(std::fabs(dimen_dist) < radius)
  {
    nbhd_other = Neighborhood(other, point, radius, (level + 1) % dimen);
    nbhd.insert(std::end(nbhd), std::begin(nbhd_other), std::end(nbhd_other));
  }

  return nbhd;
}


point_index_arr_t KDTree::Neighborhood(const point_t& point,
                                       const double& radius)
{
  size_t level = 0;
  return Neighborhood(root_, point, radius, level);
}

point_arr_t KDTree::NeighborhoodPoints(const point_t& point,
                                       const double& radius)
{
  size_t level = 0;
  auto nbhd = Neighborhood(root_, point, radius, level);

  point_arr_t point_vec;
  point_vec.resize(nbhd.size());

  std::transform(std::begin(nbhd),
                 std::end(nbhd),
                 std::begin(point_vec),
                 [](const point_index_t& point) { return point.first; });

  return point_vec;
}

index_arr_t KDTree::NeighborhoodIndices(const point_t& point,
                                        const double& radius)
{
  size_t level = 0;
  point_index_arr_t nbhd = Neighborhood(root_, point, radius, level);

  index_arr_t index_vec;
  index_vec.resize(nbhd.size());

  std::transform(nbhd.begin(),
                 nbhd.end(),
                 index_vec.begin(),
                 [](const point_index_t& point) { return point.second; });

  return index_vec;
}

} // namespace beacon