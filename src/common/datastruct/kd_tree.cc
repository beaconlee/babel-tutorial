#include "kd_tree.h"

namespace beacon
{

// square euclidean distance
double dist2(const point_t& point_a, const point_t& pont)
{
  double result = 0.;
  for(int index = 0; index < point_a.size(); ++index)
  {
    // result +=
  }
}
double dist2(const KDNodePtr&, const KDNodePtr&) {}

// euclidean distance
double dist(const point_t&, const point_t&) {}
double dist(const KDNodePtr&, const KDNodePtr&) {}

} // namespace beacon