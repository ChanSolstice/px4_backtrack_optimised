#include "px4_backtrack/domain/recorded_route.hpp"
#include <algorithm>
#include <cmath>

namespace px4_backtrack::domain {
void RecordedRoute::reset() {
  points_.clear(); arcs_.clear(); progress_=0.0;
  distance_=std::numeric_limits<double>::infinity();
}
bool RecordedRoute::build(const std::vector<RecordPoint>& points) {
  reset();
  for (auto it=points.rbegin(); it!=points.rend(); ++it) {
    if (!it->position.allFinite()) { reset(); return false; }
    if (!points_.empty() && (it->position-points_.back()).norm()<1e-9) continue;
    arcs_.push_back(points_.empty() ? 0.0 : arcs_.back()+(it->position-points_.back()).norm());
    points_.push_back(it->position);
  }
  return valid();
}
bool RecordedRoute::update(const Vec3& position) {
  if (!valid() || !position.allFinite()) return false;
  const double lo=std::max(0.0,progress_-0.3), hi=std::min(arcs_.back(),progress_+1.0);
  size_t i=std::max<size_t>(1, std::lower_bound(arcs_.begin(),arcs_.end(),lo)-arcs_.begin());
  double best=std::numeric_limits<double>::infinity(), best_s=progress_;
  for (; i<points_.size() && arcs_[i-1]<=hi; ++i) {
    const Vec3 edge=points_[i]-points_[i-1];
    const double length=arcs_[i]-arcs_[i-1];
    const double fraction=std::clamp((position-points_[i-1]).dot(edge)/edge.squaredNorm(),
        std::max(0.0,(lo-arcs_[i-1])/length), std::min(1.0,(hi-arcs_[i-1])/length));
    const Vec3 candidate=points_[i-1]+fraction*edge;
    const double d=(position-candidate).norm();
    if (d<best) { best=d; nearest_=candidate; best_s=arcs_[i-1]+fraction*length; }
  }
  distance_=best;
  progress_=std::max(progress_,best_s);
  return std::isfinite(distance_);
}
double RecordedRoute::predictedDeviation(const Vec3& position, const Vec3& velocity,
    double reaction_s, double braking_m_s2, double uncertainty_m) const {
  if (!position.allFinite() || !velocity.allFinite() || !std::isfinite(distance_))
    return std::numeric_limits<double>::infinity();
  const double outward=distance_>1e-9 ? std::max(0.0,velocity.dot(position-nearest_)/distance_) : 0.0;
  return distance_+uncertainty_m+outward*reaction_s+outward*outward/(2.0*braking_m_s2);
}
}
