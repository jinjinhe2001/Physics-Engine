
#ifndef AL_BOUNDINGBOXDATA_HPP
#define AL_BOUNDINGBOXDATA_HPP

#include <limits>

//#include "al/graphics/al_Graphics.hpp"
//#include "al/graphics/al_Mesh.hpp"
//#include "al/graphics/al_Shapes.hpp"
#include "al/math/al_Vec.hpp"

namespace al {

struct BoundingBoxData {
  Vec3f min, max;
  Vec3f cen, dim;

  BoundingBoxData();
  BoundingBoxData(const Vec3f &min_, const Vec3f &max_);

  void set(const Vec3f &min_, const Vec3f &max_);

  void setCenterDim(const Vec3f &cen_, const Vec3f &dim_);

  bool contains(const Vec3d &p);

  inline void reset();

  /**
   * @brief reset box to float value inverse extremes
   *
   * This allows building the bounding box incremementally using includePoint()
   */
  inline void resetInv() {
    max = {std::numeric_limits<float>::lowest(),
           std::numeric_limits<float>::lowest(),
           std::numeric_limits<float>::lowest()};
    min = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
           std::numeric_limits<float>::max()};
  }

  inline void includePoint(al::Vec3f &pos) {
    if (pos.x > max.x) {
      max.x = pos.x;
    } else if (pos.x < min.x) {
      min.x = pos.x;
    }
    if (pos.y > max.y) {
      max.y = pos.y;
    } else if (pos.y < min.y) {
      min.y = pos.y;
    }
    if (pos.z > max.z) {
      max.z = pos.z;
    } else if (pos.z < min.z) {
      min.z = pos.z;
    }
  }
};

} // namespace al

#endif // AL_BOUNDINGBOXDATA_HPP
