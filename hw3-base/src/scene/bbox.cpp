#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.

  Vector3D min_corner = min - r.o;
  Vector3D max_corner = max - r.o;

  double min_tx = std::min(min_corner.x / r.d.x, max_corner.x / r.d.x); // do intersection tests along each axis
  double max_tx = std::max(min_corner.x / r.d.x, max_corner.x / r.d.x);

  double min_ty = std::min(min_corner.y / r.d.y, max_corner.y / r.d.y);
  double max_ty = std::max(min_corner.y / r.d.y, max_corner.y / r.d.y);

  double min_tz = std::min(min_corner.z / r.d.z, max_corner.z / r.d.z);
  double max_tz = std::max(min_corner.z / r.d.z, max_corner.z / r.d.z);

  double min_t = std::max(min_tx, std::max(min_ty, min_tz));
  double max_t = std::min(max_tx, std::min(max_ty, max_tz));

  if ((min_t > max_t) || (min_t > r.max_t) || (max_t < r.min_t)) { // check that ts are within bounds
     return false;
  }

  t0 = min_t; // update the min and max t to t0 and t1 respectively
  t1 = max_t;
  return true;
}

void BBox::draw(Color c, float alpha) const {

  glColor4f(c.r, c.g, c.b, alpha);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
