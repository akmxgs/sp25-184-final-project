#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.

  Vector3D origin = r.o;
  Vector3D dir = r.d;
  Vector3D center = o;

  double a = dot(dir, dir);
  double b = dot((2 * (origin - center)), dir);
  double c = dot((origin - center), (origin - center)) - r2;

  double discriminant = pow(b, 2) - (4 * a * c);
  if (discriminant <= 0) { return false; }
  
  double temp1 = (-b - sqrt(discriminant)) / (2 * a);
  double temp2 = (-b + sqrt(discriminant)) / (2 * a);

  // want to make sure temp1 stays the smaller one
  if (temp1 > temp2) {
	  swap(temp1, temp2);
  }

  if ((temp1 >= r.min_t) && (temp2 <= r.max_t)) { 
	  t1 = temp1; // swap above to ensure temp1 is always the smaller one
	  t2 = temp2;
	  r.max_t = t1;
	  return true;
  }

  return false;
}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
  double t1, t2; // initialize to have addresses to pass into test helper

  return test(r, t1, t2);
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
  double t1, t2; // same as above, initialize to have addresses to pass into test helper
  if (test(r, t1, t2)) {
	  i->t = r.max_t;
	  i->n = ((i->t * r.d + r.o) - o); // norm vector calculation from center of sphere
	  i->n.normalize();
	  i->primitive = this;
	  i->bsdf = get_bsdf();
	  return true;
  }

  return false;
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
