#include "bsdf.h"
#include "bsdf.h"
#include "bsdf.h"

#include "application/visual_debugger.h"

#include <algorithm>
#include <iostream>
#include <utility>


using std::max;
using std::min;
using std::swap;

namespace CGL {

/**
 * This function creates a object space (basis vectors) from the normal vector
 */
void make_coord_space(Matrix3x3 &o2w, const Vector3D n) {

  Vector3D z = Vector3D(n.x, n.y, n.z);
  Vector3D h = z;
  if (fabs(h.x) <= fabs(h.y) && fabs(h.x) <= fabs(h.z))
    h.x = 1.0;
  else if (fabs(h.y) <= fabs(h.x) && fabs(h.y) <= fabs(h.z))
    h.y = 1.0;
  else
    h.z = 1.0;

  z.normalize();
  Vector3D y = cross(h, z);
  y.normalize();
  Vector3D x = cross(z, y);
  x.normalize();

  o2w[0] = x;
  o2w[1] = y;
  o2w[2] = z;
}

/**
 * Evaluate diffuse lambertian BSDF.
 * Given incident light direction wi and outgoing light direction wo. Note
 * that both wi and wo are defined in the local coordinate system at the
 * point of intersection.
 * \param wo outgoing light direction in local space of point of intersection
 * \param wi incident light direction in local space of point of intersection
 * \return reflectance in the given incident/outgoing directions
 */
Vector3D DiffuseBSDF::f(const Vector3D wo, const Vector3D wi) {
  // TODO (Part 3.1):
  // This function takes in both wo and wi and returns the evaluation of
  // the BSDF for those two directions.

  //return Vector3D(1.0);
  return reflectance / PI; // light scatters equally in all directions bc diffuse!
}

/**
 * Evalutate diffuse lambertian BSDF.
 */
Vector3D DiffuseBSDF::sample_f(const Vector3D wo, Vector3D *wi, double *pdf) {
  // TODO (Part 3.1):
  // This function takes in only wo and provides pointers for wi and pdf,
  // which should be assigned by this function.
  // After sampling a value for wi, it returns the evaluation of the BSDF
  // at (wo, *wi).
  // You can use the `f` function. The reference solution only takes two lines.

  //return Vector3D(1.0);
  *wi = sampler.get_sample(pdf);
  return f(wo, *wi);

}

void DiffuseBSDF::render_debugger_node()
{
  if (ImGui::TreeNode(this, "Diffuse BSDF"))
  {
    DragDouble3("Reflectance", &reflectance[0], 0.005);
    ImGui::TreePop();
  }
}

/**
 * Evalutate Emission BSDF (Light Source)
 */
Vector3D EmissionBSDF::f(const Vector3D wo, const Vector3D wi) {
  return Vector3D();
}

/**
 * Evalutate Emission BSDF (Light Source)
 */
Vector3D EmissionBSDF::sample_f(const Vector3D wo, Vector3D *wi, double *pdf) {
  *pdf = 1.0 / PI;
  *wi = sampler.get_sample(pdf);
  return Vector3D();
}

void EmissionBSDF::render_debugger_node()
{
  if (ImGui::TreeNode(this, "Emission BSDF"))
  {
    DragDouble3("Radiance", &radiance[0], 0.005);
    ImGui::TreePop();
  }
}


MultilayerBSDF::MultilayerBSDF(float d_film, float d_air_,
  float n_film, float c_interf_,
  float phong_exp,
  float ambient_coeff,
  float light_scale)
: d(d_film),         // initialize member `d` with `d_film`
d_air(d_air_),     // initialize member `d_air` with `d_air_`
n(n_film),
c_interf(c_interf_),
n_phong(phong_exp),
ca(ambient_coeff),
I0_scale(light_scale) { 
  std::cout << "Inside MultilayerBSDF constructor" << std::endl;
  std::cout << "  d_film = " << d_film << std::endl;
  std::cout << "  d_air  = " << d_air << std::endl;
  std::cout << "  n_film = " << n_film << std::endl;
  std::cout << "  c_interf = " << c_interf << std::endl;
  std::cout << "  n_phong = " << n_phong << std::endl;
  std::cout << "  ambient = " << ca << std::endl;
  std::cout << "  scale = " << I0_scale << std::endl;

  // If you initialize any objects here (like samplers), print about that too
  std::cout << "Constructed sampler..." << std::endl;
}

Vector3D MultilayerBSDF::f(const Vector3D wo, const Vector3D wi) {

  std::cout << "Inside MultilayerBSDF::f()" << std::endl;

  // === PARAMETERS ===
  double ca = 0.1;        // Ambient coefficient
  double cs = 1.0;        // Specular interference coefficient
  double m = 8.0;         // Interference power (peak sharpness)
  double n_phong = 50.0;  // Phong exponent (highlight sharpness)

  // === WAVELENGTHS FOR RGB (in nm) ===
  const double lambda_r = 650.0;
  const double lambda_g = 550.0;
  const double lambda_b = 450.0;

  // === Extract geometry ===
  Vector3D N(0, 0, 1); // Local shading normal
  Vector3D H = (wo + wi).unit(); // Half-vector
  double cos_theta = abs_cos_theta(wi); // Incident angle
  double sin_theta = sqrt(1.0 - cos_theta * cos_theta);

  // === Snell's Law ===
  double sin_theta_prime = sin_theta / n;
  sin_theta_prime = clamp(sin_theta_prime, 0.0, 1.0);
  double cos_theta_prime = sqrt(1.0 - sin_theta_prime * sin_theta_prime);

  // === Delta_b function ===
  auto delta_b = [&](double lambda_nm) {
    return (4.0 * PI / lambda_nm) * (n * d * cos_theta_prime + d_air * cos_theta);
  };

  // === R_empirical function ===
  auto R_empirical = [&](double lambda_nm) {
    double cos_db = cos(delta_b(lambda_nm));
    return (cos_db > 0.0) ? c_interf * pow(cos_db, m) : 0.0;
  };

  // === Reflectance from interference model ===
  double Rr = R_empirical(lambda_r);
  double Rg = R_empirical(lambda_g);
  double Rb = R_empirical(lambda_b);

  // === Phong term ===
  double phong = pow(clamp(dot(H, N), 0.0, 1.0), n_phong);

  // === Final spectrum ===
  Vector3D I0(1.0, 1.0, 1.0); // Assume white light for now
  Vector3D ambient = ca * I0;
  Vector3D specular = cs * Vector3D(Rr, Rg, Rb) * phong;

  return ambient + specular;
}

Vector3D MultilayerBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {
  std::cout << "Inside MultilayerBSDF::sample_f()" << std::endl;
  // Sample a direction wi according to a cosine-weighted hemisphere
  *wi = sampler.get_sample(pdf);

  // Evaluate the BSDF in that direction
  return f(wo, *wi);
}



} // namespace CGL
