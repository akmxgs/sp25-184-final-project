#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Vector3D
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // Note: When comparing Cornel Box (CBxxx.dae) results to importance sampling, you may find the "glow" around the light source is gone.
  // This is totally fine: the area lights in importance sampling has directionality, however in hemisphere sampling we don't model this behaviour.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Vector3D L_out;

  // TODO (Part 3): Write your sampling loop here
  // TODO BEFORE YOU BEGIN
  // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading 
  for (int n = 0; n < num_samples; n++) {
    // get sample direction
	Vector3D samp_dir = hemisphereSampler->get_sample();

    // calculate direction in world + create ray for intersection
    Vector3D d = o2w * samp_dir;
    Vector3D o = (d * EPS_D) + hit_p;

    Ray r = Ray(o, d);
    r.min_t = EPS_F;
    Intersection i;
	// check for intersection with scene
	bool isect_check = bvh->intersect(r, &i);

    // get bsdf and emission of surface
    if (isect_check) {
		Vector3D e = i.bsdf->get_emission();
		Vector3D bsdf = isect.bsdf->f(w_out, samp_dir);

        // accumulate L_out light contribution
        L_out = L_out + e * bsdf * cos_theta(samp_dir);
    }
  }
  // update L_out to have return value -> monte carlo estimate of direct lighting
  L_out = (L_out * (2.0 * PI)) / num_samples;

  return L_out;
}

Vector3D
PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in
  // a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);
  Vector3D L_out;

  // for every light, loop over lights
  for (SceneLight* L : scene->lights) {
      int num_samples;

      // figure out num samples, if delta light only need to do once
	  if (L->is_delta_light()) {
		  num_samples = 1;
	  }
	  else {
		  num_samples = ns_area_light;
	  }

      // for every sample for THIS light
      for (int n = 0; n < num_samples; n++) {
        // initialize addresses for light sampling
          Vector3D wi;
          double distToLight, pdf;

		  Vector3D rad = L->sample_L(hit_p, &wi, &distToLight, &pdf);

          // calculate w_in
		  Vector3D w_in = w2o * wi;

          // check if front light
          if (0 <= w_in.z) { // if z negative, light is coming from the back, we don't want
              Vector3D o = (wi * EPS_D) + hit_p;
              Ray r = Ray(o, wi);
              r.min_t = EPS_F;
			  r.max_t = distToLight - EPS_F;

              // do an occlusion test to see if you need to check for light
              Intersection i;
              bool blocked = bvh->intersect(r, &i);

              if (!blocked) {
				  Vector3D bsdf = isect.bsdf->f(w_out, w_in);

				  L_out += (rad * bsdf * cos_theta(w_in)) / pdf; // accumulate light
              }
          }
      }
	  L_out /= num_samples; // divide by number of samples
  }
  return L_out;
}

Vector3D PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // TODO: Part 3, Task 2
  // Returns the light that results from no bounces of light
  
  return isect.bsdf->get_emission();
}

Vector3D PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // TODO: Part 3, Task 3
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`

  if (direct_hemisphere_sample) {
        return estimate_direct_lighting_hemisphere(r, isect);
  }
  return estimate_direct_lighting_importance(r, isect);

}

Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d);

  Vector3D L_out(0, 0, 0);

  if (max_ray_depth == 0) {
      return L_out;
  }

  // TODO (Part 4.2): 
  // Here is where your code for sampling the BSDF,
  // performing Russian roulette step, and returning a recursively 
  // traced ray (when applicable) goes

  // reorder if statements for coherence
  // don't keep bouncing if at depth - 1
  if (isAccumBounces || r.depth == max_ray_depth - 1) {
      L_out += one_bounce_radiance(r, isect);
  }

  if (r.depth >= max_ray_depth - 1) {
      return L_out;
  }

  Vector3D w_in;
  double pdf;
  Vector3D bsdf = isect.bsdf->sample_f(w_out, &w_in, &pdf);

  // add in russian roulette probability
  float russ_roul = 1;
  
  if (max_ray_depth <= 1) {
      return L_out;
  }
  
  if (coin_flip(russ_roul) && (r.depth != max_ray_depth) && (pdf > 0)) { // if not max, keep going? prevent zero division
      Vector3D wi = o2w * w_in;
      Ray next_r = Ray(hit_p + (EPS_D * wi), wi, (int)r.depth + 1);
      Intersection i;

      if (bvh->intersect(next_r, &i)) {
          Vector3D bounce_rad = at_least_one_bounce_radiance(next_r, i);

          L_out += (bounce_rad * bsdf * cos_theta(w_in)) / pdf / russ_roul;
      }
  }

  return L_out;
}

Vector3D PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Vector3D L_out;

  // You will extend this in assignment 3-2.
  // If no intersection occurs, we simply return black.
  // This changes if you implement hemispherical lighting for extra credit.

  // The following line of code returns a debug color depending
  // on whether ray intersection with triangles or spheres has
  // been implemented.
  //
  // REMOVE THIS LINE when you are ready to begin Part 3.
  
  if (!bvh->intersect(r, &isect))
    return envLight ? envLight->sample_dir(r) : L_out;

  L_out = Vector3D(0, 0, 0);
  //L_out = (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);

  // TODO (Part 3): Return the direct illumination.
  if (isAccumBounces || max_ray_depth == 0) {
      L_out += zero_bounce_radiance(r, isect);
  }

  // TODO (Part 4): Accumulate the "direct" and "indirect"
  // parts of global illumination into L_out rather than just direct
  //L_out += one_bounce_radiance(r, isect);
  L_out += at_least_one_bounce_radiance(r, isect);

  return L_out;
}

void PathTracer::raytrace_pixel(size_t x, size_t y) {
  // TODO (Part 1.2):
  // Make a loop that generates num_samples camera rays and traces them
  // through the scene. Return the average Vector3D.
  // You should call est_radiance_global_illumination in this function.

  // TODO (Part 5):
  // Modify your implementation to include adaptive sampling.
  // Use the command line parameters "samplesPerBatch" and "maxTolerance"
  int num_samples = ns_aa;          // total samples to evaluate
  Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel

  Vector3D result(0, 0, 0); // place to add, will divide later

  // part 5 additional vars to track for adaptive sampling
  float s1 = 0;
  float s2 = 0;
  float mean = 0;
  float var = 0;
  float conv = 0;
  int samp_ct = 0;
  float lum = 0;

  for (int i = 0; i < num_samples; i++) {
      Vector2D samp = gridSampler->get_sample(); // offset
	  Vector2D pos = origin + samp; // pixel sample w offset

	  Ray r = camera->generate_ray(pos.x / sampleBuffer.w, pos.y / sampleBuffer.h); // normalize input coords by image dimensions

	  Vector3D rad = est_radiance_global_illumination(r);
	  result += rad; // accumulate

      // tip 2: add to sigmas with illumination for part 5
      lum = rad.illum();
      s1 += lum;
      s2 += lum * lum;
      samp_ct += 1;

      // break the loop if has converged every samples per batch?
      if ((samp_ct % samplesPerBatch) == 0) { // oop didn't check for zero division
          mean = s1 / (float)samp_ct;
          var = (1.0f / ((float)samp_ct - 1)) * (s2 - ((s1 * s1) / (float)samp_ct));

          conv = 1.96f * ((sqrt(var)) / sqrt(samp_ct));

          if (conv <= (maxTolerance * mean)) {
              break;
          }
      }
  }
  result /= (float)samp_ct;
  
  sampleBuffer.update_pixel(result, x, y);

  // tip 4: dill samplecount buffer w actual num samples per pixel
  sampleCountBuffer[x + y * sampleBuffer.w] = samp_ct;
}

void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
