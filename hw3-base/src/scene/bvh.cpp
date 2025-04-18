#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
    namespace SceneObjects {

        BVHAccel::BVHAccel(const std::vector<Primitive*>& _primitives,
            size_t max_leaf_size) {

            primitives = std::vector<Primitive*>(_primitives);
            root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
        }

        BVHAccel::~BVHAccel() {
            if (root)
                delete root;
            primitives.clear();
        }

        BBox BVHAccel::get_bbox() const { return root->bb; }

        void BVHAccel::draw(BVHNode* node, const Color& c, float alpha) const {
            if (node->isLeaf()) {
                for (auto p = node->start; p != node->end; p++) {
                    (*p)->draw(c, alpha);
                }
            }
            else {
                draw(node->l, c, alpha);
                draw(node->r, c, alpha);
            }
        }

        void BVHAccel::drawOutline(BVHNode* node, const Color& c, float alpha) const {
            if (node->isLeaf()) {
                for (auto p = node->start; p != node->end; p++) {
                    (*p)->drawOutline(c, alpha);
                }
            }
            else {
                drawOutline(node->l, c, alpha);
                drawOutline(node->r, c, alpha);
            }
        }

        BVHNode* BVHAccel::construct_bvh(std::vector<Primitive*>::iterator start,
            std::vector<Primitive*>::iterator end,
            size_t max_leaf_size) {

            // TODO (Part 2.1):
            // Construct a BVH from the given vector of primitives and maximum leaf
            // size configuration. The starter code build a BVH aggregate with a
            // single leaf node (which is also the root) that encloses all the
            // primitives.

            //BVHNode *node = new BVHNode(bbox);
            //node->start = start;
            //node->end = end;

            //return node;
              // count number of existing primitives
            BBox bbox, centroid_box;
            int size = 0;
            for (auto p = start; p != end; p++) {
                BBox bb = (*p)->get_bbox();
                bbox.expand(bb);
                Vector3D cent = bb.centroid();
                centroid_box.expand(cent);
                size++;
            }

            BVHNode* node = new BVHNode(bbox);

            // base case = if max number of primitives, make current node a leaf
            if (size <= max_leaf_size) {
                node->start = start;
                node->end = end;
                return node;  // Return leaf node with a range of primitives
            }

            Vector3D extent = centroid_box.extent;
            int axis = 0;
            if (extent.y > extent.x) axis = 1;
            if (extent.z > extent[axis]) axis = 2;

            // sort primitives along the chosen axis
            std::sort(start, end, [axis](const Primitive* a, const Primitive* b) {
                return a->get_bbox().centroid()[axis] < b->get_bbox().centroid()[axis];
                });

            // find the midpoint based on the spatial median of centroids
            float split_coord = centroid_box.min[axis] + extent[axis] * 0.5f;

            // Find the iterator closest to the spatial median
            auto midIter = std::partition(start, end, [axis, split_coord](const Primitive* prim) {
                return prim->get_bbox().centroid()[axis] < split_coord;
                });

            // build left + right children
            node->l = construct_bvh(start, midIter, max_leaf_size);
            node->r = construct_bvh(midIter, end, max_leaf_size);

            return node;
        }

        bool BVHAccel::has_intersection(const Ray& ray, BVHNode* node) const {
            // TODO (Part 2.3):
            // Fill in the intersect function.
            // Take note that this function has a short-circuit that the
            // Intersection version cannot, since it returns as soon as it finds
            // a hit, it doesn't actually have to find the closest hit.

            //for (auto p : primitives) {
            //  total_isects++;
            //  if (p->has_intersection(ray))
            //    return true;
            //}
            //return false;
            
            // terminate if doesn't hit bounding box
            double t0, t1;
            if (!node->bb.intersect(ray, t0, t1)) {
                return false;
            }

			// if the ray intersects the bounding box, check if its a leaf node
            if (node->isLeaf()) {
                bool hit = false;
                for (auto p = node->start; p != node->end; p++) {
                    total_isects++;
                    hit = (*p)->has_intersection(ray) || hit;
                }
                return hit;
            }
            else // check both children if not leaf
            {
                total_isects++;
                return has_intersection(ray, node->l) || has_intersection(ray, node->r);
            }
            return false;
        }

        bool BVHAccel::intersect(const Ray& ray, Intersection* i, BVHNode* node) const {
            // TODO (Part 2.3):
            // Fill in the intersect function.

            //bool hit = false;
            //for (auto p : primitives) {
            //  total_isects++;
            //  hit = p->intersect(ray, i) || hit;
            //}
            //return hit;

            // terminate if doesn't hit bounding box
            double t0, t1;
            if (!node->bb.intersect(ray, t0, t1)) {
                return false;
            }

			// if the ray intersects the bounding box, check if its a leaf node
            if (node->isLeaf()) {
                bool hit = false;
                for (auto p = node->start; p != node->end; p++) {
                    total_isects++;
                    hit = (*p)->intersect(ray, i) || hit;
                }
                return hit;
            }
            else // check both children if not leaf
            {
                total_isects++;
                bool inter = intersect(ray, i, node->l);
                return intersect(ray, i, node->r) || inter;
            }
            return false;
        } // namespace SceneObjects
    } // namespace CGL
}
