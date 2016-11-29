#include "object.hpp"
#include <fstream>
#include <sstream>
#include <map>
#include <vector>
#include <cmath>
#include <cfloat>
#include <iostream>

// 2016 Version

bool Object::intersect(Ray ray, Intersection &hit) const 
{
    // Assert the correct values of the W coords of the origin and direction.
    // You can comment this out if you take great care to construct the rays
    // properly.
    ray.origin[3] = 1;
    ray.direction[3] = 0;
	

    Ray localRay(i_transform * ray.origin, i_transform * ray.direction);
	//!!! USEFUL NOTES: to calculate depth in localIntersect(), if the intersection happens at
	//ray.origin + ray.direction * t, then t is just the depth
	//!!! USEFUL NOTES: Here direction might be scaled, so you must not renormalize it in
	//localIntersect(), or you will get a depth in local coordinate system,
	//which can't be compared with intersections with other objects
    if (localIntersect(localRay, hit))
	{
        // Assert correct values of W.
        hit.position[3] = 1;
        hit.normal[3] = 0;
        
		// Transform intersection coordinates into global coordinates.
        hit.position = transform * hit.position;
        hit.normal = (n_transform * hit.normal).normalized();
        
		return true;
    }

    return false;
}

//INTERSECTIONS 

bool Sphere::localIntersect(Ray const &ray, Intersection &hit) const 
{
	//////////////////
    // YOUR CODE HERE 
	// For this part you are required to calculate if a ray has intersected your sphere.
	// Be sure to cover all the possible intersection scenarios(zero, one, and two points of intersection).
	// Test your result by comparing the output depth image of your algorithm with the provided example solution's results.



	// Here in local coordinate system, the sphere is centered on (0, 0, 0)
	//with radius 1.0
	//
	// NOTE: hit.depth is the current closest intersection depth, so don't
	// accept any intersection that happens further away than that.

	double a = ray.direction.dot(ray.direction);
	double b = 2 * ray.direction.dot(ray.origin);
	double c = ray.origin.dot(ray.origin) - this->radius;

	double d = b*b - 4 * a*c;

	if (d <= 0) {
		return false;
	}

	double sqrtd = sqrt(d);

	double t1 = (-b + sqrtd) / (2 * a);
	double t2 = (-b - sqrtd) / (2 * a);

	if (t1 < 0 && t2 < 0) {
		return false;
	}

	if (t1 > t2 && t2 > 0) std::swap(t1, t2);
	const Vector &intersection = ray.origin + t1*ray.direction;

	if (t1 > hit.depth) {
		return false;
	}

	hit.position = intersection;
	hit.normal = (intersection).normalized();
	hit.depth = t1;

	return true;
}


bool Plane::localIntersect(Ray const &ray, Intersection &hit) const
{
	//////////////////
	// YOUR CODE HERE 
	// The implementation of this part is similar to the previous part in that you are 
	// calculating if a line has intersected your plane.Test this function in a similar 
	// way to the previous part(note that as you do new objects will appear).

	// Do not accept intersection while ray is inside the plane
	// Here in local coordinate system, the plane is at z = 0
	//
	// NOTE: hit.depth is the current closest intersection depth, so don't
	// accept any intersection that happens further away than that.

	//if (ray.origin[1] < 0) return false; // inside plane

	double t = -ray.origin[2] / ray.direction[2];

	if (t < 0) return false;
	if (t > hit.depth) return false;
	//if (double depth = ray.origin[2] + t*ray.direction[2] < hit.depth) {
	//	// hit.depth = depth;
	//	hit.position = ray.origin + t*ray.direction;
	//	hit.normal = Vector(0, 1, 0);
	//	return true;
	//}

    return false;
}

bool Mesh::intersectTriangle(Ray const &ray, Triangle const &tri, Intersection &hit) const
{
	// Extract vertex positions from the mesh data.
	Vector const &p0 = positions[tri[0].pi];
	Vector const &p1 = positions[tri[1].pi];
	Vector const &p2 = positions[tri[2].pi];

	// First calculate plane for triange

	Vector &normalToPlane = (p2 - p0).cross(p1 - p0);
	double t = (p0 - ray.origin).dot(normalToPlane) / (ray.origin.dot(normalToPlane));

	Vector &point = ray.origin + t*ray.direction;

	double sign0 = implicitLineEquation(point[0], point[1], p0[0], p0[1], p1[0], p1[1]);
	double sign1 = implicitLineEquation(point[0], point[1], p0[0], p0[1], p2[0], p2[1]);
	double sign2 = implicitLineEquation(point[0], point[1], p2[0], p2[1], p1[0], p1[1]);

	if ((sign0 >= 0 && sign1 >= 0 && sign2 >= 0) || (sign0 <= 0 && sign1 <= 0 && sign2 <= 0) && normalToPlane.dot(ray.direction) < 0 && hit.depth < point[2]) {
		hit.position = point;
		hit.depth = point[2];
		hit.normal = normalToPlane;
		return false;
	}

	//////////////////
	// YOUR CODE HERE 
	// Think back through the course and try to decide what equations might help you decide on which 
	// side of the bounding lines of the triangle the ray intersects.
	// Test this part just like the above two parts; when triangle intersection is working properly, 
	// you should be able to see full meshes appear in your scenes.
	//
	// NOTE: hit.depth is the current closest intersection depth, so don't
	// accept any intersection that happens further away than that.
	//!!! USEFUL NOTES: for the intersection point, its normal should satisfy hit.normal.dot(ray.direction) < 0

	return false;
}

bool Conic::localIntersect(Ray const &ray, Intersection &hit) const {
	//////////////////
	// YOUR CODE HERE (creative license)
    return false;
}


// Intersections!
bool Mesh::localIntersect(Ray const &ray, Intersection &hit) const
{
	// Bounding box check
	double tNear = -DBL_MAX, tFar = DBL_MAX;
	for (int i = 0; i < 3; i++) {
		if (ray.direction[i] == 0.0) {
			if (ray.origin[i] < bboxMin[i] || ray.origin[i] > bboxMax[i]) {
				// Ray parallel to bounding box plane and outside of box!
				return false;
			}
			// Ray parallel to bounding box plane and inside box: continue;
		}
		else {
			double t1 = (bboxMin[i] - ray.origin[i]) / ray.direction[i];
			double t2 = (bboxMax[i] - ray.origin[i]) / ray.direction[i];
			if (t1 > t2) std::swap(t1, t2); // Ensure t1 <= t2

			if (t1 > tNear) tNear = t1; // We want the furthest tNear
			if (t2 < tFar) tFar = t2; // We want the closest tFar

			if (tNear > tFar) 
				return false; // Ray misses the bounding box.
			if (tFar < 0) 
				return false; // Bounding box is behind the ray.
		}
	}
	// If we made it this far, the ray does intersect the bounding box.

	// The ray hits the bounding box, so check each triangle.
	bool isHit = false;
	for (size_t tri_i = 0; tri_i < triangles.size(); tri_i++) {
		Triangle const &tri = triangles[tri_i];

		if (intersectTriangle(ray, tri, hit)) {
			isHit = true;
		}
	}
	return isHit;
}

double Mesh::implicitLineEquation(double p_x, double p_y,
	double e1_x, double e1_y,
	double e2_x, double e2_y) const
{
	return (e2_y - e1_y)*(p_x - e1_x) - (e2_x - e1_x)*(p_y - e1_y);
}


