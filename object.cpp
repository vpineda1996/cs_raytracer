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
	

    Ray localRay(i_transform_fast * ray.origin, i_transform_fast * ray.direction);
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
	double c = ray.origin.dot(ray.origin) - (this->radius* this->radius);

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
	
	Vector normal = Vector(0, 0, 1, 0);
	if (ray.origin[2] < 0) normal += -1;

	if (std::abs(normal.dot(ray.direction)) < 0.00001) return false;
	
	double t = -normal.dot(ray.origin) / normal.dot(ray.direction);
	if (t < 0) return false;
	if (t > hit.depth) return false;

	Vector newPosition = Vector(ray.origin) + t*Vector(ray.direction);

	hit.position = newPosition;
	hit.normal = normal.normalized();
	hit.depth = t;

	return true;
}

inline bool Mesh::intersectTriangle(Ray const &ray, Triangle const &tri, Intersection &hit) const
{
	// Extract vertex positions from the mesh data.
	Vector const &p0 = positions[tri[0].pi];
	Vector const &p1 = positions[tri[1].pi];
	Vector const &p2 = positions[tri[2].pi];

	Vector    u, v, n;              // triangle vectors
	Vector    dir, w0, w;           // ray vectors
	double     r, a, b;              // params to calc ray-plane intersect

									// get triangle edge vectors and plane normal
	u = p1 - p0;
	v = p2 - p0;
	n = u.cross(v);              // cross product
	if (n == Vector(0))            // triangle is degenerate
		return false;                  // do not deal with this case

	dir = ray.direction;              // ray direction vector
	w0 = ray.origin - Vector(p0);
	a = -n.dot(w0);
	b = n.dot(dir);
	if (fabs(b) < 0.000001) {     // ray is  parallel to triangle plane
		if (a == 0)                 // ray lies in triangle plane
			return 2;
		else return 0;              // ray disjoint from plane
	}

	// get intersect point of ray with triangle plane
	r = a / b;
	if (r < 0.0)                    // ray goes away from triangle
		return 0;                   // => no intersect
									// for a segment, also test if (r > 1.0) => no intersect

	if (r > hit.depth) return false;
	Vector intersection = Vector(ray.origin) + r * dir;            // intersect point of ray and plane

									// is I inside T?
	double    uu, uv, vv, wu, wv, D;
	uu = u.dot(u);
	uv = u.dot(v);
	vv = v.dot(v);
	w = intersection - p0;
	wu = w.dot(u);
	wv = w.dot(v);
	D = uv * uv - uu * vv;

	// get and test parametric coords
	float s, t;
	s = (uv * wv - vv * wu) / D;
	if (s < 0.0 || s > 1.0)         // I is outside T
		return false;
	t = (uv * wu - uu * wv) / D;
	if (t < 0.0 || (s + t) > 1.0)  // I is outside T
		return false;
	// update intersection

	hit.depth = r;
	hit.position = intersection;
	hit.normal = n.normalized();

	return true;                       // I is in T
}

bool Conic::localIntersect(Ray const &ray, Intersection &hit) const {
	//////////////////
	// YOUR CODE HERE (creative license)
	return false;
	
	Vector yInv = Vector(1, -1, 1);
	Vector f = Vector(yInv) * ray.direction;
	Vector g = ray.origin;
	double h = this->radius2 * (this->zMax - this->zMin) / (this->radius2 - this->radius1) ;

	Vector raydirSq = f*f;
	double a = raydirSq.dot(Vector(yInv));
	double b = 2 * f.dot(g) + 2*h * ray.direction[1];
	Vector rayOrigSq = g*g;
	double c = rayOrigSq.dot(Vector(yInv)) - (h*h - 2*h*ray.origin[1]);


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

	if (t1 > hit.depth || intersection[1] > h  || intersection[1] < 0 ) {
		return false;
	}

	hit.position = intersection;
	hit.normal = Vector(2*intersection[0], -2*(intersection[1] - h), 2* intersection[2]).normalized();
	hit.depth = t1;

	return true;
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
	int numTriang = triangles.size();
	for (size_t tri_i = 0; tri_i < numTriang; tri_i++) {
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


