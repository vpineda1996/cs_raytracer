#include <vector>
#include <cstdlib>
#include <cfloat>
#include <cmath>
#include <map>
#include <vector>
#include <cstdio>
#include <string>
#include <fstream>
#include <algorithm>
#include <iostream>
#include <sstream>
#include "raytracer.hpp"
#include "image.hpp"
#include <omp.h>

// 2016 Version

#define EPSILON 1e-6
#define ANTIALIASRES 10
#define SOFTLIGHTRES 10

void Raytracer::render(const char *filename, const char *depth_filename, Scene const &scene)
{
    // Allocate the two images that will ultimately be saved.

    Image colorImage(scene.resolution[0], scene.resolution[1]);
    Image depthImage(scene.resolution[0], scene.resolution[1]);
    
    // Create the zBuffer.
    double *zBuffer = new double[scene.resolution[0] * scene.resolution[1]];
    for(int i = 0; i < scene.resolution[0] * scene.resolution[1]; i++) {
        zBuffer[i] = DBL_MAX;
    }

	//////////////////
	// YOUR CODE HERE 
	// calculate camera parameters for rays, refer to the slides for details
	//!!! USEFUL NOTES: tan() takes rad rather than degree, use deg2rad() to transform
	//!!! USEFUL NOTES: view plane can be anywhere, but it will be implemented differently,
	//you can find references from the course slides 22_GlobalIllum.pdf

	const Camera &cam = scene.camera;

	double top =  tan(deg2rad(cam.fov / 2)) * cam.zNear;
	double left = top * cam.aspect;

	Vector w = Vector(cam.center - cam.position).normalized(); // cam.position - cam.center;
	Vector v = Vector(cam.up).normalized();
	Vector u = Vector(w).cross(v).normalized();
	Vector originPoint = (cam.zNear * Vector(w)) - (left* Vector(u)) - (top * Vector(v));

    // Iterate over all the pixels in the image.
	#pragma omp parallel
	#pragma omp for
    for(int y = 0; y < scene.resolution[1]; y++) {
		for (int x = 0; x < scene.resolution[0]; x++) {
			std::vector<Vector> colors;
			double minDepth = scene.camera.zFar;
			for (int a = 0; a < ANTIALIASRES; a++) {
				// Generate the appropriate ray for this pixel
				Ray ray;
				if (scene.objects.empty())
				{
					ray = Ray(scene.camera.position, (Vector(-320, -320, 640) + Vector(x + 0.5, y + 0.5, 0) - scene.camera.position).normalized());
				}
				else
				{
					double xrand = double(rand() % 100) / 100, yrand = double(rand() % 100) / 100;
					Vector pixelPos = Vector((x + xrand)*(2 * left / scene.resolution[0])*u + (y + yrand )*(2 * top / scene.resolution[1])*v) + originPoint;
					ray = Ray(cam.position, (pixelPos).normalized());
				}

				// Initialize recursive ray depth.
				int ray_depth = 0;
				// Our recursive raytrace will compute the color and the z-depth
				Vector color(0);

				// This should be the maximum depth, corresponding to the far plane.
				// NOTE: This assumes the ray direction is unit-length and the
				// ray origin is at the camera position.
				double depth = scene.camera.zFar;

				// Calculate the pixel value by shooting the ray into the scene
				trace(ray, ray_depth, scene, color, depth);
				if (depth < minDepth) minDepth = depth;
				colors.push_back(color);
			}
			Vector color(0);
			for (auto colorIt = colors.begin(); colorIt != colors.end(); colorIt++) {
				color += *colorIt * 1 / colors.size();
			}

            // Depth test
            if(minDepth >= scene.camera.zNear && minDepth <= scene.camera.zFar &&
				minDepth < zBuffer[x + y*scene.resolution[0]]) {
                zBuffer[x + y*scene.resolution[0]] = minDepth;

                // Set the image color (and depth)
                colorImage.setPixel(x, y, color);
                depthImage.setPixel(x, y, (minDepth -scene.camera.zNear) /
                                        (scene.camera.zFar-scene.camera.zNear));
            }
        }

		//output step information
		if (y % 100 == 0)
		{
			printf("Row %d pixels done.\n", y);
		}
    }

	//save image
    colorImage.writeBMP(filename);
    depthImage.writeBMP(depth_filename);

	printf("Ray tracing terminated and images are saved.\n");

    delete[] zBuffer;
}


bool Raytracer::trace(Ray const &ray, int &ray_depth, Scene const &scene, Vector &rayOutColor, double &depth)
{
    // Increment the ray depth.
	ray_depth++;

    // - iterate over all objects calling Object::intersect.
    // - don't accept intersections not closer than given depth.
    // - call Raytracer::shade with the closest intersection.
    // - return true iff the ray hits an object.
	if (scene.objects.empty())
	{
		// no objects in the scene, then we render the default scene:
		// For default, we assume there's a cube centered on (0, 0, 1280 + 160) with side length 320 facing right towards the camera
		// test intersection:
		double x = 1280 / ray.direction[2] * ray.direction[0] + ray.origin[0];
		double y = 1280 / ray.direction[2] * ray.direction[1] + ray.origin[1];
		if ((x <= 160) && (x >= -160) && (y <= 160) && (y >= -160))
		{
			//if intersected:
			Material m; m.emission = Vector(16.0, 0, 0); m.reflect = 0; //just for default material, you should use the intersected object's material
			Intersection intersection;	//just for default, you should pass the intersection found by calling Object::intersect()
			rayOutColor = shade(ray, ray_depth, intersection, m, scene);
			depth = 1280;	//the depth should be set inside each Object::intersect()
		}
	}
	else
	{
		//////////////////
		// YOUR CODE HERE
		// Note that for Object::intersect(), the parameter hit is the current hit
		// your intersect() should be implemented to exclude intersection far away than hit.depth
		Intersection intersection;
		intersection.depth = depth;
		Vector finalColor = NULL;
		double tmpDepth = intersection.depth;

		for (int i = 0; i < scene.objects.size(); i++) {
			const Object *obj = scene.objects[i];
			
			if (obj->intersect(ray, intersection)) {
				finalColor = shade(ray, ray_depth, intersection, obj->material, scene);
				tmpDepth = intersection.depth;
			}
		}
		if (finalColor != NULL) rayOutColor = finalColor;
		depth = tmpDepth;
		
	}

    // Decrement the ray depth.
	ray_depth--;

    return false; 
}


Vector Raytracer::shade(Ray const &ray, int &ray_depth, Intersection const &intersection, Material const &material, Scene const &scene)
{
    // - iterate over all lights, calculating ambient/diffuse/specular contribution
    // - use shadow rays to determine shadows
    // - integrate the contributions of each light
    // - include emission of the surface material
    // - call Raytracer::trace for reflection/refraction colors
    // Don't reflect/refract if maximum ray recursion depth has been reached!
	//!!! USEFUL NOTES: attenuate factor = 1.0 / (a0 + a1 * d + a2 * d * d)..., ambient light doesn't attenuate, nor does it affected by shadow
	//!!! USEFUL NOTES: don't accept shadow intersection far away than the light position
	//!!! USEFUL NOTES: for each kind of ray, i.e. shadow ray, reflected ray, and primary ray, the accepted furthest depth are different
	Vector diffuse(0);
	Vector ambient(0);
	Vector specular(0);
	Vector normNormalized = Vector(intersection.normal).normalized();
	Vector intesectionPosition = Vector(intersection.position);
	int objectInScene = scene.objects.size();

	for (auto lightIter = scene.lights.begin(); lightIter != scene.lights.end(); lightIter++)
	{
		Vector lightPos = Vector(lightIter->position);
		//////////////////
		// YOUR CODE HERE 
		// First you can assume all the light sources are directly visible. You should calculate the ambient, diffuse, 
		// and specular terms.You should think of this part in terms of determining the color at the point where the ray 
		// intersects the scene.
		// After you finished, you will be able to get the colored resulting image with local illumination, just like in programming assignment 3.
		
		// DIFFUSE
		Vector vecToLight = Vector(lightPos - intesectionPosition).normalized();
		double lightOn = std::fmax(vecToLight.dot(normNormalized), 0.0);
		Vector tmpDiff = Vector(material.diffuse) * lightIter->diffuse * lightOn;

		// SPECULAR

		Vector viewPositionNorm = Vector(scene.camera.position - intesectionPosition).normalized();
		Vector h = Vector(viewPositionNorm + vecToLight).normalized();
		double dotFactor = std::fmax(h.dot(normNormalized), 0.0);
		double powDotFactor = std::pow(dotFactor, material.shininess);
		Vector tmpSpec = Vector(material.specular) * lightIter->specular * powDotFactor;
		
		Vector shadeRayDir = intersection.position - ray.direction*EPSILON;
		Vector lightDir = lightPos - intesectionPosition;
		double shade_factor = 1, shade_acc = 0;
		double distToLight = (lightPos - intesectionPosition).length();

		for (int x = 0; x < SOFTLIGHTRES; x++) for (int y = 0; y < SOFTLIGHTRES; y++)
		{

			Ray shadeRay = Ray(shadeRayDir, (Vector(x , 0, y) + lightDir).normalized() );
			Intersection shadInter;

			shadInter.depth = distToLight;

			for (int i = 0; i < objectInScene; i++) {
				const Object *obj = scene.objects[i];

				if (obj->intersect(shadeRay, shadInter)) {
					shade_acc += 1.0f;
				}
			}
			
		}
		shade_factor -= shade_acc / (SOFTLIGHTRES * SOFTLIGHTRES);
		tmpDiff = tmpDiff * shade_factor; 
		tmpSpec = tmpSpec * shade_factor; 

		double att = lightIter->attenuation[0] + lightIter->attenuation[1] * (distToLight) + lightIter->attenuation[2] * (distToLight) * (distToLight);

		diffuse += tmpDiff * 1 / att;
		specular += tmpSpec * 1 / att;
		ambient += Vector(material.ambient) * lightIter->ambient;
	}

	Vector reflectedLight(0);
	if ((!(ABS_FLOAT(material.reflect) < 1e-6)) && (ray_depth < MAX_RAY_RECURSION))
	{
		//////////////////
		// YOUR CODE HERE 
		// calculate reflected color using trace() recursively

		Vector color;
		Vector reflectedDirection = (-2 * Vector(ray.direction).dot(normNormalized)*normNormalized) + Vector(ray.direction);

		Ray recursiveRay = Ray(intersection.position - ray.direction * EPSILON, reflectedDirection);
		double depth = 1e10;
		trace(recursiveRay, ray_depth, scene, color, depth);
		reflectedLight = color;
		
	}
	// !!!! edited line starts
	return material.emission + ambient + diffuse + specular + material.reflect * reflectedLight;  
	// !!!! edited line ends
}