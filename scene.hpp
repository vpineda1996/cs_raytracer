#ifndef SCENE_H
#define SCENE_H

#include "object.hpp"
#include <string>
#include <vector>
#include <iostream>
#include <cmath>
#include <cfloat>
#include <fstream>
#include <sstream>
#include <map>


// A class to encapsulate all of the parameters of a light.
class PointLight 
{
public:	// Constructors
	PointLight();
	PointLight(Vector const &position, ParamList &params) : position(position) { init(params); }

	// Initialize the light's attributes from the given parameter list.
	void init(ParamList &params)
	{
		#define SET_VECTOR(_name) _name = params[#_name];	   
// !!!!  edited lines start
		SET_VECTOR(ambient)
		SET_VECTOR(diffuse)
		SET_VECTOR(specular)
// !!!! edited lines end
		SET_VECTOR(attenuation)
	}

	Vector position; // Light position

	// Ambient, diffuse, and specular light colors.	
// !!!! edited lines start
	Vector ambient;
	Vector diffuse;
	Vector specular;	  
// !!!! edited lines end

	// Attenuation coefficients type.
	// attenuation[0] = CONSTANT
	// attenuation[1] = LINEAR
	// attenuation[2] = QUADRATIC
	Vector attenuation;
};

struct Camera
{
	Camera(void) : fov(45), aspect(1.0), zNear(1.0), zFar(10000.0),
		position(0.0, 0.0, 0.0), center(0.0, 0.0, 1.0), up(0.0, 1.0, 0.0) {}

	double fov;
	double aspect;
	double zNear;
	double zFar;
	Vector position;
	Vector center;
	Vector up;
};


// A class that stores all of the parameters, materials, and objects in a 
// scene that we want to render.
class Scene {
public:
	Scene(void)
	{
		resolution[0] = resolution[1] = 640;
	}
public:
    // Resolution of output image, in pixels (Width and height)
    int resolution[2]; 

    // The camera to use when rendering the scene.
    Camera camera;
    
    // Mapping of material names to the materials themselves.
    std::map<std::string, Material> materials;

    // A list of point lights.
    std::vector<PointLight> lights; 

    // List of pointers to Objects in the scene.
    // Note that Object is an abstract class, so these will actually be
    // Spheres, Planes, Meshes, etc.
    std::vector<Object*> objects;
};


#endif
