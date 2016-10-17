/**
 * ray_tracer.cpp
 * CS230
 * -------------------------------
 * Implement ray tracer here.
 */

#define SET_RED(P, C)   (P = (((P) & 0x00ffffff) | ((C) << 24)))
#define SET_GREEN(P, C)  (P = (((P) & 0xff00ffff) | ((C) << 16)))
#define SET_BLUE(P, C) (P = (((P) & 0xffff00ff) | ((C) << 8)))

#include "ray_tracer.h"

using namespace std;

const double Object::small_t = 1e-6;
//--------------------------------------------------------------------------------
// utility functions
//--------------------------------------------------------------------------------
double sqr(const double x)
{
	return x*x;
}

Pixel Pixel_Color(const Vector_3D<double>& color)
{
	Pixel pixel = 0;
	SET_RED(pixel, (unsigned char)(min(color.x, 1.0) * 255));
	SET_GREEN(pixel, (unsigned char)(min(color.y, 1.0) * 255));
	SET_BLUE(pixel, (unsigned char)(min(color.z, 1.0) * 255));
	return pixel;
}
//--------------------------------------------------------------------------------
// Shader
//--------------------------------------------------------------------------------
Vector_3D<double> Phong_Shader::
Shade_Surface(const Ray& ray, const Object& intersection_object, const Vector_3D<double>& intersection_point, const Vector_3D<double>& same_side_normal) const
{
	Vector_3D<double> diffuseAndSpecular;
	Vector_3D<double> color;
	for (long index = 0; index < (long)world.lights.size(); ++index) {
                Light *light = NULL;
                light = world.lights.at(index);
                Ray lightRay = Ray();
		Vector_3D<double> lightDir = intersection_point - light->position;
		lightDir = lightDir* (1/lightDir.Length());
                lightRay.direction = (lightDir.Normalized())*-1.0;
                bool isShadowed = false;
		if (world.enable_shadows) {
		  lightRay.endpoint = intersection_point;
			const Object* obj = world.Closest_Intersection(lightRay);
			if (!lightRay.semi_infinite) {
			  isShadowed = true;
			}
		}
		if (!isShadowed) {
			Vector_3D<double> l = intersection_point - light->position;
			l=l*-1.0;
			l.Normalize();
			double lDotN = Vector_3D<double>::Dot_Product(l, same_side_normal);
		        Vector_3D<double> diffuse = color_diffuse * lDotN;
			Vector_3D<double> v = (intersection_point - world.camera.position);
			v.Normalize();
			Vector_3D<double> r = (same_side_normal * 2 * lDotN) - l;
			r.Normalize();
			Vector_3D<double> specular = color_specular*powf(Vector_3D<double>::Dot_Product(r,v), specular_power) ;
			diffuseAndSpecular = diffuseAndSpecular + (diffuse + specular) *light->Emitted_Light(lightRay);
		}
		}	
	color = color_ambient*0.5 +  diffuseAndSpecular;
	return color;
}

Vector_3D<double> Reflective_Shader::
Shade_Surface(const Ray& ray, const Object& intersection_object, const Vector_3D<double>& intersection_point, const Vector_3D<double>& same_side_normal) const
{
	Vector_3D<double> color;
	Vector_3D<double> reflectColor;
	color = Phong_Shader::Shade_Surface(ray,intersection_object,intersection_point,same_side_normal);
	if(ray.recursion_depth<world.recursion_depth_limit){
        Ray reflectRay = Ray();
	reflectRay.endpoint = intersection_point;
	reflectRay.direction = ray.direction - same_side_normal * 2 * Vector_3D<double>::Dot_Product(ray.direction, same_side_normal);
	reflectRay.direction.Normalize();
	reflectRay.recursion_depth = ray.recursion_depth+1;
	reflectColor = world.Cast_Ray(reflectRay,ray);
	}
	return color + reflectColor*reflectivity;
}

Vector_3D<double> Flat_Shader::
Shade_Surface(const Ray& ray, const Object& intersection_object, const Vector_3D<double>& intersection_point, const Vector_3D<double>& same_side_normal) const
{
	return color;
}

//--------------------------------------------------------------------------------
// Objects
//--------------------------------------------------------------------------------
// determine if the ray intersects with the sphere
// if there is an intersection, set t_max, current_object, and semi_infinite as appropriate and return true
bool Sphere::
Intersection(Ray& ray) const
{
	Sphere sphere(*this);
	double endptmincenter = Vector_3D<double>(ray.endpoint - sphere.center).Length_Squared();
	double b = ray.direction.Dot_Product(ray.direction, ray.endpoint - sphere.center);
	double descriminant = sqr(b) - endptmincenter + sqr(sphere.radius);
	if (descriminant < 0) {
		return false;
	}
	double d = sqrt(descriminant);
	double posRoot = -b + d;
	double negRoot = -b - d;
	double minDistance = 0;
	if(posRoot <= negRoot){
	  minDistance = posRoot;	
        }
        else{
	  minDistance = negRoot;	
        }
	if (minDistance > small_t) {
		if (ray.t_max == 0) {
			ray.t_max = minDistance;
			ray.semi_infinite = false;
			ray.current_object = &sphere;
			return true;
		}
		else if (minDistance < ray.t_max) {
			ray.t_max = minDistance;
			ray.semi_infinite = false;
			ray.current_object = &sphere;
			return true;
		}
	}
	return false;
}

Vector_3D<double> Sphere::
Normal(const Vector_3D<double>& location) const
{
	Vector_3D<double> normal;
	normal = location - center;
	normal.Normalize();
	return normal;
}

// determine if the ray intersects with the sphere
// if there is an intersection, set t_max, current_object, and semi_infinite as appropriate and return true
bool Plane::
Intersection(Ray& ray) const
{
	Plane plane(*this);
	double denominator = ray.direction.Dot_Product(ray.direction, normal);
	if (denominator != 0) {
	  Vector_3D<double> vec = plane.x1 - ray.endpoint;
		double numer = Vector_3D<double>::Dot_Product(vec, plane.normal);
		double distance = numer / denominator;
		if (distance > small_t) {
			if (ray.t_max == 0) {
				ray.t_max = distance;
				ray.semi_infinite = false;
				ray.current_object = &plane;
				return true;
			}
			else if (distance < ray.t_max) {
				ray.t_max = distance;
				ray.semi_infinite = false;
				ray.current_object = &plane;
				return true;
			}
		}
	}
	return false;
}

Vector_3D<double> Plane::
Normal(const Vector_3D<double>& location) const
{
	return normal;
}
//--------------------------------------------------------------------------------
// Camera
//--------------------------------------------------------------------------------
// Find the world position of the input pixel
Vector_3D<double> Camera::
World_Position(const Vector_2D<int>& pixel_index)
{
	double width = film.pixel_grid.m;
	double height = film.pixel_grid.n;
	double aspectRatio = (double)width / (double)height;

	double actualX = ((pixel_index.x + 0.5) / width)* aspectRatio - (((width - height) / (double)height) / 2);
	double actualY = ((height - pixel_index.y) + 0.5) / height;

	Vector_3D<double> result = look_vector + (horizontal_vector*(actualX - 0.5) - vertical_vector*(actualY - 0.5));
	result.Normalize();
	return result;
}
//--------------------------------------------------------------------------------
// Render_World
//--------------------------------------------------------------------------------
// Find the closest object of intersection and return a pointer to it
//   if the ray intersects with an object, then ray.t_max, ray.current_object, and ray.semi_infinite will be set appropriately
//   if there is no intersection do not modify the ray and return 0
const Object* Render_World::
Closest_Intersection(Ray& ray)
{
	double minDistance = 0;
	Object *closestObject = NULL;
	for (long index = 0; index < (long)objects.size(); ++index) {
		Object *object = NULL;
		object = objects.at(index);
		if (object->Intersection(ray)) {
			double distance = ray.t_max;
			if (minDistance == 0 && distance > 0) {
				minDistance = distance;
				closestObject = object;
			} else 
			if (distance < minDistance) {
				minDistance = distance;
				closestObject = object;
			}
		}
	}
	if (minDistance == 0) {
		return 0;
	}
	else {
	        ray.t_max = minDistance;
		ray.semi_infinite = false;
		ray.current_object = closestObject;
		return closestObject;
	}
}

// set up the initial view ray and call 
void Render_World::
Render_Pixel(const Vector_2D<int>& pixel_index)
{
	Ray ray;
	ray.endpoint = camera.position;
	ray.direction = camera.World_Position(pixel_index);
	Ray dummy_root;
	Vector_3D<double> color = Cast_Ray(ray, dummy_root);
	camera.film.Set_Pixel(pixel_index, Pixel_Color(color));
}

// cast ray and return the color of the closest intersected surface point, 
// or the background color if there is no object intersection
Vector_3D<double> Render_World::
Cast_Ray(Ray& ray, const Ray& parent_ray)
{
	Vector_3D<double> color;
	const Object* obj = Closest_Intersection(ray);
	if (obj != 0) {
		double distance = ray.t_max;
		Vector_3D<double> normal = obj->Normal(ray.Point(distance));
		normal.Normalize();
		color = obj->material_shader->Shade_Surface(ray, *obj, ray.Point(distance),normal );
	}
	return color;
}
