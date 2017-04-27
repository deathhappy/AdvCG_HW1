#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include "algebra3.h"

#define PI 3.1415926535898
#define Tan(th) tan(PI/180*(th))
#define Cos(th) cos(PI/180*(th))

class object{
public:
    float r,g,b,Ka,Kd,Ks,exp,rl,rr,Nr;
    
};

class cSphere : public object{
public:
    vec4 Sphere;
};

class cTriangle : public object{
public:
    vec3 T1,T2,T3;
};

class cLight{
public:
	vec3 Light;
};

class ray{
public:
	vec3 start;
	vec3 dir;
	setray(vec3 instart, vec3 indir){
		start = instart;
		dir = indir;
	}
};

class RayData{
public:
	bool isIntersected = false;
	bool IntersectedTriangle = false;
	bool IntersectedSph = false;

	vec3  hitpoint;
    vec3  startToHitpoint;
    vec3  hitpointToLight;
    vec3  normal;
    vec3  half;

    Pixel color = {255, 255, 255};
    float distance;

};

class Raytrace{
public:
	RayTrace(vec3 eye,
              vec3 dir,
              vec3 vert,
              float angle,
              int RW, int RH,
              std::vector<cTriangle> triangle,
              std::vector<cSphere> sphere,
              std::vector<cLight> light) {

    eye_ = eye;
    dir_ = dir;
    vert_ = vert;
    angle_ = angle;
    RW_ = RW;
    RH_ = RH;
    scale = RW_ / RH_;

    hori_ = vert_ ^ -dir_;

    dir_  =  dir_.normalize();
    hori_ = hori_.normalize();
    vert_ = vert_.normalize();
    

    horiU_ = hori_ * ( dir_.length() * Tan(angle_/2) / (RW_/2) ) * scale;
    vertU_ = vert_ * ( dir_.length() * Tan(angle_/2) / (RH_/2) );
    dirU_  =  dir_ - RW_/2 * horiU_ + RH_/2 * vertU_;

    triangle_ = triangle;
    sphere_ = sphere;
    light_ = light;
}
    

private:
	vec3 eye_, dir_, hori_, vert_;
	vec3 horiU_, vertU_, dirU_;
	int RH_, RW_;
	float angle_, scale;

	std::vector<cTriangle> triangle_;
    std::vector<cSphere>   sphere_;
    std::vector<cLight>    light_;

};
/*
RayData IntersectedSph(vec3 &ray, vec4 &sph) {

    ///  AT^2 + BT + C = 0
    float A = ray[0]*ray[0]+ray[1]*ray[1]+ray[2]*ray[2];
    float B = 2*(ray[0]*(eye[0]-sph[0])+ray[1]*(eye[1]-sph[1])+ray[2]*(eye[2]-sph[2]));
    float C = (eye[0]-sph[0])*(eye[0]-sph[0])+(eye[1]-sph[1])*(eye[1]-sph[1])+(eye[2]-sph[2])*(eye[2]-sph[2])-sph[3]*sph[3];

    if ( ( B*B - 4*A*C ) >= 0 ) { return true; }

    return false;
}

*/