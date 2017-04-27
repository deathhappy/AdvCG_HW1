#ifndef _RAYTRACE_H
#define _RAYTRACE_H

#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include "algebra3.h"

#define PI 3.1415926535898
#define Tan(th) tan(PI/180*(th))
#define Cos(th) cos(PI/180*(th))

class Material{
public:
    float Ka,Kd,Ks,exp,rl,rr,Nr = 1;
    vec3 color;
};

class cSphere{
public:
    vec3 Sphere;
    float radius;
    Material m;
};

class cTriangle{
public:
    vec3 T1,T2,T3;
    Material m;
};

class cLight{
public:
    vec3 light;
    vec3 color = vec3(1, 1, 1);
};

class Ray{
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

    vec3  hitpoint;
    vec3  startToHitpoint;
    vec3  hitpointToLight;
    vec3  normal;
    vec3  half;

    char type;
    int index;

    float t = -1;
    Material m;

};

class Raytrace{
public:
    Raytrace(vec3 eye,
              vec3 dir,
              vec3 vert,
              float angle,
              int RW, int RH,
              std::vector<cTriangle> triangle,
              std::vector<cSphere> sphere,
              std::vector<cLight> light,
              int depth) {

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
    this->depth = depth;
}
    
    RayData IntersectedSph(Ray&, cSphere&, int);
    RayData IntersectedTriangle(Ray&, cTriangle&, int);
    RayData Intersected(Ray&);
    RayData PhongShading(RayData&);
    RayData Shadow(RayData&);
    vec3 Reflection(RayData&, vec3&, int);
    vec3 Refraction(RayData&, vec3&, float,int);
    vec3 Raytracing(int, int);

private:
    vec3 eye_, dir_, hori_, vert_;
    vec3 horiU_, vertU_, dirU_;
    int RH_, RW_, depth;
    float angle_,scale;
    vec3 background = vec3(0.1, 0.1, 0.1);

    std::vector<cTriangle> triangle_;
    std::vector<cSphere>   sphere_;
    std::vector<cLight>    light_;

};

RayData Raytrace::IntersectedSph(Ray &ray, cSphere &sph, int i) {

    RayData data;
    ///  AT^2 + BT + C = 0
    float A = ray.dir[0] * ray.dir[0] + ray.dir[1] * ray.dir[1] + ray.dir[2] * ray.dir[2];
    float B = 2 * (ray.dir[0] * (ray.start[0] - sph.Sphere[0] ) + ray.dir[1] * (ray.start[1] - sph.Sphere[1]) + ray.dir[2] * (ray.start[2] - sph.Sphere[2]));
    float C = (ray.start[0] - sph.Sphere[0]) * (ray.start[0] - sph.Sphere[0]) + (ray.start[1] - sph.Sphere[1]) * (ray.start[1] - sph.Sphere[1]) + (ray.start[2] - sph.Sphere[2]) * (ray.start[2] - sph.Sphere[2]) - sph.radius * sph.radius;

    if ((B * B - 4 * A * C) >= 0) { 
        float t = ( -B - sqrt(B * B - 4 * A * C)) / 2;

        data.isIntersected = true;
        data.hitpoint = ray.start + t * ray.dir;
        data.t = t;
        data.normal = (data.hitpoint - sph.Sphere).normalize();
        data.m = sph.m;
        data.index = i;
        data.type = 's';

        return data;
    }

    return data;
}

RayData Raytrace::IntersectedTriangle(Ray &ray, cTriangle &tri, int i){
    // https://github.com/substack/ray-triangle-intersection
    RayData data;
    vec3 a = tri.T1, b = tri.T2, c = tri.T3;

    float E = 0.0001;
    vec3 s1 = b - a;
    vec3 s2 = c - a;
    vec3 tvec, normal;

    vec3 pvec = ray.dir ^ s2;
    float det = s1 * pvec;

    if(det > 0 ){
        tvec = ray.start - a;
        normal = s1 ^ s2;
    }else{
        tvec = a - ray.start;
        det *= -1;
        normal = s2 ^ s1;
    }


    if( det < E ){ return data; }

    float u = tvec * pvec;
    if( u < 0 || u > det ){ return data; }
    vec3 qvec = tvec ^ s1;
    float v = ray.dir * qvec;
    if( v < 0 || u + v > det ){ return data; }

    float t = ( s2 * qvec ) / det;

    data.isIntersected = true;
    data.hitpoint = ray.start + t * ray.dir;
    data.t = t;
    data.normal = normal.normalize();
    data.m = tri.m;
    data.index = i;
    data.type = 't';

    return data;
}

RayData Raytrace::Intersected(Ray &ray){
    RayData data;
    for(std::size_t i = 0; i < sphere_.size(); i++){
        RayData temp = IntersectedSph(ray ,sphere_[i], i);
        if(data.t < 0 && temp.t > 0)    data = temp;
        if(data.t > 0 && temp.t > 0 && data.t > temp.t){
            data = temp;
        }
    }

    for(std::size_t i = 0; i < triangle_.size(); i++){
        RayData temp = IntersectedTriangle(ray ,triangle_[i], i);
        if(data.t < 0 && temp.t > 0)    data = temp;
        if(data.t > 0 && temp.t > 0 && data.t > temp.t){
            data = temp;
        }
    }

    return data;

}

RayData Raytrace::PhongShading(RayData &raydata){
    Material material = raydata.m;
    float Ka = material.Ka, Kd = material.Kd, Ks = material.Ks;

    for(std::size_t i = 0; i < light_.size(); i++){
    // Ambient
    vec3 ambient = Ka * light_[i].color;
    ambient = prod(ambient, material.color);

    // Diffuse
    vec3 lightdir = (light_[i].light - raydata.hitpoint).normalize();
    vec3 diffuse = MAX(raydata.normal * lightdir, 0.0) * light_[0].color;
    diffuse = Kd * prod(diffuse, material.color);

    //Specular
    float exp = material.exp;
    vec3 viewdir = (eye_ - raydata.hitpoint).normalize();
    vec3 H = (lightdir + viewdir).normalize();
    vec3 specular = Ks * light_[i].color * pow(MAX(raydata.normal * H, 0.0), exp);

    raydata.m.color = ambient + diffuse + specular;
    }
    return raydata;
}

RayData Raytrace::Shadow(RayData &raydata){
    for(std::size_t i = 0; i < light_.size(); i++){
    vec3 lightdir = raydata.hitpoint - light_[i].light;
    Ray ray;
    ray.setray(light_[i].light, lightdir.normalize());
    RayData data = Intersected(ray);
    if(abs((raydata.hitpoint - light_[i].light).length() - (data.hitpoint - light_[i].light).length()) > 0.001)
        raydata.m.color *= 0.2;
    }
    return raydata;
}

vec3 Raytrace::Reflection(RayData &raydata, vec3 &raydir, int count){
    if(raydata.m.rl <= 0)   return background;
    vec3 reflectdir = (raydir - 2 * raydir * raydata.normal * raydata.normal).normalize();
    Ray reflectray;
    reflectray.setray(raydata.hitpoint + 0.01 * raydata.normal, reflectdir);
    RayData reflectdata = Intersected(reflectray);
    if(reflectdata.t > 0 && count > 0){
        reflectdata = PhongShading(reflectdata);
        reflectdata = Shadow(reflectdata);

        return raydata.m.rl * (reflectdata.m.color + Reflection(reflectdata, reflectdir, count - 1) + Refraction(reflectdata, reflectdir, reflectdata.m.Nr, count - 1));
    }
    
    return background;
}

vec3 Raytrace::Refraction(RayData &raydata, vec3 &raydir, float Nr, int count){
    if(raydata.m.rr <= 0)   return background;
    RayData in,out;

    float N = Nr / raydata.m.Nr;
    float cosI = -(raydata.normal * raydir);
    float sinT2 = pow(N, 2) * (1.0 - pow(cosI, 2));
    if(sinT2 > 1.0){
        vec3 reflectray = Reflection(raydata, raydir, count);
        return raydata.m.rl * reflectray;
    }        
    float cosT = sqrt(1.0 - sinT2);
    vec3 refractdir = (N * raydir + (N * cosI - cosT) * raydata.normal).normalize();
    Ray inrefractray;
    inrefractray.setray(raydata.hitpoint + 0.01 * refractdir, refractdir);
    in = Intersected(inrefractray);
    
    if(in.t > 0 && count > 0){
        in = PhongShading(in);
        in = Shadow(in);
        if(in.type == raydata.type && in.index == raydata.index){
            in.normal = -in.normal;
            in.m.Nr = 1;
        }
        return raydata.m.rr * (in.m.color + Reflection(in, refractdir, count - 1) + Refraction(in, refractdir, raydata.m.Nr, count - 1));
    }
    return background;
}


vec3 Raytrace::Raytracing(int x, int y){
    vec3 ray1 = dirU_ + x * horiU_ - y * vertU_;
    ray1 = ray1.normalize();
    Ray ray;
    ray.setray(eye_, ray1);

    RayData data = Intersected(ray);
    if(data.t > 0){
        data = PhongShading(data);
        data = Shadow(data);
        //int count = depth;
        //data.m.color += Reflection(data, ray1, count);     
        //data.m.color += Refraction(data, ray1, 1.0, count);

    }else{
        //background
        data.m.color = background;
    }

    return data.m.color;

}

#endif