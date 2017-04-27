#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include "algebra3.h"
#include "imageIO.h"
#include "raytrace.h"

#define PI 3.1415926535898
#define Tan(th) tan(PI/180*(th))

using namespace std;

vec3 eye, dir, vert= vec3(0, 1, 0), hori;
float Fangle;
int Rw, Rh;

float scale;

cSphere Si;
cTriangle Ti;
cLight Li;
bool SorT=true;

vector<cSphere> vSphere;
vector<cTriangle> vTriangle;
vector<cLight> vLight;

void readFile() {
    ifstream ifile("hw1_input.txt");
    string test;
    while( ifile >> test ) {
        switch (test[0]) {
            case 'E':
                ifile >> eye[0] >> eye[1] >> eye[2];
                break;
            case 'V':
                ifile >> dir[0] >> dir[1] >> dir[2];
                break;
            case 'F':
                ifile >> Fangle;
                break;
            case 'R':
                ifile >> Rw >> Rh;
                break;
            case 'S':
                ifile >> Si.Sphere[0] >> Si.Sphere[1] >> Si.Sphere[2] >> Si.Sphere[3];
                //vSphere.push_back(Si);
                SorT = true;
                break;
            case 'T':
                ifile >> Ti.T1[0] >> Ti.T1[1] >> Ti.T1[2] >> Ti.T2[0] >> Ti.T2[1] >> Ti.T2[2] >> Ti.T3[0] >> Ti.T3[1] >> Ti.T3[2];
                //vTriangle.push_back(Ti);
                SorT = false;
                break;
            case 'L':
                ifile >> Li.Light[0] >> Li.Light[1] >> Li.Light[2];
                vLight.push_back(Li);
                break;
            case 'M':
                if(SorT){
                    ifile >> Si.r >> Si.g >> Si.b >> Si.Ka >> Si.Kd >> Si.Ks >> Si.exp >> Si.rl >> Si.rr >> Si.Nr;
                    vSphere.push_back(Si);
                }
                else{
                    ifile >> Ti.r >> Ti.g >> Ti.b >> Ti.Ka >> Ti.Kd >> Ti.Ks >> Ti.exp >> Ti.rl >> Ti.rr >> Ti.Nr;
                    vTriangle.push_back(Ti);
                }
                break;
        }
    }
}



bool IntersectedTriangle(vec3 &ray, vec3 &a, vec3 &b, vec3 &c){
    // https://github.com/substack/ray-triangle-intersection
    float E = 0.0001;
    vec3 s1 = b - a;
    vec3 s2 = c - a;
    vec3 tvec;

    vec3 pvec = ray ^ s2;
    float det = s1 * pvec;

    if(det > 0 ){
        tvec = eye - a;
    }else{
        tvec = a - eye;
        det *= -1;
    }


    if( det < E ){ return false; }

    float u = tvec * pvec;
    if( u < 0 || u > det ){ return false; }
    vec3 qvec = tvec ^ s1;
    float v = ray * qvec;
    if( v < 0 || u + v > det ){ return false; }

    float t = ( s2 * qvec ) / det;
    vec3 out = vec3(eye[0] + t * ray[0], eye[1] + t * ray[1], eye[2] + t * ray[2]);

    return true;



}

int main()
{
    readFile();
    scale = Rw / Rh;
    

    for (unsigned int t = 0; t < vTriangle.size(); ++t) {
        Ti = vTriangle[t];
        cout << Ti.r << Ti.g << Ti.b << Ti.Ka << Ti.Kd << Ti.Ks << Ti.exp << Ti.rl << Ti.rr << Ti.Nr << '\n';

    }
            /// test if it intersected with all spheres.
    for (unsigned int s = 0; s < vSphere.size(); ++s) {
        Si = vSphere[s];
        cout << Si.r << Si.g << Si.b << Si.Ka << Si.Kd << Si.Ks << Si.exp << Si.rl << Si.rr << Si.Nr << '\n';
        
    }
    /*
    for (int j = 0; j < Rh; j++) {
        for (int i = 0; i < Rw; i++) {
            /// generate the ray.
            vec3 ray = dir + i * hori - j * vert;
            ray = ray.normalize();
            /// test if it intersected with all triangles.
            for (unsigned int t = 0; t < vTriangle.size(); t++) {
                

            }
            /// test if it intersected with all spheres.
            for (unsigned int s = 0; s < Sphere.size(); ++s) {
                
            }
        }
    }*/
        return 0;
}
