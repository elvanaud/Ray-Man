#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

typedef struct
{
    double x,y,z;
} Vec3;

typedef struct
{
    Vec3 color;
    int specLevel;
    double specularStrength;
} Material;

typedef struct
{
    Vec3 center;
    double r;
    Material mat;
} Sphere;

typedef struct
{
    Vec3 pos;
    Vec3 color;
    double ambientStrength;
    double diffuseStrength;
} Light;

double max(double a, double b)
{
    return a < b ? b : a;
}

double min(double a, double b)
{
    return a >= b ? b : a;
}

double vec_dot(Vec3 a, Vec3 b)
{
    return a.x*b.x+a.y*b.y+a.z*b.z;
}

Vec3 vec_mul(Vec3 v, double a)
{
    Vec3 res = {v.x*a,v.y*a,v.z*a};
    return res;
}

Vec3 vec_substract(Vec3 a, Vec3 b)
{
    Vec3 res = {a.x-b.x, a.y-b.y, a.z-b.z};
    return res;
}

Vec3 vec_add(Vec3 a, Vec3 b)
{
    Vec3 res = {a.x+b.x, a.y+b.y, a.z+b.z};
    return res;
}

Vec3 vec_prod(Vec3 a, Vec3 b)
{
    Vec3 res = {a.x*b.x, a.y*b.y, a.z*b.z};
    return res;
}

Vec3 vec_normalize(Vec3 v)
{
    double length = sqrt(vec_dot(v,v));
    return vec_mul(v,1.0/length);
}

double degrees_to_radians(double a)
{
    return a/180.0 * M_PI; //180° = pi rad
}

int sphere_intersect(Sphere s, Vec3 origin, Vec3 d,Vec3*inter)
{
    Vec3 oc = vec_substract(s.center,origin);
    double tc = vec_dot(oc,d);
    if(tc < 0.0)
        return 0;
    double ta2 = s.r*s.r-vec_dot(oc,oc)+tc*tc;
    if(ta2 < 0.0)
        return 0; //No collision
    double ta = sqrt(ta2);
    Vec3 p1 = vec_mul(d,(tc-ta)); //Intersection points
    Vec3 p2 = vec_mul(d,(tc+ta));
    if(inter != NULL)
        *inter=p1;
    return 1;
}

Vec3 reflect(Vec3 n, Vec3 d)
{
    /*Vec3 dst = vec_substract(v,n);
    dst = vec_mul(dst,-1.0);
    Vec3 res = vec_normalize(vec_add(n,dst));
    return res;*/
    return vec_substract(d,vec_mul(n,2*vec_dot(n,d)));
}

double clamp_rand(double min, double max)
{
    double range = max -min;
    double step = RAND_MAX/range;
    return min+rand()/step;
}

int main(int argc, char *argv[])
{
    srand(time(NULL));
    //int w = 2000;
    //int h = 1500;

    int w = 1280;
    int h = 900;

    int center_image_x = w/2;
    int center_image_y = h/2;

    char * image = malloc(sizeof(char)*(w*h*3));

    /*Light light = {{7.0,5.0,-1.0},{0.0,1.0,1.0}};
    light.ambientStrength = 0.1;

    Light light2 = {{-5.0,-3.0,-1.0},{1.0,0.0,0.0}};
    light2.ambientStrength = 0.2;*/
    //Vec3 ambient = vec_mul(light.color,light.ambientStrength); //?
    //double specularStrength = 1.0;
    const int NB_LIGHTS = 4;
    Light *lights = malloc(NB_LIGHTS*sizeof(Light));
    for(int i = 0; i < NB_LIGHTS; i++)
    {
        Light l = {{clamp_rand(-100,100), clamp_rand(-100,100),clamp_rand(-100,100)},{clamp_rand(0,1.0),clamp_rand(0,1.0),clamp_rand(0,1.0)}};
        l.ambientStrength = clamp_rand(0.0, 0.5);
        lights[i] = l;
    }

    Vec3 origin = {0.0, 0.0, 0.0};

    const int NB_OBJECTS = 30;
    Sphere *objects = malloc(NB_OBJECTS*sizeof(Sphere));
    for(int i = 0; i < NB_OBJECTS; i++)
    {
        Sphere t = {    {clamp_rand(-5,5),clamp_rand(-5,5),clamp_rand(5,30.0)},
                        clamp_rand(0.2,2.0),
                        {{clamp_rand(0,1.0),clamp_rand(0,1.0),clamp_rand(0,1.0)},rand()%10<5?32:256,clamp_rand(0.1,2.0)}};
        objects[i] = t;
    }

    for(int y = 0; y < h; y++)
    {
        for(int x = 0; x < w; x++)
        {
            int index = (y*w+x)*3;

            double fov = 80.0;
            double fov_rad = degrees_to_radians(fov/2.0);
            double step = tan(fov_rad)*2.0/w;
            Vec3 view_vec = {(x-center_image_x)*step,-(y-center_image_y)*step*h/w,1};
            view_vec = vec_normalize(view_vec);
            Vec3 inter;
            Vec3 finalColor = {230,210,250};
            finalColor = vec_mul(finalColor,1.0/255.0);
            double nearestObject = 1000000000.0;
            for(int i = 0; i < NB_OBJECTS; i++)
            {
                if((sphere_intersect(objects[i],origin, view_vec,&inter) == 1) && (inter.z <= nearestObject))
                {
                    nearestObject = inter.z;
                    finalColor = origin;
                    for(int l = 0; l < NB_LIGHTS; l++)
                    {
                        Vec3 light_dir = vec_normalize(vec_substract(lights[l].pos,inter));

                        int shadowed = 0;
                        for(int obstacle = 0; obstacle < NB_OBJECTS; obstacle++)
                        {
                            if(sphere_intersect(objects[obstacle],inter,light_dir,NULL) == 1)
                            {
                                shadowed = 1;
                                break;
                            }
                        }

                        Vec3 ambient = vec_mul(lights[l].color,lights[l].ambientStrength);

                        Vec3 normale = vec_normalize(vec_substract(inter,objects[i].center));
                        double luminosity = vec_dot(light_dir,normale);

                        luminosity = max(luminosity,0.0);
                        Vec3 diffuse = vec_mul(lights[l].color,luminosity);

                        Vec3 reflectedLight = reflect(normale,vec_mul(light_dir,-1.0));
                        double spec = pow(max(vec_dot(reflectedLight,vec_mul(view_vec,-1.0)),0.0),objects[i].mat.specLevel);
                        Vec3 specular = vec_mul(lights[l].color,spec*objects[i].mat.specularStrength);

                        finalColor = vec_add(finalColor, vec_prod(vec_add(vec_mul(vec_add(diffuse,specular),1.0-shadowed),ambient),objects[i].mat.color));
                        //finalColor = vec_prod(specular,objects[i].mat.color);
                    }
                }
            }

            finalColor = vec_mul(finalColor,255);

            image[index+0] = max(0,min(finalColor.x,255));
            image[index+1] = max(0,min(finalColor.y,255));
            image[index+2] = max(0,min(finalColor.z,255));
        }
    }

    int result = stbi_write_png("output.png", w, h, 3, image, 0);
    if(result == 0)
    {
        printf("Error writing output file\n");
    }


    printf("RayMan\n");
    return 0;
}
