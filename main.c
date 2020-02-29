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
    Vec3 center;
    double r;
    Vec3 color;
} Sphere;

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

double clamp_rand(double min, double max)
{
    double range = max -min;
    double step = RAND_MAX/range;
    return min+rand()/step;
}

int main(int argc, char *argv[])
{
    srand(time(NULL));
    int w = 400;
    int h = 400;

    int center_image_x = w/2;
    int center_image_y = h/2;

    char * image = malloc(sizeof(char)*(w*h*3));

    Sphere s = {{0.0,1.0,4.0},1.0, {220,150,150}};
    Sphere s2 = {{0.0,-3.0,7.0},3.0, {50,190,170}};
    Vec3 light = {7.0,5.0,-1.0};

    Vec3 origin = {0.0, 0.0, 0.0};

    //const int NB_OBJECTS = 3;
    //Sphere objects[3] = {s,s2,{{1.0,0.3,2.5},1.0,{380,200,400}}};

    const int NB_OBJECTS = 30;
    Sphere *objects = malloc(NB_OBJECTS*sizeof(Sphere));
    for(int i = 0; i < NB_OBJECTS; i++)
    {
        Sphere t = {    {clamp_rand(-5,5),clamp_rand(-5,5),clamp_rand(5,30.0)},
                        clamp_rand(0.2,2.0),
                        {clamp_rand(0,255),clamp_rand(0,255),clamp_rand(0,255)}};
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
            Vec3 finalColor = {150,120,220};
            double nearestObject = 1000000000.0;
            for(int i = 0; i < NB_OBJECTS; i++)
            {
                if((sphere_intersect(objects[i],origin, view_vec,&inter) == 1) && (inter.z <= nearestObject))
                {
                    nearestObject = inter.z;
                    Vec3 light_dir = vec_normalize(vec_substract(light,inter));
                    //Vec3 color;
                    int shadowed = 0;
                    for(int obstacle = 0; obstacle < NB_OBJECTS; obstacle++)
                    {
                        if(sphere_intersect(objects[obstacle],inter,light_dir,NULL) == 1)
                        {
                            finalColor = vec_mul(objects[i].color,0.4);
                            shadowed = 1;
                            break;
                        }
                    }
                    if(!shadowed)
                    {
                        Vec3 normale = vec_normalize(vec_substract(inter,objects[i].center));
                        double luminosity = vec_dot(light_dir,normale);
                        luminosity *= luminosity; //squared
                        //luminosity = 1.0 - luminosity;
                        //luminosity = max(luminosity,0.8);
                        luminosity = luminosity*0.6+0.4;
                        //luminosity*=1.35;
                        finalColor = vec_mul(objects[i].color,luminosity);
                    }

                }
            }

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
