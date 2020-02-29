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

int main(int argc, char *argv[])
{
    int w = 400;
    int h = 400;

    int center_image_x = w/2;
    int center_image_y = h/2;

    char * image = malloc(sizeof(char)*(w*h*3));

    Sphere s = {{0.0,0,3.5},1.0, {220,150,150}};
    Sphere s2 = {{0.0,-3.0,5.0},3.0, {50,190,170}};
    Vec3 light = {-10.0,5.0,0.0};

    Vec3 origin = {0.0, 0.0, 0.0};

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

            if(sphere_intersect(s,origin, view_vec,&inter) == 1)
            {
                Vec3 light_dir = vec_normalize(vec_substract(light,inter));
                Vec3 color;
                if(sphere_intersect(s,inter,light_dir,NULL) == 1 || sphere_intersect(s2,inter,light_dir,NULL) == 1) //light blocked => shadow
                {
                    color = vec_mul(s.color,0.6);
                    //color = vec_clamp(color,0,255);
                }
                else
                {
                    Vec3 normale = vec_normalize(vec_substract(inter,s.center));
                    double luminosity = vec_dot(light_dir,normale);
                    luminosity *= luminosity; //squared
                    //luminosity = 1.0 - luminosity;
                    //luminosity = max(luminosity,0.8);
                    luminosity = luminosity*0.4+0.6;
                    //luminosity*=1.35;
                    color = vec_mul(s.color,luminosity);
                }

                image[index+0] = max(0,min(color.x,255));
                image[index+1] = max(0,min(color.y,255));
                image[index+2] = max(0,min(color.z,255));
            }
            else if(sphere_intersect(s2,origin,view_vec,&inter) == 1)
            {
                Vec3 light_dir = vec_normalize(vec_substract(light,inter));
                Vec3 color;
                if(sphere_intersect(s,inter,light_dir,NULL) == 1 || sphere_intersect(s2,inter,light_dir,NULL) == 1) //light blocked => shadow
                {
                    color = vec_mul(s2.color,0.6);
                    //color = vec_clamp(color,0,255);
                }
                else
                {
                    Vec3 normale = vec_normalize(vec_substract(inter,s2.center));
                    double luminosity = vec_dot(light_dir,normale);
                    luminosity *= luminosity; //squared
                    //luminosity = 1.0 - luminosity;
                    luminosity = luminosity*0.4+0.6;
                    //luminosity*=1.35;
                    color = vec_mul(s2.color,luminosity);
                }

                image[index+0] = max(0,min(color.x,255));
                image[index+1] = max(0,min(color.y,255));
                image[index+2] = max(0,min(color.z,255));
            }
            else
            {
                image[index+0] = 150;
                image[index+1] = 120;
                image[index+2] = 220;
            }

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
