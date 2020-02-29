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
} Sphere;

double vec_dot(Vec3 a, Vec3 b)
{
    return a.x*b.x+a.y*b.y+a.z*b.z;
}

Vec3 vec_mul(Vec3 v, double a)
{
    Vec3 res = {v.x*a,v.y*a,v.z*a};
    return res;
}

Vec3 vec_normalize(Vec3 v)
{
    double length = sqrt(vec_dot(v,v));
    return vec_mul(v,1.0/length);
}

int sphere_intersect(Sphere s, Vec3 d,Vec3*inter)
{
    double tc = vec_dot(s.center,d);
    double ta2 = s.r*s.r-vec_dot(s.center,s.center)+tc*tc;
    if(ta2 < 0.0)
        return 0; //No collision
    double ta = sqrt(ta2);
    Vec3 p1 = vec_mul(d,(tc-ta)); //Intersection points
    Vec3 p2 = vec_mul(d,(tc+ta));
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

    /*for(int y = 0; y < h; y++)
    {
        for(int x = 0; x < w; x++)
        {
            int index = (y*w+x)*3;
            image[index+0] = 150;
            image[index+1] = 120;
            image[index+2] = 220;

        }
    }*/

    /*double sphere_x = 20;
    double sphere_y = 0;
    double sphere_z = 100; //Positive axis: further away
    double sphere_radius = 50;*/
    Sphere s = {{0.0,0,3.5},3.0};

    for(int y = 0; y < h; y++)
    {
        for(int x = 0; x < w; x++)
        {
            int index = (y*w+x)*3;

            /*double src_x = x - center_image_x;
            double src_y = y - center_image_y;
            double src_z = 0.1;*/
            Vec3 view_vec = {(x-center_image_x)/200.0,-(y-center_image_y)/200.0,0.15};
            view_vec = vec_normalize(view_vec);
            Vec3 inter;

            //double distance = (src_x-sphere_x)*(src_x-sphere_x)+(src_y-sphere_y)*(src_y-sphere_y);
            //if(distance <= sphere_radius*sphere_radius)
            if(sphere_intersect(s,view_vec,&inter) == 1)
            {
                image[index+0] = 220;
                image[index+1] = 150;
                image[index+2] = 150;

                if(inter.y < 1.0)
                {
                    image[index+0] = 0;
                    image[index+1] = 0;
                }
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
