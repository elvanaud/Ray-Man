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
    int mirror;
    int refract;
} Material;

typedef struct
{
    Vec3 center;
    double r;
    Material mat;
} Sphere;

typedef struct
{
    Vec3 vertex0,vertex1,vertex2;
    Material mat;
} Triangle;

enum {SPHERE,TRIANGLE};

typedef struct
{
    int type;
    union
    {
        Sphere sphere;
        Triangle triangle;
    };
} Object;

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

int vec_equals(Vec3 a, Vec3 b)
{
    return a.x==b.x&&a.y==b.y&&a.z==b.z;
}

Vec3 vec_cross(Vec3 a, Vec3 b)
{
    Vec3 res = {
    a.y*b.z - a.z*b.y,
    a.z*b.x - a.x*b.z,
    a.x*b.y - a.y*b.x};
    return res;
}

double degrees_to_radians(double a)
{
    return a/180.0 * M_PI; //180� = pi rad
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
    if(tc-ta <= 0.0) //first intersection behind us or is equal to origin
    {
        if(tc+ta <= 0.0) //useless condition as it can only happen if tc < 0
            return 0;
        p1 = p2;
    }
    if(inter != NULL)
        *inter=p1;
    return 1;
}
//Copied from wikipedia
int triangle_intersect(Vec3 rayOrigin, Vec3 rayVector, Triangle inTriangle, Vec3* outIntersectionPoint)
{
    const float EPSILON = 0.0000001;
    Vec3 vertex0 = inTriangle.vertex0;
    Vec3 vertex1 = inTriangle.vertex1;
    Vec3 vertex2 = inTriangle.vertex2;
    Vec3 edge1, edge2, h, s, q;
    float a,f,u,v;
    edge1 = vec_substract(vertex1,vertex0);
    edge2 = vec_substract(vertex2,vertex0);
    h = vec_cross(rayVector,edge2);
    a = vec_dot(edge1,h);
    if (a > -EPSILON && a < EPSILON)
        return 0;    // This ray is parallel to this triangle.
    f = 1.0/a;
    s = vec_substract(rayOrigin,vertex0);
    u = f * vec_dot(s,h);
    if (u < 0.0 || u > 1.0)
        return 1;
    q = vec_cross(s,edge1);
    v = f * vec_dot(rayVector,q);
    if (v < 0.0 || u + v > 1.0)
        return 0;
    // At this stage we can compute t to find out where the intersection point is on the line.
    float t = f * vec_dot(edge2,q);
    if (t > EPSILON) // ray intersection
    {
        *outIntersectionPoint = vec_add(rayOrigin,vec_mul(rayVector,t));
        return 1;
    }
    else // This means that there is a line intersection but not a ray intersection.
        return 0;
}

Vec3 reflect(Vec3 n, Vec3 d)
{
    return vec_substract(d,vec_mul(n,2*vec_dot(n,d)));
}

Vec3 refract(Vec3 n, Vec3 i, double ratio) //Copied from wikipedia (Vecorial form of Snell-Descartes)
{
    double cosTheta1 = vec_dot(n,vec_mul(i,-1.0));

    if(cosTheta1 < 0.0)
    {
        n = vec_mul(n,-1.0);
        ratio = 1.0 / ratio;
        cosTheta1 = -cosTheta1;
    }
    double tmp = 1-(ratio*ratio*(1-cosTheta1*cosTheta1));
    if(tmp < 0.0)
        return i;
    double cosTheta2 = sqrt(tmp); //negative sqrt?
    Vec3 refracted = vec_add(vec_mul(i,ratio), vec_mul(n,ratio*cosTheta1-cosTheta2));
    refracted = vec_normalize(refracted);
    return refracted;
}

double clamp_rand(double min, double max)
{
    double range = max -min;
    double step = RAND_MAX/range;
    return min+rand()/step;
}

int objects_intersect(Vec3 origin, Vec3 view_vec,Object * objects,const int NB_OBJECTS, Vec3 * outInter, Vec3 * outNorm,Material*outMat)
{
    Vec3 inter;
    Vec3 normal;
    Material mat;
    double farthest = 1000000.0;
    int intersection = 0;

    for(int obstacle = 0; obstacle < NB_OBJECTS; obstacle++)
    {
        Vec3 tmpInter;
        Vec3 tmpNormal;
        Material tmpMat;

        if(objects[obstacle].type == SPHERE)
        {
            Sphere obj = objects[obstacle].sphere;
            if(sphere_intersect(obj,origin,view_vec,&tmpInter))
            {
                tmpNormal = vec_normalize(vec_substract(tmpInter,obj.center));
                tmpMat = obj.mat;
                intersection = 1;

                if(tmpInter.z <= farthest) //TODO: maybe factorize code here (careful might cause graphical glitches if only put after the if else
                {
                    farthest = tmpInter.z,
                    inter = tmpInter;
                    normal = tmpNormal;
                    mat = tmpMat;
                }
            }
        }
        else if(objects[obstacle].type == TRIANGLE)
        {
            Triangle obj = objects[obstacle].triangle;
            if(triangle_intersect(origin,view_vec,obj,&tmpInter))
            {
                tmpNormal = vec_normalize(vec_cross(vec_substract(obj.vertex0,obj.vertex1),vec_substract(obj.vertex2,obj.vertex1)));
                tmpMat = obj.mat;
                intersection = 1;

                if(tmpInter.z <= farthest)
                {
                    farthest = tmpInter.z,
                    inter = tmpInter;
                    normal = tmpNormal;
                    mat = tmpMat;
                }
            }
        }


    }

    if(intersection && outInter != NULL)
        *outInter = inter;
    if(intersection && outNorm != NULL)
        *outNorm = normal;
    if(intersection && outMat != NULL)
        *outMat = mat;
    return intersection;
}

Vec3 launchRay(Vec3 origin, Vec3 view_vec, Object * objects, const int NB_OBJECTS, Light*lights, const int NB_LIGHTS, int iter)
{
    Vec3 black = {0.0, 0.0, 0.0};
    Vec3 red = {1.0,0.0,0.0};
    Vec3 finalColor = {230,210,250};
    finalColor = vec_mul(finalColor,1.0/255.0);
    if(iter <= 0)
        return finalColor;
    iter--;

    Vec3 inter;
    Vec3 normale;
    Material mat;
    if(objects_intersect(origin,view_vec,objects,NB_OBJECTS,&inter,&normale,&mat))
    {
        finalColor = black;

        if(mat.mirror)
        {
            finalColor = launchRay(inter,reflect(normale,view_vec),objects, NB_OBJECTS, lights, NB_LIGHTS,iter);
        }
        else if(mat.refract)
        {

            Vec3 refract_dir = refract(vec_mul(normale,1.0),vec_mul(view_vec,1.0),1.0/1.33);
            inter = vec_dot(normale,refract_dir) > 0.0 ? vec_add(inter,vec_mul(normale,0.001)) : vec_add(inter,vec_mul(normale,-0.001));
            finalColor = launchRay(inter,refract_dir,objects, NB_OBJECTS, lights, NB_LIGHTS, iter);
        }

        for(int l = 0; l < NB_LIGHTS; l++)
        {
            Vec3 light_dir = vec_normalize(vec_substract(lights[l].pos,inter));

            int shadowed = objects_intersect(inter,light_dir,objects,NB_OBJECTS,NULL,NULL,NULL);

            Vec3 ambient = vec_mul(lights[l].color,lights[l].ambientStrength);

            double luminosity = vec_dot(light_dir,normale);
            luminosity = max(luminosity,0.0);
            Vec3 diffuse = vec_mul(lights[l].color,luminosity);

            Vec3 reflectedLight = reflect(normale,vec_mul(light_dir,-1.0));
            double spec = pow(max(vec_dot(reflectedLight,vec_mul(view_vec,-1.0)),0.0),mat.specLevel);
            Vec3 specular = vec_mul(lights[l].color,spec*mat.specularStrength);
            if(mat.mirror || mat.refract)
            {
                Vec3 blue = {0.0,0.0,0.0};
                ambient = diffuse = blue;
            }

            finalColor = vec_add(finalColor, vec_prod(vec_add(vec_mul(vec_add(diffuse,specular),1.0-shadowed),ambient),mat.color));
        }
    }
    return finalColor;
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
    Object *objects = malloc(NB_OBJECTS*sizeof(Object));
    for(int i = 0; i <= NB_OBJECTS-1; i++)
    {
        int mirorring = rand()%100<10?1:0;
        int refracting = rand()%100<10 && !mirorring?1:0;
        Sphere t = {    {clamp_rand(-5,5),clamp_rand(-5,5),clamp_rand(5,30.0)},
                        clamp_rand(0.2,2.0),
                        {{clamp_rand(0,1.0),clamp_rand(0,1.0),clamp_rand(0,1.0)},rand()%10<5?32:256,clamp_rand(0.1,2.0),mirorring,refracting}};
        Object o; o.type = SPHERE;
        o.sphere = t;
        objects[i] = o;
    }
    /*Sphere s1 = {{0.0,1.0,5.0},1.0,{{1.0,0.0,0.0},32,0.1,0,0}};
    Object o1 = {SPHERE}; o1.sphere = s1;
    objects[0] = o1;
    Sphere transparent_sphere = {{0.5,0.2,3.5},0.7,{{0.0,1.0,1.0},32,0.1,0,1}};
    printf("%d\n",transparent_sphere.mat.refract);
    Object o2 = {SPHERE}; o2.sphere = transparent_sphere;
    objects[NB_OBJECTS-1] = o2;*/

    for(int y = 0; y < h; y++)
    {
        for(int x = 0; x < w; x++)
        {
            int index = (y*w+x)*3;

            double fov = 80.0;
            double fov_rad = degrees_to_radians(fov/2.0);
            double step = tan(fov_rad)*2.0;
            Vec3 view_vec = {(x-center_image_x)*step/w,-(y-center_image_y)*step/h,1};
            view_vec = vec_normalize(view_vec);

            Vec3 finalColor = launchRay(origin, view_vec, objects, NB_OBJECTS, lights, NB_LIGHTS, 40);

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
