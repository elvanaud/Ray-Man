#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

typedef struct
{
    double x,y,z;
} Vec3;

typedef struct
{
    double s,x,y,z;
} Quat;

typedef struct
{
    Vec3 color;
    int specLevel;
    double specularStrength;
    int mirror;
    int refract;

    unsigned char * texture;
    unsigned char * normalMap;
    int w,h;

    double w0,w1,w2;
    int type;
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
    Vec3 texCoord0,texCoord1,texCoord2;
} Triangle;

typedef struct
{
    Triangle faces[12];
    //Material mat;
} Cubemap;

enum {SPHERE,TRIANGLE,CUBEMAP};

typedef struct
{
    int type;
    union
    {
        Sphere sphere;
        Triangle triangle;
        Cubemap cubemap;
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
    const double EPSILON = 0.00000001;
    Vec3 vertex0 = inTriangle.vertex0;
    Vec3 vertex1 = inTriangle.vertex1;
    Vec3 vertex2 = inTriangle.vertex2;
    Vec3 edge1, edge2, h, s, q;
    double a,f,u,v;
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
        return 0;
    q = vec_cross(s,edge1);
    v = f * vec_dot(rayVector,q);
    if (v < 0.0 || u + v > 1.0)
        return 0;
    // At this stage we can compute t to find out where the intersection point is on the line.
    double t = f * vec_dot(edge2,q);
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

Vec3 refract(Vec3 n, Vec3 i, double ratio) //Copied from wikipedia (Vectorial form of Snell-Descartes)
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

double tri_area(Triangle tri)
{
    Vec3 base = vec_substract(tri.vertex2, tri.vertex0);
    Vec3 side = vec_substract(tri.vertex1,tri.vertex0);
    double t = vec_dot(side,vec_normalize(base));
    double h2 = max(0.0,vec_dot(side,side)-t*t);
    double area = sqrt(h2*vec_dot(base,base))/2.0;
    return area;
}
void barycentric_interpolation(Triangle tri, Vec3 p, double * w0, double * w1, double * w2)
{
    double areaTotal = tri_area(tri);
    Triangle t0 = {tri.vertex1,tri.vertex2,p};
    Triangle t1 = {tri.vertex0,tri.vertex2,p};
    Triangle t2 = {tri.vertex0,tri.vertex1,p};
    *w0 = tri_area(t0)/areaTotal;
    *w1 = tri_area(t1)/areaTotal;
    *w2 = tri_area(t2)/areaTotal;
}

Quat quat_product(Quat a, Quat b)
{
    Vec3 vecA = {a.x,a.y,a.z};
    Vec3 vecB = {b.x,b.y,b.z};

    Vec3 vecPart = vec_add(vec_add(vec_mul(vecA,b.s),vec_mul(vecB,a.s)),vec_cross(vecA,vecB));

    Quat result = {
        a.s*b.s-vec_dot(vecA,vecB),
        vecPart.x, vecPart.y, vecPart.z};
    return result;
}

Quat quat_conjugate(Quat p, Quat q)
{
    Quat qInverse = {q.s,-q.x,-q.y,-q.z}; //Only if q is normalized
    Quat result = quat_product(quat_product(q,p),qInverse);
    return result;
}

Vec3 vec_rotate(Vec3 axis, double angle, Vec3 v)
{
    angle /= 2.0;
    Quat rotation = {cos(angle),sin(angle)*axis.x,sin(angle)*axis.y,sin(angle)*axis.z};
    Quat q = {0,v.x,v.y,v.z};
    Quat result = quat_conjugate(q,rotation);
    Vec3 rotated = {result.x,result.y,result.z};
    return rotated;
}

Vec3 vec_align(Vec3 dir, Vec3 v)
{
    Vec3 axis = vec_cross(dir,v);
    double angle = acos(vec_dot(dir,v));
    Vec3 rotated = vec_rotate(axis,angle,v);
    return rotated;
}

Triangle triangle_rotate(Triangle tri, Vec3 axis, double angle)
{
    tri.vertex0 = vec_rotate(axis,angle,tri.vertex0);
    tri.vertex1 = vec_rotate(axis,angle,tri.vertex1);
    tri.vertex2 = vec_rotate(axis,angle,tri.vertex2);
    return tri;
}

Vec3 vec_linear_interpolation(Vec3 a, Vec3 b, double r)
{
    Vec3 res = vec_add(a,vec_mul(vec_substract(b,a),r));
    return res;
}

Vec3 texture_sample_nearest(unsigned char * texture, Vec3 tex, int w, int h, int n)
{
    int texX = (w-1)*tex.x;
    int texY = (h-1)*tex.y;
    if(n!=3)n=3; //sorry i don't handle this for now
    Vec3 color = {texture[(texY*w+texX)*n],texture[(texY*w+texX)*n+1],texture[(texY*w+texX)*n+2]};
    color = vec_mul(color,1.0/255.0);
    return color;
}

Vec3 texture_sample_bilinear(unsigned char * texture, Vec3 tex, int w, int h, int n)
{
    int left = w*tex.x; //Careful 1.0*w = w(can't index texture[w]) see sample nearest ^
    int top = h*tex.y;
    int right = ceil(w*tex.x);
    int bottom = ceil(h*tex.y);

    Vec3 topLeftCoord = {left/(double)w,top/(double)h};
    Vec3 bottomLeftCoord = {left/(double)w,bottom/(double)h};
    Vec3 topRightCoord = {right/(double)w,top/(double)h};
    Vec3 bottomRightCoord = {right/(double)w,bottom/(double)h};

    Vec3 topLeft = texture_sample_nearest(texture,topLeftCoord,w,h,n);
    Vec3 bottomLeft = texture_sample_nearest(texture,bottomLeftCoord,w,h,n);
    Vec3 topRight = texture_sample_nearest(texture,topRightCoord,w,h,n);
    Vec3 bottomRight = texture_sample_nearest(texture,bottomRightCoord,w,h,n);

    Vec3 tmpLeft = vec_linear_interpolation(topLeft,bottomLeft,h*tex.y-top);
    Vec3 tmpRight = vec_linear_interpolation(topRight,bottomRight,h*tex.y-top);

    Vec3 color = vec_linear_interpolation(tmpLeft,tmpRight,w*tex.x-left);
    return color;
}

Cubemap make_cubemap(char * path)
{
    char * names[] = {"front.jpg","back.jpg","right.jpg","left.jpg","top.jpg","bottom.jpg"};
    Material mat;
    Cubemap cube = {{
        {{-0.5,-0.5,0.5},{0.5,-0.5,0.5},{-0.5,0.5,0.5},mat,{0.0,1.0,0.0},{1.0,1.0,0.0},{0.0,0.0,0.0}}, //Front
        {{-0.5,0.5,0.5},{0.5,0.5,0.5},{0.5,-0.5,0.5},mat,{0.0,0.0,0.0},{1.0,0.0,0.0},{1.0,1.0,0.0}},

        {{0.5,-0.5,-0.5},{-0.5,-0.5,-0.5},{0.5,0.5,-0.5},mat,{0.0,1.0,0.0},{1.0,1.0,0.0},{0.0,0.0,0.0}}, //Back
        {{0.5,0.5,-0.5},{-0.5,-0.5,-0.5},{-0.5,0.5,-0.5},mat,{0.0,0.0,0.0},{1.0,1.0,0.0},{1.0,0.0,0.0}},

        {{-0.5,-0.5,-0.5},{-0.5,-0.5,0.5},{-0.5,0.5,-0.5},mat,{0.0,1.0,0.0},{1.0,1.0,0.0},{0.0,0.0,0.0}}, //Left (careful their left and right are inverted)
        {{-0.5,0.5,-0.5},{-0.5,0.5,0.5},{-0.5,-0.5,0.5},mat,{0.0,0.0,0.0},{1.0,0.0,0.0},{1.0,1.0,0.0}},

        {{0.5,-0.5,0.5},{0.5,-0.5,-0.5},{0.5,0.5,0.5},mat,{0.0,1.0,0.0},{1.0,1.0,0.0},{0.0,0.0,0.0}}, //Right
        {{0.5,0.5,0.5},{0.5,-0.5,-0.5},{0.5,0.5,-0.5},mat,{0.0,0.0,0.0},{1.0,1.0,0.0},{1.0,0.0,0.0}},

        {{-0.5,0.5,0.5},{0.5,0.5,0.5},{-0.5,0.5,-0.5},mat,{1.0,0.0,0.0},{0.0,0.0,0.0},{1.0,1.0,0.0}}, //Top
        {{-0.5,0.5,-0.5},{0.5,0.5,0.5},{0.5,0.5,-0.5},mat,{1.0,1.0,0.0},{0.0,0.0,0.0},{0.0,1.0,0.0}},

        {{-0.5,-0.5,-0.5},{0.5,-0.5,-0.5},{-0.5,-0.5,0.5},mat,{1.0,0.0,0.0},{0.0,0.0,0.0},{0.0,1.0,0.0}}, //Bottom
        {{-0.5,-0.5,0.5},{0.5,-0.5,-0.5},{0.5,-0.5,0.5},mat,{1.0,1.0,0.0},{0.0,0.0,0.0},{0.0,1.0,0.0}}}};



    for(int i = 0; i < 6; i++)
    {
        char * pathName = malloc(sizeof(char)*(1+strlen(path)+strlen(names[i])));
        strcpy(pathName,path);
        strcat(pathName,names[i]);
        Material tri_mat;
        int n;
        tri_mat.texture = stbi_load(pathName,&tri_mat.w,&tri_mat.h,&n,3);
        cube.faces[i*2+0].mat = tri_mat;
        cube.faces[i*2+1].mat = tri_mat;
    }

    return cube;
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
                tmpMat.type = SPHERE;

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
                tmpNormal = vec_normalize(vec_cross(vec_substract(obj.vertex1,obj.vertex0),vec_substract(obj.vertex2,obj.vertex0)));
                tmpMat = obj.mat;
                intersection = 1;
                //Barycentric interpolation here
                barycentric_interpolation(obj,tmpInter,&tmpMat.w0,&tmpMat.w1,&tmpMat.w2);
                tmpMat.type = TRIANGLE;

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

Vec3 launchRay(Vec3 origin, Vec3 view_vec, Object * objects, const int NB_OBJECTS, Light*lights, const int NB_LIGHTS, int iter, Cubemap skymap)
{
    Vec3 black = {0.0, 0.0, 0.0};
    Vec3 red = {1.0,0.0,0.0};
    Vec3 finalColor = {230,210,250};
    finalColor = vec_mul(finalColor,1.0/255.0);
    if(iter <= 0)
        return finalColor; //TODO: adapt this and make refractions work
    iter--;

    Vec3 inter;
    Vec3 normale;
    Material mat;
    if(objects_intersect(origin,view_vec,objects,NB_OBJECTS,&inter,&normale,&mat))
    {
        finalColor = black;

        if(mat.mirror)
        {
            finalColor = launchRay(inter,reflect(normale,view_vec),objects, NB_OBJECTS, lights, NB_LIGHTS,iter,skymap);
        }
        else if(mat.refract)
        {

            Vec3 refract_dir = refract(vec_mul(normale,1.0),vec_mul(view_vec,1.0),1.0/1.33);
            inter = vec_dot(normale,refract_dir) > 0.0 ? vec_add(inter,vec_mul(normale,0.001)) : vec_add(inter,vec_mul(normale,-0.001));
            finalColor = launchRay(inter,refract_dir,objects, NB_OBJECTS, lights, NB_LIGHTS, iter,skymap);
        }
        if(mat.type == TRIANGLE)
        {
            Vec3 t0 = {0.0,1.0,0.0};
            Vec3 t1 = {1.0,1.0,0.0};
            Vec3 t2 = {0.0,0.0,0.0};

            Vec3 tex = vec_add(vec_mul(t0,mat.w0),vec_add(vec_mul(t1,mat.w1),vec_mul(t2,mat.w2)));
            mat.color = texture_sample_bilinear(mat.texture,tex,mat.w,mat.h,3);
            Vec3 triangleNormal = normale;
            normale = vec_normalize(texture_sample_bilinear(mat.normalMap,tex,mat.w,mat.h,3));
            normale = vec_align(triangleNormal,normale);
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
            double v = vec_dot(reflectedLight,vec_mul(view_vec,-1.0));
            double spec = pow(max(v,0.0),mat.specLevel);
            Vec3 specular = vec_mul(lights[l].color,spec*mat.specularStrength);
            if(mat.mirror || mat.refract)
            {
                Vec3 blue = {0.0,0.0,0.0};
                ambient = diffuse = blue;
            }

            finalColor = vec_add(finalColor, vec_prod(vec_add(vec_mul(vec_add(diffuse,specular),1.0-shadowed),ambient),mat.color));
        }
    }
    else //Skybox
    {
        for(int i=0; i<12;i++)
        {
            Triangle tri = skymap.faces[i];
            Vec3 sky_origin = {0.0,0.0,0.0};
            if(triangle_intersect(sky_origin,view_vec,tri,&inter))
            {
                double w0,w1,w2;
                barycentric_interpolation(tri,inter,&w0,&w1,&w2);

                Vec3 tex = vec_add(vec_mul(tri.texCoord0,w0),vec_add(vec_mul(tri.texCoord1,w1),vec_mul(tri.texCoord2,w2)));
                finalColor = texture_sample_bilinear(tri.mat.texture,tex,tri.mat.w,tri.mat.h,3);
                break;
            }
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

    const int NB_OBJECTS = 20;
    Object *objects = malloc(NB_OBJECTS*sizeof(Object));
    /*Sphere s0 = {{0.0,-0.5,2.0},1.0,{{0.0,0.0,0.0},32,1.0,1,0}};
    Object o0; o0.type = SPHERE; o0.sphere = s0;
    objects[0] = o0;*/
    for(int i = 0; i < NB_OBJECTS; i++)
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
    Material tri_mat = {{0.5,0.4,0.4},32,1.0,0,0};
    int n;
    //tri_mat.texture = stbi_load("texture_ciment.jpg", &tri_mat.w, &tri_mat.h, &n, 3);
    tri_mat.texture = stbi_load("Rock_038_baseColor.jpg", &tri_mat.w, &tri_mat.h, &n, 3);
    tri_mat.normalMap = stbi_load("Rock_038_normal.jpg", &tri_mat.w, &tri_mat.h, &n, 3);
    Triangle t1 = {{-100.0,-50.0,2.0},{100.0,-50.0,2.0},{-100.0,-50.0,500.0},tri_mat}; //Triangle t1 = {{-0.9,-2.0,2.0},{0.5,-2.0,2.0},{-0.9,-2.0,500.0},tri_mat};

    Object o_tri; o_tri.type = TRIANGLE; o_tri.triangle=t1;
    objects[NB_OBJECTS-1] = o_tri;


    /*Triangle t2 = {{-0.5,-0.5,0.0},{0.5,-0.5,0.0},{0.0,0.5,0.0},tri_mat};
    Vec3 axis = {1.0,1.0,0.0};
    double angle = M_PI/4.0;
    t2 = triangle_rotate(t2,axis,angle); //rotates vectors not coordinates (rotates around origin (camera position))
    t2.vertex0.z += 1.0;
    t2.vertex1.z += 1.0;
    t2.vertex2.z += 1.0;
    Object o_tri2; o_tri2.type = TRIANGLE; o_tri2.triangle=t2;
    objects[NB_OBJECTS-2] = o_tri2;*/

    Cubemap skymap = make_cubemap("skybox/");

    for(int y = 0; y < h; y++)
    {
        for(int x = 0; x < w; x++)
        {
            int index = (y*w+x)*3;

            double fov = 80.0;
            double fov_rad = degrees_to_radians(fov/2.0);
            double step = tan(fov_rad)*2.0;
            Vec3 view_vec = {(x-center_image_x)*step/w,-(y-center_image_y+200.2)*step/h,1.0};
            view_vec = vec_normalize(view_vec);

            Vec3 finalColor = launchRay(origin, view_vec, objects, NB_OBJECTS, lights, NB_LIGHTS, 40,skymap);

            finalColor = vec_mul(finalColor,255);

            image[index+0] = max(0,min(finalColor.x,255));
            image[index+1] = max(0,min(finalColor.y,255));
            image[index+2] = max(0,min(finalColor.z,255));
        }
    }

    int result = stbi_write_png("images/output.png", w, h, 3, image, 0);
    if(result == 0)
    {
        printf("Error writing output file\n");
    }

    printf("RayMan\n");
    stbi_image_free(tri_mat.texture);
    stbi_image_free(tri_mat.normalMap);
    return 0;
}
