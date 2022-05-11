// SoftRasterizer.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <memory>
#include <math.h>
//#include <vector>
#include "model.h"
#include "tgaimage.h"
using namespace std;
const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
const TGAColor green = TGAColor(0, 255, 0, 255);
const int width = 800;
const int height = 800;
const int depth = 255;

void line(int x0, int y0, int x1, int y1, TGAImage& image, TGAColor color) {
    bool steep = false;
    if (std::abs(x0 - x1) < std::abs(y0 - y1)) {
        std::swap(x0, y0);
        std::swap(x1, y1);
        steep = true;
    }
    if (x0 > x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }
    for (int x = x0; x <= x1; x++) {
        float t = (x - x0) / (float)(x1 - x0);
        int y = y0 * (1. - t) + y1 * t;
        if (steep) {
            image.set(y, x, color);
        }
        else {
            image.set(x, y, color);
        }
    }
}


Vec3f barycentric(Vec3f A, Vec3f B, Vec3f C, Vec3f P)
{
    Vec3f AB = B - A;
    Vec3f AC = C - A;
    Vec3f PA = A - P;
    Vec3f s[2];
    s[0] = Vec3f(AB.x, AC.x, PA.x);
    s[1] = Vec3f(AB.y, AC.y, PA.y);
    //Vec3f u = cross(s[0], s[1]);
    Vec3f u = s[0] ^ s[1];
    if (abs(u.z) > 1e-2) {    //返回质心坐标
        return Vec3f((1.f - u.x / u.z - u.y / u.z), u.x / u.z, u.y / u.z);
    }
    return Vec3f(-1, 1, 1);
}

//Vec3f barycentric(Vec3f A, Vec3f B, Vec3f C, Vec3f P) {
//    Vec3f s[2];
//    //����[AB,AC,PA]��x��y����
//    for (int i = 2; i--; ) {
//        s[i][0] = C[i] - A[i];
//        s[i][1] = B[i] - A[i];
//        s[i][2] = A[i] - P[i];
//    }
//    //[u,v,1]��[AB,AC,PA]��Ӧ��x��y��������ֱ�����Բ��
//    //Vec3f u = cross(s[0], s[1]);
//    Vec3f u = s[0]^ s[1];
//    //���㹲��ʱ���ᵼ��u[2]Ϊ0����ʱ����(-1,1,1)
//    if (std::abs(u[2]) > 1e-2)
//        //��1-u-v��u��vȫΪ����0��������ʾ�����������ڲ�
//        return Vec3f(1.f - (u.x + u.y) / u.z, u.y / u.z, u.x / u.z);
//    return Vec3f(-1, 1, 1);
//}
 //4维矩阵表示3维向量
Vec3f m2v(Matrix m)
{
    return Vec3f(m[0][0] / m[3][0], m[1][0] / m[3][0], m[2][0] / m[3][0]);
}

//3维向量转4维矩阵
Matrix v2m(Vec3f v)
{
    Matrix m(4, 1);  //4行1列的矩阵
    m[0][0] = v.x;
    m[1][0] = v.y;
    m[2][0] = v.z;
    m[3][0] = 1.f;
    return m;
}

//将x,y坐标放width×height，z坐标转成[0,255)
Matrix viewport(int x, int y, int w, int h)
{
    Matrix m = Matrix::identity(4);
    //平移
    m[0][3] = x + w / 2.f;
    m[1][3] = y + h / 2.f;
    m[2][3] = depth / 2.0f;
    //缩放
    m[0][0] = w / 2.f;
    m[1][1] = h / 2.f;
    m[2][2] = depth / 2.f;
    return m;
}

void triangle(const Vec3f pts[], vector<vector<float>> &zbuffer, TGAImage& image, TGAColor color)
{
    Vec2f bboxmin(numeric_limits<float>::max(), numeric_limits<float>::max());
    Vec2f bboxmax(-numeric_limits<float>::max(), -numeric_limits<float>::max());
    Vec2f clamp(image.get_width() - 1, image.get_height() - 1);
    //cout << pts[0].x<<" "<< pts[1].x << endl;
    for (int i = 0; i < 3; i++) {
        //for (int j = 0; j < 2; j++) {
        //    bboxmin[j] = max(0.f, min(bboxmin[j], pts[i][j]));
        //    bboxmax[j] = min(clamp[j], max(bboxmax[j], pts[i][j]));
        //}
        bboxmin[0] = max(0.f, min(bboxmin[0], pts[i].x));
        bboxmin[1] = max(0.f, min(bboxmin[1], pts[i].y));
        bboxmax[0] = min(clamp[0], max(bboxmax[0], pts[i].x));
        bboxmax[1] = min(clamp[1], max(bboxmax[1], pts[i].y));
    }
    Vec3f P;
    //cout << bboxmin[0] << bboxmin[1]<< bboxmax[0]<< bboxmax[1]<<endl;
    //cout << bboxmin[0] <<" "<< bboxmax.x<<" "<< bboxmin.y<<" "<< bboxmax.y << endl;
    for (P.x = bboxmin[0]; P.x < bboxmax[0]; P.x++) {
        for (P.y = bboxmin[1]; P.y < bboxmax[1]; P.y++) {
            //计算质心
            if (P.x > 600 && P.y > 500)
            {
                P.x += 0.01;
            }
            Vec3f bc_screen = barycentric(pts[0], pts[1], pts[2], P);
            if (bc_screen.x < 0 || bc_screen.y < 0 || bc_screen.z < 0) continue;
            P.z = 0;
            for (int i = 0; i < 3; i++) P.z += pts[i].z * bc_screen[i];
           // cout << P.z<<" "<< zbuffer[int(P.x)][int(P.y)] << endl;
            //if (zbuffer[int(P.x)][int(P.y)] < P.z) {
                image.set(P.x, P.y, color);
                zbuffer[int(P.x)][int(P.y)] = P.z;
           // }
        }
    }
}

Vec3f world2screen(Vec3f v) {
    return Vec3f(int((v.x + 1.) * width / 2. + .5), int((v.y + 1.) * height / 2. + .5), v.z);
}

int main()
{
    auto model = make_shared<Model>("obj1/african_head.obj");
    TGAImage image(width, height, TGAImage::RGB);
    Vec3f light_dir(0, 0, -1);
    vector<vector<float>> zbuffer(width,vector<float>(height, -std::numeric_limits<float>::max()));
    for (int i = 0; i < model->nfaces(); i++) {
        vector<int> face = model->face(i);
        Vec3f screen_coords[3];
        Vec3f world_coords[3];
        for (int j = 0; j < 3; j++) {
            Vec3f v = model->vert(face[j]);
            screen_coords[j] = world2screen(model->vert(face[j]));
            world_coords[j] = v;
            //screen_coords[j] = Vec3f(int((v.x + 1.) * width / 2+.5), int((v.y + 1.) * height / 2+.5), v.z);
        }
        Vec3f n = (world_coords[2] - world_coords[0])^(world_coords[1] - world_coords[0]);
        n.normalize();
        float intensity = n * light_dir;
        intensity = 1;
        //if(intensity > 0)
            triangle(screen_coords,zbuffer, image, TGAColor(255 * intensity, 255 * intensity, 255 * intensity, 255));
    }
    image.flip_vertically();
    image.write_tga_file("output.tga");
    return 0;
}