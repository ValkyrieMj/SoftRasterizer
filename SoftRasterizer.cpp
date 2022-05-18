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
auto model = make_shared<Model>("obj1/african_head.obj");

//摄像机位置
Vec3f eye(2, 1, 3);
//焦点位置
Vec3f center(0, 0, 1);

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

//将x,y坐标放width×height，z坐标转成[0,255)，这里相机位置，直接这样就行了
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

void triangle1(const Vec3f pts[], vector<vector<float>> &zbuffer, TGAImage& image, TGAColor color)
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

void triangle(Vec3i t0, Vec3i t1, Vec3i t2, Vec2i uv0, Vec2i uv1, Vec2i uv2, TGAImage& image, float intensity[], vector<vector<float>>& zbuffer) {   //三角形vec坐标与贴图uv坐标对应
    if (t0.y == t1.y && t0.y == t2.y) return;
    float ity0 = intensity[0];
    float ity1 = intensity[1];
    float ity2 = intensity[2];
    //分割成两个三角形
    if (t0.y > t1.y) { std::swap(t0, t1); std::swap(ity0, ity1); std::swap(uv0, uv1); }
    if (t0.y > t2.y) { std::swap(t0, t2); std::swap(ity0, ity2); std::swap(uv0, uv2); }
    if (t1.y > t2.y) { std::swap(t1, t2); std::swap(ity1, ity2); std::swap(uv1, uv2); }
    //用高度做循环控制
    int total_height = t2.y - t0.y;
    for (int i = 0; i < total_height; i++) {
        //判断属于哪一部分以确定高度
        bool second_half = i > t1.y - t0.y || t1.y == t0.y;
        int segment_height = second_half ? t2.y - t1.y : t1.y - t0.y;
        //计算当前的比例
        float alpha = (float)i / total_height;
        float beta = (float)(i - (second_half ? t1.y - t0.y : 0)) / segment_height; // be careful: with above conditions no division by zero here
        //A表示t0与t2之间的点
        //B表示t0与t1之间的点
        Vec3i A = t0 + Vec3f(t2 - t0) * alpha;
        Vec3i B = second_half ? t1 + Vec3f(t2 - t1) * beta : t0 + Vec3f(t1 - t0) * beta;
        //计算A,B两点的光照强度
        float ityA = ity0 + (ity2 - ity0) * alpha;
        float ityB = second_half ? ity1 + (ity2 - ity1) * beta : ity0 + (ity1 - ity0) * beta;
        //计算UV
        Vec2i uvA = uv0 + (uv2 - uv0) * alpha;
        Vec2i uvB = second_half ? uv1 + (uv2 - uv1) * beta : uv0 + (uv1 - uv0) * beta;

        //保证B在A的右边
        if (A.x > B.x) { std::swap(A, B); std::swap(ityA, ityB); }
        //用横坐标作为循环控制，对这一行进行着色
        for (int j = A.x; j <= B.x; j++) {
            //计算当前点在AB之间的比例
            float phi = B.x == A.x ? 1. : (float)(j - A.x) / (float)(B.x - A.x);
            //计算出当前点的坐标,A，B保存了z轴信息
            Vec3i   P = Vec3f(A) + Vec3f(B - A) * phi;
            float ityP = ityA + (ityB - ityA) * phi;
            ityP = std::min(1.f, std::abs(ityP) + 0.01f);;
            Vec2i uvP = uvA + (uvB - uvA) * phi;
            //float disP = disA + (disB - disA) * phi;
            if (P.x >= width || P.y >= height || P.x < 0 || P.y < 0) continue;
            if (zbuffer[P.x][P.y] < P.z) {
                zbuffer[P.x][P.y] = P.z;
                TGAColor color = model->diffuse(uvP);
                image.set(P.x, P.y, TGAColor(color.bgra[2], color.bgra[1], color.bgra[0]) * ityP);
                //image.set(P.x, P.y, TGAColor(255,255,255)* ityP);
            }
        }
    }
}


Matrix lookat(Vec3f eye, Vec3f center, Vec3f up)
{
    Vec3f z = (eye - center).normalize();
    Vec3f x = (up^z).normalize();
    Vec3f y = (z ^ x).normalize();
    Matrix rotation = Matrix::identity(4);
    Matrix translation = Matrix::identity(4);
    for (int i = 0; i < 3; i++) {
        rotation[0][i] = x[i];
        rotation[1][i] = y[i];
        rotation[2][i] = z[i];
    }
    for (int i = 0; i < 3; i++)
        //translation[i][3] = -center[i];
        rotation[i][3] = -center[i];
    return rotation * translation;
}

Vec3f world2screen(Vec3f v) {
    return Vec3f(int((v.x + 1.) * width / 2. + .5), int((v.y + 1.) * height / 2. + .5), v.z);
}

int main()
{

    TGAImage image(width, height, TGAImage::RGB);
    Vec3f camara(0, 0, 3);
    Vec3f light_dir = Vec3f(0, -1, -1).normalize();
    vector<vector<float>> zbuffer(width,vector<float>(height, -std::numeric_limits<float>::max()));
    Matrix Projection = Matrix::identity(4);
    //Projection[3][2] = -1.f / camara.z;
    Projection[3][2] = -1.f / (eye - center).norm();
    //Matrix ViewPort = viewport(width / 8, height / 8, width * 3 / 4, height * 3 / 4);
    Matrix ViewPort = viewport(width / 8, height / 8, width * 4 / 4, height * 4 / 4);
    Matrix ModelView = lookat(eye, center, Vec3f(0, 1, 0));
    for (int i = 0; i < model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
        Vec3i screen_coords[3];
        //Vec3f world_coords[3];
        float distance[3];
        float intensity[3];
        for (int j = 0; j < 3; j++) {
            Vec3f v = model->vert(face[j]);
            Matrix m_v = ModelView * Matrix(v);
            screen_coords[j] = m2v(ViewPort * Projection * ModelView* v2m(v));
            //screen_coords[j] = m2v(Projection * ViewPort * v2m(v));
            //world_coords[j] = v;
            intensity[j] = model->norm(i, j) * light_dir;
            //Vec3f new_v = Vec3f(m_v);
           // distance[j] = std::pow((std::pow(new_v.x - eye.x, 2.0f) + std::pow(new_v.y - eye.y, 2.0f) + std::pow(new_v.z - eye.z, 2.0f)), 0.5f);

        }
        //Vec3f n = (world_coords[2] - world_coords[0])^(world_coords[1] - world_coords[0]);
        //n.normalize();
        //float intensity = n * light_dir;
        //intensity = std::min(std::abs(intensity), 1.f);
        Vec2i uv[3];
        for (int k = 0; k < 3; k++) {
            uv[k] = model->uv(i, k);  //i为第几个面，k为顶点索引，uv[k]为视角转换后的贴图坐标，与screen_coords对应
        }
        //绘制三角形
        triangle(screen_coords[0], screen_coords[1], screen_coords[2], uv[0], uv[1], uv[2], image, intensity, zbuffer);
        //triangle3(screen_coords[0], screen_coords[1], screen_coords[2], intensity[0], intensity[1], intensity[2], uv[0], uv[1], uv[2], image, zbuffer);
    }
    image.flip_vertically();
    image.write_tga_file("output.tga");
    return 0;
}




