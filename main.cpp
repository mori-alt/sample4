#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#define EIGEN_DONT_VECTORIZE

#include <GL/freeglut.h>
#include <windows.h>
#include <time.h>

#define _USE_MATH_DEFINES

#include <math.h>
#include <vector>
#include <iostream>

#include "Camera.h"

inline double rand48(){
    return (double)rand()/RAND_MAX;
}

enum MouseMode {
    MM_CAMERA
};

// 光線が当たるときの判定に使うベクトルの情報
struct RayHit {
    double t; // ベクトルが面にあたる時のスカラー倍
    double alpha; // 面を決めるベクトル
    double beta;
    int mesh_idx; // 面の識別
    int tri_idx;
};

// 四角形の構造決定
struct RectangleObj {
    Eigen::Vector3d centerPos;
    Eigen::Vector3d widthVec;
    Eigen::Vector3d heightVec;
    Eigen::Vector3d color;
    Eigen::Vector3d n;
    bool is_light;
};

// 三角形の構造決定
struct TriangleObj {
    Eigen::Vector3d v1;
    Eigen::Vector3d v2;
    Eigen::Vector3d v3;
    Eigen::Vector3d n;
    Eigen::Vector3d color;
};

// 無限遠点の決定
const double __FAR__ = 1.0e33;

// フィルムサイズ指定
const int g_FilmWidth = 640;
const int g_FilmHeight = 480;
float *g_FilmBuffer = nullptr;
GLuint g_FilmTexture = 0;

bool g_DrawFilm = true;

int width = 640;
int height = 480;

MouseMode g_MouseMode = MM_CAMERA;
int mx, my;

double g_FrameSize_WindowSize_Scale_x = 1.0;
double g_FrameSize_WindowSize_Scale_y = 1.0;

Camera g_Camera;

RectangleObj rects[2];
TriangleObj tris[1];

constexpr int LIGHT_RECTANGLE_INDEX = 1;

void initFilm() {
    g_FilmBuffer = (float *) malloc(sizeof(float) * g_FilmWidth * g_FilmHeight * 3);
    memset(g_FilmBuffer, 0, sizeof(float) * g_FilmWidth * g_FilmHeight * 3);

    glGenTextures(1, &g_FilmTexture);
    glBindTexture(GL_TEXTURE_2D, g_FilmTexture);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, g_FilmWidth, g_FilmHeight, 0, GL_RGB, GL_FLOAT, g_FilmBuffer);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
}

void updateFilm() {
    glBindTexture(GL_TEXTURE_2D, g_FilmTexture);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, g_FilmWidth, g_FilmHeight, GL_RGB, GL_FLOAT, g_FilmBuffer);
}

void drawFilm() {
    Eigen::Vector3d screen_center = g_Camera.getEyePoint() - g_Camera.getZVector() * g_Camera.getFocalLength();
    Eigen::Vector3d p1 = screen_center - g_Camera.getXVector() * g_Camera.getScreenWidth() * 0.5 -
                         g_Camera.getYVector() * g_Camera.getScreenHeight() * 0.5;
    Eigen::Vector3d p2 = screen_center + g_Camera.getXVector() * g_Camera.getScreenWidth() * 0.5 -
                         g_Camera.getYVector() * g_Camera.getScreenHeight() * 0.5;
    Eigen::Vector3d p3 = screen_center + g_Camera.getXVector() * g_Camera.getScreenWidth() * 0.5 +
                         g_Camera.getYVector() * g_Camera.getScreenHeight() * 0.5;
    Eigen::Vector3d p4 = screen_center - g_Camera.getXVector() * g_Camera.getScreenWidth() * 0.5 +
                         g_Camera.getYVector() * g_Camera.getScreenHeight() * 0.5;

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, g_FilmTexture);

    glBegin(GL_TRIANGLES);
    glColor3f(1.0, 1.0, 1.0);

    glTexCoord2f(0.0, 1.0);
    glVertex3f(p1.x(), p1.y(), p1.z());
    glTexCoord2f(1.0, 1.0);
    glVertex3f(p2.x(), p2.y(), p2.z());
    glTexCoord2f(1.0, 0.0);
    glVertex3f(p3.x(), p3.y(), p3.z());

    glTexCoord2f(0.0, 1.0);
    glVertex3f(p1.x(), p1.y(), p1.z());
    glTexCoord2f(1.0, 0.0);
    glVertex3f(p3.x(), p3.y(), p3.z());
    glTexCoord2f(0.0, 0.0);
    glVertex3f(p4.x(), p4.y(), p4.z());

    glEnd();

    glDisable(GL_TEXTURE_2D);
}

void clearRayTracedResult() {
    memset(g_FilmBuffer, 0, sizeof(float) * g_FilmWidth * g_FilmHeight * 3);
}

void makeRectangle(const Eigen::Vector3d &centerPos, const Eigen::Vector3d &widthVec, const Eigen::Vector3d &heightVec,
                   const Eigen::Vector3d &color, RectangleObj &out_Rect) {
    out_Rect.centerPos = centerPos;
    out_Rect.widthVec = widthVec;
    out_Rect.heightVec = heightVec;
    out_Rect.color = color;

    const Eigen::Vector3d v1 = centerPos - widthVec - heightVec;
    const Eigen::Vector3d v2 = centerPos + widthVec + heightVec;
    const Eigen::Vector3d v3 = centerPos - widthVec + heightVec;

    out_Rect.n = ((v1 - v3).cross(v2 - v3)).normalized();
}

void makeTriangle(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, const Eigen::Vector3d &v3,
                   const Eigen::Vector3d &color, TriangleObj &out_Tri) {
    out_Tri.v1 = v1;
    out_Tri.v2 = v2;
    out_Tri.v3 = v3;
    out_Tri.color = color;

    // 法線計算　ワンチャン怪しい
    // 法線はこっち向いてることにしてる
    out_Tri.n = ((v2 - v1).cross(v3 - v1)).normalized();
}

// 近い面の計算
void raySquareIntersection(const RectangleObj &rect, const int in_Triangle_idx, const Ray &in_Ray, RayHit &out_Result) {
    out_Result.t = __FAR__;

    const Eigen::Vector3d v1 = rect.centerPos - rect.widthVec - rect.heightVec;
    const Eigen::Vector3d v2 = rect.centerPos + rect.widthVec + rect.heightVec;
    const Eigen::Vector3d v3 = rect.centerPos - rect.widthVec + rect.heightVec;

    Eigen::Vector3d triangle_normal = (v1 - v3).cross(v2 - v3);
    triangle_normal.normalize();

    const double denominator = triangle_normal.dot(in_Ray.d);
    if (denominator >= 0.0)
        return;

    const double t = triangle_normal.dot(v3 - in_Ray.o) / denominator;
    if (t <= 0.0)
        return;

    const Eigen::Vector3d x = in_Ray.o + t * in_Ray.d;

    Eigen::Matrix<double, 3, 2> A;
    A.col(0) = v1 - v3;
    A.col(1) = v2 - v3;

    Eigen::Matrix2d ATA = A.transpose() * A;
    const Eigen::Vector2d b = A.transpose() * (x - v3);

    const Eigen::Vector2d alpha_beta = ATA.inverse() * b;

    if (alpha_beta.x() < 0.0 || 1.0 < alpha_beta.x() || alpha_beta.y() < 0.0 || 1.0 < alpha_beta.y()) return;

    out_Result.t = t;
    out_Result.alpha = alpha_beta.x();
    out_Result.beta = alpha_beta.y();
}

// レイ飛ばす
void rayTracing(const Ray &in_Ray, RayHit &io_Hit) {
    double t_min = __FAR__; // 無限遠の指定
    double alpha_I = 0.0, beta_I = 0.0;
    int mesh_idx = -1; // レイが当たっているのかを判定したいので、面が当たっていないindexは-1にしておく

    for (int i = 0; i < 2; i++) {// 面を2個作っているから2回ループ
        // 計算用の値を決定
        RayHit temp_hit{};
        raySquareIntersection(rects[i], i, in_Ray, temp_hit);
        if (temp_hit.t < t_min) { // スカラー倍の値で小さいものが見つかるたびに（　近い面が見つかる度に　）　その値に更新する
            t_min = temp_hit.t;
            alpha_I = temp_hit.alpha;
            beta_I = temp_hit.beta;
            mesh_idx = i;
        }
    }

    io_Hit.t = t_min; // 最小のスカラー倍の計算　一番近い面の計算
    io_Hit.alpha = alpha_I;
    io_Hit.beta = beta_I;
    io_Hit.mesh_idx = mesh_idx;
}

// 今は面があるだけ
// 上の面を光源　下を普通の面にして　反射をモンテカルロ積分で表現する
// レイを飛ばす
// 光源かどうかの判定
// 光源だったら光源の色
// 拡散面　その点から出てくるLoutを計算する　その時にレンダリング方程式利用する
void Learn() {
    for (int y = 0; y < g_FilmHeight; y++) {
        for (int x = 0; x < g_FilmWidth; x++) {
            const int pixel_idx = y * g_FilmWidth + x;

            // 各ピクセルの真ん中の座標を見てる
            const double p_x = (x + 0.5) / g_FilmWidth;
            const double p_y = (y + 0.5) / g_FilmHeight;

            // レイ作ってる（目から飛ばすもの）
            Ray ray;
            // rayに値をセット（目の位置を計算してフィルムのある一点を通る方向成分を持つ単位ベクトルを計算）
            g_Camera.screenView(p_x, p_y, ray);

            // 計算する変数準備
            RayHit ray_hit;
            // レイを飛ばす
            rayTracing(ray, ray_hit);

            if (ray_hit.mesh_idx >= 0) // もし当たって入れば ray_hit.mesh_idx に当たってるメッシュのインデックスが入って帰ってくる
            {
                // この中で光源かどうかの場合分け
                if (rects[ray_hit.mesh_idx].is_light) { // 光源ならそのまま光らせる
                    g_FilmBuffer[pixel_idx * 3] = rects[ray_hit.mesh_idx].color.x();
                    g_FilmBuffer[pixel_idx * 3 + 1] = rects[ray_hit.mesh_idx].color.y();
                    g_FilmBuffer[pixel_idx * 3 + 2] = rects[ray_hit.mesh_idx].color.z();
                }
                else { // 光源でないなら反射を計算する 面で積分するようにする
                    // 拡散面の
                    Eigen::Vector3d x = ray.o + ray_hit.t * ray.d;
                    // サンプル数
                    const int n = 10;
                    Ray _ray;
                    RayHit _ray_hit;
                    Eigen::Vector3d count{0, 0, 0}; // 光を積もらせる
                    double Lin = 10; // 光の強さ
                    Eigen::Vector3d kd{1.0, 1.0, 1.0}; // 反射吸収率
                    double _x = 0; // 面上のある値のx
                    double _y = 0; // 面上のある値のy
                    double _z = 0; // 面上のある値のz
                    // この中でモンテカルロ積分
                    for (int i = 0; i < n; i++) {
                       const double _lx = 2 * rects[1].widthVec.norm();
                       const double _ly = 2 * rects[1].widthVec.norm();

                       const double _ox = rects[1].centerPos.x() - rects[1].widthVec.x();
                       const double _oy = rects[1].centerPos.y() - rects[1].heightVec.y();

                       _x = _ox + _lx * rand48();
                       _y = _oy + _ly * rand48();
                       _z = rects[1].centerPos.z();
                       Eigen::Vector3d xa = {_x, _y, _z};

                       _ray.o = xa;
                       _ray.d = (x - xa).normalized();
                        rayTracing(_ray, _ray_hit);

                        double  cos_theta_a = rects[1].n.dot(_ray.d);
                        double cos_theta = -rects[ray_hit.mesh_idx].n.dot(_ray.d);

                        double  geometry = cos_theta * cos_theta_a / (xa - x).squaredNorm();


                        count.x() += Lin * kd.x() * geometry *  rects[LIGHT_RECTANGLE_INDEX].color.x() / M_PI;
                        count.y() += Lin * kd.y() * geometry *  rects[LIGHT_RECTANGLE_INDEX].color.y() / M_PI;
                        count.z() += Lin * kd.z() * geometry *  rects[LIGHT_RECTANGLE_INDEX].color.z() / M_PI;
                        //std::cout <<  "###############" << std::endl;
                        //std::cout <<  Lin << std::endl;
                        //std::cout <<  kd.x() << std::endl;
                        //std::cout <<  geometry << std::endl;
                        //std::cout << _ray_hit.mesh_idx << std::endl;
                        //std::cout <<  rects[_ray_hit.mesh_idx].color.x() << std::endl;


                    }
                    double A = rects[1].widthVec.norm() * rects[1].heightVec.norm() * 4;
                    count = A * count / (double)n;



                    g_FilmBuffer[pixel_idx * 3] = count.x();
                    g_FilmBuffer[pixel_idx * 3 + 1] = count.y();
                    g_FilmBuffer[pixel_idx * 3 + 2] = count.z();
                }
            } else {
                g_FilmBuffer[pixel_idx * 3] = 1.0;
                g_FilmBuffer[pixel_idx * 3 + 1] = 1.0;
                g_FilmBuffer[pixel_idx * 3 + 2] = 1.0;
            }
        }
    }
    updateFilm();
    glutPostRedisplay();
}

void drawTriangleObj(const TriangleObj &rect) {
    glBegin(GL_TRIANGLES);
    glColor3f(rect.color(0), rect.color(1), rect.color(2));

    glVertex3f(rect.v1.x(), rect.v1.y(), rect.v1.z());
    glVertex3f(rect.v2.x(), rect.v2.y(), rect.v2.z());
    glVertex3f(rect.v3.x(), rect.v3.y(), rect.v3.z());

    glEnd();
}

void drawRectangleObj(const RectangleObj &rect) {
    glBegin(GL_TRIANGLES);
    glColor3f(rect.color(0), rect.color(1), rect.color(2));
    Eigen::Vector3d vertice1 = rect.centerPos + rect.widthVec + rect.heightVec;
    Eigen::Vector3d vertice2 = rect.centerPos - rect.widthVec + rect.heightVec;
    Eigen::Vector3d vertice3 = rect.centerPos - rect.widthVec - rect.heightVec;
    Eigen::Vector3d vertice4 = rect.centerPos + rect.widthVec - rect.heightVec;

    glVertex3f(vertice1(0), vertice1(1), vertice1(2));
    glVertex3f(vertice2(0), vertice2(1), vertice2(2));
    glVertex3f(vertice3(0), vertice3(1), vertice3(2));

    glVertex3f(vertice1(0), vertice1(1), vertice1(2));
    glVertex3f(vertice3(0), vertice3(1), vertice3(2));
    glVertex3f(vertice4(0), vertice4(1), vertice4(2));

    glEnd();
}

void mouseDrag(int x, int y) {
    int _dx = x - mx, _dy = y - my;
    mx = x;
    my = y;

    double dx = double(_dx) / double(width);
    double dy = -double(_dy) / double(height);

    if (g_MouseMode == MM_CAMERA) {
        double scale = 2.0;

        g_Camera.rotateCameraInLocalFrameFixLookAt(dx * scale);
        glutPostRedisplay();
    }
}

void mouseDown(int x, int y) {
    mx = x;
    my = y;
}

void mouse(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
        mouseDown(x, y);
}

void key(unsigned char key, int x, int y) {
    switch (key) {
        case 'C':
        case 'c':
            g_MouseMode = MM_CAMERA;
            break;
        case 'f':
        case 'F':
            g_DrawFilm = !g_DrawFilm;
            glutPostRedisplay();
            break;
    }
}

void projection_and_modelview(const Camera &in_Camera) {
    const double fovy_deg = (2.0 * 180.0 / M_PI) * atan(0.024 * 0.5 / in_Camera.getFocalLength());

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fovy_deg, double(width) / double(height), 0.01 * in_Camera.getFocalLength(), 1000.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    const Eigen::Vector3d lookAtPoint = in_Camera.getLookAtPoint();
    gluLookAt(in_Camera.getEyePoint().x(), in_Camera.getEyePoint().y(), in_Camera.getEyePoint().z(), lookAtPoint.x(),
              lookAtPoint.y(), lookAtPoint.z(), in_Camera.getYVector().x(), in_Camera.getYVector().y(),
              in_Camera.getYVector().z());
}

void drawFloor() {
    glBegin(GL_TRIANGLES);
    for (int j = -20; j < 20; j++) {
        for (int i = -20; i < 20; i++) {
            int checker_bw = (i + j) % 2;
            if (checker_bw == 0) {
                glColor3f(0.3, 0.3, 0.3);

                glVertex3f(i * 0.5, 0.0, j * 0.5);
                glVertex3f(i * 0.5, 0.0, (j + 1) * 0.5);
                glVertex3f((i + 1) * 0.5, 0.0, j * 0.5);

                glVertex3f(i * 0.5, 0.0, (j + 1) * 0.5);
                glVertex3f((i + 1) * 0.5, 0.0, (j + 1) * 0.5);
                glVertex3f((i + 1) * 0.5, 0.0, j * 0.5);
            }
        }
    }
    glEnd();
}

void display() {
    glViewport(0, 0, width * g_FrameSize_WindowSize_Scale_x, height * g_FrameSize_WindowSize_Scale_y);

    glClearColor(1.0, 0.5, 1.0, 0.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    projection_and_modelview(g_Camera);

    glEnable(GL_DEPTH_TEST);

    // フィルムの処理　光の反射を計算する
    Learn();

    if (g_DrawFilm)
        drawFilm();

    drawRectangleObj(rects[0]);
    drawRectangleObj(rects[1]);
    drawTriangleObj(tris[0]);

    glDisable(GL_DEPTH_TEST);

    glutSwapBuffers();
}

void resize(int w, int h) {
    width = w;
    height = h;
}

int main(int argc, char *argv[]) {
    // カメラの設定
    g_Camera.setEyePoint(Eigen::Vector3d{0.0, 1.0, 4.0});
    g_Camera.lookAt(Eigen::Vector3d{0.0, 0.5, 0.0}, Eigen::Vector3d{0.0, 1.0, 0.0});

    glutInit(&argc, argv);
    glutInitWindowSize(width, height);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_MULTISAMPLE);

    glutCreateWindow("Hello world!!");

    // With retina display, frame buffer size is twice the window size.
    // Viewport size should be set on the basis of the frame buffer size, rather than the window size.
    // g_FrameSize_WindowSize_Scale_x and g_FrameSize_WindowSize_Scale_y account for this factor.
    GLint dims[4] = {0};
    glGetIntegerv(GL_VIEWPORT, dims);
    g_FrameSize_WindowSize_Scale_x = double(dims[2]) / double(width);
    g_FrameSize_WindowSize_Scale_y = double(dims[3]) / double(height);

    // 引数1つ目がセンターポジション　二つ目が横幅　3つ目が立幅　4つ目が色指定 5つ目は識別番号
    // kd　は拡散面の色によって定義される　面の色の最大値をとることが多いらしい
    //makeRectangle(Eigen::Vector3d{0.0, 0.0, -1.0}, Eigen::Vector3d{1.0, 0.0, 0.0}, Eigen::Vector3d{0.0, 0.0, -1.0},
    //              Eigen::Vector3d{1.0, 0.00, 0.00}, rects[0]);
    makeTriangle(Eigen::Vector3d{0.0, 0.0, -1.0}, Eigen::Vector3d{1.0, 0.0, 0.0}, Eigen::Vector3d{0.0, 0.0, 1.0}, Eigen::Vector3d{1.0, 0.00, 0.00}, tris[0]);
    makeRectangle(Eigen::Vector3d{0.0, 1.0, -2.0}, Eigen::Vector3d{1.0, 0.0, 0.0}, Eigen::Vector3d{0.0, 1.0, 0.0},Eigen::Vector3d{0.90, 0.90, 0.90}, rects[0]);
    rects[1].is_light = false;
    rects[0].is_light = true;

    srand(time(NULL));

    // ここの関数たちがループされる
    glutDisplayFunc(display);
    glutReshapeFunc(resize);
    glutMouseFunc(mouse);
    glutMotionFunc(mouseDrag);
    glutKeyboardFunc(key);

    // フィルム初期化
    initFilm();
    clearRayTracedResult();


    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

    glutMainLoop();
    return 0;
}
