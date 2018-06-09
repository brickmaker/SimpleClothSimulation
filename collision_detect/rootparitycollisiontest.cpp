#include "rootparitycollisiontest.h"
#include "Matrix.h"
#include <cstdlib>

namespace rootparity {
#ifndef M_PI
    const double M_PI = 3.1415926535;
#endif
    namespace   // unnamed namespace for local functions
    {
        template<class T>
        T plane_dist(const Vec<3, T> &x, const Vec<3, T> &q, const Vec<3, T> &r, const Vec<3, T> &p) {
            return dot(x - p, cross(q - p, r - p));
        }

        // ----------------------------------------

        template<class T>
        bool
        implicit_surface_function(const Vec<3, T> &x, const Vec<3, T> &q0, const Vec<3, T> &q1, const Vec<3, T> &q2,
                                  const Vec<3, T> &q3, T &out) {

            T g012 = plane_dist(x, q0, q1, q2);
            T g132 = plane_dist(x, q1, q3, q2);
            T h12 = g012 * g132;
            T g013 = plane_dist(x, q0, q1, q3);
            T g032 = plane_dist(x, q0, q3, q2);
            T h03 = g013 * g032;
            out = h12 - h03;
            return out > 0;
        }

        template<class T>
        bool
        implicit_surface_function(const Vec<3, T> &x, const Vec<3, T> &q0, const Vec<3, T> &q1, const Vec<3, T> &q2,
                                  const Vec<3, T> &q3) {

            T g012 = plane_dist(x, q0, q1, q2);
            T g132 = plane_dist(x, q1, q3, q2);
            T h12 = g012 * g132;
            T g013 = plane_dist(x, q0, q1, q3);
            T g032 = plane_dist(x, q0, q3, q2);
            T h03 = g013 * g032;
            T out = h12 - h03;
            return out > 0;
        }

        // Determine whether point P in triangle ABC
        bool PointinTriangle(Vec3d A, Vec3d B, Vec3d C, Vec3d P) {
            if(A==C||A==B||B==C)
                return false;
            Vec3d v0 = C - A;
            Vec3d v1 = B - A;
            Vec3d v2 = P - A;
            Vec3d t0 = v0,t1 = v1;
            normalize(t0);
            normalize(t1);
            if(t0==t1||t0==-t1)
                return false;

            double dot00 = dot(v0, v0);
            double dot01 = dot(v0, v1);
            double dot02 = dot(v0, v2);
            double dot11 = dot(v1, v1);
            double dot12 = dot(v1, v2);

            double inverDeno = 1 / (dot00 * dot11 - dot01 * dot01);

            double u = (dot11 * dot02 - dot01 * dot12) * inverDeno;
            if (u < 0 || u > 1) // if u out of range, return directly
            {
                return false;
            }

            float v = (dot00 * dot12 - dot01 * dot02) * inverDeno;
            if (v < 0 || v > 1) // if v out of range, return directly
            {
                return false;
            }

            return u + v <= 1;
        }
    }


    bool RootParityCollisionTest::edge_edge_collision() {
        // 0-2 0-3 1-2 1-3;
        // ��Χ��
        if (!AABB_edge_edge())
            return false;

        int S;
        int N = 0;
        while (++N) {
            S = 0;
            FLAGTYPE flag = right;
            randGenerateRay(N-1);

            S += ray_vs_bilinear_patch(m_x0old - m_x2old, m_x0old - m_x3old, m_x1old - m_x2old, m_x1old - m_x3old,
                                       flag);
            S += ray_vs_bilinear_patch(m_x0new - m_x2new, m_x0new - m_x3new, m_x1new - m_x2new, m_x1new - m_x3new,
                                       flag);

            if (pointOnFace == flag)            //  �����������
                return true;
            else if (throughEdge == flag)        //  ������ڱ�Ե
                continue;

            S += ray_vs_bilinear_patch(m_x0old - m_x2old, m_x0old - m_x3old, m_x0new - m_x2new, m_x0new - m_x3new,
                                       flag);//0-2 0-3
            S += ray_vs_bilinear_patch(m_x0old - m_x2old, m_x1old - m_x2old, m_x0new - m_x2new, m_x1new - m_x2new,
                                       flag);//0-2 1-2
            S += ray_vs_bilinear_patch(m_x0old - m_x3old, m_x1old - m_x3old, m_x0new - m_x3new, m_x1new - m_x3new,
                                       flag);//0-3 1-3
            S += ray_vs_bilinear_patch(m_x1old - m_x2old, m_x1old - m_x3old, m_x1new - m_x2new, m_x1new - m_x3new,
                                       flag);//1-2 1-3


            if (pointOnFace == flag)            //  �����������
                return true;
            else if (throughEdge == flag)        //  ������ڱ�Ե
                continue;
            else
                return S & 1;
        }

        assert(!"Should not get here");
        return false;
    }


    bool RootParityCollisionTest::point_triangle_collision() {
        // 0-1 0-2 0-3
        // ��Χ��
        if (!AABB_point_tri())
            return false;
        int N = 0;
        int S;
        while (++N) {
            S = 0;
            FLAGTYPE flag = right;
            randGenerateRay(N-1);

            S += ray_vs_bilinear_patch(m_x0old - m_x1old, m_x0old - m_x2old, m_x0new - m_x1new, m_x0new - m_x2new,
                                       flag);
            S += ray_vs_bilinear_patch(m_x0old - m_x3old, m_x0old - m_x2old, m_x0new - m_x3new, m_x0new - m_x2new,
                                       flag);
            S += ray_vs_bilinear_patch(m_x0old - m_x1old, m_x0old - m_x3old, m_x0new - m_x1new, m_x0new - m_x3new,
                                       flag);

            if (pointOnFace == flag)            //  �����������
                return true;
            else if (throughEdge == flag)        //  ������ڱ�Ե
                continue;

            S += ray_vs_triangle(m_x0old - m_x1old, m_x0old - m_x2old, m_x0old - m_x3old, flag);
            S += ray_vs_triangle(m_x0new - m_x1new, m_x0new - m_x2new, m_x0new - m_x3new, flag);

            if (pointOnFace == flag)            //  �����������
                return true;
            else if (throughEdge == flag)        //  ������ڱ�Ե
                continue;
            else
                return S & 1;
        }

        assert(!"Should not get here");
        return false;
    }


    Vec3d standard[5] = {Vec3d(1),Vec3d(-1),Vec3d(1,1,-1),Vec3d(1,-1,-1)};

    void RootParityCollisionTest::randGenerateRay(int N) {
        if(N<5)
            m_ray = standard[N];
        else
            m_ray = Vec3d(double(rand()), double(rand()), double(rand()));
        //m_ray = Vec3d(0,0,1);

    }

    bool RootParityCollisionTest::AABB_edge_edge() {
        // 0-2 0-3 1-2 1-3;
        Vec3d xmin1, xmin2, xmin, xmax1, xmax2, xmax;

        minmax(m_x0old - m_x2old, m_x0old - m_x3old, m_x1old - m_x2old, m_x1old - m_x3old, xmin1, xmax1);
        minmax(m_x0new - m_x2new, m_x0new - m_x3new, m_x1new - m_x2new, m_x1new - m_x3new, xmin1, xmax1);
        minmax(xmin1, xmin2, xmax1, xmax2, xmin, xmax);
        return aabb_contains_origin(xmin, xmax);
    }

    bool RootParityCollisionTest::AABB_point_tri() {
        Vec3d xmin1, xmin2, xmin, xmax1, xmax2, xmax;

        minmax(m_x0old - m_x1old, m_x0old - m_x2old, m_x0old - m_x3old, xmin1, xmax1);
        minmax(m_x0new - m_x1new, m_x0new - m_x2new, m_x0new - m_x3new, xmin2, xmax2);
        minmax(xmin1, xmin2, xmax1, xmax2, xmin, xmax);
        return aabb_contains_origin(xmin, xmax);
    }

    bool RootParityCollisionTest::ray_vs_triangle(const Vec3d &x0, const Vec3d &x1, const Vec3d &x2, FLAGTYPE &flag) {
        // ���Բο�https://lisabug.github.io/2015/05/01/Ray-tracer-triangle-intersection/
        // ����ͨ��������
        double belta, gamma, t;
        double a;
        a = _matrix::det3x3(
                x0[0] - x1[0], x0[0] - x2[0], m_ray[0],
                x0[1] - x1[1], x0[1] - x2[1], m_ray[1],
                x0[2] - x1[2], x0[2] - x2[2], m_ray[2]
        );
        belta = _matrix::det3x3(
                x0[0], x0[0] - x2[0], m_ray[0],
                x0[1], x0[1] - x2[1], m_ray[1],
                x0[2], x0[2] - x2[2], m_ray[2]
        );
        gamma = _matrix::det3x3(
                x0[0] - x1[0], x0[0], m_ray[0],
                x0[1] - x1[1], x0[1], m_ray[1],
                x0[2] - x1[2], x0[2], m_ray[2]
        );
        t = _matrix::det3x3(
                x0[0] - x1[0], x0[0] - x2[0], x0[0],
                x0[1] - x1[1], x0[1] - x2[1], x0[1],
                x0[2] - x1[2], x0[2] - x2[2], x0[2]
        );

        if (a == 0) // ƽ��
        {
            if (t == 0) // �غ�
            {
                if (PointinTriangle(x0, x1, x2, Vec3d(0, 0, 0))) {
                    flag = pointOnFace;
                    //ontriangle = true;
                }
            }
            return false;
        }
        belta /= a;
        gamma /= a;
        t /= a;
        if (belta > 1 || belta < 0)
            return false;
        if (gamma > 1 || gamma < 0)
            return false;
        if (1 - belta - gamma > 1 || 1 - belta - gamma < 0)
            return false;

        if (0 == t) {
            flag = pointOnFace;
            return true;
        }


        // ���ô����߽�
        if (1 - belta - gamma == 1 || belta == 0 || gamma == 0) {
            flag = throughEdge;
            return false;
        }

        if (t > 0) {
            return true;
        }

        return false;
    }

    bool RootParityCollisionTest::ray_vs_triangle(const Vec3d &x0, const Vec3d &x1, const Vec3d &x2, FLAGTYPE &flag,
                                                  bool &ontriangle) {
        ontriangle = false;
        // ���Բο�https://lisabug.github.io/2015/05/01/Ray-tracer-triangle-intersection/
        // ����ͨ��������
        double belta, gamma, t;
        double a;
        a = _matrix::det3x3(
                x0[0] - x1[0], x0[0] - x2[0], m_ray[0],
                x0[1] - x1[1], x0[1] - x2[1], m_ray[1],
                x0[2] - x1[2], x0[2] - x2[2], m_ray[2]
        );
        belta = _matrix::det3x3(
                x0[0], x0[0] - x2[0], m_ray[0],
                x0[1], x0[1] - x2[1], m_ray[1],
                x0[2], x0[2] - x2[2], m_ray[2]
        );
        gamma = _matrix::det3x3(
                x0[0] - x1[0], x0[0], m_ray[0],
                x0[1] - x1[1], x0[1], m_ray[1],
                x0[2] - x1[2], x0[2], m_ray[2]
        );
        t = _matrix::det3x3(
                x0[0] - x1[0], x0[0] - x2[0], x0[0],
                x0[1] - x1[1], x0[1] - x2[1], x0[1],
                x0[2] - x1[2], x0[2] - x2[2], x0[2]
        );

        if (a == 0) // ƽ��
        {
            if (t == 0) // �غ�
            {
                if (PointinTriangle(x0, x1, x2, Vec3d(0, 0, 0))) {
                    // ������Ϊ�а�һ���ı��β�����������εĲ��������Ե����������ϲ������������
                    //flag = pointOnFace;
                    ontriangle = true;
                }
            }
            return false;
        }
        belta /= a;
        gamma /= a;
        t /= a;
        if (belta > 1 || belta < 0)
            return false;
        if (gamma > 1 || gamma < 0)
            return false;
        if (1 - belta - gamma > 1 || 1 - belta - gamma < 0)
            return false;

        // ���ô����߽�
        if (1 - belta - gamma == 1 || belta == 0 || gamma == 0) {
            flag = throughEdge;
            return false;
        }

        if (t > 0) {
            return true;
        }

        return false;
    }


    bool
    RootParityCollisionTest::ray_vs_bilinear_patch(const Vec3d &x0, const Vec3d &x1, const Vec3d &x2, const Vec3d &x3,
                                                   FLAGTYPE &flag) {
        // ��Ӧ��ϵ�� 0-2 1-3
        // �ж�bilinear patch�ǲ�����һ��ƽ����
        if (0 == plane_dist(x0, x1, x2, x3)) {
            //����������μ���
            return ray_vs_triangle(x0, x1, x2, flag) || ray_vs_triangle(x1, x2, x3, flag);
        }
        if(x0==x2 && x1==x3)
        {
            return false;
        }
        if(x0==x2)
        {
            return ray_vs_triangle(x0, x1, x3, flag);
        }
        if(x1==x3)
        {
            return ray_vs_triangle(x0, x1, x2, flag);
        }

        // �����ڲ�����������

        bool inTetrahedron = false;
        double dist[4];
        Vec3d origin(0);
        dist[0] = plane_dist(origin, x0, x2, x3);
        dist[1] = plane_dist(origin, x0, x3, x1);
        dist[2] = plane_dist(origin, x0, x1, x2);
        dist[3] = plane_dist(origin, x1, x3, x2);

        if (dist[0] > 0 && dist[1] > 0 && dist[2] > 0 && dist[3] > 0)
            inTetrahedron = true;
        if (dist[0] < 0 && dist[1] < 0 && dist[2] < 0 && dist[3] < 0)
            inTetrahedron = true;
        if (dist[0] == 0 || dist[1] == 0 || dist[2] == 0 || dist[3] == 0)
            inTetrahedron = true;

        if (!inTetrahedron) {
            //bool ontriangle[2];
            //bool result = ray_vs_triangle(x0, x1, x3, flag, ontriangle[0]) ^ ray_vs_triangle(x0, x2, x3, flag, ontriangle[1]);
            //if (ontriangle[0] ^ ontriangle[1])
            //{
            //	flag = pointOnFace;
            //}
            //return result;
            return ray_vs_triangle(x0, x1, x3, flag) ^ ray_vs_triangle(x0, x2, x3, flag);
        }

        // �ж�x���߶�03�Ƿ�ͬ������03ȡ�е��ж�
        double distance;
        implicit_surface_function(origin, x0, x1, x2, x3, distance);
        if (0 == distance) {
            //flag = pointOnFace;
            return false;
        }

        bool isTest03 = implicit_surface_function((x0 + x3) / 2, x0, x1, x2, x3) ^
                        implicit_surface_function(origin, x0, x1, x2, x3);

        // ���㽻�����
        if (isTest03) {
            return ray_vs_triangle(x0, x1, x3, flag) ^ ray_vs_triangle(x0, x2, x3, flag);
        } else {
            return ray_vs_triangle(x1, x2, x0, flag) ^ ray_vs_triangle(x1, x2, x3, flag);
        }

        assert(!"Should not get here");
        return false;
    }

}





