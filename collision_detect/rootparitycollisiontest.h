//
//
//  Tyson Brochu 2011
//

#ifndef NEWTUNICATE_ROOTPARITYCOLLISIONTEST_H
#define NEWTUNICATE_ROOTPARITYCOLLISIONTEST_H

#include "vec.h"

enum FLAGTYPE {
    right,
    throughEdge,
    pointOnFace
};


namespace rootparity {


    typedef Vec<3, bool> Vec3b;
    typedef Vec<4, bool> Vec4b;


    class RootParityCollisionTest {

    public:

        /// Constructor, take a reference to the input vertex locations at t=0 and t=1.  User also specifies whether the vertices 
        /// should be interpreted as representing an edge-edge collision test, or point-triangle.
        ///
        inline RootParityCollisionTest(const Vec3d &_x0old, const Vec3d &_x1old, const Vec3d &_x2old,
                                       const Vec3d &_x3old,
                                       const Vec3d &_x0new, const Vec3d &_x1new, const Vec3d &_x2new,
                                       const Vec3d &_x3new,
                                       bool _is_edge_edge) :
                m_x0old(_x0old), m_x1old(_x1old), m_x2old(_x2old), m_x3old(_x3old),
                m_x0new(_x0new), m_x1new(_x1new), m_x2new(_x2new), m_x3new(_x3new),
                m_is_edge_edge(_is_edge_edge),
                m_ray() {
        }

        /// Returns true if there is an odd number of ray intersections (corresponding to an odd number of roots of F in the domain).
        ///
        inline bool run_test();

        /// Run edge-edge continuous collision detection
        ///
        bool edge_edge_collision();

        /// Run point-triangle continuous collision detection
        ///
        bool point_triangle_collision();


        void printpoint();

        void printRay();

    private:

        /// Input vertex locations.
        /// If point-triangle collision test, the point is vertex 0, and the triangle is vertices (1,2,3).
        /// If edge-edge collision test, the edges are (0,1) and (2,3).
        ///
        const Vec3d &m_x0old, m_x1old, m_x2old, m_x3old, m_x0new, m_x1new, m_x2new, m_x3new;

        const bool m_is_edge_edge;
        Vec3d m_ray;

        // Functions

        bool AABB_edge_edge();

        bool AABB_point_tri();

        bool ray_vs_triangle(const Vec3d &x0, const Vec3d &x1, const Vec3d &x2, FLAGTYPE &flag, bool &ontriangle);

        bool ray_vs_triangle(const Vec3d &x0, const Vec3d &x1, const Vec3d &x2, FLAGTYPE &flag);

        bool ray_vs_bilinear_patch(const Vec3d &x0, const Vec3d &x1, const Vec3d &x2, const Vec3d &x3, FLAGTYPE &flag);

        void randGenerateRay(int N);


    };


    inline bool aabb_contains_origin(const Vec3d &xmin, const Vec3d &xmax) {
        return (xmin[0] <= 0 && xmin[1] <= 0 && xmin[2] <= 0) && (xmax[0] >= 0 && xmax[1] >= 0 && xmax[2] >= 0);
    }


    inline bool RootParityCollisionTest::run_test() {
        if (m_is_edge_edge) {
            return edge_edge_collision();
        } else {
            return point_triangle_collision();
        }
    }

    inline void rootparity::RootParityCollisionTest::printpoint() {
        std::cout << m_x0old - m_x2old << std::endl << m_x0old - m_x3old << std::endl << m_x1old - m_x2old << std::endl
                  << m_x1old - m_x3old << std::endl << std::endl <<
                  m_x0new - m_x2new << std::endl << m_x0new - m_x3new << std::endl << m_x1new - m_x2new << std::endl
                  << m_x1new - m_x3new << std::endl;
    }

    inline void rootparity::RootParityCollisionTest::printRay() {
        std::cout << m_ray << std::endl;
    }

} // namespace rootparity

#endif
