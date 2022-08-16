#include "cmesh/igl/booleanigl.h"
#include <Eigen/Core>
#include <igl/copyleft/cgal/mesh_boolean.h>
#include <igl/parallel_for.h>
#include <igl/write_triangle_mesh.h>
#include <igl/read_triangle_mesh.h>

namespace cmesh
{
    typedef struct Emesh {
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
    }emesh;

    void T2E(trimesh::TriMesh* Mesh1, emesh& emesh)
    {
        if (!Mesh1)
        {
            return;
        }

        // allocate space for vertices
        const int number_of_vertices = Mesh1->vertices.size();
        emesh.V.resize(number_of_vertices, 3);
        for (int i = 0; i < number_of_vertices; i++)
        {
            const trimesh::point& p = Mesh1->vertices[i];
            emesh.V(i, 0) = p.x;
            emesh.V(i, 1) = p.y;
            emesh.V(i, 2) = p.z;
        }

        int number_of_triangles = Mesh1->faces.size();
        // allocate space for triangles
        emesh.F.resize(number_of_triangles, 3);
        for (int i = 0; i < number_of_triangles; i++)
        {
            trimesh::TriMesh::Face& f = Mesh1->faces[i];
            emesh.F(i, 0) = f.x;
            emesh.F(i, 1) = f.y;
            emesh.F(i, 2) = f.z;
        }
    }

    void E2T(trimesh::TriMesh* Mesh1, emesh& emesh)
    {
        if (!Mesh1)
        {
            return;
        }
        Mesh1->vertices.clear();
        Mesh1->faces.clear();
        const int number_of_vertices = emesh.V.size();
        Mesh1->vertices.reserve(number_of_vertices / 3);
        for (int i = 0; i < number_of_vertices / 3; i++)
        {
            Mesh1->vertices.push_back(trimesh::point(emesh.V(i, 0), emesh.V(i, 1), emesh.V(i, 2)));
        }

        int number_of_triangles = emesh.F.size();
        // allocate space for triangles
        Mesh1->faces.reserve(number_of_triangles / 3);
        for (int i = 0; i < number_of_triangles / 3; i++)
        {
            trimesh::TriMesh::Face f;
            f.x = emesh.F(i, 0);
            f.y = emesh.F(i, 1);
            f.z = emesh.F(i, 2);
            Mesh1->faces.push_back(f);
        }
    }

    trimesh::TriMesh* cxBooleanOperateMeshIGLObj(trimesh::TriMesh* Mesh1, trimesh::TriMesh* Mesh2, MeshBooleanType meshBooleanType, ccglobal::Tracer* tracer)
    {

        emesh e1, e2, e0;

        if (tracer)
        {
            tracer->progress(0.2f);
            if (tracer->interrupt())
                return nullptr;
        }

        T2E(Mesh1, e1);
        if (tracer)
        {
            tracer->progress(0.3f);
            if (tracer->interrupt())
                return nullptr;
        }
        T2E(Mesh2, e2);
        if (tracer)
        {
            tracer->progress(0.4f);
            if (tracer->interrupt())
                return nullptr;
        }

        igl::copyleft::cgal::mesh_boolean(e1.V, e1.F, e2.V, e2.F,
            (igl::MeshBooleanType)meshBooleanType,
            e0.V, e0.F);

        if (tracer)
        {
            tracer->progress(0.8f);
            if (tracer->interrupt())
                return nullptr;
        }

        trimesh::TriMesh* out = new trimesh::TriMesh();
        E2T(out, e0);
        out->need_normals();

        if (tracer)
        {
            tracer->progress(1.0f);
            if (tracer->interrupt())
                return nullptr;
        }

        return out;

    }

    trimesh::TriMesh* cxBooleanOperateMeshIGLObj(std::string& Mesh1Name, std::string& Mesh2Name, MeshBooleanType meshBooleanType, ccglobal::Tracer* tracer)
    {

        Eigen::MatrixXd V1, V2, Vo;
        Eigen::MatrixXi F1, F2, Fo;
        igl::read_triangle_mesh(Mesh1Name, V1, F1);
        igl::read_triangle_mesh(Mesh2Name, V2, F2);

        igl::copyleft::cgal::mesh_boolean(V1, F1, V2, F2,
            (igl::MeshBooleanType)meshBooleanType,
            Vo, Fo);

        emesh e0;
        e0.V = Vo;
        e0.F = Fo;
        trimesh::TriMesh* out = new trimesh::TriMesh();;
        E2T(out, e0);
        out->need_normals();
        return out;

        return nullptr;
    }
}





