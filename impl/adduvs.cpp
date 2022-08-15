#include "cmesh/uvs/adduvs.h"
#include "ccglobal/tracer.h"

#include "ccglobal/spycc.h"


namespace cmesh
{
    void getAcrossEdge(trimesh::TriMesh* mesh, int fn,int fedgen,int edgeIndex)//edgeIndex: 0 1 2
    {

    }

    void getNeedUVsFaces(trimesh::TriMesh* mesh, std::vector<int>& faceIndex)
    {
        for (size_t i = 0; i < mesh->faceUVs.size(); i++)
        {
            trimesh::TriMesh::Face& f = mesh->faceUVs[i];
            if (f[0] < 0 || f[1] < 0 || f[2] < 0)
            {
                faceIndex.push_back(i);
            }
        }
    }

    bool checkIsBoundary(trimesh::TriMesh* mesh, int fn)
    {
        trimesh::TriMesh::Face& facross_edge = mesh->across_edge[fn];
        for (size_t i = 0; i < 3; i++)
        {
            trimesh::TriMesh::Face& fuv = mesh->faceUVs[facross_edge[i]];
            if (fuv.x >=0 || fuv.y >= 0 || fuv.z >= 0)
            {
                return true;
            }
        }
        return false;
    }

    void getNeedUVsFacesBoundary(trimesh::TriMesh* mesh, std::vector<int>& faceIndex)
    {
        for (size_t i = 0; i < mesh->faceUVs.size(); i++)
        {
            trimesh::TriMesh::Face& f = mesh->faceUVs[i];
            if (f[0] < 0 || f[1] < 0 || f[2] < 0)
            {
                if (checkIsBoundary(mesh,i))
                {
                    faceIndex.push_back(i);
                }              
            }
        }
    }

    int getsamePlaneFaceIndex(trimesh::TriMesh* mesh, int fn)
    {
        trimesh::TriMesh::Face& f = mesh->faces[fn];
        trimesh::TriMesh::Face& facross_edge = mesh->across_edge[fn];
        trimesh::vec3 nf = trimesh::normalized(trimesh::trinorm(mesh->vertices[f.x], mesh->vertices[f.y], mesh->vertices[f.z]));
        for (size_t i = 0; i < 3; i++)
        {
            trimesh::TriMesh::Face& facrossedgeOpposite = mesh->faces[facross_edge[i]];
            trimesh::vec3 nfacross = trimesh::normalized(trimesh::trinorm(mesh->vertices[facrossedgeOpposite.x], mesh->vertices[facrossedgeOpposite.y], mesh->vertices[facrossedgeOpposite.z]));
            if (std::abs(nf.at(0) - nfacross.at(0)) < 0.01
                && std::abs(nf.at(1) - nfacross.at(1)) < 0.01
                && std::abs(nf.at(2) - nfacross.at(2)) < 0.01)
            {
                return i;
            }
        }
        return 0;
    }

    void dealUVs(trimesh::TriMesh* mesh , int fn,int samePlaneFaceIndex ,bool needSameUVs)
    {
        trimesh::TriMesh::Face& facross_edge = mesh->across_edge[fn];
        trimesh::TriMesh::Face& f = mesh->faces[fn];
        trimesh::TriMesh::Face& fuv = mesh->faceUVs[fn];
        for (size_t j = 0; j < 3; j++)
        {//across edge 
            int n = (j + samePlaneFaceIndex) % 3;
            trimesh::TriMesh::Face & facrossedgeOpposite = mesh->faces[facross_edge[n]];
            int acrossfn = facross_edge[n];
            trimesh::TriMesh::Face & acrossedgeOpposite = mesh->across_edge[acrossfn];
            int accrossfnOpposite = 0;
            for (size_t k = 0; k < 3; k++)
            {
                if (acrossedgeOpposite[k] == fn)
                {
                    accrossfnOpposite = k;
                    break;
                }
            }

            if (needSameUVs)
            {

                //if (f[n] == facrossedgeOpposite[accrossfnOpposite])//Opposite
                {
                    int fuv1 = mesh->faceUVs[acrossfn][accrossfnOpposite];
                    if (fuv[n] < 0 && fuv1 >= 0)
                        fuv[n] = fuv1;
                }
            }

            if (f[(n + 1) % 3] == facrossedgeOpposite[(accrossfnOpposite + 2) % 3]
                && f[(n + 2) % 3] == facrossedgeOpposite[(accrossfnOpposite + 1) % 3])//Opposite
            {
                int fuv1 = mesh->faceUVs[acrossfn][(accrossfnOpposite + 2) % 3];
                int fuv2 = mesh->faceUVs[acrossfn][(accrossfnOpposite + 1) % 3];
                if (fuv[(n + 1) % 3] < 0 && fuv1 >=0 )
                    fuv[(n + 1) % 3] = fuv1;
                if (fuv[(n + 2) % 3] < 0 && fuv2 >=0)
                    fuv[(n + 2) % 3] = fuv2;
            }
        }
    }

    void addUVs1(trimesh::TriMesh* mesh, ccglobal::Tracer* tracer)
    {
        if (!mesh)
        {
            if (tracer)
                tracer->failed("mesh is empty");
            return;
        }

        if (mesh->faces.size() < mesh->faceUVs.size())
        {
            if (tracer)
                tracer->failed("faces.size() != faceUVs.size()");
            return;
        }

        if (mesh->faces.size() != mesh->faceUVs.size())
        {
            int len = mesh->faces.size() - mesh->faceUVs.size();
            mesh->faceUVs.insert(mesh->faceUVs.end(), len, trimesh::TriMesh::Face(-1,-1,-1));
        }

        mesh->clear_normals();
        mesh->need_normals();
        mesh->clear_across_edge();
        mesh->need_across_edge();

        std::vector<int> faceIndex;
        int preNeedUVsFacesNum = 0;
        bool needSameUVs = true;
        while (1)
        {
            faceIndex.clear();
            getNeedUVsFaces(mesh, faceIndex);
            if (preNeedUVsFacesNum == faceIndex.size()) //recycle
            {
                needSameUVs = true;
            }
            else
                needSameUVs = false;
            if (faceIndex.size() == mesh->faces.size())
            {
                if (tracer)
                    tracer->failed("faceUVs is empty");
                return;
            }
            if (faceIndex.empty())
            {
                return;
            }

            for (size_t i = 0; i < faceIndex.size(); i++)
            {
                int& f = faceIndex[i];
                if (checkIsBoundary(mesh, f))
                {
                    int samePlaneFaceIndex = getsamePlaneFaceIndex(mesh, f);
                    dealUVs(mesh, f, samePlaneFaceIndex, needSameUVs);
                }
            }
            preNeedUVsFacesNum = faceIndex.size();
        }
    }

    void facrossFace(trimesh::TriMesh* mesh, int nf, std::vector<bool>& isnewEdge)
    {
        trimesh::TriMesh::Face& f = mesh->faces[nf];
        trimesh::TriMesh::Face& facross_edge = mesh->across_edge[nf];

        int samePlaneFaceIndex = getsamePlaneFaceIndex(mesh, nf);
        int n = 0;
        for (size_t i = 0; i < 3; i++)
        {
            n = (i + samePlaneFaceIndex) % 3;
            if (mesh->faceUVs[facross_edge[n]].x >= 0 && isnewEdge[facross_edge[n]])
            {
                int acrossfn = facross_edge[n];
                trimesh::TriMesh::Face& acrossedgeOpposite = mesh->across_edge[acrossfn];
                for (size_t k = 0; k < 3; k++)
                {
                    if (acrossedgeOpposite[k] == nf)
                    {
                        int fuv0 = mesh->faceUVs[acrossfn][k];
                        int fuv1 = mesh->faceUVs[acrossfn][(k + 2) % 3];
                        int fuv2 = mesh->faceUVs[acrossfn][(k + 1) % 3];

                        float midPx = (mesh->UVs[fuv1].x + mesh->UVs[fuv2].x) / 2.0;
                        float midPy = (mesh->UVs[fuv1].y + mesh->UVs[fuv2].y) / 2.0;

                        float p0x = mesh->UVs[fuv0].x;
                        float p0y = mesh->UVs[fuv0].y;

                        float  px = midPx * 2.0 - p0x;
                        float  py = midPy * 2.0 - p0y;

                        if (px > 1 || px < 0 || py>1 || py < 0)
                        {
                            //px = px > 1 || px < 0 ? 0.0 : px;
                            //py = py > 1 || py < 0 ? 0.0 : py;
                            //mesh->faceUVs[nf][n] = mesh->UVs.size();
                            //mesh->UVs.push_back(trimesh::vec2(px, py));

                            //if (mesh->faceUVs[nf][(n + 1) % 3] < 0 && fuv1 >= 0)
                            //    mesh->faceUVs[nf][(n + 1) % 3] = fuv1;
                            //if (mesh->faceUVs[nf][(n + 2) % 3] < 0 && fuv2 >= 0)
                            //    mesh->faceUVs[nf][(n + 2) % 3] = fuv2;
                        }
                        else
                        {
                            mesh->faceUVs[nf][n] = mesh->UVs.size();
                            mesh->UVs.push_back(trimesh::vec2(px, py));

                            if (mesh->faceUVs[nf][(n + 1) % 3] < 0 && fuv1 >= 0)
                                mesh->faceUVs[nf][(n + 1) % 3] = fuv1;
                            if (mesh->faceUVs[nf][(n + 2) % 3] < 0 && fuv2 >= 0)
                                mesh->faceUVs[nf][(n + 2) % 3] = fuv2;

                            if (mesh->textureIDs[acrossfn] > -1)
                            {
                                mesh->textureIDs[nf] = mesh->textureIDs[acrossfn];
                            }

                            isnewEdge[nf] = true;
                            return;
                        }



                    }
                }
            }
        }
        return;
    }

    void facrossFace(trimesh::TriMesh* mesh,int nf)
    {
        trimesh::TriMesh::Face& f = mesh->faces[nf];
        trimesh::TriMesh::Face& facross_edge = mesh->across_edge[nf];

        int samePlaneFaceIndex = getsamePlaneFaceIndex(mesh, nf);
        int n = 0;
        for (size_t i = 0; i < 3; i++)
        {
            n = (i + samePlaneFaceIndex) % 3;
            if (mesh->faceUVs[facross_edge[n]].x >= 0)
            {
                int acrossfn = facross_edge[n];
                trimesh::TriMesh::Face& acrossedgeOpposite = mesh->across_edge[acrossfn];
                for (size_t k = 0; k < 3; k++)
                {
                    if (acrossedgeOpposite[k] == nf)
                    {
                        int fuv0 = mesh->faceUVs[acrossfn][k];
                        int fuv1 = mesh->faceUVs[acrossfn][(k + 2) % 3];
                        int fuv2 = mesh->faceUVs[acrossfn][(k + 1) % 3];

                        float midPx = (mesh->UVs[fuv1].x + mesh->UVs[fuv2].x) / 2.0;
                        float midPy = (mesh->UVs[fuv1].y + mesh->UVs[fuv2].y) / 2.0;

                        float p0x = mesh->UVs[fuv0].x;
                        float p0y = mesh->UVs[fuv0].y;

                        float  px = midPx * 2.0 - p0x;
                        float  py = midPy * 2.0 - p0y;

                        if (px>1 || px <0 || py>1 || py < 0)
                        {
                            //px = px > 1 || px < 0 ? 0.0 : px;
                            //py = py > 1 || py < 0 ? 0.0 : py;
                            //mesh->faceUVs[nf][n] = mesh->UVs.size();
                            //mesh->UVs.push_back(trimesh::vec2(px, py));

                            //if (mesh->faceUVs[nf][(n + 1) % 3] < 0 && fuv1 >= 0)
                            //    mesh->faceUVs[nf][(n + 1) % 3] = fuv1;
                            //if (mesh->faceUVs[nf][(n + 2) % 3] < 0 && fuv2 >= 0)
                            //    mesh->faceUVs[nf][(n + 2) % 3] = fuv2;
                        }
                        else
                        {
                            mesh->faceUVs[nf][n] = mesh->UVs.size();
                            mesh->UVs.push_back(trimesh::vec2(px, py));

                            if (mesh->faceUVs[nf][(n + 1) % 3] < 0 && fuv1 >= 0)
                                mesh->faceUVs[nf][(n + 1) % 3] = fuv1;
                            if (mesh->faceUVs[nf][(n + 2) % 3] < 0 && fuv2 >= 0)
                                mesh->faceUVs[nf][(n + 2) % 3] = fuv2;

                            if (mesh->textureIDs[acrossfn] > -1)
                            {
                                mesh->textureIDs[nf] = mesh->textureIDs[acrossfn];
                            }
                            return;
                        }


                        
                    }
                }
            }
        }
        return ;
    }

    void facrossFaceSame(trimesh::TriMesh* mesh, int nf)
    {
        trimesh::TriMesh::Face& f = mesh->faces[nf];
        trimesh::TriMesh::Face& facross_edge = mesh->across_edge[nf];

        int samePlaneFaceIndex = getsamePlaneFaceIndex(mesh, nf);
        int n = 0;
        for (size_t i = 0; i < 3; i++)
        {
            n = (i + samePlaneFaceIndex) % 3;
            if (mesh->faceUVs[facross_edge[n]].x >= 0)
            {
                int acrossfn = facross_edge[n];
                trimesh::TriMesh::Face& acrossedgeOpposite = mesh->across_edge[acrossfn];
                for (size_t k = 0; k < 3; k++)
                {
                    if (acrossedgeOpposite[k] == nf)
                    {
                        int fuv0 = mesh->faceUVs[acrossfn][k];
                        int fuv1 = mesh->faceUVs[acrossfn][(k + 2) % 3];
                        int fuv2 = mesh->faceUVs[acrossfn][(k + 1) % 3];

                        if (mesh->faceUVs[nf][(n + 1) % 3] < 0 && fuv1 >= 0)
                            mesh->faceUVs[nf][(n + 1) % 3] = fuv1;
                        if (mesh->faceUVs[nf][(n + 2) % 3] < 0 && fuv2 >= 0)
                            mesh->faceUVs[nf][(n + 2) % 3] = fuv2;
                        if (mesh->faceUVs[nf][n] < 0 && fuv0 >= 0)
                            mesh->faceUVs[nf][n] = fuv0;
                        return;
                    }
                }
            }
        }
        return;
    }

    void addUVs(trimesh::TriMesh* mesh, ccglobal::Tracer* tracer)
    {
        if (!mesh)
        {
            if (tracer)
                tracer->failed("mesh is empty");
            return;
        }

        if (mesh->faces.size() < mesh->faceUVs.size())
        {
            if (tracer)
                tracer->failed("faces.size() != faceUVs.size()");
            return;
        }

        std::vector<bool> isnewEdge(mesh->faceUVs.size(), true);

        if (mesh->faces.size() != mesh->faceUVs.size())
        {
            int len = mesh->faces.size() - mesh->faceUVs.size();
            mesh->faceUVs.insert(mesh->faceUVs.end(), len, trimesh::TriMesh::Face(-1, -1, -1));

            isnewEdge.insert(isnewEdge.end(), len, false);

            int lenId = mesh->faces.size() - mesh->textureIDs.size();
            mesh->textureIDs.insert(mesh->textureIDs.end(), lenId, 0);
        }

        mesh->clear_normals();
        mesh->need_normals();
        mesh->clear_across_edge();
        mesh->need_across_edge();

        //get first point;
        std::vector<int> faceIndex;
        bool firstCnt = true;
        while (1)
        {
            faceIndex.clear();
            getNeedUVsFacesBoundary(mesh, faceIndex);
            if (faceIndex.empty())
            {
                break;
            }          

            if (firstCnt)
            {
                for (size_t i = 0; i < faceIndex.size(); i++)
                {
                    facrossFace(mesh, faceIndex[0]);
                }

                isnewEdge[faceIndex[0]] = true;
                firstCnt = false;
            }
            else
            {
                for (size_t i = 0; i < faceIndex.size(); i++)
                {
                    facrossFaceSame(mesh, faceIndex[i]);
                }
            }   
        }
    }
}