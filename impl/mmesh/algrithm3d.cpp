#include "algrithm3d.h"

#define ARC_PER_BLOCK 5
namespace mmesh
{
	bool rayIntersectTriangle(const trimesh::vec3& orig, const trimesh::vec3& dir,
        trimesh::vec3& v0, trimesh::vec3& v1, trimesh::vec3& v2, trimesh::vec3& baryPosition)
	{
        auto E1 = v1 - v0;
        auto E2 = v2 - v0;

        auto P = trimesh::cross(dir, E2);
        auto det = trimesh::dot(E1, P);
        auto s = orig - v0;

        if(det < 0)
        {
            det = -det;
            s = v0 - orig;
        }
        if (det < 0.0001f)
            return false;

        auto f = 1.0f / det;
        auto dSP = trimesh::dot(s, P);
        baryPosition.at(0) = f * dSP;
        if (baryPosition.at(0) < 0.0f)
            return false;
        if (baryPosition.at(0) > 1.0f)
            return false;
        auto q = trimesh::cross(s, E1);
        auto dDQ = trimesh::dot(dir, q);
        baryPosition.y = f * dDQ;
        if (baryPosition.at(1) < 0.0f)
            return false;
        if (baryPosition.at(1) + baryPosition.at(0) > 1.0f)
            return false;
        auto dEQ = trimesh::dot(E2, q);
        baryPosition.at(2) = f * dEQ;
        return baryPosition.at(2) >= 0.0f;
	}

	bool rayIntersectTriangle(const trimesh::vec3& orig, const trimesh::vec3& dir,
		trimesh::vec3& v0, trimesh::vec3& v1, trimesh::vec3& v2, float* t, float* u, float* v)
	{
		trimesh::vec3 E1 = v1 - v0;
		trimesh::vec3 E2 = v2 - v0;

		trimesh::vec3 P = trimesh::cross(dir, E2);
		float det = trimesh::dot(E1, P);

		// keep det > 0, modify T accordingly
		trimesh::vec3 T;
		if (det > 0)
		{
			T = orig - v0;
		}
		else
		{
			T = v0 - orig;
			det = -det;
		}

		// If determinant is near zero, ray lies in plane of triangle
		if (det < 0.0001f)
			return false;

		// Calculate u and make sure u <= 1
		*u = trimesh::dot(T, P);
		if (*u < 0.0f || *u > det)
			return false;

		trimesh::vec3 Q = trimesh::cross(T, E1);

		// Calculate v and make sure u + v <= 1
		*v = trimesh::dot(dir, Q);
		if (*v < 0.0f || *u + *v > det)
			return false;

		// Calculate t, scale parameters, ray intersects triangle
		*t = trimesh::dot(E2, Q);

		float fInvDet = 1.0f / det;
		*t *= fInvDet;
		*u *= fInvDet;
		*v *= fInvDet;

		return true;
	}

	bool rayIntersectPlane(const trimesh::vec3& orig, const trimesh::vec3& dir, trimesh::vec3& vertex, trimesh::vec3& normal, float& t)
	{
		float l = trimesh::dot(normal, dir);
		if (l == 0.0f) return false;

		t = trimesh::dot(normal, (vertex - orig)) / l;
		return true;
	}

	bool PointinTriangle(const trimesh::vec3& A, const trimesh::vec3& B, const trimesh::vec3& C, const trimesh::vec3& P)
	{
		trimesh::vec3 v0 = C - A;
		trimesh::vec3 v1 = B - A;
		trimesh::vec3 v2 = P - A;

		float dot00 = v0.dot(v0);
		float dot01 = v0.dot(v1);
		float dot02 = v0.dot(v2);
		float dot11 = v1.dot(v1);
		float dot12 = v1.dot(v2);

		float inverDeno = 1.0 / (dot00 * dot11 - dot01 * dot01);

		float u = (dot11 * dot02 - dot01 * dot12) * inverDeno;
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

	trimesh::box3 extendBox(const trimesh::box3& b, float r)
	{
		if (r <= 0.0f)
			r = 0.1f;

		trimesh::box3 rb = b;
		trimesh::vec3 offset = rb.size() * r;
		rb += (b.max + offset);
		rb += (b.min - offset);
		return rb;
	}

	float GetArea(const trimesh::vec3& A, const trimesh::vec3& B, const trimesh::vec3& C)
	{
		//double ab1 = A.distanceToPoint(B); 
		//double ac1 = A.distanceToPoint(C);
		//double bc1 = B.distanceToPoint(C);

		double ab = trimesh::distance(A, B);
		double ac = trimesh::distance(A, C);
		double bc = trimesh::distance(B, C);

		double p = (ab + bc + ac) / 2;
		double t = p * (p - ab) * (p - ac) * (p - bc);
		if (t < 0)
		{
			t = 0;
		}
		return sqrt(t);
	}


	void offsetPoints(std::vector<trimesh::vec3>& points, const trimesh::vec3& offset)
	{
		for (trimesh::vec3& point : points)
			point += offset;
	}

	void applyMatrix2Points(std::vector<trimesh::vec3>& points, const trimesh::fxform& xf)
	{
		for (trimesh::vec3& point : points)
			point = xf * point;
	}

	trimesh::box3 pointsBox(const std::vector<trimesh::vec3>& points)
	{
		trimesh::box3 box;
		for (const trimesh::vec3& point : points)
			box += point;
		return box;
	}

	float getAngelOfTwoVector(const trimesh::vec& pt1, const trimesh::vec& pt2, const trimesh::vec& c)
	{
		float theta = atan2(pt1.y - c.y, pt1.x - c.x) - atan2(pt2.y - c.y, pt2.x - c.x);
		if (theta > M_PIf)
			theta -= 2 * M_PIf;
		if (theta < -M_PIf)
			theta += 2 * M_PIf;

		theta = theta * 180.0 / M_PIf;
		if (theta < 0)
		{
			theta = 360 + theta;
		}
		return theta;
	}

	void getDevidePoint(const trimesh::vec& p0, const trimesh::vec& p1,
		std::vector<trimesh::vec>& out, float theta, bool clockwise)
	{
		if (theta <= 0.02f)
		{
			return;
		}

		//int x = 1, y = 2;//��ת�ĵ�
		//int dx = 1, dy = 1;//��������ת�ĵ�

		float block = ARC_PER_BLOCK;

		int count = theta > block ? theta / block : 2;
		float angle = theta > block ? block : theta / 2.0;

		if (theta < block)
		{
			block = theta / 2.0;
		}
		//int count = theta / ARC_PER_BLOCK;
		//int angle = ARC_PER_BLOCK;
		out.reserve(count);
		for (int i = 1; i < count; i++)
		{
			//if (clockwise)
			//{
			//    angle = -15 * i;
			//}
			//else
			//{
			//    angle = 15 * i;
			//}
			////int angle = 45 * i;//��ʱ��
			////int angle = -15 * i;//˳ʱ��
			//trimesh::vec _out = p0;
			//_out.x = (p0.x - p1.x) * cos(angle * PI / 180) - (p0.y - p1.y) * sin(angle * PI / 180) + p1.x;
			//_out.y = (p0.y - p1.y) * cos(angle * PI / 180) + (p0.x - p1.x) * sin(angle * PI / 180) + p1.y;
			trimesh::vec _out = p0;
			if (clockwise)
			{
				angle = block * i;
				_out.x = (p1.x - p0.x) * cos(angle * M_PIf / 180) + (p1.y - p0.y) * sin(angle * M_PIf / 180) + p0.x;
				_out.y = (p1.y - p0.y) * cos(angle * M_PIf / 180) - (p1.x - p0.x) * sin(angle * M_PIf / 180) + p0.y;
			}
			else
			{
				angle = block * i;

				_out.x = (p1.x - p0.x) * cos(angle * M_PIf / 180) - (p1.y - p0.y) * sin(angle * M_PIf / 180) + p0.x;
				_out.y = (p1.y - p0.y) * cos(angle * M_PIf / 180) + (p1.x - p0.x) * sin(angle * M_PIf / 180) + p0.y;

			}

			out.push_back(_out);
		}
	}
}