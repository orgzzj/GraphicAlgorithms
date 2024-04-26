#include "BasicGraphicsAlgorithms.hpp"

Vector3D BasicGraphicsAlgorithms::CalculatePlaneNormal(const Vector3D& p1, const Vector3D& p2, const Vector3D& p3)
{
	//算法原理参考链接：https://blog.csdn.net/sinat_41104353/article/details/84963016

	double y21 = p2.y - p1.y;
	double z31 = p3.z - p1.z;
	double y31 = p3.y - p1.y;
	double z21 = p2.z - p1.z;
	double x31 = p3.x - p1.x;
	double x21 = p2.x - p1.x;

	double a = y21 * z31 - y31 * z21;
	double b = z21 * x31 - z31 * x21;
	double c = x21 * y31 - x31 * y21;

	return Vector3D(a, b, c);
}

Vector3D BasicGraphicsAlgorithms::CalculatePlaneProjectionPoint(const Vector3D& px, const Vector3D& normal, const Vector3D& origin)
{
	//算法原理参考链接：https://zhuanlan.zhihu.com/p/594862976

	Vector3D origin_to_px = px - origin;
	double len_normal = normal.MouldLength();
	if (len_normal == 0.0)
	{
		//平面法向量为0向量，即平面不存在，所以投影点不存在
		return NAN;
	}
	double t = (normal.PointMultiplication(origin_to_px)) / (normal.PointMultiplication(normal));
	return px - normal * t;
}

bool BasicGraphicsAlgorithms::IsPointsCollinear(const std::vector<Vector3D>& points)
{
	if (points.size() <= 2)
		return true;

	/*
	* 判断方式如下：
	* 在点集中任选两个不重合的点p1和p2，计算p1指向p2的方向向量n12
	* 在计算p1指向其他点pi的方向向量n1i，如果n1i=λ*n12，则说明p1，p2，pi三点共线，
	* 如果p1到集合任意一点的方向向量都满足n1i=λi*n12则说明所有点都共线，
	* 但凡有三个点不共线，例如p1，p2，p3不共线，那么n13!=λ*n12
	*/

	Vector3D init_dir;
	Vector3D init_pos = points.at(0);
	bool is_find_init_dir = false;
	for (int i = 1; i < points.size(); i++)
	{
		Vector3D dir = points.at(i) - init_pos;

		if (!is_find_init_dir)
		{
			if (fabs(dir.MouldLength()) > 1.0e-8)
			{
				//p1和pi两点不重合
				is_find_init_dir = true;
				init_dir = dir;
			}
		}
		else
		{
			double k = init_dir / dir;
			if (std::isnan(k))
			{
				//两条向量方向不相同/相反
				return false;
			}
		}
	}

	return true;
}

bool BasicGraphicsAlgorithms::IsPointsCoplanar(const std::vector<Vector3D>& points)
{

	return false;
}

bool BasicGraphicsAlgorithms::PointInLineSegmentCell(const std::vector<Vector3D>& points, const Vector3D& p)
{
	if (points.size() != 2)
		return false;
	const Vector3D& p0 = points.at(0);
	const Vector3D& p1 = points.at(1);
	double r = (p - p0) / (p1 - p0);
	return r >= 0 && r <= 1 ? true : false;
}

bool BasicGraphicsAlgorithms::PointInLineSegmentCell(const double* points, const Vector3D& p)
{
	if (!points)
		return false;

	// 创建两个端点的Vector3D
	const Vector3D p0 = Vector3D(points[0], points[1], points[2]);
	const Vector3D p1 = Vector3D(points[3], points[4], points[5]);
	double r = (p - p0) / (p1 - p0);

	return r >= 0 && r <= 1 ? true : false;
}

int BasicGraphicsAlgorithms::PointInPolygonCell(const std::vector<Vector3D>& points, const Vector3D& p)
{
	int n = points.size();
	if (n < 3)
		return false;
	Vector3D positive_dir;
	double sum_angle = 0;
	for (int i = 0; i <= n - 1; i++)
	{
		//判断点p是否为多边形顶点points
		if ((points[i] - p).MouldLength() < 1.0e-5) {
			return 1;
		}

		Vector3D n1 = points.at(i) - p;
		Vector3D n2 = points.at((i + 1) % n) - p;
		Vector3D forkn = n1.ForkMultiplication(n2);
		if (i == 0)
			positive_dir = forkn.UnitVector();
		//计算两个向量夹角
		double angle = n1.GetAngle(n2);
		if (i != 0)
		{
			//判断夹角正负
			double cos_angle = forkn.PointMultiplication(positive_dir) / forkn.MouldLength();
			if (cos_angle < 0)
				angle = -angle;
		}
		else
		{
			positive_dir = forkn.UnitVector();
		}
		// 如果夹角接近180度，则认为点在边界上
		if (fabs(angle - PI) < 1.0e-5)
			return 2;

		sum_angle += angle;
	}

	return fabs(fabs(sum_angle) - 2 * PI) < 1.0e-5 ? 1 : 0;
}

bool BasicGraphicsAlgorithms::PointInPolygonCell(const double* points, const Vector3D& p, const int& point_num)
{
	if (point_num < 3 || !points)
		return false;
	Vector3D positive_dir;
	double sum_angle = 0;
	for (int i = 0; i < point_num; i++)
	{
		Vector3D n1(points[3 * i] - p.x, points[3 * i + 1] - p.y, points[3 * i + 2] - p.z);
		Vector3D n2(points[3 * ((i + 1) % point_num)] - p.x, points[3 * ((i + 1) % point_num) + 1] - p.y, points[3 * ((i + 1) % point_num) + 2] - p.z);
		Vector3D forkn = n1.ForkMultiplication(n2);
		//计算两个向量夹角
		double angle = n1.GetAngle(n2);
		if (i != 0)
		{
			//判断夹角正负
			double cos_angle = forkn.PointMultiplication(positive_dir) / forkn.MouldLength();
			if (cos_angle < 0)
				angle = -angle;
		}
		else
		{
			positive_dir = forkn.UnitVector();
		}
		sum_angle += angle;
	}

	return fabs(fabs(sum_angle) - 2 * PI) < 1.0e-5 ? true : false;

}

bool BasicGraphicsAlgorithms::PointInTetrahedronCell(const std::vector<Vector3D>& points, const Vector3D& p)
{
	return false;
}

bool BasicGraphicsAlgorithms::PointInTetrahedronCell(const double* points, const Vector3D& p, const int& point_num) {

	if (point_num != 4)
		return false;

	Vector3D vertices[4];
	for (int i = 0; i < 4; ++i)
	{
		vertices[i] = Vector3D(points[3 * i], points[3 * i + 1], points[3 * i + 2]);
	}

	for (int i = 0; i < 4; ++i)
	{
		// 找到除去i点的其余三点
		Vector3D& a = vertices[(i + 1) % 4];
		Vector3D& b = vertices[(i + 2) % 4];
		Vector3D& c = vertices[(i + 3) % 4];
		Vector3D edge1 = a - vertices[i];
		Vector3D edge2 = b - vertices[i];
		Vector3D normal = edge1.ForkMultiplication(edge2);
		Vector3D vectorToPoint = p - vertices[i];
		double dot = normal.PointMultiplication(vectorToPoint);
		if (dot < 0)
		{
			return false;
		}
	}
	return true;
}

bool BasicGraphicsAlgorithms::PointInHexahedronCell(const std::vector<Vector3D>& points, const Vector3D& p)
{
	return false;
}

bool BasicGraphicsAlgorithms::PointInWedgeCell(const std::vector<Vector3D>& points, const Vector3D& p)
{
	return false;
}

bool BasicGraphicsAlgorithms::PointInPyramidCell(const std::vector<Vector3D>& points, const Vector3D& p)
{
	return false;
}

bool BasicGraphicsAlgorithms::PointInPolyhedronCell(const std::vector<Vector3D>& points, const std::vector<std::vector<uint64_t>>& topology, const Vector3D& p)
{
	if (points.size() < 3)
		return false;

	// 初始化随机种子
	srand(static_cast<unsigned int>(time(0)));

	//生成随机方向向量的射线
	double azimuth = ((double)rand() / RAND_MAX) * 2 * PI;  // 方位角 [0, 2π)
	double z = ((double)rand() / RAND_MAX) * 2 - 1.0;        // Z值 [-1, 1]
	double theta = acos(z);                                   // 天顶角 [0, π]
	double x = sin(theta) * cos(azimuth);
	double y = sin(theta) * sin(azimuth);
	Vector3D ray = Vector3D(x, y, z);

	int intersections = 0;

	//寻找每个面的点索引
	for (const auto& sequence : topology)
	{
		std::vector<Vector3D> pts;

		//通过点索引寻找点坐标
		for (const auto& pointIndex : sequence)
		{
			if (pointIndex < points.size())
			{
				pts.push_back(points[pointIndex]);
			}
			else
				return false;
		}

		int type = RaySurfaceCellAcrossPoint(p, ray, pts);

		// 若type等于3，增加扰动量改变射线方向，直到type不等于3
		while (type == 3)
		{
			// 初始化随机种子
			srand(static_cast<unsigned int>(time(0)));
			Vector3D ray_new = ray;
			//生成随机方向向量的射线
			double azimuth = ((double)rand() / RAND_MAX) * 2 * PI;  // 方位角 [0, 2π)
			double dz = ((double)rand() / RAND_MAX) * 2 - 1.0;        // Z值 [-1, 1]
			double theta = acos(z);                                   // 天顶角 [0, π]
			double dx = sin(theta) * cos(azimuth);
			double dy = sin(theta) * sin(azimuth);

			ray_new = ray + Vector3D(dx, dy, dz);
			type = RaySurfaceCellAcrossPoint(p, ray_new, pts);

		}
		if (type == 2)
			return 1;
		else if (type == 1)
			intersections++;

	}
	return (intersections % 2) == 1;
}

int BasicGraphicsAlgorithms::RaySurfaceCellAcrossPoint(const Vector3D& p, const Vector3D& v, const std::vector<Vector3D>& points)
{
	//http://give.zju.edu.cn/cgcourse/new/frame/index.html 第二章五节

	if (points.size() < 3)
		return 4;
	Vector3D x;
	Vector3D normal = CalculatePlaneNormal(points[0], points[1], points[2]).UnitVector();

	//判断射线所在直线与平面单元所在的平面是否相交
	double denominator = v.PointMultiplication(normal);
	//射线源点与平面的距离
	double numerator = (points[0] - p).PointMultiplication(normal);

	//判断射线的起始点是否在单元内
	if (abs(numerator) <= 1.0e-06 && PointInPolygonCell(points, p))
		return 2;

	if (abs(denominator) <= 1.0e-06)
		return 0;

	//参数变量可参照vtkPlane.cxx的IntersectWithLine函数。
	double t = numerator / denominator;

	//t>0射线与平面的交点在射线的正方向上
	if (t > 0)
	{
		//计算交点坐标
		x = p + v * t;

		int type = PointInPolygonCell(points, x);
		//交点在平面单元边界上
		if (type == 2)
			return 3;

		else if (type == 0)
			return 0;

		return 1;
	}
	else
		return 0;
}

Vector3D BasicGraphicsAlgorithms::LineSurfaceAcrossPoint(const std::vector<Vector3D>& surface_points, const std::vector<Vector3D>& line_points)
{
	return Vector3D();
}

Vector3D BasicGraphicsAlgorithms::LineSegmentCellBarycenter(const std::vector<Vector3D>& points)
{
	if (points.size() != 2)
		return NAN;
	return (points[0] + points[1]) / 2;
}

Vector3D BasicGraphicsAlgorithms::LineSegmentCellBarycenter(const double* points)
{
	if (!points)
		return NAN;
	const Vector3D p1 = Vector3D(points[0], points[1], points[2]);
	const Vector3D p2 = Vector3D(points[3], points[4], points[5]);
	return (p1 + p2) / 2;
}

Vector3D BasicGraphicsAlgorithms::TriangleCellBarycenter(const std::vector<Vector3D>& points)
{
	if (points.size() != 3)
		return NAN;
	const Vector3D& p1 = points[0];
	const Vector3D& p2 = points[1];
	const Vector3D& p3 = points[2];
	return (p1 + p2 + p3) / 3;
}

Vector3D BasicGraphicsAlgorithms::TriangleCellBarycenter(const double* points)
{
	if (!points)
		return NAN;
	const Vector3D p1 = Vector3D(points[0], points[1], points[2]);
	const Vector3D p2 = Vector3D(points[3], points[4], points[5]);
	const Vector3D p3 = Vector3D(points[6], points[7], points[8]);
	return (p1 + p2 + p3) / 3;

}

Vector3D BasicGraphicsAlgorithms::PolygonCellBarycenter(const std::vector<Vector3D>& points)
{
	int n = points.size();
	if (n < 4)
		return NAN;
	double S = 0;
	Vector3D barycenter;
	const Vector3D& p1 = points[0];
	//先假设第一个划分三角形两条边叉乘结果方向为正方向，对应的三角形面积是正数
	//依次遍历每个划分的三角形，如果方向某个三角形两条边叉乘结果方向是负方向，对应的三角形面积是负数
	Vector3D positive_dir;
	for (int i = 1; i < n - 1; i++)
	{
		const Vector3D& p2 = points[i];
		const Vector3D& p3 = points[i + 1];
		//计算三角形面积
		Vector3D n1 = p2 - p1;
		Vector3D n2 = p3 - p1;
		Vector3D forkn = n1.ForkMultiplication(n2);
		double cell_s = forkn.MouldLength() / 2;
		if (i != 1)
		{
			//判断正负
			//计算当前划分的三角形叉乘后的向量与正方向的方向是相同还是相反
			double cos_angle = positive_dir.PointMultiplication(forkn) / forkn.MouldLength();
			//共面的多边形中向量叉乘的结果方向要么是同向，要么是反向，所以在这个问题里面反向向量判断可以为跟正向向量的夹角大于90
			//如果大于90°就可以认为反向
			if (cos_angle < 0)
				cell_s = -cell_s;
		}
		else
		{
			positive_dir = forkn.UnitVector();
		}
		S += cell_s;
		//计算当前划分的三角形重心坐标
		barycenter = barycenter + (p1 + p2 + p3) * cell_s / 3;
	}

	return barycenter / S;
}

Vector3D BasicGraphicsAlgorithms::PolygonCellBarycenter(const double* points, const int& point_num)
{
	if (point_num < 4 || !points)
		return NAN;
	double S = 0;
	Vector3D barycenter;
	const Vector3D p1 = Vector3D(points[0], points[1], points[2]);
	//先假设第一个划分三角形两条边叉乘结果方向为正方向，对应的三角形面积是正数
	//依次遍历每个划分的三角形，如果方向某个三角形两条边叉乘结果方向是负方向，对应的三角形面积是负数
	Vector3D positive_dir;
	for (int i = 1; i < point_num - 1; i++)
	{
		const Vector3D p2 = Vector3D(points[3 * i], points[3 * i + 1], points[3 * i + 2]);
		const Vector3D p3 = Vector3D(points[3 * i + 3], points[3 * i + 4], points[3 * i + 5]);
		//计算三角形面积
		Vector3D n1 = p2 - p1;
		Vector3D n2 = p3 - p1;
		Vector3D forkn = n1.ForkMultiplication(n2);
		double cell_s = forkn.MouldLength() / 2;
		if (i != 1)
		{
			//判断正负
			//计算当前划分的三角形叉乘后的向量与正方向的方向是相同还是相反
			double cos_angle = positive_dir.PointMultiplication(forkn) / forkn.MouldLength();
			//共面的多边形中向量叉乘的结果方向要么是同向，要么是反向，所以在这个问题里面反向向量判断可以为跟正向向量的夹角大于90
			//如果大于90°就可以认为反向
			if (cos_angle < 0)
				cell_s = -cell_s;
		}
		else
		{
			positive_dir = forkn.UnitVector();
		}
		S += cell_s;
		//计算当前划分的三角形重心坐标
		barycenter = barycenter + (p1 + p2 + p3) * cell_s / 3;
	}
	return barycenter / S;


}

Vector3D BasicGraphicsAlgorithms::TetrahedronCellBarycenter(const std::vector<Vector3D>& points)
{
	if (points.size() != 4)
		return NAN;
	const Vector3D& p1 = points[0];
	const Vector3D& p2 = points[1];
	const Vector3D& p3 = points[2];
	const Vector3D& p4 = points[3];
	Vector3D barycenter = (p2 + p3 + p4) / 3;
	Vector3D n = barycenter - p1;
	double d = n.MouldLength();
	n = n.UnitVector();
	return p1 + n * 0.75 * d;
}

Vector3D  BasicGraphicsAlgorithms::TetrahedronCellBarycenter(const double* points)
{
	if (!points)
		return NAN;
	const Vector3D p1 = Vector3D(points[0], points[1], points[2]);
	const Vector3D p2 = Vector3D(points[3], points[4], points[5]);
	const Vector3D p3 = Vector3D(points[6], points[7], points[8]);
	const Vector3D p4 = Vector3D(points[9], points[10], points[11]);
	Vector3D barycenter = (p2 + p3 + p4) / 3;
	Vector3D n = barycenter - p1;
	double d = n.MouldLength();
	n = n.UnitVector();
	return p1 + n * 0.75 * d;

}

Vector3D BasicGraphicsAlgorithms::polyhedronCellBarycenter(const std::vector<Vector3D>& points)
{
	return Vector3D();
}

Vector3D BasicGraphicsAlgorithms::VTKLineSegmentCellCentre(const std::vector<Vector3D>& points)
{
	if (points.size() != 2)
		return NAN;
	return (points[0] + points[1]) / 2;
}

Vector3D BasicGraphicsAlgorithms::VTKLineSegmentCellCentre(double* points)
{
	if (!points)
		return NAN;
	Vector3D p1(points[0], points[1], points[2]);
	Vector3D p2(points[3], points[4], points[5]);
	return (p1 + p2) / 2;
}

Vector3D BasicGraphicsAlgorithms::VTKTriangleCellCentre(const std::vector<Vector3D>& points)
{
	if (points.size() != 3)
		return NAN;
	return (points[0] + points[1] + points[2]) / 3;
}

Vector3D BasicGraphicsAlgorithms::VTKTriangleCellCentre(double* points)
{
	if (!points)
		return NAN;
	Vector3D p1(points[0], points[1], points[2]);
	Vector3D p2(points[3], points[4], points[5]);
	Vector3D p3(points[6], points[7], points[8]);
	return (p1 + p2 + p3) / 3;
}

Vector3D BasicGraphicsAlgorithms::VTKQuadrilateralCellCentre(const std::vector<Vector3D>& points)
{
	if (points.size() != 4)
		return NAN;
	return (points[0] + points[1] + points[2] + points[3]) / 3;
}

Vector3D BasicGraphicsAlgorithms::VTKQuadrilateralCellCentre(double* points)
{
	if (!points)
		return NAN;
	Vector3D p1(points[0], points[1], points[2]);
	Vector3D p2(points[3], points[4], points[5]);
	Vector3D p3(points[6], points[7], points[8]);
	Vector3D p4(points[9], points[10], points[11]);
	return (p1 + p2 + p3 + p4) / 4;
}

Vector3D BasicGraphicsAlgorithms::VTKPolygonCellCentre(const std::vector<Vector3D>& points)
{
	int point_num = points.size();
	if (point_num < 5)
		return NAN;
	double r = 0.5;
	double s = 0.5;
	//多边形第一个点为坐标原点，第二个点与第一个点构成s轴，t=n x s
	Vector3D p1 = points[0];
	Vector3D p2 = points[1];
	Vector3D p3 = points[2];
	Vector3D normal = BasicGraphicsAlgorithms::CalculatePlaneNormal(p1, p2, p3);
	Vector3D s_dir = p2 - p1;
	Vector3D t_dir = normal.ForkMultiplication(s_dir);
	double smin = 0;
	double smax = 0;
	double tmin = 0;
	double tmax = 0;
	for (int i = 1; i < point_num; i++)
	{
		Vector3D p = points[i];
		Vector3D dir = p - p1;
		double s_value = dir.PointMultiplication(s_dir) / s_dir.MouldLength();
		double t_value = dir.PointMultiplication(t_dir) / t_dir.MouldLength();
		smax = s_value > smax ? s_value : smax;
		smin = s_value < smin ? s_value : smin;
		tmax = t_value > tmax ? t_value : tmax;
		tmin = t_value < tmin ? t_value : tmin;
	}
	//校正ost坐标系，满足在o's't'坐标系中各个点的st坐标各个分量在[0,1]之间
	Vector3D new_origin = p1 + s_dir * smin + t_dir * tmin;
	Vector3D new_s = p1 + s_dir * smax + t_dir * tmin - new_origin;
	Vector3D new_t = p1 + s_dir * smin + t_dir * tmax - new_origin;

	return new_origin + new_s * 0.5 + new_t * 0.5;
}

Vector3D BasicGraphicsAlgorithms::VTKPolygonCellCentre(double* points, const int& point_num)
{
	if (!points || point_num < 5)
		return NAN;
	double r = 0.5;
	double s = 0.5;
	//多边形第一个点为坐标原点，第二个点与第一个点构成s轴，t=n x s
	Vector3D p1(points[0], points[1], points[2]);
	Vector3D p2(points[3], points[4], points[5]);
	Vector3D p3(points[6], points[7], points[8]);
	Vector3D normal = BasicGraphicsAlgorithms::CalculatePlaneNormal(p1, p2, p3);
	Vector3D s_dir = p2 - p1;
	Vector3D t_dir = normal.ForkMultiplication(s_dir);
	double smin = 0;
	double smax = 0;
	double tmin = 0;
	double tmax = 0;
	for (int i = 1; i < point_num; i++)
	{
		Vector3D p(points[i * 3], points[i * 3 + 1], points[i * 3 + 2]);
		Vector3D dir = p - p1;
		double s_value = dir.PointMultiplication(s_dir) / s_dir.MouldLength();
		double t_value = dir.PointMultiplication(t_dir) / t_dir.MouldLength();
		smax = s_value > smax ? s_value : smax;
		smin = s_value < smin ? s_value : smin;
		tmax = t_value > tmax ? t_value : tmax;
		tmin = t_value < tmin ? t_value : tmin;
	}
	//校正ost坐标系，满足在o's't'坐标系中各个点的st坐标各个分量在[0,1]之间
	Vector3D new_origin = p1 + s_dir * smin + t_dir * tmin;
	Vector3D new_s = p1 + s_dir * smax + t_dir * tmin - new_origin;
	Vector3D new_t = p1 + s_dir * smin + t_dir * tmax - new_origin;

	return new_origin + new_s * 0.5 + new_t * 0.5;
}

Vector3D BasicGraphicsAlgorithms::VTKTetrahedronCellCentre(const std::vector<Vector3D>& points)
{
	if (points.size() != 4)
		return NAN;
	Vector3D p1 = points[0];
	Vector3D p2 = points[1];
	Vector3D p3 = points[2];
	Vector3D p4 = points[3];
	double r = 0.25;
	double s = 0.25;
	double t = 0.25;
	return p1 * (1 - r - s - t) + p2 * r + p3 * s + p4 * t;
}

Vector3D BasicGraphicsAlgorithms::VTKTetrahedronCellCentre(double* points)
{
	if (!points)
		return NAN;
	Vector3D p1(points[0], points[1], points[2]);
	Vector3D p2(points[3], points[4], points[5]);
	Vector3D p3(points[6], points[7], points[8]);
	Vector3D p4(points[9], points[10], points[11]);
	double r = 0.25;
	double s = 0.25;
	double t = 0.25;
	return p1 * (1 - r - s - t) + p2 * r + p3 * s + p4 * t;
}

Vector3D BasicGraphicsAlgorithms::VTKHexahedronCellCentre(const std::vector<Vector3D>& points)
{
	if (points.size() != 8)
		return NAN;
	Vector3D p1 = points[0];
	Vector3D p2 = points[1];
	Vector3D p3 = points[2];
	Vector3D p4 = points[3];
	Vector3D p5 = points[4];
	Vector3D p6 = points[5];
	Vector3D p7 = points[6];
	Vector3D p8 = points[7];
	double r = 0.5;
	double s = 0.5;
	double t = 0.5;
	return p1 * (1 - r) * (1 - s) * (1 - t) + p2 * r * (1 - s) * (1 - t) + p3 * r * s * (1 - t) + p4 * (1 - r) * s * (1 - t) + p5 * (1 - r) * (1 - s) * t + p6 * r * (1 - s) * t +
		p7 * r * s * t + p8 * (1 - r) * s * t;
}

Vector3D BasicGraphicsAlgorithms::VTKHexahedronCellCentre(double* points)
{
	if (!points)
		return NAN;
	Vector3D p1(points[0], points[1], points[2]);
	Vector3D p2(points[3], points[4], points[5]);
	Vector3D p3(points[6], points[7], points[8]);
	Vector3D p4(points[9], points[10], points[11]);
	Vector3D p5(points[12], points[13], points[14]);
	Vector3D p6(points[15], points[16], points[17]);
	Vector3D p7(points[18], points[19], points[20]);
	Vector3D p8(points[21], points[22], points[23]);
	double r = 0.5;
	double s = 0.5;
	double t = 0.5;
	return p1 * (1 - r) * (1 - s) * (1 - t) + p2 * r * (1 - s) * (1 - t) + p3 * r * s * (1 - t) + p4 * (1 - r) * s * (1 - t) + p5 * (1 - r) * (1 - s) * t + p6 * r * (1 - s) * t +
		p7 * r * s * t + p8 * (1 - r) * s * t;
}

Vector3D BasicGraphicsAlgorithms::VTKWedgeCellCentre(const std::vector<Vector3D>& points)
{
	if (points.size() != 6)
		return NAN;
	Vector3D p1 = points[0];
	Vector3D p2 = points[1];
	Vector3D p3 = points[2];
	Vector3D p4 = points[3];
	Vector3D p5 = points[4];
	Vector3D p6 = points[5];
	double r = 0.333333;
	double s = 0.333333;
	double t = 0.5;
	return p1 * (1 - r - s) * (1 - t) + p2 * r * (1 - t) + p3 * s * (1 - t) + p4 * (1 - r - s) * t + p5 * r * t + p6 * s * t;
}

Vector3D BasicGraphicsAlgorithms::VTKWedgeCellCentre(double* points)
{
	if (!points)
		return NAN;
	Vector3D p1(points[0], points[1], points[2]);
	Vector3D p2(points[3], points[4], points[5]);
	Vector3D p3(points[6], points[7], points[8]);
	Vector3D p4(points[9], points[10], points[11]);
	Vector3D p5(points[12], points[13], points[14]);
	Vector3D p6(points[15], points[16], points[17]);
	double r = 0.333333;
	double s = 0.333333;
	double t = 0.5;
	return p1 * (1 - r - s) * (1 - t) + p2 * r * (1 - t) + p3 * s * (1 - t) + p4 * (1 - r - s) * t + p5 * r * t + p6 * s * t;
}

Vector3D BasicGraphicsAlgorithms::VTKPyramidCellCentre(const std::vector<Vector3D>& points)
{
	if (points.size() != 5)
		return NAN;
	Vector3D p1 = points[0];
	Vector3D p2 = points[1];
	Vector3D p3 = points[2];
	Vector3D p4 = points[3];
	Vector3D p5 = points[4];
	double r = 0.4;
	double s = 0.4;
	double t = 0.2;
	return p1 * (1 - r) * (1 - s) * (1 - t) + p2 * r * (1 - s) * (1 - t) + p3 * r * s * (1 - t) + p4 * (1 - r) * s * (1 - t) + p5 * t;
}

Vector3D BasicGraphicsAlgorithms::VTKPyramidCellCentre(double* points)
{
	if (!points)
		return NAN;
	Vector3D p1(points[0], points[1], points[2]);
	Vector3D p2(points[3], points[4], points[5]);
	Vector3D p3(points[6], points[7], points[8]);
	Vector3D p4(points[9], points[10], points[11]);
	Vector3D p5(points[12], points[13], points[14]);
	double r = 0.4;
	double s = 0.4;
	double t = 0.2;
	return p1 * (1 - r) * (1 - s) * (1 - t) + p2 * r * (1 - s) * (1 - t) + p3 * r * s * (1 - t) + p4 * (1 - r) * s * (1 - t) + p5 * t;
}

void BasicGraphicsAlgorithms::FindNeighborCell(uint64_t* cell_ptr, uint32_t* cell_points_num, const std::unordered_map<uint64_t, const std::vector<CellInfo>>& data, std::vector<CellInfo>& neighbor)
{
	neighbor.clear();
	std::unordered_map<uint64_t*, uint32_t> neighbor_map;
	//获取当前单元每个点ID，获取包含每个点的单元
	//如果出现当某个邻居单元与其有两个重合点，那么他将会被添加两次，这样就是采用unordered_map/map好处，如果是vector还需要去重
	for (int i = 0; i < *cell_points_num; i++)
	{
		auto iter = data.find(cell_ptr[i]);
		if (iter != data.end())
		{
			auto cells_info = iter->second;
			for (const auto& info : cells_info)
			{
				neighbor_map.emplace(info.start_ptr, info.points_num);
			}
		}
	}

	for (const auto& info : neighbor_map)
	{
		neighbor.emplace_back(info.first, info.second);
	}
}

// template <typename T>
// inline bool BasicGraphicsAlgorithms::FindPointInCell(YJVisDataSet *data, const Vector3D &point, std::vector<std::pair<Vector3D, T>> &cell)
// {
//     return false;
// }