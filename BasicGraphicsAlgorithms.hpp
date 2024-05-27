#ifndef BASIC_GRAPHICS_ALGORITHMS_H
#define BASIC_GRAPHICS_ALGORITHMS_H

#include <vector>
#include <set>
#include <unordered_map>
#include "DataType.hpp"



//基本图形学算法

class BasicGraphicsAlgorithms
{
public:
	/*
	* 功能：根据三点计算平面法向量
	* 输入参数：p1，p2，p3表示确定平面的三个点坐标
	* 返回值：平面法向量（非单位向量）
	*/
	static Vector3D CalculatePlaneNormal(const Vector3D& p1, const Vector3D& p2, const Vector3D& p3);

	/*
	* 功能：计算点在平面上的投影点
	* 输入参数：
	*	px：准备投影的点
	*	normal：投影平面法向量
	*	origin：投影平面上一个点
	* 返回值：投影点坐标
	*/
	static Vector3D CalculatePlaneProjectionPoint(const Vector3D& px, const Vector3D& normal, const Vector3D& origin);

	/*
	* 功能：判断输入的点是否共线
	* 输入参数：
	*	points：点坐标
	* 返回值：共线（true），不共线（false）
	*/
	static bool IsPointsCollinear(const std::vector<Vector3D>& points);

	/*
	* 功能：判断输入的点是否共面
	* 输入参数：
	*	points：点坐标
	* 返回值：共面（true），不共面（false）
	*/
	static bool IsPointsCoplanar(const std::vector<Vector3D>& points);

	/*
	* 功能：判断当前点在哪个单元内
	* 输入参数：
	* 	data：单元数据
	*   point：待查找的点坐标
	*   cell：返回查找的单元
	* 返回值：查找到当前点所在的单元（true）,没有查找到（false）
	*/
	// template<typename T>
	// static bool FindPointInCell(YJVisDataSet *data,const Vector3D& point,std::vector<std::pair<Vector3D, T>>& cell);

	/*
	* 功能：判断点是否在线段单元上
	* 输入参数：
	*   points：点集
	*   p：判断点坐标
	* 返回值：在（true），不在（false）
	*/
	static bool PointInLineSegmentCell(const std::vector<Vector3D>& points, const Vector3D& p);
	//输入参数：points单元点集，p待判断的点
	static bool PointInLineSegmentCell(const double* points, const Vector3D& p);
	/*
	* 功能：判断点是否在多边形单元上
	* 输入参数：
	*   points：点集
	*   p：判断点坐标
	* 返回值：在（true），不在（false），在边界上（2）
	*/
	static int PointInPolygonCell(const std::vector<Vector3D>& points, const Vector3D& p);
	//输入参数：points单元点集，p待判断的点，point_num点的个数
	static bool PointInPolygonCell(const double* points, const Vector3D& p, const int& point_num);

	/*
	* 功能：判断点是否在四面体单元上
	* 输入参数：
	*   points：点集
	*   p：判断点坐标
	* 返回值：在（true），不在（false）
	*/
	static bool PointInTetrahedronCell(const std::vector<Vector3D>& points, const Vector3D& p);
	static bool PointInTetrahedronCell(const double* points, const Vector3D& p, const int& point_num);
	/*
	* 功能：判断点是否在六面体单元上
	* 输入参数：
	*   points：点集
	*   p：判断点坐标
	* 返回值：在（true），不在（false）
	*/
	static bool PointInHexahedronCell(const std::vector<Vector3D>& points, const Vector3D& p);

	/*
	* 功能：判断点是否在三棱柱（楔）单元上
	* 输入参数：
	*   points：点集
	*   p：判断点坐标
	* 返回值：在（true），不在（false）
	*/
	static bool PointInWedgeCell(const std::vector<Vector3D>& points, const Vector3D& p);

	/*
	* 功能：判断点是否在四棱锥（金字塔）单元上
	* 输入参数：
	*   points：点集
	*   p：判断点坐标
	* 返回值：在（true），不在（false）
	*/
	static bool PointInPyramidCell(const std::vector<Vector3D>& points, const Vector3D& p);

	/*
	* 功能：判断点是否在多面体单元上
	* 输入参数：
	*   points：点集
	*   p：判断点坐标
	* 返回值：在（true），不在（false）
	*/
	static bool PointInPolyhedronCell(const std::vector<Vector3D>& points, const std::vector<std::vector<uint64_t>>& topology, const Vector3D& p);

	/*
	* 功能：计算射线和平面单元的交点
	* 输入参数：
	*   p：射线的起始点
	*   v：射线的方向向量
	*	points：点集
	* 返回值：无交点（0），有交点（1），射线的起始点在单元内（2）,射线过边界点（3）,传入的点构不成平面单元（4）
	*/
	static int RaySurfaceCellAcrossPoint(const Vector3D& p, const Vector3D& v, const std::vector<Vector3D>& points);

	/*
	* 功能：计算线和面的交点
	* 输入参数：
	*   surface_points：面上的不共线的三个点（不共线的三个点确定一个平面）
	*   line_points：线
	* 返回值：交点坐标，如果返回(NAN,NAN,NAN)表示无交点
	*/
	static Vector3D LineSurfaceAcrossPoint(const std::vector<Vector3D>& surface_points, const std::vector<Vector3D>& line_points);

	/*
	* 功能：计算线段单元重心
	* 输入参数：
	*   points：点集
	* 返回值：重心坐标
	*/
	static Vector3D LineSegmentCellBarycenter(const std::vector<Vector3D>& points);
	//输入参数：points单元点集
	static Vector3D LineSegmentCellBarycenter(const double* points);


	/*
	* 功能：计算三角形单元重心
	* 输入参数：
	*   points：点集
	* 返回值：重心坐标
	*/
	static Vector3D TriangleCellBarycenter(const std::vector<Vector3D>& points);
	//输入参数：points单元点集
	static Vector3D TriangleCellBarycenter(const double* points);

	/*
	* 功能：计算多边形（n>=4）单元重心
	* 输入参数：
	*   points：点集
	* 返回值：重心坐标
	*/
	static Vector3D PolygonCellBarycenter(const std::vector<Vector3D>& points);
	//输入参数：points单元点集，point_num点数
	static Vector3D PolygonCellBarycenter(const double* points, const int& point_num);

	/*
	* 功能：计算四面体单元重心
	* 输入参数：
	*   points：点集
	* 返回值：重心坐标
	*/
	static Vector3D TetrahedronCellBarycenter(const std::vector<Vector3D>& points);
	//输入参数：points单元点集
	static Vector3D TetrahedronCellBarycenter(const double* points);

	/*
	* 功能：计算三维多面体单元重心
	* 输入参数：
	*   points：点集
	* 返回值：重心坐标
	*/
	static Vector3D polyhedronCellBarycenter(const std::vector<Vector3D>& points);

	/*
	* 功能：vtk中计算线段单元中心
	* 输入参数：
	*	points：点集
	* 返回值：中心坐标
	*/
	static Vector3D VTKLineSegmentCellCentre(const std::vector<Vector3D>& points);
	static Vector3D VTKLineSegmentCellCentre(double* points);

	/*
	* 功能：vtk中计算三角形单元中心
	* 输入参数：
	*	points：点集
	* 返回值：中心坐标
	*/
	static Vector3D VTKTriangleCellCentre(const std::vector<Vector3D>& points);
	static Vector3D VTKTriangleCellCentre(double* points);

	/*
	* 功能：vtk中计算四边形单元中心
	* 输入参数：
	*	points：点集
	* 返回值：中心坐标
	*/
	static Vector3D VTKQuadrilateralCellCentre(const std::vector<Vector3D>& points);
	static Vector3D VTKQuadrilateralCellCentre(double* points);

	/*
	* 功能：vtk中计算多边形单元中心（n>4）
	* 输入参数：
	*	points：点集
	* 返回值：中心坐标
	*/
	static Vector3D VTKPolygonCellCentre(const std::vector<Vector3D>& points);
	static Vector3D VTKPolygonCellCentre(double* points, const int& point_num);

	/*
	* 功能：vtk中计算四面体单元中心
	* 输入参数：
	*	points：点集
	* 返回值：中心坐标
	*/
	static Vector3D VTKTetrahedronCellCentre(const std::vector<Vector3D>& points);
	static Vector3D VTKTetrahedronCellCentre(double* points);

	/*
	* 功能：vtk中计算六面体单元中心
	* 输入参数：
	*	points：点集
	* 返回值：中心坐标
	*/
	static Vector3D VTKHexahedronCellCentre(const std::vector<Vector3D>& points);
	static Vector3D VTKHexahedronCellCentre(double* points);

	/*
	* 功能：vtk中计算楔单元中心
	* 输入参数：
	*	points：点集
	* 返回值：中心坐标
	*/
	static Vector3D VTKWedgeCellCentre(const std::vector<Vector3D>& points);
	static Vector3D VTKWedgeCellCentre(double* points);

	/*
	* 功能：vtk中计算金字塔单元中心
	* 输入参数：
	*	points：点集
	* 返回值：中心坐标
	*/
	static Vector3D VTKPyramidCellCentre(const std::vector<Vector3D>& points);
	static Vector3D VTKPyramidCellCentre(double* points);

	/*
	* 功能：寻找与某个多边形相邻的多边形（这里的相邻指两个单元有共同的点，所以这个方法不会认为下图中AB是相邻的单元）
	*
	*  ----------------
	*  |              |
	*  |              |-------
	*  |       A      |      |
	*  |              |    B |
	*  |              |-------
	*  ----------------
	*  输入参数:
	*         cell_ptr：要查找其邻居的单元
	*         data:     包含某个顶点单单元集合的集合
	*         cell_points_num：当前单元的点数
	*         neighbor:查找的结果
	*/
	static void FindNeighborCell(uint64_t* cell_ptr, uint32_t* cell_points_num, const std::unordered_map<uint64_t, const std::vector<CellInfo>>& data, std::vector<CellInfo>& neighbor);
};
#endif
