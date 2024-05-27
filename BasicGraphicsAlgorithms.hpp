#ifndef BASIC_GRAPHICS_ALGORITHMS_H
#define BASIC_GRAPHICS_ALGORITHMS_H

#include <vector>
#include <set>
#include <unordered_map>
#include "DataType.hpp"



//����ͼ��ѧ�㷨

class BasicGraphicsAlgorithms
{
public:
	/*
	* ���ܣ������������ƽ�淨����
	* ���������p1��p2��p3��ʾȷ��ƽ�������������
	* ����ֵ��ƽ�淨�������ǵ�λ������
	*/
	static Vector3D CalculatePlaneNormal(const Vector3D& p1, const Vector3D& p2, const Vector3D& p3);

	/*
	* ���ܣ��������ƽ���ϵ�ͶӰ��
	* ���������
	*	px��׼��ͶӰ�ĵ�
	*	normal��ͶӰƽ�淨����
	*	origin��ͶӰƽ����һ����
	* ����ֵ��ͶӰ������
	*/
	static Vector3D CalculatePlaneProjectionPoint(const Vector3D& px, const Vector3D& normal, const Vector3D& origin);

	/*
	* ���ܣ��ж�����ĵ��Ƿ���
	* ���������
	*	points��������
	* ����ֵ�����ߣ�true���������ߣ�false��
	*/
	static bool IsPointsCollinear(const std::vector<Vector3D>& points);

	/*
	* ���ܣ��ж�����ĵ��Ƿ���
	* ���������
	*	points��������
	* ����ֵ�����棨true���������棨false��
	*/
	static bool IsPointsCoplanar(const std::vector<Vector3D>& points);

	/*
	* ���ܣ��жϵ�ǰ�����ĸ���Ԫ��
	* ���������
	* 	data����Ԫ����
	*   point�������ҵĵ�����
	*   cell�����ز��ҵĵ�Ԫ
	* ����ֵ�����ҵ���ǰ�����ڵĵ�Ԫ��true��,û�в��ҵ���false��
	*/
	// template<typename T>
	// static bool FindPointInCell(YJVisDataSet *data,const Vector3D& point,std::vector<std::pair<Vector3D, T>>& cell);

	/*
	* ���ܣ��жϵ��Ƿ����߶ε�Ԫ��
	* ���������
	*   points���㼯
	*   p���жϵ�����
	* ����ֵ���ڣ�true�������ڣ�false��
	*/
	static bool PointInLineSegmentCell(const std::vector<Vector3D>& points, const Vector3D& p);
	//���������points��Ԫ�㼯��p���жϵĵ�
	static bool PointInLineSegmentCell(const double* points, const Vector3D& p);
	/*
	* ���ܣ��жϵ��Ƿ��ڶ���ε�Ԫ��
	* ���������
	*   points���㼯
	*   p���жϵ�����
	* ����ֵ���ڣ�true�������ڣ�false�����ڱ߽��ϣ�2��
	*/
	static int PointInPolygonCell(const std::vector<Vector3D>& points, const Vector3D& p);
	//���������points��Ԫ�㼯��p���жϵĵ㣬point_num��ĸ���
	static bool PointInPolygonCell(const double* points, const Vector3D& p, const int& point_num);

	/*
	* ���ܣ��жϵ��Ƿ��������嵥Ԫ��
	* ���������
	*   points���㼯
	*   p���жϵ�����
	* ����ֵ���ڣ�true�������ڣ�false��
	*/
	static bool PointInTetrahedronCell(const std::vector<Vector3D>& points, const Vector3D& p);
	static bool PointInTetrahedronCell(const double* points, const Vector3D& p, const int& point_num);
	/*
	* ���ܣ��жϵ��Ƿ��������嵥Ԫ��
	* ���������
	*   points���㼯
	*   p���жϵ�����
	* ����ֵ���ڣ�true�������ڣ�false��
	*/
	static bool PointInHexahedronCell(const std::vector<Vector3D>& points, const Vector3D& p);

	/*
	* ���ܣ��жϵ��Ƿ�����������Ш����Ԫ��
	* ���������
	*   points���㼯
	*   p���жϵ�����
	* ����ֵ���ڣ�true�������ڣ�false��
	*/
	static bool PointInWedgeCell(const std::vector<Vector3D>& points, const Vector3D& p);

	/*
	* ���ܣ��жϵ��Ƿ�������׶������������Ԫ��
	* ���������
	*   points���㼯
	*   p���жϵ�����
	* ����ֵ���ڣ�true�������ڣ�false��
	*/
	static bool PointInPyramidCell(const std::vector<Vector3D>& points, const Vector3D& p);

	/*
	* ���ܣ��жϵ��Ƿ��ڶ����嵥Ԫ��
	* ���������
	*   points���㼯
	*   p���жϵ�����
	* ����ֵ���ڣ�true�������ڣ�false��
	*/
	static bool PointInPolyhedronCell(const std::vector<Vector3D>& points, const std::vector<std::vector<uint64_t>>& topology, const Vector3D& p);

	/*
	* ���ܣ��������ߺ�ƽ�浥Ԫ�Ľ���
	* ���������
	*   p�����ߵ���ʼ��
	*   v�����ߵķ�������
	*	points���㼯
	* ����ֵ���޽��㣨0�����н��㣨1�������ߵ���ʼ���ڵ�Ԫ�ڣ�2��,���߹��߽�㣨3��,����ĵ㹹����ƽ�浥Ԫ��4��
	*/
	static int RaySurfaceCellAcrossPoint(const Vector3D& p, const Vector3D& v, const std::vector<Vector3D>& points);

	/*
	* ���ܣ������ߺ���Ľ���
	* ���������
	*   surface_points�����ϵĲ����ߵ������㣨�����ߵ�������ȷ��һ��ƽ�棩
	*   line_points����
	* ����ֵ���������꣬�������(NAN,NAN,NAN)��ʾ�޽���
	*/
	static Vector3D LineSurfaceAcrossPoint(const std::vector<Vector3D>& surface_points, const std::vector<Vector3D>& line_points);

	/*
	* ���ܣ������߶ε�Ԫ����
	* ���������
	*   points���㼯
	* ����ֵ����������
	*/
	static Vector3D LineSegmentCellBarycenter(const std::vector<Vector3D>& points);
	//���������points��Ԫ�㼯
	static Vector3D LineSegmentCellBarycenter(const double* points);


	/*
	* ���ܣ����������ε�Ԫ����
	* ���������
	*   points���㼯
	* ����ֵ����������
	*/
	static Vector3D TriangleCellBarycenter(const std::vector<Vector3D>& points);
	//���������points��Ԫ�㼯
	static Vector3D TriangleCellBarycenter(const double* points);

	/*
	* ���ܣ��������Σ�n>=4����Ԫ����
	* ���������
	*   points���㼯
	* ����ֵ����������
	*/
	static Vector3D PolygonCellBarycenter(const std::vector<Vector3D>& points);
	//���������points��Ԫ�㼯��point_num����
	static Vector3D PolygonCellBarycenter(const double* points, const int& point_num);

	/*
	* ���ܣ����������嵥Ԫ����
	* ���������
	*   points���㼯
	* ����ֵ����������
	*/
	static Vector3D TetrahedronCellBarycenter(const std::vector<Vector3D>& points);
	//���������points��Ԫ�㼯
	static Vector3D TetrahedronCellBarycenter(const double* points);

	/*
	* ���ܣ�������ά�����嵥Ԫ����
	* ���������
	*   points���㼯
	* ����ֵ����������
	*/
	static Vector3D polyhedronCellBarycenter(const std::vector<Vector3D>& points);

	/*
	* ���ܣ�vtk�м����߶ε�Ԫ����
	* ���������
	*	points���㼯
	* ����ֵ����������
	*/
	static Vector3D VTKLineSegmentCellCentre(const std::vector<Vector3D>& points);
	static Vector3D VTKLineSegmentCellCentre(double* points);

	/*
	* ���ܣ�vtk�м��������ε�Ԫ����
	* ���������
	*	points���㼯
	* ����ֵ����������
	*/
	static Vector3D VTKTriangleCellCentre(const std::vector<Vector3D>& points);
	static Vector3D VTKTriangleCellCentre(double* points);

	/*
	* ���ܣ�vtk�м����ı��ε�Ԫ����
	* ���������
	*	points���㼯
	* ����ֵ����������
	*/
	static Vector3D VTKQuadrilateralCellCentre(const std::vector<Vector3D>& points);
	static Vector3D VTKQuadrilateralCellCentre(double* points);

	/*
	* ���ܣ�vtk�м������ε�Ԫ���ģ�n>4��
	* ���������
	*	points���㼯
	* ����ֵ����������
	*/
	static Vector3D VTKPolygonCellCentre(const std::vector<Vector3D>& points);
	static Vector3D VTKPolygonCellCentre(double* points, const int& point_num);

	/*
	* ���ܣ�vtk�м��������嵥Ԫ����
	* ���������
	*	points���㼯
	* ����ֵ����������
	*/
	static Vector3D VTKTetrahedronCellCentre(const std::vector<Vector3D>& points);
	static Vector3D VTKTetrahedronCellCentre(double* points);

	/*
	* ���ܣ�vtk�м��������嵥Ԫ����
	* ���������
	*	points���㼯
	* ����ֵ����������
	*/
	static Vector3D VTKHexahedronCellCentre(const std::vector<Vector3D>& points);
	static Vector3D VTKHexahedronCellCentre(double* points);

	/*
	* ���ܣ�vtk�м���Ш��Ԫ����
	* ���������
	*	points���㼯
	* ����ֵ����������
	*/
	static Vector3D VTKWedgeCellCentre(const std::vector<Vector3D>& points);
	static Vector3D VTKWedgeCellCentre(double* points);

	/*
	* ���ܣ�vtk�м����������Ԫ����
	* ���������
	*	points���㼯
	* ����ֵ����������
	*/
	static Vector3D VTKPyramidCellCentre(const std::vector<Vector3D>& points);
	static Vector3D VTKPyramidCellCentre(double* points);

	/*
	* ���ܣ�Ѱ����ĳ����������ڵĶ���Σ����������ָ������Ԫ�й�ͬ�ĵ㣬�����������������Ϊ��ͼ��AB�����ڵĵ�Ԫ��
	*
	*  ----------------
	*  |              |
	*  |              |-------
	*  |       A      |      |
	*  |              |    B |
	*  |              |-------
	*  ----------------
	*  �������:
	*         cell_ptr��Ҫ�������ھӵĵ�Ԫ
	*         data:     ����ĳ�����㵥��Ԫ���ϵļ���
	*         cell_points_num����ǰ��Ԫ�ĵ���
	*         neighbor:���ҵĽ��
	*/
	static void FindNeighborCell(uint64_t* cell_ptr, uint32_t* cell_points_num, const std::unordered_map<uint64_t, const std::vector<CellInfo>>& data, std::vector<CellInfo>& neighbor);
};
#endif
