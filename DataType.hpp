#ifndef CUSTOM_DATA_TYPE_H
#define CUSTOM_DATA_TYPE_H

#include <cmath>
#include <vector>
#include <stdint.h>
#include <time.h>
#define PI 3.141592653589793

//����ε�Ԫ��ֵ����
enum class PolygonInterpolationMethod
{
    MVC,          //��ֵ����ϵ��ֵ��
    DISTANCE      //��Ȩ�����ֵ��
};

//��Ԫ����
enum class CellType
{
    LineSegmentCell,    //�߶ε�Ԫ
    QuadrilateralCell,  //�ı��ε�Ԫ
    TriangleCell,       //�����ε�Ԫ
    PolygonCell,        //����ε�Ԫ
    TetrahedronCell,    //�����嵥Ԫ
    HexahedronCell,     //�����嵥Ԫ
    TriangularPrism,    //��������Ш����Ԫ
    Pyramid             //����׶������������Ԫ
};

//΢�ַ�������㷽��
enum class DifferentialEquation
{
    OL,        //ŷ����
    RK2,       //2�����������
    RK4        //4�����������
};

//��άʸ������������ά��Ϊ0��Ҳ���Ա�ʾ��άʸ��
struct Vector3D
{
    Vector3D() {}
    Vector3D(double xx, double yy, double zz) :x(xx), y(yy), z(zz) {}
    Vector3D(double value)
    {
        x = value;
        y = value;
        z = value;
    };
    double x = 0;
    double y = 0;
    double z = 0;
    //����ģ��
    double MouldLength() const
    {
        return std::sqrt(x * x + y * y + z * z);
    }
    //���㵥λ����
    Vector3D UnitVector() const
    {
        double len = MouldLength();
        return len != 0 ? Vector3D(x / len, y / len, z / len) : NAN;
    }
    //���ؼ��������
    Vector3D operator-(const Vector3D& v) const
    {
        return Vector3D(x - v.x, y - v.y, z - v.z);
    }
    //���ؼӷ������
    Vector3D operator+(const Vector3D& v) const
    {
        return Vector3D(x + v.x, y + v.y, z + v.z);
    }
    //���س˷������
    Vector3D operator*(double n) const
    {
        return Vector3D(x * n, y * n, z * n);
    }
    //���س��������
    Vector3D operator/(double n) const
    {
        return Vector3D(x / n, y / n, z / n);
    }
    //���س��������
    double operator/(const Vector3D& v) const
    {
        std::vector<double> k;
        if (v.x != 0)
        {
            double kx = x / v.x;
            k.push_back(kx);
        }
        else
        {
            if (x != 0)
            {
                return NAN;
            }
        }

        if (v.y != 0)
        {
            double ky = y / v.y;
            k.push_back(ky);
        }
        else
        {
            if (y != 0)
            {
                return NAN;
            }
        }

        if (v.z != 0)
        {
            double kz = z / v.z;
            k.push_back(kz);
        }
        else
        {
            if (z != 0)
            {
                return NAN;
            }
        }

        int size = k.size();
        if (size > 1)
        {
            for (int i = 1; i < size; i++)
            {
                if (fabs(k.at(1) - k.at(i)) > 1.0e-8)
                {
                    //������������ı�ֵ����ȣ�˵������������������ͬ���෴
                    return NAN;
                }
            }
        }
        return k.at(1);
    }
    //�������������
    double operator[](int n) const
    {
        if (n < 0 || n>2)
        {
            //�׳��쳣������Խ��
            throw "access is out of bounds";
        }
        else
        {
            if (n == 0)
                return x;
            else if (n == 1)
                return y;
            else
                return z;
        }
    }
    //�����������
    double PointMultiplication(const Vector3D& v) const
    {
        return x * v.x + y * v.y + z * v.z;
    }
    //�����������
    Vector3D ForkMultiplication(const Vector3D& v) const
    {
        return Vector3D(y * v.z - v.y * z, v.x * z - v.z * x, x * v.y - v.x * y);
    }
    //�������������ļн�
    //����ֵ����
    double GetAngle(const Vector3D& v) const
    {
        double lens = v.MouldLength() * MouldLength();
        double cos_angle = PointMultiplication(v) / lens;
        if (cos_angle < -1)
            cos_angle = -1;
        else if (cos_angle > 1)
            cos_angle = 1;
        return acos(cos_angle);
    }
};

struct TrajectoryVector
{
    //�켣��
    std::vector<Vector3D> points;
    //�켣�����Ƿ����
    bool end = false;
};

struct VectorResult
{
    Vector3D pos;    //ʸ����ʾλ��
    Vector3D vector; //ʸ��
};

struct CellInfo
{
    CellInfo(uint64_t* ptr, uint32_t num) :start_ptr(ptr), points_num(num) {};
    uint64_t* start_ptr;  //��Ԫ��ʼ��ַ
    uint32_t points_num;  //��Ԫ����
};

#endif
