#ifndef CUSTOM_DATA_TYPE_H
#define CUSTOM_DATA_TYPE_H

#include <cmath>
#include <vector>
#include <stdint.h>
#include <time.h>
#define PI 3.141592653589793

//多边形单元插值方法
enum class PolygonInterpolationMethod
{
    MVC,          //均值坐标系插值法
    DISTANCE      //加权距离插值法
};

//单元类型
enum class CellType
{
    LineSegmentCell,    //线段单元
    QuadrilateralCell,  //四边形单元
    TriangleCell,       //三角形单元
    PolygonCell,        //多边形单元
    TetrahedronCell,    //四面体单元
    HexahedronCell,     //六面体单元
    TriangularPrism,    //三棱柱（楔）单元
    Pyramid             //四棱锥（金字塔）单元
};

//微分方程组解算方法
enum class DifferentialEquation
{
    OL,        //欧拉法
    RK2,       //2阶龙格库塔法
    RK4        //4阶龙格库塔法
};

//三维矢量，当第三个维度为0，也可以表示二维矢量
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
    //计算模长
    double MouldLength() const
    {
        return std::sqrt(x * x + y * y + z * z);
    }
    //计算单位向量
    Vector3D UnitVector() const
    {
        double len = MouldLength();
        return len != 0 ? Vector3D(x / len, y / len, z / len) : NAN;
    }
    //重载减法运算符
    Vector3D operator-(const Vector3D& v) const
    {
        return Vector3D(x - v.x, y - v.y, z - v.z);
    }
    //重载加法运算符
    Vector3D operator+(const Vector3D& v) const
    {
        return Vector3D(x + v.x, y + v.y, z + v.z);
    }
    //重载乘法运算符
    Vector3D operator*(double n) const
    {
        return Vector3D(x * n, y * n, z * n);
    }
    //重载除法运算符
    Vector3D operator/(double n) const
    {
        return Vector3D(x / n, y / n, z / n);
    }
    //重载除法运算符
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
                    //出现两个坐标的比值不相等，说明两个向量方向不是相同或相反
                    return NAN;
                }
            }
        }
        return k.at(1);
    }
    //重载索引运算符
    double operator[](int n) const
    {
        if (n < 0 || n>2)
        {
            //抛出异常，索引越界
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
    //两个向量点乘
    double PointMultiplication(const Vector3D& v) const
    {
        return x * v.x + y * v.y + z * v.z;
    }
    //两个向量叉乘
    Vector3D ForkMultiplication(const Vector3D& v) const
    {
        return Vector3D(y * v.z - v.y * z, v.x * z - v.z * x, x * v.y - v.x * y);
    }
    //计算两个向量的夹角
    //返回值弧度
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
    //轨迹点
    std::vector<Vector3D> points;
    //轨迹计算是否结束
    bool end = false;
};

struct VectorResult
{
    Vector3D pos;    //矢量显示位置
    Vector3D vector; //矢量
};

struct CellInfo
{
    CellInfo(uint64_t* ptr, uint32_t num) :start_ptr(ptr), points_num(num) {};
    uint64_t* start_ptr;  //单元起始地址
    uint32_t points_num;  //单元点数
};

#endif
