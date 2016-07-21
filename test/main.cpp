#include "geom/algorithms.h"

#include <iostream>
#include <memory>

typedef std::vector<Eigen::Vector3d> T_Point;
typedef T_Point::const_iterator CIT_Point;
typedef std::vector<Eigen::Vector2d> T_Point2D;
typedef T_Point2D::const_iterator CIT_Point2D;

void ErrorIfDifferent(const Eigen::VectorXd& a, const Eigen::VectorXd& b, const std::string& in, int& err)
{
    if(a != b)
    {
        err = -1;
        std::cout << "In " << in << "expected ("
                  <<  a(0) << ", " << a(1)
                  <<"), got ("
                  <<  b(0) << ", " << b(1)
                  << ")" << std::endl;
    }
}

const std::size_t size = 11;
char scr [size][size];

void clear()
{
    for(std::size_t i = 0; i <size; ++i)
    {
        for(std::size_t j = 0; j <size; ++j)
        {
            scr[i][j] = '-';
        }
    }
}

void drawMatrix()
{
    for(std::size_t i = 0; i <size; ++i)
    {
        for(std::size_t j = 0; j <size; ++j)
        {
            std::cout << " " << scr[j][i];
        }
        std::cout <<  std::endl;
    }
    std::cout <<  std::endl;
}

void loadHull(const T_Point& hull, const char sign)
{
    // increment of 0.2
    // starting from -1
    for(std::size_t i = 0; i <size; ++i)
    {
        double x = i * 0.2 -1;
        for(std::size_t j = 0; j <size; ++j)
        {
            double y = j * 0.2 -1;
            if(geom::containsHull(hull.begin(), hull.end(), Eigen::Vector3d(x,y,0)))
            {
                scr[i][j] = sign;
            }
        }
    }
}

void hullGiftWrapping2DTest(int& ret)
{
    T_Point2D points;
    points.push_back(Eigen::Vector2d(1,1));
    points.push_back(Eigen::Vector2d(-1,1));
    points.push_back(Eigen::Vector2d(-1,-1));
    points.push_back(Eigen::Vector2d(1,-1));
    points.push_back(Eigen::Vector2d(0,0));
    T_Point2D hull = geom::hullGiftWrapping<T_Point2D, 2>(points.begin(), points.end());
    T_Point2D expectedHull;
    expectedHull.push_back(Eigen::Vector2d(-1,1));
    expectedHull.push_back(Eigen::Vector2d(1,1));
    expectedHull.push_back(Eigen::Vector2d(1,-1));
    expectedHull.push_back(Eigen::Vector2d(-1,-1));
    expectedHull.push_back(Eigen::Vector2d(-1,1));
    if(hull.size() != expectedHull.size())
    {
        std::cout << "error, expected and found hull do not have the same size" << std::endl;
        ret = -1;
    }
     CIT_Point2D cit2 = expectedHull.begin();
    for(CIT_Point2D cit = hull.begin();
        cit != hull.end() && cit2 != expectedHull.end(); ++cit, ++cit2)
    {
        ErrorIfDifferent(*cit2, *cit, "Convex Hull test: ", ret);
    }
}

void hullGiftWrappingTest(int& ret)
{
    T_Point points;
    points.push_back(Eigen::Vector3d(1,1,0));
    points.push_back(Eigen::Vector3d(-1,1,0));
    points.push_back(Eigen::Vector3d(-1,-1,0));
    points.push_back(Eigen::Vector3d(1,-1,0));
    points.push_back(Eigen::Vector3d(0,0,0));
    T_Point hull = geom::hullGiftWrapping<T_Point>(points.begin(), points.end());
    T_Point expectedHull;
    expectedHull.push_back(Eigen::Vector3d(-1,1,0));
    expectedHull.push_back(Eigen::Vector3d(1,1,0));
    expectedHull.push_back(Eigen::Vector3d(1,-1,0));
    expectedHull.push_back(Eigen::Vector3d(-1,-1,0));
    expectedHull.push_back(Eigen::Vector3d(-1,1,0));
     CIT_Point cit2 = expectedHull.begin();
    if(hull.size() != expectedHull.size())
    {
        std::cout << "error, expected and found hull do not have the same size" << std::endl;
        ret = -1;
    }
    for(CIT_Point cit = hull.begin();
        cit != hull.end() && cit2 != expectedHull.end(); ++cit, ++cit2)
    {
        ErrorIfDifferent(*cit2, *cit, "Convex Hull 2D test: ", ret);
    }
}

void ContainsTest(int& ret)
{
    std::vector<bool> expected;
    expected.push_back(true);
    expected.push_back(true);
    expected.push_back(true);
    expected.push_back(false);
    T_Point tested;
    tested.push_back(Eigen::Vector3d(0,0,0));
    tested.push_back(Eigen::Vector3d(0.499,0.5,0));
    tested.push_back(Eigen::Vector3d(0,0.999,0));
    tested.push_back(Eigen::Vector3d(-1.0000001,0,0));

    T_Point points;
    points.push_back(Eigen::Vector3d(-1,0,0));
    points.push_back(Eigen::Vector3d(0,1,0));
    points.push_back(Eigen::Vector3d(1,0,0));
    points.push_back(Eigen::Vector3d(0,-1,0));
    points.push_back(Eigen::Vector3d(0,0,0));
    T_Point hull = geom::hullGiftWrapping<T_Point>(points.begin(), points.end());
    for(std::size_t i = 0; i < tested.size(); ++i)
    {
        if(geom::containsHull(hull.begin(), hull.end(), tested[i]) != expected[i])
        {
            ret = -1;
            std::cout << "Failed in Contains test, for point " << i << std::endl;
        }
    }

    loadHull(hull, '+');
    drawMatrix();
    clear();
}

void hullGiftWrappingIntersectionTest(int& ret)
{
    T_Point a, b;
    a.push_back(Eigen::Vector3d(-1,0,0));
    a.push_back(Eigen::Vector3d(-0.5,0.5,0));
    a.push_back(Eigen::Vector3d(0.5,0,0));
    a.push_back(Eigen::Vector3d(-0.5,-1,0));

    b.push_back(Eigen::Vector3d(-0.6,-1,0));
    b.push_back(Eigen::Vector3d(-0.6,1,0));
    b.push_back(Eigen::Vector3d(1,1,0));
    b.push_back(Eigen::Vector3d(1,-1,0));
    T_Point ha = geom::hullGiftWrapping<T_Point>(a.begin(), a.end());
    T_Point hb = geom::hullGiftWrapping<T_Point>(b.begin(), b.end());
    loadHull(ha, '+');
    drawMatrix();
    clear();
    loadHull(hb, '/');
    drawMatrix();
    loadHull(ha, '+');
    T_Point inter = geom::computeIntersection<T_Point, 3, double, Eigen::Vector3d,
            Eigen::Ref<Eigen::Vector3d>&,
            const Eigen::Ref<const Eigen::Vector3d>&, T_Point::const_iterator>(ha.begin(),ha.end(),hb.begin(),hb.end());

    loadHull(inter, '*');
    drawMatrix();
    clear();

    for(std::size_t i = 0; i < 10000; ++i)
    {
        Eigen::Vector3d test;
        test[0] = 2*((double)(rand()) / (double)(RAND_MAX) - 0.5);
        test[1] = 2*((double)(rand()) / (double)(RAND_MAX) - 0.5);
        bool inInter =geom::containsHull(inter.begin(), inter.end(),test);
        bool inEach= geom::containsHull(hb.begin(), hb.end(), test) &&
                geom::containsHull(ha.begin(), ha.end(), test);
        if(inInter && !inEach)
        {
            ret = -1;
            std::cout << "Failed in hullGiftWrappingIntersectionTest, false positive " << test << std::endl;
        }
        if(!inInter && inEach)
        {
            ret = -1;
            std::cout << "Failed in hullGiftWrappingIntersectionTest, false negative " << test << std::endl;
        }
    }
}

void hullGiftWrappingIntersectionNoIntersectionTest(int& ret)
{
    T_Point a, b;
    a.push_back(Eigen::Vector3d(-1,1,0));
    a.push_back(Eigen::Vector3d(-0,1,0));
    a.push_back(Eigen::Vector3d(0,0,0));
    a.push_back(Eigen::Vector3d(-1,0,0));


    b.push_back(Eigen::Vector3d(0.1,1,0));
    b.push_back(Eigen::Vector3d(1,1,0));
    b.push_back(Eigen::Vector3d(1,0,0));
    b.push_back(Eigen::Vector3d(0.1,0,0));
    T_Point ha = geom::hullGiftWrapping<T_Point>(a.begin(), a.end());
    T_Point hb = geom::hullGiftWrapping<T_Point>(b.begin(), b.end());
    loadHull(ha, '+');
    drawMatrix();
    clear();
    loadHull(hb, '/');
    drawMatrix();
    loadHull(ha, '+');
    T_Point inter = geom::computeIntersection<T_Point, 3, double, Eigen::Vector3d,
            Eigen::Ref<Eigen::Vector3d>&,
            const Eigen::Ref<const Eigen::Vector3d>&, T_Point::const_iterator>(ha.begin(),ha.end(),hb.begin(),hb.end());

    //clear();
    loadHull(inter, '*');
    drawMatrix();
    clear();

    if(!inter.empty())
    {
        ret = -1;
        std::cout << "Failed in hullGiftWrappingIntersectionNoIntersectionTest, intersection should be empty" << std::endl;
    }
}

// TODO NORM DOT CROSS EN 2D

int main()
{
    srand(0);
    std::cout << "running test cases..." << std::endl;
    int ret = 0;
    clear();
    hullGiftWrappingTest(ret);
    hullGiftWrapping2DTest(ret);
    ContainsTest(ret);
    hullGiftWrappingIntersectionTest(ret);
    hullGiftWrappingIntersectionNoIntersectionTest(ret);
    if(ret == -1)
        std::cout << "test exited with errors" << std::endl;
    else
        std::cout << "test exited normally" << std::endl;
    return ret;
}
