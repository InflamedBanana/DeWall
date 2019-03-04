#include <iostream>
#include "Vect3.h"
//#include "Triangle.h"
#include <vector>
#include "Math3D.h"
#include "Delaunay.h"
using namespace std;


template<typename T> void ShowTest( const T& _toShow )
{
	cout << _toShow << endl;
}

void Vect3Test()
{
  auto test1 = Vect3<float>();
  auto test2 = Vect3<float>(.25f);
  auto test3 = Vect3<float>(1.f, 2.f, 3.f);
  auto test4 = test3;
  auto test5(Vect3<float>(5.f, 2.f, 3.f));

  auto test6 = test1 + test2;
  auto test7 = test1 - test2;
  auto test8 = test2;
  auto test9 = test3 / 2.f;
  auto test10 = test4;
  test10 *= 2.f;
  auto test11 = test4;
  test11 /= 2.f;

  cout << test1 << endl;
  cout << test2 << endl;
  cout << test3 << endl;
  cout << test4 << endl;
  cout << test5 << endl;
  cout << test6 << endl;
  cout << test7 << endl;
  cout << test8 << endl;
  cout << test9 << endl;
  cout << test10 << endl;
  cout << test11 << endl;
}

int main()
{
  //Vect3Test();

 //Vect3<int> test(4, 21, 0);
 //Vect3<int> test2(0, 1, 0);
 //Vect3<int> test3(23, 7, 0);

 /* Vect3<float> planeNorm(0.f, 1.f, 0.f);
  Vect3<float> planePoint(0.f, 0.f, 0.f);
  Vect3<float> rayStart(1.f, 1.f, 1.f);
  Vect3<float> rayDir(-.5f, -.23f, 0.f);

  Vect3<float> intersection(.0f, .0f, .0f);

  int intersectStatus = Math3D::RayPlaneIntersection(
      rayStart, rayDir, planePoint, planeNorm, intersection);

  if (intersectStatus == 0)
    cout << "No Intersect" << endl;
  else if (intersectStatus == 1)
    cout << "intersection is : " << intersection << endl;
  else
    cout << "In Plane" << endl; */
 
    /*Vect3<float> normal(1.f,0.f, 0.f);
    Vect3<float> pointPos(-1.f, 0.f, 0.f);
    Vect3<float> pointPos2(1.f, 0.f, 0.f);*/
    vector<Vect3<float>> pointSet;

    pointSet.reserve(9);
    pointSet.push_back(Vect3<float> (-1.f, .0f, .5f));
    pointSet.push_back(Vect3<float> (-.7f, .0f, .45f));
    pointSet.push_back(Vect3<float> (-.02f, .0f, .9f));
    pointSet.push_back(Vect3<float> (-.89f, .0f, .75f));
    pointSet.push_back(Vect3<float> (-.3f, .0f, .3f));
    
    pointSet.push_back(Vect3<float> (1.f, .0f, .5f));
    pointSet.push_back(Vect3<float> (.0f, .0f, .45f));
    pointSet.push_back(Vect3<float> (.02f, .0f, .9f));
    pointSet.push_back(Vect3<float> (.89f, .0f, .75f));
    pointSet.push_back(Vect3<float> (.3f, .0f, .3f));
    pointSet.push_back(Vect3<float> (.2f, .0f, .59f));

    float a = 3, b = 4, c = 5;
    
    float r = ((a * b * c) / sqrt(((a + b + c)*(b + c - a)*(c + a - b)*(a + b - c))));

    cout << r << endl;
    //Delaunay::Triangulate(pointSet, nullptr);


	system("PAUSE");

	return 0;
}