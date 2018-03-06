#include <vector> 
#include <iostream> 
#include <math.h>
using namespace std;


template <class T, class U>
class Quaternion{
public: 
	Quaternion(const vector<T> n, const U a);
	Quaternion(const vector<T> n);
	vector<T>* inverse();
	T magnitude();
	void printq();
	void invert();
	void lmultiply(Quaternion* q);
	void rmultiply(Quaternion* q);

private: 
	T w, x, y, z; 
};

/*Create a quaternion from an axis angle and an attitude vector*/
template <class T, class U>
Quaternion<T,U>::Quaternion(vector<T> n, U a){
	a = a/360*(T)M_PI*2;
	w = cos(a/2);
	x = n[0]*sin(a/2); 
	y = n[1]*sin(a/2);
	z = n[2]*sin(a/2);
}
template <class T, class U>
Quaternion<T,U>::Quaternion(vector<T> n):
w(n[0]),x(n[1]),y(n[2]),z(n[3])
{}

template <class T, class U>
T Quaternion<T,U>::magnitude(){
	return x*x+y*y+z*z+w*w;
}

template <class T, class U>
vector<T>* Quaternion<T,U>::inverse(){	
	static vector<T> quatInv_ptr = {w,-x,-y,-z};
	return &quatInv_ptr;
}

template <class T, class U>
void Quaternion<T,U>::invert(){	
	T m = x*x+y*y+z*z+w*w;
	w = w/m;
	x = -x/m; 
	y = -y/m; 
	z = -z/m;
}

template <class T, class U>
void Quaternion<T,U>::lmultiply(Quaternion* q){
	T xOld,yOld,zOld,wOld; 
	xOld = x; 
	yOld = y; 
	zOld = z;
	wOld = w; 
	w = (*q).w*wOld-(*q).x*xOld-(*q).y*yOld-(*q).z*zOld;
	x = (*q).x*wOld+(*q).w*xOld-(*q).z*yOld+(*q).y*zOld;
	y = (*q).y*wOld+(*q).z*xOld+(*q).w*yOld-(*q).x*zOld;
	z = (*q).z*wOld-(*q).y*xOld+(*q).x*yOld+(*q).w*zOld;
}
template <class T, class U>
void Quaternion<T,U>::rmultiply(Quaternion* q){
	T xOld,yOld,zOld,wOld; 
	xOld = x; 
	yOld = y; 
	zOld = z;
	wOld = w; 
	w = wOld*(*q).w-(*q).x*xOld-(*q).y*yOld-(*q).z*zOld;
	x = xOld*(*q).w+(*q).x*wOld-(*q).y*zOld+(*q).z*yOld;
	y = yOld*(*q).w+(*q).x*zOld+(*q).y*wOld-(*q).z*xOld;
	z = zOld*(*q).w-(*q).x*yOld+(*q).y*xOld+(*q).z*wOld;
}
template <class T, class U>
void Quaternion<T,U>::printq()
{
	cout<<"{"<<w<<","<<x<<","<<y<<","<<z<<"}"<<endl;
}

int  main()
{
	vector<double> m1 ={1, 0, 0};
	double r1 = 30;
	vector<double> n0 = {0, 1, 0, 0};
	vector<double> n1 = {0, 1, 0, 0};

	Quaternion <double,double> q0(m1, r1);
	Quaternion <double,double> q1(*(q0.inverse()));	
	cout<<q1.magnitude()<<endl;
	q1.printq();
	q0.printq();
	q1.lmultiply(&q0);
	q1.printq();


return 0;
}

