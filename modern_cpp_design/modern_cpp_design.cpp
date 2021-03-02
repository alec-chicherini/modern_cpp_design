//My examples - while reading modern c++ design - Generic programming and design pattern applied - Andrei Alexandrescu book

#include <iostream>
//PART1 strategy class develepment
#define PART1

#ifdef PART1

template<class A, class B> 
class C {
public:
    C() { std::cout << "general C ctor" << std::endl; }

    void f(){ std::cout << "general C func" << std::endl; }
    
};


template<> void C<char,char>::f() { std::cout << "specialised<char,char> func" << std::endl; };

template<class A>
class C<int,A>
{
public:
    C() { std::cout << "specialised<int,T>   C ctor" << std::endl; }
    void f() { std::cout << "specialised<int,T> C func" << std::endl; }
};

template<class T>class X { public: T x; };
template<class T>class XY { public: T x;  T y; };

template<class T>
class allocator_heap {
public:
    static T create(T x)
    {
        std::cout << "allocator_heap call" << std::endl;
        auto res = new T(x);
        return *res;
    }
};

template <class T>
class allocator_stack {
public:
    static T create(T x)
    {
        std::cout << "allocator_stack call" << std::endl;
        return T(x);
    }
};

template<class Policy,class Type,class T>
class creator_master : public Policy, public Type {
public:
    creator_master() {};
    creator_master(T _x)                            { this->x = Policy().create(_x); }
    creator_master(T _x, T _y) :creator_master(_x)  { this->y = Policy().create(_y); }
};


template<class Policy, class Type, class T>
class creator : public creator_master<Policy, Type, T>
{
public:
    creator(T _x) : creator_master<Policy, Type, T>(_x) {};
    creator(T _x, T _y) :creator_master<Policy, Type, T>(_x, _y) {};

};


template<template<class>class Policy, template<class>class Type, class T = int>
class creator2 : public creator_master<Policy<int>, Type<int>, int>
{
public:
    creator2(T _x) : creator_master<Policy<int>, Type<int>, T>(_x) {};
    creator2(T _x, T _y) :creator_master<Policy<int>, Type<int>, T>(_x, _y) {};

};

template<class Policy, class Type=XY<int>, class T = int>
class creator3 : public creator_master<Policy, Type,int>
{
public:
    creator3(int _x) : creator_master<Policy, Type, T>(_x) {};
    creator3(int _x, int _y) :creator_master<Policy, Type, T>(_x, _y) {};
};

template<template<class>class Type, class Policy = allocator_stack<int>, class T = int>
class creator4: public creator_master<Policy, Type<int>, int>
{
public:
    creator4(int _x) : creator_master<Policy, Type<int>, T>(_x) {};
    creator4(int _x, int _y) :creator_master<Policy, Type<int>, T>(_x,_y) {};
};

#endif
int main()
{
    C<float, float> c1;
    C<int, float> c2;
    C<float, int> c3;
    C<int, int> c4;
    C<char, char> c5;
    std::cout << std::endl;
    c1.f();
    c2.f();
    c3.f();
    c4.f();
    c5.f();
    std::cout << std::endl;
  
    creator<allocator_stack<int>,X<int>,int> i1(1);
    std::cout << "i1.x = "<< i1.x << std::endl;

    creator2<allocator_heap,X> i2(3);
    std::cout << "i2.x = "<< i2.x << std::endl;
    
    creator2<allocator_stack,XY> i3(1,2);
    std::cout << "i3.x = " << i3.x << std::endl;
    std::cout << "i3.y = " << i3.y << std::endl;

    creator3<allocator_heap<int>> i4(4,5);
    std::cout << "i4.x = " << i4.x << std::endl;
    std::cout << "i4.y = " << i4.y << std::endl;

    creator4<XY> i5(6,7);
    std::cout << "i5.x = " << i5.x << std::endl;
    std::cout << "i5.y = " << i5.y << std::endl;
   
};

