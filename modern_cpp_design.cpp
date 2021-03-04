//My examples - while reading modern c++ design - Generic programming and design pattern applied - Andrei Alexandrescu book
///returning type of variables and some objects using boost library.

#include <iostream>
#include <typeinfo>
#include <boost\type_index.hpp>
#include <type_traits>

template<typename T>
std::string boost_type_name()
{
    return boost::typeindex::type_id_with_cvr<T>().pretty_name();
}



//PART1 strategy class develepment
//PART2 tools
#define PART2

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
protected:
    ~allocator_heap() = default;
};

template <class T>
class allocator_stack {
public:
    static T create(T x)
    {
        std::cout << "allocator_stack call" << std::endl;
        return T(x);
    }
protected:
    ~allocator_stack()=default;
};

template<class Policy,class Type,class T>
class creator_master : public Policy, public Type {
public:
    creator_master() {};
    creator_master(T _x)                            { this->x = Policy().create(_x); }
    creator_master(T _x, T _y) :creator_master(_x)  { this->y = Policy().create(_y); }
protected:
    ~creator_master() = default;
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


///////////////////////////////////////////////////////////////////////////////////////////////////
#include <thread>
#include <future>

template<template <class>class ThreadingPolicy, class Func>
struct do_code_execute
{


};

template <template <class>class ThreadingPolicy, class ReturnType>
class code_execute
{

    ReturnType res;
public:

    code_execute(ReturnType(*func_)())
    {
        res = ThreadingPolicy<ReturnType>().launch(func_);
    }

    ReturnType getResult()
    {
        return res;
    }
};


template<class ReturnType> struct launch_std_async
{ 
    ReturnType launch(ReturnType(*func_)())
    {
        std::future<ReturnType> fwd = std::async(std::launch::async, func_);
        std::cout << "launch_std_async" << std::endl;
        return fwd.get();
    }
};

template<class ReturnType> struct launch_this_thread
{
    ReturnType launch(ReturnType(*func_)())
    {
        std::cout << "launch_this_thread" << std::endl;
        return func_();
    }
};

template<class ReturnType> struct launch_std_thread
{
    ReturnType launch(ReturnType(*func_)())
    {
        std::cout << "launch_std_thread" << std::endl;
        ReturnType res;
        auto func_wrap = [&]() {res=func_();};
        std::thread tr(func_wrap);
        tr.join();

        return res;
    }
};
//based on answer from stack overflow
//https://stackoverflow.com/questions/66459261/how-to-pass-template-parameter-from-class-constructor

template<template<class>class Policy,typename Func>
auto create_code_execute(Func f)
{
    return code_execute<Policy, decltype(f())>(f);
}

#endif

#ifdef PART2
////////////////////////////////////2.1////////////////////////////////////////
template<class To, class From>
To safe_reinterpret_cast(From from)
{
    static_assert(sizeof(From) >= sizeof(To),"safe_reinterpret_cast ERROR");
    return reinterpret_cast<To>(from);
}

////////////////////////////////////2.2////////////////////////////////////////
class A1 ; class B1 ;
class A2 ; class B2 ;

template<class A, class B>
class W ;

template<>
class W<A1, B1>;

template<class A>
class W<A, B2>;

template<typename Arg>
class C;

template<class Args>
class  W<C<Args>, B1>;

template <class T, class U> T f(U arg);
//template <class U> void f<void, U>(U arg);
template <class T> T f(A1 arg);

////////////////////////////////////2.3////////////////////////////////////////
class I {
public:
    virtual int f() = 0;
};

template<class F, class T>
auto adapter(const F& func, const T& args) {

    class local :public I
    {
    public:
        local(const F& func, const T& args) :
            func_(func), args_(args) {};
          virtual int f() {
              return func_(args_);
               
          }
    private:
        F func_;
        T args_;
    };
    auto res = new local(func, args);
   
    return res->f() ;
}
////////////////////////////////////2.4////////////////////////////////////////

template<int v>
struct Int2Type
{
    enum { value = v };
};

template<typename T, int whatNumber>
class I2T{
private:
    void f(T x, Int2Type<1>) { std::cout << "call 1st version: " << x << std::endl; }
    void f(T x, Int2Type<2>) { std::cout << "call 2nd version: " << x << std::endl; }
    void f(T x, Int2Type<3>) { std::cout << "call 3rd version: " << x << std::endl; }

public:
    void f(T x)
    {
        f(x, Int2Type<whatNumber>());
    }
};
////////////////////////////////////2.5////////////////////////////////////////
template<typename T>
struct Type2Type {
    //using type = T;
};

template<class T, class U>
int* func(U arg, Type2Type<T>)
{
    std::cout << "call int version: " << arg << std::endl;
    return new T(arg);
};

template<class U>
float* func(U arg, Type2Type<float>)
{
    std::cout << "call float version: " << arg << std::endl;
    return new float(arg);
};
////////////////////////////////////2.6////////////////////////////////////////
template<bool flag, typename T, typename U>
struct Select
{
    using result = T;
};

template<typename T, typename U>
struct Select<false, T, U>
{
    using result = U ;
};

template<typename T, bool isPolimorphic>
struct NiftyContainer
{
    using type = typename Select<isPolimorphic, T*, T>::result ;
};

////////////////////////////////////2.7////////////////////////////////////////
#include <type_traits>
#include <vector>

#endif

int main()
{
#ifdef PART1
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

    std::cout << std::endl;

    auto lambda = []()-> int{
        int count = 1000;
        auto x = count;
        while (count--) { x += count;}
        return x; };


    code_execute<launch_std_async, decltype(lambda())> ce1(lambda);
    std::cout << "ce1.getResult() = " << ce1.getResult() << std::endl;

    code_execute<launch_this_thread, decltype(lambda())> ce2(lambda);
    std::cout << "ce2.getResult() = " << ce2.getResult() << std::endl;

    code_execute<launch_std_thread, decltype(lambda())> ce3(lambda);
    std::cout << "ce3.getResult() = " << ce3.getResult() << std::endl;

    //code_execute<launch_std_async> ce4(lambda);
    //std::cout << "ce4.getResult() = " << ce4.getResult() << std::endl;

    auto ce5 = create_code_execute<launch_std_thread>(lambda);
    std::cout << "ce5.getResult() = " << ce5.getResult() << std::endl;

#endif

#ifdef PART2
   
    long long x=4;
    std::string* z = safe_reinterpret_cast<std::string*>(x);

    ////////////////////////////////////2.3////////////////////////////////////////
    std::cout<<"like adapter call :"<<adapter([](int i) {return i + 1; }, 5)<<std::endl;
    std::cout << std::endl;

    ////////////////////////////////////2.4////////////////////////////////////////
    I2T<int, 1> i2t1; i2t1.f(1);
    I2T<float, 2> i2t2; i2t2.f(2.01f);
    I2T<char, 3> i2t3; i2t3.f('&');
    std::cout << std::endl;

    ////////////////////////////////////2.5////////////////////////////////////////
    int* pInt = func(1, Type2Type<int>());
    float* pFloat = func(2.34f, Type2Type<float>());
    int i = 5;
    int* pInt2 = func(i, Type2Type<decltype(i)>());
    std::cout << std::endl;

    ////////////////////////////////////2.6////////////////////////////////////////
    NiftyContainer<int, true>::type NC1; std::cout << boost_type_name<decltype(NC1)>() << std::endl;
    NiftyContainer<float, false>::type NC2; std::cout << boost_type_name<decltype(NC2)>() << std::endl;

    ////////////////////////////////////2.7////////////////////////////////////////
    //looks like standart creators read this book and made this built-in functions. I just use it.
    std::cout << std::is_convertible<double, int>::value << " "
              << std::is_convertible<char, char*>::value << " "
              << std::is_convertible<size_t, std::vector<int>>::value << std::endl;

    std::cout << std::is_same<double, int>::value << " "
              << std::is_same<int, std::int32_t>::value << " "
              << std::is_same<int, std::int64_t>::value << std::endl;



#endif
};

