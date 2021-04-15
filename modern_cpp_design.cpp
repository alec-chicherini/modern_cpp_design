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
//PART3 TypeList
//PART4 small objects in memory
//PART5 Functor class
//PART6 Singleton
//PART7 Smart ptr
#define PART7

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
class A1 {}; class B1;
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
////////////////////////////////////2.8////////////////////////////////////////
#include <typeinfo>

////////////////////////////////////2.10////////////////////////////////////////
template<typename T>
class TypeTraits {
private:
    template<class U>
    struct isPointer {
        enum { result = false };
    };

    template<class U>
    struct isPointer<U*> {
        enum { result = true };
    };
    template<class U> struct unConst          {typedef U result;};
    template<class U> struct unConst<const U> { typedef U result; };

public:
enum { isPtr = isPointer<T>::result };
typedef unConst<T>::result UnConst;
};
#endif

#ifdef PART3



template <typename ...Ts>
struct newTypeList
{
    newTypeList()
    {
        std::cout << sizeof...(Ts) << std::endl;
 
    }

    template <int N>
    using Ntype = typename std::tuple_element_t<N, std::tuple<Ts...>>;
};

#include <tuple>

class Widget {};
class ScrollBar     :public Widget {};
class Button        :public Widget {};
class GraphicButton :public Button {};

#define SUPERSUBCLASS(BASE,DERIVED) std::cout<< "std::is_base_of<"<<boost_type_name<BASE>()<<","<<boost_type_name<DERIVED>()<<"> = "<<std::is_base_of<BASE, DERIVED>()<<std::endl;

////////////////////////////////////3.13////////////////////////////////////////

template<template<class>class TargetClass, class ... Ts>
struct GenScatterHierarchy:TargetClass<Ts>... {

    GenScatterHierarchy() {};

    template<typename T>
    T getValue()
    {
        return (static_cast<TargetClass<T>>(*this)).value_;
    }

    template<typename T>
    void setValue(T x) {

        (static_cast<TargetClass<T>&>(*this)).value_ = x;
    }
};

template<class T>
struct Holder
{
    T value_;
};

//https://stackoverflow.com/a/66627180/12575933
template<class T, class, class ...Ts>
struct delete_untill_ :delete_untill_<T, Ts...> {};

template<class T, class...Ts>
struct delete_untill_<T, T, Ts...>
{
    using type = std::tuple<T,Ts...>;
};

template<typename ... Ts>
using delete_untill_T = typename delete_untill_<Ts...>::type;

//template<typename...Ts>
//using tuple_cat_t = decltype(std::tuple_cat(std::declval<Ts>()...));
//
//template<class T, class ... Ts>
//using delete_untill_T = tuple_cat_t
//<
//    typename std::conditional
//    <
//    std::is_same_v<T,Ts>||false,
//    std::tuple<Ts>,
//    std::tuple<>
//    >::type...
//
//> ;


struct null_class {};
#include <type_traits>
template<typename F, typename... Args>
using call_t = decltype(std::declval<F>()(std::declval<Args>()...));


template<class T, class... Ts>
struct EventHandler : public EventHandler<Ts...>
{
    EventHandler() {};
    
    virtual void OnEvent(T& obj) {std::cout<<"onEvent call - > object type : "<< boost_type_name<T>()<<std::endl; };
    
    template<class objType>
    void onEventCall(objType& obj)
    {
      delete_untill_T<objType, T, Ts...> del_tup;
      auto res_del_tuple = rewrap_args(del_tup);
      static_cast<decltype(res_del_tuple) & > (*this).OnEvent(obj);
    }

    template <typename ...Args>
    EventHandler<Args...> rewrap_args(std::tuple<Args...>& t) { return EventHandler<Args...>(); };
};

template<class T>
struct EventHandler<T> : public null_class
{
    std::tuple<T> params;
public:
    EventHandler() {};

    virtual void OnEvent(T obj) final { std::cout << "onEvent call - > object type : " << boost_type_name<T>() << std::endl; };

    void onEventCall(T obj)
    {//how to choose what to cast here to choose correct instance???
        static_cast<EventHandler<T>&>(*this).OnEvent(obj);
    }
};

#endif

#ifdef PART4
#include <vector>
#include <array>
#endif

#ifdef PART5


void A() { std::cout << "A call" << std::endl; };
void B() { std::cout << "B call" << std::endl; };


#include<functional>
#include<tuple>
#include<type_traits>

template<class F>
struct Functor
{
    F* pF;

    template<typename=std::enable_if_t<std::is_same_v<F,Functor>>>
    Functor(Functor& f) { pF = f.pF; }

    Functor(F& f) { pF = &f; };
    

    template<typename ...Ts>
    //auto operator()(Ts...params) { return std::apply(*pF, std::make_tuple(params...)); };
    auto operator()(Ts...params) { return std::invoke(*pF, params...); };

    template<typename OBJ, typename ...Ts>
    auto operator()(OBJ obj,Ts...params) { return std::invoke(*pF,obj, params...); };

};

template<typename F,typename T>
Functor(Functor<F> f, T param)->Functor<decltype(*f.pF)>;

struct C
{
    void memberFunc(int i, float f) 
    {
            std::cout << "call C::func(" << i << "," << f << ")" << std::endl;
    }
};

void func(int i, float f) {
    std::cout << "call func(" << i << "," << f << ")" << std::endl;
};

struct TestFunctor
{
    void operator()(int i, float f) {
        std::cout << "call functor(" << i << "," << f << ")" << std::endl;
    }
};

auto lambda = [](int i, float f) {std::cout << "call lambda(" << i << "," << f << ")" << std::endl; };



#endif

#ifdef PART6

void atexit_1();
void atexit_2();

static int x = std::atexit(atexit_1);
static int y = std::atexit(atexit_2);

void atexit_1()
{
	std::cout << "atexit 1 call. x = " << x << std::endl;
}

void atexit_2()
{
	std::cout << "atexit 2 call. y = " << y << std::endl;
}

///////////////////////Singleton///////////////////////
#include <memory>
#include<type_traits>
#include<concepts>

template<typename T>
struct Creator {

	virtual T* Create() = 0;
	virtual void Destroy(T* t) = 0;
	virtual ~Creator() {};
};

template<typename T>
struct Creator<std::shared_ptr<T>>
{
	virtual std::shared_ptr<T> Create() = 0;
	virtual  void Destroy(std::shared_ptr<T> t) = 0;
	virtual ~Creator() {};
};

template<typename T>
struct CreateStatic :public Creator<T>
{
	static T* Create() {
		static T* obj = new T();;
		return obj;
	}
    static void Destroy(T* t)
    {
    delete  t;
    std::cout << "Static version destroyed" << std::endl; 
    }
    ~CreateStatic() { };
};

template<typename T>
struct CreateUsingNew :public Creator<T>
{
	static T* Create() { return new T; }
    static void Destroy(T* t) {
        delete  t; 
        std::cout << "New version destroyed" << std::endl; }
	~CreateUsingNew() {  };
};


template<typename T>
struct CreateUsingUniquePtr :public Creator<std::shared_ptr<T>>
{
    static std::shared_ptr<T> Create() { return std::move(std::make_unique<T>());}
    static void Destroy(std::shared_ptr<T> t)
    {
        t.~shared_ptr();
        std::cout << "shared_ptr version destroyed" << std::endl;
    }
    ~CreateUsingUniquePtr() {  };
};

#include <thread>
#include <mutex>
#include<vector>
#include<algorithm>
template<typename T>
struct Threading
{
    virtual void LockGuard() = 0;
   
};

template<typename T>
struct stdmutex :public Threading<T>
{
    static void LockGuard()
    {
        std::mutex mtx;
        std::lock_guard<std::mutex> lock(mtx);
    };
    void Destroy() {}
};

template<typename T>
struct noLock :public Threading<T>
{
   static void LockGuard(){};
   
};


template<
    class T,
    template<class> class CreationPolicy,
    template<class> class ThreadingModel>
class SingletonCreator
{ 
public:
    
    using CreateReturnType = decltype(std::declval<CreationPolicy<T>>().Create());
    
public:
    CreateReturnType Instance()
    {
        if (!_pInstance)
        {
            ThreadingModel<T>::LockGuard();
            if (!_pInstance)
            {
                _pInstance = CreationPolicy<T>::Create();
               
            }
        }
        return std::move(_pInstance);
    };

    ~SingletonCreator() { CreationPolicy<T>::Destroy(_pInstance); };
private:
   
    CreateReturnType  _pInstance = nullptr;

  
};

struct Keyboard_ { std::string state = "Keyboard Ok"; }; 
using Keyboard =  SingletonCreator<Keyboard_, CreateUsingNew, noLock>;
struct DisplayImpl { std::string state = "Display Ok"; }; 
using Display =  SingletonCreator<DisplayImpl, CreateStatic, stdmutex>;
struct LogImpl { std::string state = "Log Ok"; }; 
using Log =  SingletonCreator<LogImpl, CreateUsingUniquePtr, stdmutex>;
////////////////////////////////////////////////////////
#endif

#ifdef PART7
class A
{
public:
    A() = delete;
    
   explicit A(int x) { std::cout << x << std::endl; }
};

void f(A a) {};

struct refCounter
{
    size_t count=0;
};

template<typename T>
struct shared
{
    static void clone(refCounter* ref) { ref->count++; }
    static void release(refCounter* ref) { ref->count--; }
};

template<typename T>
struct unique
{
    static void clone(refCounter* ref) { ref->count = 1; }
    static void release(refCounter* ref) { ref->count = 0; }
};

    template
        <
        typename T,
        template<class>class CounterPolicy
        >
        class SmartPtr
    {
    public:
        SmartPtr(T value) {
            pointee = new (T)(value);
            counter = new refCounter();
            CounterPolicy<T>::clone(counter);
        }

        SmartPtr(SmartPtr<T,CounterPolicy>& sp)
        {
            pointee = sp.get();
            counter = sp.getCounter();
            CounterPolicy<T>::clone(counter);
        }

        refCounter* getCounter() { return counter; }
        T* get() { return pointee; }

        size_t use_count() { return counter->count; }

        void operator=(SmartPtr<T, CounterPolicy>& sp)
        {
            if (std::is_same_v<decltype(sp), SmartPtr<T,shared>&>) {
                if (pointee != sp.get())
                {
                    auto right_counter = sp.getCounter();
                    CounterPolicy<T>::release(counter);
                    CounterPolicy<T>::clone(right_counter);

                    if (counter->count == 0) { this->~SmartPtr(); }

                    this->counter = right_counter;
                    this->pointee = sp.get();

                }
            }
            else if (std::is_same_v<decltype(sp), SmartPtr<T, unique>&>)
            {
                    delete this->counter;
                    delete this->pointee;

                    this->counter = sp.getCounter();
                    this->pointee = sp.get();

                    sp.setNullptr();
                  
            }
        }

        void setNullptr() {
            counter = nullptr;
            pointee = nullptr;
        };

        void print() {
            if(counter && pointee)
            std::cout << "*counter = " << counter->count << " *pointee = " << *pointee<<  std::endl;
            else
            std::cout << "*counter = " << "[nullptr]" << " *pointee = " << "[nullptr]" << std::endl;
        }
        ~SmartPtr() {
            if(counter && pointee)
            if (counter->count>1) 
                CounterPolicy<T>::release(counter);
            else
            {
                delete counter; delete pointee;
            }
        }
    private:
        T* pointee;
        refCounter* counter;
    };


    


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
    std::cout << std::endl;
    ////////////////////////////////////2.8////////////////////////////////////////

    int x28(4);
    std::cout << typeid(int).name() << std::endl;
    std::cout << typeid(int&).name() << std::endl;
    std::cout << typeid(int&&).name() << std::endl;
    std::cout << typeid(A1).name() << std::endl;
    
    //std::cout << tinfo<<std::endl;
    std::cout << std::endl;

    ////////////////////////////////////2.10////////////////////////////////////////
    int i_value;
    int* i_ptr;

   std::cout <<"is i_value ptr? = "<< TypeTraits<decltype(i_value)>::isPtr<<std::endl;
   std::cout <<"is i_ptr ptr?   = " << TypeTraits<decltype(i_ptr)>::isPtr << std::endl;

   const int const_int(4);
   std::cout << "const_int = " << boost_type_name<decltype(const_int)>() << std::endl;
  
   TypeTraits<decltype(const_int)>::UnConst some_int(5);
   std::cout << "some_int  = " << boost_type_name<decltype(some_int)>() << std::endl;

#endif

#ifdef PART3
   //creating and out size
   newTypeList<Widget, ScrollBar, Button,GraphicButton> nt1;

   //access to specified number of type in pack
   std::cout << boost_type_name <decltype(nt1)::Ntype<0>>() << std::endl;
   std::cout << boost_type_name <decltype(nt1)::Ntype<1>>() << std::endl;
   std::cout << boost_type_name <decltype(nt1)::Ntype<2>>() << std::endl;
   std::cout << boost_type_name <decltype(nt1)::Ntype<3>>() << std::endl;
   std::cout << std::endl;
   //derived fromTo
   //std::cout<< "std::is_base_of<Widget, Button>() = "<<std::is_base_of<Widget, Button>()<<std::endl;
   SUPERSUBCLASS(Widget, Button);
   SUPERSUBCLASS(Widget, ScrollBar);
   SUPERSUBCLASS(GraphicButton, Button);
   SUPERSUBCLASS(Button, GraphicButton);
   SUPERSUBCLASS(GraphicButton, Widget);
   SUPERSUBCLASS(Button, Widget);

   //hierarchy
   GenScatterHierarchy<Holder, int, float> obj;
  
  //Holder<std::string>::value_="4";

  std::cout << (static_cast<Holder<int>  &>(obj)).value_ << std::endl;
  std::cout << (static_cast<Holder<float>&>(obj)).value_ << std::endl;

  obj.setValue(int(4));
  obj.setValue(float(5.6));

  std::cout << obj.getValue<int>  () << std::endl;
  std::cout << obj.getValue<float>() << std::endl;
  std::cout << std::endl;
  //GenLinearHierarchy<EventHandler, Widget, Button, GraphicButton> LinearHierarchyObj;
  
  EventHandler<Widget, Button, GraphicButton> EventHandlerObj;
  
  Widget w; Button b; GraphicButton gb;

  EventHandlerObj.OnEvent(w);
  EventHandlerObj.OnEvent(b);
  EventHandlerObj.OnEvent(gb);
  std::cout << std::endl;
  static_cast<EventHandler<Widget,Button, GraphicButton>&>(EventHandlerObj).OnEvent(w);
  static_cast<EventHandler<Button,GraphicButton>&>(EventHandlerObj).OnEvent(b);
  static_cast<EventHandler<GraphicButton>&>(EventHandlerObj).OnEvent(gb);
  std::cout << std::endl;
  EventHandlerObj.onEventCall(gb);
  EventHandlerObj.onEventCall(b);
  EventHandlerObj.onEventCall(w);


#endif

#ifdef PART4

  int x = 1 + 2;

  int& x2 = *new int(3 + 4);
  std::cout << "x = " << x << " x2 = " << x2 << std::endl;
  delete &x2;

  constexpr int count = 10000;
  auto stringP = (std::string*)std::malloc(count * sizeof(std::string));
 
  std::vector<std::string> stringVec;

  std::array<std::string, count> stringArray;

  int c = count;
  while (c--)
  {
      new(stringP + c) std::string(std::to_string(c));
      stringVec.push_back(std::to_string(c));
      stringArray[c] = std::to_string(c);
  }

  std::cout << "sizeof(stringP) = " << sizeof(std::string) * count << std::endl;
  std::cout << "sizeof(stringVec) = " << sizeof(stringVec)+sizeof(std::string)* stringVec.capacity() << std::endl;
  std::cout << "sizeof(stringArray) = " << sizeof(stringArray) << std::endl;

  using std::string;
  c = count;
  while (c--)
  {
      stringP[c].~string();
  }
  std::free(stringP);

#endif 

#ifdef PART5
  void (*pA)() = &A;
  void (*pB)() = &B;
  A();
  B();
  pA();
  pB();
  (*pA)();
  (*pB)();
  pA = &B;
  pB = &A;
  pA();
  pB();

  TestFunctor functor;
  auto pMemberFunc = &C::memberFunc;

  Functor cmd1(functor); 
  Functor cmd2(func);
  Functor cmd3(lambda);
  Functor cmd4(pMemberFunc);
  auto cmd5 = std::bind_front(cmd2, 13);
  Functor cmd6(cmd1);

  cmd1(1, 2.3f);
  cmd2(4, 5.6f);
  cmd3(7, 8.9f);
  cmd4(C(),10, 11.12f);
  cmd5(14.15);
  cmd6(16, 17.18);
  

#endif

#ifdef PART6


  int x = 1;
  int y = 3;
  ::x = 5;
  ::y = 7;

  static int* p=new int(9);
  std::cout <<" *p = " << *p << std::endl;
  new(p) int(10);
  std::cout <<" *p = " << *p << std::endl;

 Keyboard K; std::cout<<K.Instance()->state<<std::endl;
 Display D; std::cout << D.Instance()->state << std::endl;
 Log L; std::cout << L.Instance()->state << std::endl;

 K.Instance();

  std::cout << "returning from main" << std::endl;
#endif

#ifdef PART7
  A a(char('1'));
  //A b = char('1');
  f(A(5));

  SmartPtr<int, shared> sp1(15);
  SmartPtr<int, shared> sp2(sp1);
  SmartPtr<int, shared> sp3(115);
  SmartPtr<int, shared> sp4(sp3);
  sp1.print();
  sp2.print();
  sp3.print();
  sp4.print();
  sp4=sp1;
  std::cout<<std::endl;
  sp1.print();
  sp2.print();
  sp3.print();
  sp4.print();
  sp3 = sp1;

  std::cout << std::endl;
  sp1.print();
  sp2.print();
  sp3.print();
  sp4.print();

  SmartPtr<int, unique> sp5(66);
  SmartPtr<int, unique> sp6(77);
  std::cout << std::endl;
  sp5.print();
  sp6.print();

  sp5 = sp6;
  sp5.print();
  sp6.print();

#endif
 };

