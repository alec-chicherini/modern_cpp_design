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
//PART8 Factory
//PART9 Abstract Factory
//PART10 Visitor
//PART11 Multimethod
#define PART11

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


#ifdef PART8
#include <map>
    template
        <
        class Product,
        typename Key,
        typename Creator = Product*(*)()
        >
        class Factory
    {
    private:
        using Map = typename std::map<Key, Creator>;
        Map creators_;
    public:
        bool Register(const Key& id, Creator creator)
        {
            const auto [it, success] = creators_.insert(Map::value_type(id,creator));
            return success;
        }
        bool Unregistered(const Key& id)
        {
            return creators_.erase(id)==1;
        }

        Product* CreateObject(const Key& id)
        {
            auto it = creators_.find(id);
            if (it != creators_.end())
                return (it->second)();
            else {
            std::runtime_error("CrateObject(Key&) Error - Wrong key.");

            return nullptr;
        }
            return nullptr;
        }
    };

    enum class rotateType{ROTATE_CUSTOM, ROTATE_HITBOX, ROTATE_MIDDLE};
    enum class resizeType{RESIZE_HITBOX , RESIZE_MIDDLE};

    template<typename T=int>
    class Object
    {
    private:
        T _x;
        T _y;

    public:
        virtual void move(T dx, T dy) = 0;
        virtual void rotate(float degree, rotateType rt= rotateType::ROTATE_MIDDLE, T x=0, T y=0) = 0;
        virtual void resize(float sx, float sy, resizeType rt = resizeType::RESIZE_HITBOX) = 0;
        virtual std::string info() = 0;

    };

    struct Color {
        uint8_t r;
        uint8_t g;
        uint8_t b;
        uint8_t alpha = 1;
    }default_color(0,0,0,1);

    //_x,_y - point to rotate. x, y - rotation point, angle - angle to rotate in degrees
    template<typename T>
    inline void RotateHelper(T& _x, T& _y, T& x, T& y, float angle) noexcept
    {
        T dx = _x - x;
        T dy = _y - y;
        _x = (T)(dx * cos(angle) - dy * sin(angle));
        _y = (T)(dx * sin(angle) + dy * cos(angle));
        _x += dx;
        _y += dy;
    };

    //_x,_y,_w,_h hitbox params for line (x1,y1) - (x2,y2)
    template<typename T>
    inline void HitboxHelperLine(T& _x, T& _y, T& _w, T& _h, T& x1, T& y1, T& x2, T& y2) noexcept
    {
        _x = std::min(x1, x2);
        _y = std::min(y1, y2);
        _w = std::max(x1, x2) - _x;
        _h = std::max(y1, y2) - _y;
    };

    template<typename T = int>
    class Point :public Object<T>
    {
    private:
        T _x;
        T _y;
        T _w; //width
        T _h; //height
        Color color;

    public:
        Point() :_x((T)0.0), _y((T)0.0), _h((T)0.0), _w((T)0.0), color(default_color) {};
        virtual void move(T dx, T dy) override
        {
            _x += dx;
            _y += dy;
            _w = _x;
            _h = _y;
        };

        virtual std::string info() override
        {
            std::string res;
            res += "_x = "; res += std::to_string(_x); res += ";";
            res += "_y = "; res += std::to_string(_y); res += ";";
            res += "_w = "; res += std::to_string(_w); res += ";";
            res += "_h = "; res += std::to_string(_h); res += ";";

            return res;
        };
        virtual void rotate(float degree, rotateType rt = rotateType::ROTATE_MIDDLE, T x = 0, T y = 0) override
        {
        
            switch (rt)
            {
            case rotateType::ROTATE_CUSTOM:
            {
                ::RotateHelper(_x, _y, x, y, degree);
                _w = _x;
                _h = _y;
               
                break; 
            }
            case rotateType::ROTATE_HITBOX:
                break;
            case rotateType::ROTATE_MIDDLE:
                break;
            default:
                break;
            }
        };

    private:
        //Point::resize() Error - cannot resize point.
        virtual void resize(float sx, float sy, resizeType rt = resizeType::RESIZE_HITBOX) override {};
    };

    template<typename T = int>
    class Line :public Object<T>
    {
    private:
        T _x;
        T _y;
        T _w; //width
        T _h; //height
        T _x1, _x2, _y1, _y2;
        Color color;

    public:

        Line(T x1, T y1, T x2, T y2):_x1(x1), _y1(y1), _x2(x2), _y2(y2), _x((T)0.0f), _y((T)0.0f), color(default_color)
        {
            ::HitboxHelperLine(_x, _y, _w, _h, x1, y1, x2, y2);

        }
        Line() : Line(0,0,0,0){}

        virtual std::string info() override
        {
            std::string res;
            res += "_x = "; res += std::to_string(_x); res += ";";
            res += "_y = "; res += std::to_string(_y); res += ";";
            res += "_w = "; res += std::to_string(_w); res += ";";
            res += "_h = "; res += std::to_string(_h); res += ";";
            res += "_x1 = "; res += std::to_string(_x1); res += ";";
            res += "_y1 = "; res += std::to_string(_y1); res += ";";
            res += "_x2 = "; res += std::to_string(_x2); res += ";";
            res += "_y2 = "; res += std::to_string(_y2); res += ";";
            return res;
        };
        virtual void move(T dx, T dy) override
        {
            _x +=dx;
            _y +=dy;
            _x1 += dx; _x2 += dx;
            _y1 += dy; _y2 += dy;
        }; 

        virtual void rotate(float degree, rotateType rt = rotateType::ROTATE_MIDDLE, T x = 0, T y = 0) override
        {
            switch (rt)
            {
            case rotateType::ROTATE_CUSTOM:
            {            
            ::RotateHelper(_x1, _y1, x, y, degree);
            ::RotateHelper(_x2, _y2, x, y, degree);
            ::HitboxHelperLine(_x, _y, _w, _h, _x1, _y1, _x2, _y2);
                break; 
            }
            case rotateType::ROTATE_HITBOX:
            {
                ::RotateHelper(_x1, _y1, _x, _y, degree);
                ::RotateHelper(_x2, _y2, _x, _y, degree);
                break;
            }
            case rotateType::ROTATE_MIDDLE:
            {
                T mx = (T)((_x1 + _x2) / 2.0);
                T my = (T)((_y1 + _y2) / 2.0);
                ::RotateHelper(_x1, _y1, mx, my, degree);
                ::RotateHelper(_x2, _y2, mx, my, degree);
                ::HitboxHelperLine(_x, _y, _w, _h, _x1, _y1, _x2, _y2);
                break;
            }
            default:
                break;
            }
        };

        virtual void resize(float sx, float sy, resizeType rt = resizeType::RESIZE_HITBOX) override
        {
            switch (rt)
            {
            case resizeType::RESIZE_HITBOX:
            {
				auto max_x = std::max(_x1, _x2);
				if (_x1 == max_x)_x1 = (T)(max_x - std::min(_x1, _x2) * sx);
				else _x2 = max_x - std::min(_x1, _x2);

				auto max_y = std::max(_y1, _y2);
				if (_y1 == max_y)_y1 = (T)(max_y - std::min(_y1, _y2) * sy);
				else _y2 = max_y - std::min(_y1, _y2);

                ::HitboxHelperLine(_x, _y, _w, _h, _x1, _y1, _x2, _y2);

                break;
            };
            case resizeType::RESIZE_MIDDLE:
            {
                auto mx = (_x1 + _x2) / 2.0;
                auto my = (_y1 + _y2) / 2.0;

                _x1 = (T)((_x1 - mx) * sx + mx);
                _x2 = (T)((_x2 - mx) * sx + mx);
                _y1 = (T)((_y1 - my) * sx + my);
                _y2 = (T)((_y2 - my) * sx + my);
                ::HitboxHelperLine(_x, _y, _w, _h, _x1, _y1, _x2, _y2);

                break;
            };
            default:
                break;
            }
        };
    };


    Object<int>* createPoint() { return new Point<int>; }
    Object<int>* createLine() { return new Line<int>; }




#endif

#ifdef PART9

#include <tuple>
#include <type_traits>
#include <any>

    struct Unit { Unit() { std::cout << "Unit ctor. -> "; } };

    struct Archer : public Unit { Archer() { std::cout << "Archer ctor. ->  " ;   } };
    struct Warior : public Unit { Warior() { std::cout << "Warior ctor. ->  " ;    } };
    struct Pikeman : public Unit { Pikeman() { std::cout << "Pikeman ctor. ->  " ; } };
    struct Balista : public Unit { Balista() { std::cout << "Balista ctor. ->  " ; } };

   

    struct EasyArcher :public Archer { EasyArcher() { std::cout << "EasyArcher ctor. " << std::endl; } };
    struct HardArcher :public Archer { HardArcher() { std::cout << "HardArcher ctor. " << std::endl; } };
    struct InsaneArcher :public Archer { InsaneArcher() { std::cout << "InsaneArcher ctor. " << std::endl; } };

    struct EasyWarior :public Warior { EasyWarior() { std::cout << "EasyWarior ctor. " << std::endl; } };
    struct HardWarior :public Warior { HardWarior() { std::cout << "HardWarior ctor. " << std::endl; } };
    struct InsaneWarior :public Warior { InsaneWarior() { std::cout << "InsaneWarior ctor. " << std::endl; } };

    struct EasyPikeman :public Pikeman { EasyPikeman() { std::cout << "EasyPikeman ctor. " << std::endl; } };
    struct HardPikeman :public Pikeman { HardPikeman() { std::cout << "HardPikeman ctor. " << std::endl; } };
    struct InsanePikeman :public Pikeman { InsanePikeman() { std::cout << "InsanePikeman ctor. " << std::endl; } };

    struct EasyBalista :public Balista { EasyBalista() { std::cout << "EasyBalista ctor. " << std::endl; } };
    struct HardBalista :public Balista { HardBalista() { std::cout << "HardBalista ctor. " << std::endl; } };
    struct InsaneBalista :public Balista { InsaneBalista() { std::cout << "InsaneBalista ctor. " << std::endl; } };

 //version 1


	struct AbstractFactoryImpl
	{
		virtual Archer* CreateArcher() = 0;
		virtual Warior* CreateWarior() = 0;
		virtual Pikeman* CreatePikeman() = 0;
		virtual Balista* CreateBalista() = 0;
	};

    struct AbstractFactory:public AbstractFactoryImpl
    {
        virtual Archer* CreateArcher() { return new Archer; };
        virtual Warior* CreateWarior() { return new Warior; };
        virtual Pikeman* CreatePikeman() { return new Pikeman; };
        virtual Balista* CreateBalista() { return new Balista; };
    };


	template<class...Ts>
	struct ConcreteFactory :public AbstractFactory
	{
        using params = std::tuple<Ts...>;
        using A = typename std::tuple_element_t<0, params>;
        using W = typename std::tuple_element_t<1, params>;
        using P = typename std::tuple_element_t<2, params>;
        using B = typename std::tuple_element_t<3, params>;

		virtual Archer* CreateArcher()override 
        {
            static_assert(std::is_base_of_v<Archer,A>,"Wrong list of types in Concrete Factory: must be <Archer,Warior,Pikeman,Balista>");
            return new A;
        };
		virtual Warior* CreateWarior()override 
        {
            static_assert(std::is_base_of_v<Warior, W>, "Wrong list of types in Concrete Factory: must be <Archer,Warior,Pikeman,Balista>");
            return new W;
        };
		virtual Pikeman* CreatePikeman()override 
        {
            static_assert(std::is_base_of_v<Pikeman, P>, "Wrong list of types in Concrete Factory: must be <Archer,Warior,Pikeman,Balista>");
            return new P;
        };
		virtual Balista* CreateBalista()override 
        {
            static_assert(std::is_base_of_v<Balista, B>, "Wrong list of types in Concrete Factory: must be <Archer,Warior,Pikeman,Balista>");
            return new B;
        };
	};

//version 2
    template<typename T>
    struct AbstractFactoryImpl_
    {
        virtual T* Create() = 0;
    };


    template<typename T>
    struct AbstractFactory_:public AbstractFactoryImpl_<T>
    {
        virtual T* Create() override
        {
            return new T;
        }
    };

    //struct Ptr_ {};


    template<class ...Ts >
    struct ConcreteFactory_ :  public AbstractFactory_<Ts>...
    {

        using params = std::tuple<Ts...>;
        using A = typename std::tuple_element_t<0, params>;
        using W = typename std::tuple_element_t<1, params>;
        using P = typename std::tuple_element_t<2, params>;
        using B = typename std::tuple_element_t<3, params>;

        template<class T> 
        T* Create()
        {
            if constexpr(std::is_base_of_v<T, A>) return AbstractFactory_<A>::Create();
            else if constexpr (std::is_base_of_v<T, W>) return AbstractFactory_<W>::Create();
            else if constexpr (std::is_base_of_v<T, P>) return AbstractFactory_<P>::Create();
            else if constexpr (std::is_base_of_v<T, B>) return AbstractFactory_<B>::Create();
        }
    };

    //version 3

    template<typename T>
    struct AbstarctFactoryImpl__
    {
        virtual ~AbstarctFactoryImpl__() = default;
       virtual T* Create(T) = 0;
    };

    template<typename ...Ts>
    struct AbstractFactory__ :virtual AbstarctFactoryImpl__<Ts>...
    {
        using AbstarctFactoryImpl__<Ts>::Create...;
    };

    template<typename T1, typename T2>
    struct ConcreteFactoryImpl__ : virtual AbstarctFactoryImpl__<T1>
    {
        T1* Create(T1) override { return new T2; }
    };

    template<typename Base, typename ...Ts>
    struct ConcreteFactory__;

    template <typename ...Ts1, typename ...Ts2>
    struct ConcreteFactory__ <AbstractFactory__<Ts1...>, Ts2...> : AbstractFactory__<Ts1...>, ConcreteFactoryImpl__<Ts1, Ts2>...
    {
        using ConcreteFactoryImpl__<Ts1, Ts2>::Create...;
    };

#endif

#ifdef PART10
    //version 1
    class Big;
    class Small;

    class Visitor {
    public:
        virtual void visit(Big* b)=0;
        virtual void visit(Small* s)=0;
        virtual ~Visitor() = default;
    };

    class Object
    {
    public:
        virtual void accept(Visitor*) = 0;
        virtual std::string get_name() = 0;
        virtual void set_name(std::string) = 0;
        virtual ~Object() = default;
    };

    class Big :public Object
    {
    public:
         Big() 
        {
            name= "BiG ObjecT";
        };

        void accept(Visitor* v) override
        {
            v->visit(this);
        };

        std::string get_name() 
        {
            return name;
        };

        void set_name(std::string str)
        {
            name = str;
        };

    private:
        std::string name;
    };

    class Small :public Object
    {
    public:
         Small() 
        {
            name = "SmaLL ObjEct";
        };

        void accept(Visitor* v) override
        {
            v->visit(this);
        };

        std::string get_name()
        {
            return name;
        };

        void set_name(std::string str)
        {
            name = str;
        };

    private:
        std::string name;
    };

    class CaseVisitor:public Visitor
    {
    public:
        void visit(Big* b) override
        {
            auto name = b->get_name();
            for (auto& c : name)c = toupper(c);
            b->set_name(name);
        };

        void visit(Small* b) override
        {
            auto name = b->get_name();
            for (auto& c : name)c = tolower(c);
            b->set_name(name);
        };
    };

#include <vector>
    void print(std::vector<Object*> objs)
    {
        for (auto& o : objs)
            std::cout << o->get_name() << "-";
        std::cout << std::endl;
    }

    
    /*template<class T>
    class Visitor_ {
    public:
        virtual void visit(T* b) = 0;
        virtual ~Visitor_() = default;
    };

    template<class T>
    class CaseVisitor_ :public Visitor_<T>
    {
        void visit(T* t)override
        {
            auto name = t->get_name();
            for (auto& c : name)
            {
                if constexpr (std::is_same_v<T, Small>)c = tolower(c);
                if constexpr (std::is_same_v<T, Big>)c = toupper(c);
            }
            b->set_name(name);
        };
    };*/

   



#endif

#ifdef PART11
	//overloaded
	//void f(int x) {};
	//void f(int x, int y) {};

	//template<typename T>
	//void f(T x) {};

	//struct V {
	//	virtual void f() {};
	//};
	//struct dV :V {
	//	void f()override {};
	//};


	struct X {};
	struct X1 :X {};
	struct X2 :X {};
    struct X3 :X {};
    struct Y {};
    struct Y1 :Y {};
    struct Y2 :Y {};
    struct Y3 :Y {};

    std::string f(X1, Y1) { return "f(X1, Y1)"; }
    std::string f(X1, Y2) { return "f(X1, Y2)"; }
    std::string f(X1, Y3) { return "f(X1, Y3)"; }
    std::string f(X2, Y1) { return "f(X2, Y1)"; }
    std::string f(X2, Y2) { return "f(X2, Y2)"; }
    std::string f(X2, Y3) { return "f(X2, Y3)"; }
    std::string f(X3, Y1) { return "f(X3, Y1)"; }
    std::string f(X3, Y2) { return "f(X3, Y2)"; }
    std::string f(X3, Y3) { return "f(X3, Y3)"; }

 //version x
#include <map>
#include <functional>
    std::string f_X1_Y1() { return f(X1(), Y1());};
    std::string f_X1_Y2() { return f(X1(), Y2()); };
    std::string f_X1_Y3() { return f(X1(), Y3()); };
    std::string f_X2_Y1() { return f(X2(), Y1()); };
    std::string f_X2_Y2() { return f(X2(), Y2()); };
    std::string f_X2_Y3() { return f(X2(), Y3()); };
    std::string f_X3_Y1() { return f(X3(), Y1()); };
    std::string f_X3_Y2() { return f(X3(), Y2()); };
    std::string f_X3_Y3() { return f(X3(), Y3()); };

    class crossExecutor
    {
        std::map<std::string, std::function<std::string(void)>> fMap;

    public:
           crossExecutor() {
            fMap["X1Y1"] = f_X1_Y1;
            fMap["X1Y2"] = f_X1_Y2;
            fMap["X1Y3"] = f_X1_Y3;

            fMap["X2Y1"] = f_X2_Y1;
            fMap["X2Y2"] = f_X2_Y2;
            fMap["X2Y3"] = f_X2_Y3;

            fMap["X3Y1"] = f_X3_Y1;
            fMap["X3Y2"] = f_X3_Y2;
            fMap["X3Y3"] = f_X3_Y3;
        };
           std::string exec(std::string left, std::string right) 
           {
               std::string result;

               try 
               { 
                     result = fMap.at(left + right)();
                     return result;
               }
               catch (std::out_of_range e)
               {
                   std::cout << "WARNING: cross function doesn`t exist. Left - Right value changed." << std::endl;
               };

               try 
               {
                   result = fMap.at(right + left)();
                   return result;
               }
               catch (std::out_of_range e)
               {
                   std::cout << "ERROR: cross function doesn`t exist" << std::endl;
               };
           }
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

#ifdef PART8

  Factory<Object<int>, std::string> fac1;
  fac1.Register("point", createPoint);
  fac1.Register("line", createLine);

  auto point1 = fac1.CreateObject("point");
  auto point2 = fac1.CreateObject("point");

  auto line1 = fac1.CreateObject("line");
  auto line2 = fac1.CreateObject("line");

  std::cout << point1->info() << std::endl;
  std::cout << point2->info() << std::endl;
  std::cout << line1->info() << std::endl;
  std::cout << line2->info() << std::endl;

  Line<int> line3(0, 0, 15, 20);
  std::cout << line3.info() << std::endl;
  line3.move(2, 2);
  std::cout << line3.info() << std::endl;
  line3.resize(2, 0.5f);                  //wrong resize!
  std::cout << line3.info() << std::endl;
  line3.rotate(180);
  std::cout << line3.info() << std::endl;
  

#endif

#ifdef PART9

  //version 1
  using EasyUnitFactory     = typename ConcreteFactory<EasyArcher,EasyWarior, EasyPikeman, EasyBalista>;
  using HardUnitFactory     = typename ConcreteFactory<HardArcher,HardWarior, HardPikeman, HardBalista>;
  using InsaneUnitFactory   = typename ConcreteFactory<InsaneArcher,InsaneWarior, InsanePikeman, InsaneBalista>;

  AbstractFactory* pFactory;

  pFactory = new EasyUnitFactory;
  Warior* pWarior = pFactory->CreateWarior();
  delete pFactory;

  pFactory = new HardUnitFactory;
  Archer* pArcher = pFactory->CreateArcher();
  delete pFactory;

  pFactory = new InsaneUnitFactory;
  Pikeman* pPikeman = pFactory->CreatePikeman();
  Balista* pBalista = pFactory->CreateBalista();
  delete pFactory;

  delete pWarior;
  delete pArcher;
  delete pPikeman;
  delete pBalista;


  //version 2
  std::cout << std::endl;
  using EasyUnitFactory_ = typename ConcreteFactory_<EasyArcher, EasyWarior, EasyPikeman, EasyBalista>;
  using HardUnitFactory_ = typename ConcreteFactory_<HardArcher, HardWarior, HardPikeman, HardBalista>;
  using InsaneUnitFactory_ = typename ConcreteFactory_<InsaneArcher, InsaneWarior, InsanePikeman, InsaneBalista>;


  auto pFactory_E = new EasyUnitFactory_;
  Warior* pWarior_ = pFactory_E->Create<Warior>();
  delete pFactory_E;

  auto pFactory_H = new HardUnitFactory_;
  Archer* pArcher_ = pFactory_H->Create<Archer>();
  delete pFactory_H;

  auto pFactory_I = new InsaneUnitFactory_;
  Pikeman* pPikeman_ = pFactory_I->Create<Pikeman>();
  Balista* pBalista_ = pFactory_I->Create<Balista>();
  delete pFactory_I;

  delete pWarior_;
  delete pArcher_;
  delete pPikeman_;
  delete pBalista_;

  //version 3
  std::cout << std::endl;
  using EasyUnitFactory__   = typename ConcreteFactory__<AbstractFactory__<Archer, Warior, Pikeman, Balista>, EasyArcher, EasyWarior, EasyPikeman, EasyBalista>;
  using HardUnitFactory__   = typename ConcreteFactory__<AbstractFactory__<Archer, Warior, Pikeman, Balista>, HardArcher, HardWarior, HardPikeman, HardBalista>;
  using InsaneUnitFactory__ = typename ConcreteFactory__<AbstractFactory__<Archer, Warior, Pikeman, Balista>, InsaneArcher, InsaneWarior, InsanePikeman, InsaneBalista>;

  auto dummyWarior = Warior(); std::cout << std::endl;
  auto dummyArcher = Archer(); std::cout << std::endl;
  auto dummyBalista = Balista(); std::cout << std::endl;
  auto dummyPikeman = Pikeman(); std::cout << std::endl;

  auto create = [&]<typename T,typename ... Ts1, typename ... Ts2>
      (ConcreteFactory__<AbstractFactory__<Ts1...>, Ts2...>& factory, T Unit) 
  {
      return factory.Create(Unit);
  };

  EasyUnitFactory__   pFactory_E_;// = new EasyUnitFactory__;
  HardUnitFactory__   pFactory_H_;// = new HardUnitFactory__;
  InsaneUnitFactory__ pFactory_I_;// = new InsaneUnitFactory__;


  auto pEasyWaryor = create(pFactory_E_, dummyWarior);
  auto pEasyArcher = create(pFactory_E_, dummyArcher);
  auto pHardBalista = create(pFactory_H_, dummyBalista);
  auto pInsanePikeman = create(pFactory_I_, dummyPikeman);


#endif

#ifdef PART10

  std::vector<Object*> Objs;
  
  Objs.push_back(new Big());
  Objs.push_back(new Small());
  Objs.push_back(new Big());
  Objs.push_back(new Small());

  print(Objs);

  CaseVisitor* cv = new CaseVisitor;
  for (auto& o : Objs) o->accept(cv);
  //same
  /*for (auto& o : Objs) { 
      
      if(dynamic_cast<Big*>(o))
      cv->visit(static_cast<Big*>(o));

      if (dynamic_cast<Small*>(o))
          cv->visit(static_cast<Small*>(o));
  }*/

  print(Objs);
  
#endif

#ifdef PART11

  //f(1);
  //f(1.2f);
  //V v; v.f();
  //dV dv; dv.f();

  std::vector<std::string> Xs = {"X1","X2","X3"};
  std::vector<std::string> Ys = {"Y1","Y2","Y3"};

  crossExecutor cE;

  for (auto& x : Xs)
      for (auto& y : Ys) 
      {
          std::cout << "x = " << x << " y = " << y << " call ->" << cE.exec(x, y) << std::endl;
          std::cout << "y = " << y << " x = " << x << " call ->" << cE.exec(y, x) << std::endl;
      }

  cE.exec("A", "B");

#endif
 };

