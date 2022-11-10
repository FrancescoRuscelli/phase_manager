#ifndef CLASS_RESOLVER_H
#define CLASS_RESOLVER_H

#include <iostream>

// magic stuff to check if class has given member

template <class...>
struct types
{
    using type=types;
};

template <class...>
struct voider
{
    using type=void;
};

template <class...Ts>
using void_t = typename voider<Ts...>::type;

namespace details
{

template <template <class...> class Z,
         class types,
         class = void>
struct can_apply : std::false_type {};

template <template <class...> class Z,
         class... Ts>
struct can_apply<Z,
                 types<Ts...>,
                 void_t<Z<Ts...>>
                 > : std::true_type {};

} // namespace details

template <template <class...> class Z, class...Ts>
using can_apply = details::can_apply<Z, types<Ts...>>;

// the type of os << X, in a SFINAE friendly manner:
template<class X>
using set_bounds_result = decltype( std::declval<X>().setBounds(std::declval<int>()) );

// turn the above into a "is there a print operator" test:
template<class X>
using has_set_bounds = can_apply< set_bounds_result, X >;



//// SFINAE test
//template <typename T>
//class has_set_bounds
//{
//    typedef char one;
//    struct two { char x[2]; };

//    template <typename C> static one test( decltype(&C::setBounds) ) ; //(std::declval<int>())
//    template <typename C> static two test(...);

//public:
//    enum { value = sizeof(test<T>(0)) == sizeof(char) };
//};


#endif // CLASS_RESOLVER_H
