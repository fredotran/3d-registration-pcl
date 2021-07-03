#include <iostream>
#include <tuple>

std::tuple<int, int> divide(int dividend, int divisor) {
    return  std::make_tuple(dividend / divisor, dividend % divisor);
}

int main()
{
    if (__cplusplus == 201703L)
        std::cout << "C++17" << std::endl;
    else if (__cplusplus == 201402L)
        std::cout << "C++14" << std::endl;
    else if (__cplusplus == 201103L)
        std::cout << "C++11" << std::endl;
    else if (__cplusplus == 199711L)
        std::cout << "C++98" << std::endl;
    else
        std::cout << "pre-standard C++" << std::endl;

    int quotient, remainder;

    std::tie(quotient, remainder) = divide(14, 3);

    std::cout << quotient << ',' << remainder << std::endl;
}


