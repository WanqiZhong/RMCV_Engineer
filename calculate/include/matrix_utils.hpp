#ifndef _H_MATRIX_UTILS
#define _H_MATRIX_UTILS

#include <type_traits>
#include "Eigen/Core"
#include "Eigen/Dense"
#include <toml.hpp>

template <int x, int y>
inline constexpr bool greater = x > y;

template <typename T, int R, int C>
typename std::enable_if<
    greater<R, 1> &&
    greater<C, 1> &&
    std::is_arithmetic_v<T>>::type
toml_to_matrix(const toml::value &V, Eigen::Matrix<T, R, C> &M)
{
    for (int i = 0; i < R; ++i)
    {
        for (int j = 0; j < C; ++j)
        {
            if constexpr (std::disjunction<
                              std::is_same<T, double>,
                              std::is_same<T, float>>::value)
            {
                M(i, j) = V[i][j].as_floating();
            }
            else if constexpr (std::is_same<T, int>::value)
            {
                M(i, j) = V[i][j].as_integer();
            }
        }
    }
}

template <typename T, int R, int C>
typename std::enable_if<
    R == 1 ||
    C == 1 &&
        std::is_arithmetic_v<T>>::type
toml_to_matrix(const toml::value &V, Eigen::Matrix<T, R, C> &M)
{
    constexpr int len = R * C;
    T value{};
    for (int i = 0; i < len; ++i)
    {
        if constexpr (std::disjunction<
                          std::is_same<T, double>,
                          std::is_same<T, float>>::value)
        {
            value = V[i].as_floating();
        }
        else if constexpr (std::is_same<T, int>::value)
        {
            value = V[i].as_integer();
        }
        if constexpr (R == 1)
        {
            M(0, i) = value;
        }
        else
        {
            M(i, 0) = value;
        }
    }
}

#endif