#ifndef _H_MATRIX_UTILS
#define _H_MATRIX_UTILS

#include <toml.hpp>
#include <type_traits>

#include "Eigen/Core"
#include "Eigen/Dense"

namespace toml {
    template<int x, int y>
    inline constexpr bool greater = x > y;
}


template<size_t Expected>
[[noreturn]] inline void
throw_bad_array_size(const std::string &funcname, size_t actual, const toml::value &v) {
    throw std::out_of_range(toml::detail::format_underline(
        toml::concat_to_string(funcname, "expected array of size ", Expected), 
        {
            {v.location(), toml::concat_to_string("but got ", v.as_array().size())}
        }
    ));
}

template <typename T, int R, int C>
typename std::enable_if<
        toml::greater<R, 1> &&
        toml::greater<C, 1> &&
    std::is_arithmetic_v<T>>::type
toml_to_matrix(const toml::value &V, Eigen::Matrix<T, R, C> &M) {
    for (int i = 0; i < R; ++i) {
        if(V.as_array().size() != R) {
            throw_bad_array_size<R>("toml_to_matrix(): ", V.as_array().size(), V);
        }
        auto &row = V.as_array().at(i);
        for (int j = 0; j < C; ++j) {
            if(row.as_array().size() != C) {
                throw_bad_array_size<C>("toml_to_matrix(): ", row.as_array().size(), row);
            }
            auto &val = row.as_array().at(j);
            if constexpr (std::disjunction<
                              std::is_same<T, double>,
                              std::is_same<T, float>>::value) {
                M(i, j) = val.as_floating();
            } else if constexpr (std::is_same<T, int>::value) {
                M(i, j) = val.as_integer();
            }
        }
    }
}

template <typename T, int R, int C>
typename std::enable_if<
    R == 1 ||
    C == 1 &&
        std::is_arithmetic_v<T>>::type
toml_to_matrix(const toml::value &V, Eigen::Matrix<T, R, C> &M) {
    constexpr int len = R * C;
    T value{};
    if(V.as_array().size() != len) {
        throw_bad_array_size<len>("toml_to_matrix(): ", V.as_array().size(), V);
    }
    for (int i = 0; i < len; ++i) {
        auto &val = V.as_array().at(i);
        if constexpr (std::disjunction<
                          std::is_same<T, double>,
                          std::is_same<T, float>>::value) {
            value = val.as_floating();
        } else if constexpr (std::is_same<T, int>::value) {
            value = val.as_integer();
        }
        if constexpr (R == 1) {
            M(0, i) = value;
        } else {
            M(i, 0) = value;
        }
    }
}

#endif