#pragma once

#include "Eigen/Core"
#include <cstdint>

#include "stamp.h"

namespace cityfly::common {

using scalar_t = double;

template <int _dim> using Vector = Eigen::Vector<scalar_t, _dim>;

template <size_t _rows, size_t _cols>
using Matrix = Eigen::Matrix<scalar_t, _rows, _cols, Eigen::RowMajor>;

using Vector2 = Vector<2>;
using Vector3 = Vector<3>;
using Vector4 = Vector<4>;
using Vector5 = Vector<5>;
using Vector6 = Vector<6>;

using Matrix2 = Matrix<2, 2>;
using Matrix3 = Matrix<3, 3>;
using Matrix4 = Matrix<4, 4>;
using Matrix5 = Matrix<5, 5>;
using Matrix6 = Matrix<6, 6>;
using Matrix9 = Matrix<9, 9>;

} // namespace cityfly::common
