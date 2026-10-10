#ifndef TEST_SUPPORT_H
#define TEST_SUPPORT_H
#include <gtest/gtest.h>
#include <manipulator/forward.h>
#include <manipulator/transform.h>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>
constexpr long double pi = 3.141592653589793238462643383279502884L;
constexpr double rotation_tolerance = MANIPULATOR_USE_FLOAT ? 2e-5 : 2e-12;
inline manipulator_transform identity()
{
    manipulator_transform t{};
    EXPECT_EQ(manipulator_transform_identity(&t), MANIPULATOR_OK);
    return t;
}
inline manipulator_transform sentinel()
{
    manipulator_transform t;
    for (auto &row : t.m) for (auto &x : row) x = 37;
    return t;
}
inline void same_bytes(const manipulator_transform &a, const manipulator_transform &b)
{ EXPECT_EQ(std::memcmp(&a, &b, sizeof(a)), 0); }
inline void matrix_near(const manipulator_transform &a, const manipulator_transform &b, double tol = rotation_tolerance)
{
    for (size_t i = 0; i < 4; ++i)
        for (size_t j = 0; j < 4; ++j) EXPECT_NEAR(a.m[i][j], b.m[i][j], tol) << i << ',' << j;
}
inline manipulator_dh_joint joint(manipulator_real a = 0)
{ return {MANIPULATOR_REVOLUTE, a, 0, 0, 0}; }
#endif
