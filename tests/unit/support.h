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
constexpr double rotation_tolerance = K_USE_FLOAT ? 2e-5 : 2e-12;
inline k_transform identity()
{
    k_transform t{};
    EXPECT_EQ(k_transform_identity(&t), K_OK);
    return t;
}
inline k_transform sentinel()
{
    k_transform t;
    for (auto &row : t.m) for (auto &x : row) x = 37;
    return t;
}
inline void same_bytes(const k_transform &a, const k_transform &b)
{ EXPECT_EQ(std::memcmp(&a, &b, sizeof(a)), 0); }
inline void matrix_near(const k_transform &a, const k_transform &b, double tol = rotation_tolerance)
{
    for (size_t i = 0; i < 4; ++i)
        for (size_t j = 0; j < 4; ++j) EXPECT_NEAR(a.m[i][j], b.m[i][j], tol) << i << ',' << j;
}
inline k_dh_joint joint(k_real a = 0)
{ return {K_REVOLUTE, a, 0, 0, 0}; }
struct TestModel : k_model
{
    std::vector<k_joint_config> storage;
    TestModel() : k_model{} {}
};
inline k_status test_model_init(TestModel *model, const k_dh_joint *joints, size_t count)
{
    model->storage.resize(count);
    k_status status = k_model_begin(model, model->storage.data(), count, count);
    const k_joint_limits limits{K_UNBOUNDED, 0, 0};
    const auto transform = identity();
    for (size_t i = 0; status == K_OK && i < count; ++i)
    {
        status = k_model_set_joint(model, i, &joints[i]);
        if (status == K_OK) status = k_model_set_limits(model, i, &limits);
    }
    if (status == K_OK) status = k_model_set_base(model, &transform);
    if (status == K_OK) status = k_model_set_tool(model, &transform);
    return status == K_OK ? k_model_finalize(model) : status;
}
#endif
