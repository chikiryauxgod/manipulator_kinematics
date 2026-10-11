#include "support.h"
#include <manipulator/inverse.h>
#include <array>
#include <cstdint>

extern "C" k_status k_internal_jacobian(const k_model *, const k_real *, const k_transform *,
                                        k_real *);
extern "C" k_status k_internal_linear3(k_real[3][3], const k_real[3], k_real[3]);
extern "C" k_status k_internal_dls(k_real *, size_t, const k_real[3], const k_ik_options *,
                                   k_real *, k_real[3], k_real *);

static k_ik_options options(const std::vector<k_real> &scales, const std::vector<k_real> &steps,
                            size_t iterations = 500)
{
    return {iterations,
            16,
            (k_real)(K_USE_FLOAT ? 2e-5 : 1e-9),
            1,
            (k_real)0.005,
            scales.data(),
            steps.data(),
            scales.size()};
}

class InverseFamily : public testing::TestWithParam<int>
{
};
TEST_P(InverseFamily, PositionOnlyIndependentGeometry)
{
    size_t n = (size_t)GetParam();
    std::vector<k_dh_joint> geometry(n, joint((k_real)0.15));
    geometry[0] = {K_REVOLUTE, 0, (k_real)(pi / 2), (k_real)0.1, 0};
    TestModel model;
    ASSERT_EQ(test_model_init(&model, geometry.data(), n), K_OK);
    std::vector<k_real> scales(n, 1), steps(n, (k_real)0.2), initial(n, (k_real)0.15);
    auto config = options(scales, steps);
    long double angle = 0, radial = 0, height = .1;
    for (size_t i = 1; i < n; ++i)
    {
        angle += .2L + .02L * i;
        radial += (long double)geometry[i].a * std::cos(angle);
        height += (long double)geometry[i].a * std::sin(angle);
    }
    k_vec3 target{(k_real)(std::cos(.3L) * radial), (k_real)(std::sin(.3L) * radial),
                  (k_real)height};
    size_t needed = 0;
    ASSERT_EQ(k_ik_workspace_size(n, &needed), K_OK);
    std::vector<k_real> work(needed + 1, 77), result(n + 1, 88);
    const auto before = initial;
    k_ik_report report{};
    ASSERT_EQ(k_inverse_position(&model, &target, initial.data(), n, &config, work.data(), needed,
                                 result.data(), n, &report),
              K_OK);
    EXPECT_EQ(report.termination, K_OK);
    EXPECT_TRUE(report.residual_valid);
    EXPECT_LE(report.residual, config.position_tolerance);
    EXPECT_LE(report.iteration_attempts, config.max_iterations);
    EXPECT_LE(report.fk_evaluations,
              1 + 2 * config.max_backtracking_trials * report.iteration_attempts);
    EXPECT_EQ(result[n], 88);
    EXPECT_EQ(work[needed], 77);
    EXPECT_EQ(initial, before);
    k_transform tool{};
    ASSERT_EQ(k_forward(&model, result.data(), n, &tool), K_OK);
    EXPECT_NEAR(tool.m[0][3], target.x, config.position_tolerance);
    EXPECT_NEAR(tool.m[1][3], target.y, config.position_tolerance);
    EXPECT_NEAR(tool.m[2][3], target.z, config.position_tolerance);
}
INSTANTIATE_TEST_SUITE_P(Sizes, InverseFamily, testing::Values(3, 4, 5, 6, 7, 16, 64));

TEST(Inverse, FixedAndActiveLimitsDoNotDistortSearchDirection)
{
    k_dh_joint geometry[] = {{K_REVOLUTE, 1, (k_real)(pi / 2), 0, (k_real)(pi / 2)},
                             {K_PRISMATIC, 0, (k_real)(pi / 2), 1, (k_real)(pi / 2)},
                             {K_PRISMATIC, 0, 0, -2, 0}};
    std::vector<k_real> scales{1, 1, (k_real).1}, steps{1, 1, 10};
    auto config = options(scales, steps, 1);
    config.damping = (k_real).1;
    k_real initial[3] = {0, 0, 0}, work[18], output[3] = {91, 92, 93};
    k_vec3 target{(k_real)1.2, 0, 0};
    for (auto limits : {k_joint_limits{K_BOUNDED, 0, 0}, k_joint_limits{K_BOUNDED, -1, 0}})
    {
        TestModel model;
        ASSERT_EQ(test_model_init(&model, geometry, 3), K_OK);
        model.joints[0].limits = limits;
        k_ik_report report{};
        EXPECT_EQ(
            k_inverse_position(&model, &target, initial, 3, &config, work, 18, output, 3, &report),
            K_NO_CONVERGENCE);
        EXPECT_EQ(report.accepted_steps, 1u);
        EXPECT_LT(report.residual, (k_real)std::sqrt(1.04));
        EXPECT_EQ(output[0], 91);
        EXPECT_EQ(output[1], 92);
        EXPECT_EQ(output[2], 93);
        if (limits.min == limits.max)
        {
            config.max_iterations = 500;
            ASSERT_EQ(k_inverse_position(&model, &target, initial, 3, &config, work, 18, output, 3,
                                         &report),
                      K_OK);
            EXPECT_EQ(output[0], 0);
            config.max_iterations = 1;
            output[0] = 91;
            output[1] = 92;
            output[2] = 93;
        }
    }
}

TEST(Inverse, JacobianMatchesCentralDifferencesForMixedSpatialChain)
{
    k_dh_joint geometry[] = {{K_REVOLUTE, (k_real).3, (k_real).8, (k_real).1, (k_real).2},
                             {K_PRISMATIC, (k_real).2, (k_real)-.6, (k_real).1, (k_real).4},
                             {K_REVOLUTE, (k_real).1, (k_real).3, (k_real)-.2, (k_real)-.2}};
    TestModel model;
    ASSERT_EQ(test_model_init(&model, geometry, 3), K_OK);
    model.world_base = {{{0, -1, 0, 2}, {1, 0, 0, -1}, {0, 0, 1, 3}, {0, 0, 0, 1}}};
    model.last_tool.m[0][3] = (k_real).2;
    model.last_tool.m[2][3] = (k_real).1;
    k_real positions[3] = {(k_real).3, (k_real).2, (k_real)-.4}, columns[9];
    k_transform tool{};
    ASSERT_EQ(k_forward(&model, positions, 3, &tool), K_OK);
    ASSERT_EQ(k_internal_jacobian(&model, positions, &tool, columns), K_OK);
    const k_real h = K_USE_FLOAT ? (k_real).002 : (k_real)1e-5;
    for (size_t i = 0; i < 3; ++i)
    {
        auto original = positions[i];
        k_transform plus{}, minus{};
        positions[i] = original + h;
        ASSERT_EQ(k_forward(&model, positions, 3, &plus), K_OK);
        positions[i] = original - h;
        ASSERT_EQ(k_forward(&model, positions, 3, &minus), K_OK);
        positions[i] = original;
        for (size_t r = 0; r < 3; ++r)
            EXPECT_NEAR(columns[3 * i + r], (plus.m[r][3] - minus.m[r][3]) / (2 * h),
                        K_USE_FLOAT ? 2e-4 : 2e-9);
    }
}

TEST(Inverse, LinearSystemAndEffectiveRegularization)
{
    k_real matrix[3][3] = {{4, 1, 1}, {1, 3, 0}, {1, 0, 2}}, rhs[3] = {9, 7, 7},
           out[3] = {91, 92, 93};
    ASSERT_EQ(k_internal_linear3(matrix, rhs, out), K_OK);
    EXPECT_NEAR(out[0], 1, rotation_tolerance);
    EXPECT_NEAR(out[1], 2, rotation_tolerance);
    EXPECT_NEAR(out[2], 3, rotation_tolerance);
    matrix[0][0] = -1;
    auto before = std::array<k_real, 3>{out[0], out[1], out[2]};
    EXPECT_EQ(k_internal_linear3(matrix, rhs, out), K_NUMERIC_ERROR);
    EXPECT_EQ(std::memcmp(out, before.data(), sizeof(out)), 0);
    std::vector<k_real> scales{1}, steps{1};
    auto config = options(scales, steps);
    config.damping = (k_real)1e-20;
    k_real columns[] = {1000, 1000, 0}, error[] = {1, 1, 0}, step[1], gradient[3], effective;
    EXPECT_EQ(k_internal_dls(columns, 1, error, &config, step, gradient, &effective), K_OK);
    EXPECT_GT(effective, config.damping);
    EXPECT_TRUE(std::isfinite(step[0]));
    EXPECT_GT(step[0], 0);
}

TEST(Inverse, ErrorsPreserveOutputAndReportLastAcceptedPosition)
{
    auto geometry = joint(1);
    TestModel model;
    ASSERT_EQ(test_model_init(&model, &geometry, 1), K_OK);
    std::vector<k_real> scales{1}, steps{(k_real).1};
    auto config = options(scales, steps, 1);
    k_real initial = 0, work[6] = {}, output = 91;
    k_ik_report report{};
    k_vec3 target{2, 0, 0};
    EXPECT_EQ(
        k_inverse_position(&model, &target, &initial, 1, &config, work, 6, &output, 1, &report),
        K_STALLED);
    EXPECT_EQ(output, 91);
    EXPECT_EQ(report.residual, 1);
    EXPECT_EQ(report.accepted_steps, 0u);
    target = {0, 1, 0};
    EXPECT_EQ(
        k_inverse_position(&model, &target, &initial, 1, &config, work, 6, &output, 1, &report),
        K_NO_CONVERGENCE);
    EXPECT_EQ(output, 91);
    EXPECT_EQ(report.accepted_steps, 1u);
    EXPECT_LT(report.residual, (k_real)std::sqrt(2));
    EXPECT_EQ(
        k_inverse_position(&model, &target, &initial, 1, &config, work, 5, &output, 1, &report),
        K_BUFFER_TOO_SMALL);
    EXPECT_FALSE(report.residual_valid);
    config.damping = 0;
    EXPECT_EQ(
        k_inverse_position(&model, &target, &initial, 1, &config, work, 6, &output, 1, &report),
        K_INVALID_OPTIONS);
    config.damping = (k_real).1;
    target.x = std::numeric_limits<k_real>::quiet_NaN();
    EXPECT_EQ(
        k_inverse_position(&model, &target, &initial, 1, &config, work, 6, &output, 1, &report),
        K_NONFINITE_INPUT);
    EXPECT_EQ(output, 91);
    size_t needed = 88;
    EXPECT_EQ(k_ik_workspace_size(SIZE_MAX, &needed), K_INVALID_MODEL);
    EXPECT_EQ(needed, 88u);
    EXPECT_EQ(k_ik_workspace_size(0, &needed), K_INVALID_MODEL);
    EXPECT_EQ(k_ik_workspace_size(1, nullptr), K_INVALID_ARGUMENT);
}

TEST(Inverse, AlreadyAtTargetLastIterationAndSeparateInstances)
{
    k_dh_joint geometry{K_PRISMATIC, 0, 0, 0, 0};
    TestModel first, second;
    ASSERT_EQ(test_model_init(&first, &geometry, 1), K_OK);
    ASSERT_EQ(test_model_init(&second, &geometry, 1), K_OK);
    std::vector<k_real> scales{1}, steps{1};
    auto config = options(scales, steps, 1);
    config.damping = (k_real)1e-6;
    k_real initial = 0, work[6], output = 91;
    k_vec3 target{0, 0, 0};
    k_ik_report report{};
    ASSERT_EQ(
        k_inverse_position(&first, &target, &initial, 1, &config, work, 6, &output, 1, &report),
        K_OK);
    EXPECT_EQ(report.iteration_attempts, 0u);
    EXPECT_EQ(report.fk_evaluations, 1u);
    EXPECT_EQ(output, 0);
    target.z = (k_real).1;
    ASSERT_EQ(
        k_inverse_position(&second, &target, &initial, 1, &config, work, 6, &output, 1, &report),
        K_OK);
    EXPECT_EQ(report.accepted_steps, 1u);
    EXPECT_EQ(report.fk_evaluations, 2u);
    first.joints[0].limits = {K_BOUNDED, 0, 0};
    EXPECT_EQ(
        k_inverse_position(&first, &target, &initial, 1, &config, work, 6, &output, 1, &report),
        K_STALLED);
    ASSERT_EQ(
        k_inverse_position(&second, &target, &initial, 1, &config, work, 6, &output, 1, nullptr),
        K_OK);
}

TEST(Inverse, InvalidArgumentsOptionsAndBoundsPreserveOutput)
{
    k_dh_joint geometry{K_PRISMATIC, 0, 0, 0, 0};
    TestModel model;
    ASSERT_EQ(test_model_init(&model, &geometry, 1), K_OK);
    std::vector<k_real> scales{1}, steps{1};
    auto config = options(scales, steps);
    k_real initial = 0, work[6], output = 91;
    k_vec3 target{0, 0, 1};
    k_ik_report report{};
    auto call = [&](const k_model *m, const k_vec3 *t, const k_real *seed, size_t count,
                    const k_ik_options *o, k_real *w, size_t capacity, k_real *out,
                    size_t output_capacity) {
        return k_inverse_position(m, t, seed, count, o, w, capacity, out, output_capacity, &report);
    };
    EXPECT_EQ(call(nullptr, &target, &initial, 1, &config, work, 6, &output, 1),
              K_INVALID_ARGUMENT);
    EXPECT_EQ(call(&model, nullptr, &initial, 1, &config, work, 6, &output, 1), K_INVALID_ARGUMENT);
    EXPECT_EQ(call(&model, &target, nullptr, 1, &config, work, 6, &output, 1), K_INVALID_ARGUMENT);
    EXPECT_EQ(call(&model, &target, &initial, 0, &config, work, 6, &output, 1), K_SIZE_MISMATCH);
    EXPECT_EQ(call(&model, &target, &initial, 1, nullptr, work, 6, &output, 1), K_INVALID_ARGUMENT);
    EXPECT_EQ(call(&model, &target, &initial, 1, &config, nullptr, 6, &output, 1),
              K_INVALID_ARGUMENT);
    EXPECT_EQ(call(&model, &target, &initial, 1, &config, work, 6, nullptr, 1), K_INVALID_ARGUMENT);
    EXPECT_EQ(call(&model, &target, &initial, 1, &config, work, 6, &output, 0), K_BUFFER_TOO_SMALL);
    auto valid = config;
    for (k_real value : {(k_real)0, (k_real)-1, std::numeric_limits<k_real>::infinity(),
                         std::numeric_limits<k_real>::quiet_NaN()})
    {
        config.position_tolerance = value;
        EXPECT_EQ(call(&model, &target, &initial, 1, &config, work, 6, &output, 1),
                  K_INVALID_OPTIONS);
        config = valid;
        scales[0] = value;
        EXPECT_EQ(call(&model, &target, &initial, 1, &config, work, 6, &output, 1),
                  K_INVALID_OPTIONS);
        scales[0] = 1;
        steps[0] = value;
        EXPECT_EQ(call(&model, &target, &initial, 1, &config, work, 6, &output, 1),
                  K_INVALID_OPTIONS);
        steps[0] = 1;
    }
    config.max_iterations = SIZE_MAX;
    EXPECT_EQ(call(&model, &target, &initial, 1, &config, work, 6, &output, 1), K_INVALID_OPTIONS);
    config = valid;
    config.max_backtracking_trials = SIZE_MAX;
    EXPECT_EQ(call(&model, &target, &initial, 1, &config, work, 6, &output, 1), K_INVALID_OPTIONS);
    config = valid;
    config.joint_count = 2;
    EXPECT_EQ(call(&model, &target, &initial, 1, &config, work, 6, &output, 1), K_SIZE_MISMATCH);
    config = valid;
    initial = std::numeric_limits<k_real>::infinity();
    EXPECT_EQ(call(&model, &target, &initial, 1, &config, work, 6, &output, 1), K_NONFINITE_INPUT);
    initial = 0;
    model.joints[0].limits = {K_BOUNDED, 1, 2};
    EXPECT_EQ(call(&model, &target, &initial, 1, &config, work, 6, &output, 1), K_JOINT_LIMIT);
    k_model incomplete{};
    EXPECT_EQ(call(&incomplete, &target, &initial, 1, &config, work, 6, &output, 1), K_NOT_READY);
    EXPECT_EQ(output, 91);
    EXPECT_FALSE(report.residual_valid);
}

TEST(Inverse, StepLimitsDoNotChangeRegularizedDirection)
{
    std::vector<k_real> scales{1, 2}, limits{100, 100};
    auto config = options(scales, limits);
    k_real columns[6] = {1, 1, 0, 0, 1, 1}, copy[6], error[3] = {1, 2, 3};
    std::memcpy(copy, columns, sizeof(copy));
    k_real first[2], second[2], gradient[3], damping;
    ASSERT_EQ(k_internal_dls(columns, 2, error, &config, first, gradient, &damping), K_OK);
    limits[0] = (k_real).001;
    ASSERT_EQ(k_internal_dls(copy, 2, error, &config, second, gradient, &damping), K_OK);
    EXPECT_NEAR(first[0] * second[1], first[1] * second[0], K_USE_FLOAT ? 1e-7 : 1e-12);
    EXPECT_LE(std::fabs(second[0]), limits[0]);
}

TEST(Inverse, SmallLengthScaleKeepsRegularizationRepresentable)
{
    k_dh_joint geometry{K_PRISMATIC, 0, 0, 0, 0};
    TestModel model;
    ASSERT_EQ(test_model_init(&model, &geometry, 1), K_OK);
    std::vector<k_real> scales{1}, steps{1};
    auto config = options(scales, steps);
    config.length_scale = std::numeric_limits<k_real>::min();
    k_real initial = 0, work[6], output = 91;
    k_vec3 target{0, 0, 1};
    k_ik_report report{};
    EXPECT_EQ(
        k_inverse_position(&model, &target, &initial, 1, &config, work, 6, &output, 1, &report),
        K_OK);
    EXPECT_TRUE(report.residual_valid);
    EXPECT_LE(report.residual, config.position_tolerance);
}

TEST(Inverse, NormalizationAvoidsIntermediateOverflowAndUnderflow)
{
    const k_real largest = std::numeric_limits<k_real>::max();
    const k_real smallest = std::numeric_limits<k_real>::min();
    struct Case
    {
        k_real column, joint_scale, length_scale, damping, error, expected;
    };
    const Case cases[] = {
        {4, largest * (k_real).5, largest * (k_real).5, (k_real).1, 2, (k_real)(8.0 / 16.01)},
        {1, 1, smallest, (k_real).005, 8, 8},
        {smallest, smallest, smallest, smallest, smallest, (k_real).5},
        {smallest, largest * (k_real).5, largest * (k_real).5, smallest, smallest, (k_real).5},
        {-4, largest * (k_real).5, largest * (k_real).5, (k_real).1, 2, (k_real)(-8.0 / 16.01)}};
    for (const auto &item : cases)
    {
        std::vector<k_real> scales{item.joint_scale}, steps{16};
        auto config = options(scales, steps);
        config.length_scale = item.length_scale;
        config.damping = item.damping;
        k_real columns[3] = {0, 0, item.column}, error[3] = {0, 0, item.error};
        k_real step[1], gradient[3], effective;
        ASSERT_EQ(k_internal_dls(columns, 1, error, &config, step, gradient, &effective), K_OK);
        EXPECT_NEAR(step[0], item.expected, K_USE_FLOAT ? 1e-4 : 1e-12);
        EXPECT_TRUE(std::isfinite(effective));
        EXPECT_GT(effective, 0);
    }
}

TEST(Inverse, UnrepresentableNormalizationPreservesOutput)
{
    k_dh_joint geometry{K_PRISMATIC, 0, 0, 0, 0};
    TestModel model;
    ASSERT_EQ(test_model_init(&model, &geometry, 1), K_OK);
    std::vector<k_real> scales{16}, steps{1};
    auto config = options(scales, steps);
    config.length_scale = std::numeric_limits<k_real>::min();
    k_real initial = 0, work[6], output = 91;
    k_vec3 target{0, 0, 1};
    k_ik_report report{};
    EXPECT_EQ(
        k_inverse_position(&model, &target, &initial, 1, &config, work, 6, &output, 1, &report),
        K_NUMERIC_ERROR);
    EXPECT_EQ(output, 91);
    EXPECT_EQ(report.termination, K_NUMERIC_ERROR);
    EXPECT_TRUE(report.residual_valid);
    EXPECT_EQ(report.residual, 1);
    EXPECT_EQ(report.accepted_steps, 0u);
}

TEST(Inverse, NumericCandidateFailuresUseSmallerTrialsAndPreserveDiagnostics)
{
    const k_real largest = std::numeric_limits<k_real>::max();
    k_dh_joint geometry{K_REVOLUTE, largest * (k_real).5, 0, 0, 0};
    TestModel model;
    ASSERT_EQ(test_model_init(&model, &geometry, 1), K_OK);
    model.world_base.m[0][3] = largest * (k_real).75;
    std::vector<k_real> scales{1}, limits{2};
    auto config = options(scales, limits, 1);
    config.length_scale = largest;
    config.max_backtracking_trials = 8;
    for (k_real initial : {(k_real)1.2, (k_real)(pi / 3 + 1e-6L)})
    {
        k_transform initial_tool{};
        ASSERT_EQ(k_forward(&model, &initial, 1, &initial_tool), K_OK);
        k_vec3 target{initial_tool.m[0][3], -initial_tool.m[1][3], 0};
        k_real work[6], output = 91;
        k_ik_report report{};
        auto status =
            k_inverse_position(&model, &target, &initial, 1, &config, work, 6, &output, 1, &report);
        EXPECT_EQ(output, 91);
        EXPECT_TRUE(report.residual_valid);
        if (initial > (k_real)1.1)
        {
            EXPECT_EQ(status, K_NO_CONVERGENCE);
            EXPECT_EQ(report.accepted_steps, 1u);
            EXPECT_GT(report.fk_evaluations, 2u);
            EXPECT_LT(report.residual, initial_tool.m[1][3] * 2);
        }
        else
        {
            EXPECT_EQ(status, K_NUMERIC_ERROR);
            EXPECT_EQ(report.accepted_steps, 0u);
            EXPECT_EQ(report.fk_evaluations, 17u);
            EXPECT_EQ(report.residual, initial_tool.m[1][3] * 2);
        }
    }
}
