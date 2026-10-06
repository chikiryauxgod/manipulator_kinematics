#include "support.h"
#include <cfenv>

TEST(Arithmetic, VerifiedHostRuntime)
{
    EXPECT_TRUE(std::numeric_limits<mk_real>::is_iec559);
    EXPECT_EQ(std::fegetround(),FE_TONEAREST);
    volatile mk_real small=std::numeric_limits<mk_real>::denorm_min();
    volatile mk_real one=1;
    EXPECT_EQ(small*one,small);
    EXPECT_GT(small*one,0);
}

TEST(Transform, IdentityAndNull)
{
    EXPECT_EQ(mk_transform_identity(nullptr), MK_INVALID_ARGUMENT);
    auto t = sentinel();
    ASSERT_EQ(mk_transform_identity(&t), MK_OK);
    for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) EXPECT_EQ(t.m[i][j], i == j);
    EXPECT_EQ(mk_transform_validate(nullptr), MK_INVALID_ARGUMENT);
    EXPECT_EQ(mk_transform_validate(&t), MK_OK);
}

TEST(Transform, InvalidMatricesAndNonfinite)
{
    for (mk_real value : {std::numeric_limits<mk_real>::infinity(),
                         -std::numeric_limits<mk_real>::infinity(),
                         std::numeric_limits<mk_real>::quiet_NaN()}) {
        for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) {
            auto t = identity(); t.m[i][j] = value;
            EXPECT_EQ(mk_transform_validate(&t), MK_NONFINITE_INPUT);
        }
    }
    auto t = identity(); t.m[0][0] = -1;
    EXPECT_EQ(mk_transform_validate(&t), MK_INVALID_TRANSFORM);
    t = identity(); t.m[0][0] = 2;
    EXPECT_EQ(mk_transform_validate(&t), MK_INVALID_TRANSFORM);
    t = identity(); t.m[1][0] = (mk_real)0.01;
    EXPECT_EQ(mk_transform_validate(&t), MK_INVALID_TRANSFORM);
    t = identity(); t.m[0][0] = std::numeric_limits<mk_real>::max();
    EXPECT_EQ(mk_transform_validate(&t), MK_INVALID_TRANSFORM);
    for (int j = 0; j < 4; ++j) {
        t = identity(); t.m[3][j] += (mk_real)1e-6;
        EXPECT_EQ(mk_transform_validate(&t), MK_INVALID_TRANSFORM);
    }
    t = identity(); t.m[3][0] = -mk_real(0);
    EXPECT_EQ(mk_transform_validate(&t), MK_OK);
}

TEST(Transform, ComposeNoncommutingRotationsAndTranslation)
{
    auto z = identity(), x = identity(), out = sentinel();
    z.m[0][0]=0; z.m[0][1]=-1; z.m[1][0]=1; z.m[1][1]=0;
    z.m[0][3]=1; z.m[1][3]=2;
    x.m[1][1]=0; x.m[1][2]=-1; x.m[2][1]=1; x.m[2][2]=0;
    x.m[0][3]=3; x.m[1][3]=4; x.m[2][3]=5;
    mk_transform expected = {{{0,0,1,-3}, {1,0,0,5}, {0,1,0,5}, {0,0,0,1}}};
    ASSERT_EQ(mk_transform_compose(&z, &x, &out), MK_OK);
    matrix_near(out, expected, 0);
    auto reverse = identity();
    ASSERT_EQ(mk_transform_compose(&x, &z, &reverse), MK_OK);
    EXPECT_NE(reverse.m[0][1], out.m[0][1]);
    auto aliased = z;
    ASSERT_EQ(mk_transform_compose(&aliased, &x, &aliased), MK_OK);
    matrix_near(aliased, out, 0);
    aliased = x;
    ASSERT_EQ(mk_transform_compose(&z, &aliased, &aliased), MK_OK);
    matrix_near(aliased, out, 0);
}

TEST(Transform, AllFailuresPreserveOutput)
{
    auto a=identity(), b=identity(), out=sentinel(), before=out;
    EXPECT_EQ(mk_transform_compose(nullptr, &b, &out), MK_INVALID_ARGUMENT);
    same_bytes(out,before);
    EXPECT_EQ(mk_transform_compose(&a, nullptr, &out), MK_INVALID_ARGUMENT);
    EXPECT_EQ(mk_transform_compose(&a, &b, nullptr), MK_INVALID_ARGUMENT);
    a.m[0][0]=2;
    EXPECT_EQ(mk_transform_compose(&a, &b, &out), MK_INVALID_TRANSFORM);
    same_bytes(out,before);
    a=identity(); b.m[0][0]=2;
    EXPECT_EQ(mk_transform_compose(&a, &b, &out), MK_INVALID_TRANSFORM);
    same_bytes(out,before);
    b=identity(); b.m[0][3]=std::numeric_limits<mk_real>::infinity();
    EXPECT_EQ(mk_transform_compose(&a, &b, &out), MK_NONFINITE_INPUT);
    same_bytes(out,before);
    for (mk_real sign : {mk_real(1), mk_real(-1)}) {
        a=identity(); b=identity();
        a.m[0][3]=sign*std::numeric_limits<mk_real>::max(); b.m[0][3]=a.m[0][3];
        EXPECT_EQ(mk_transform_compose(&a, &b, &out), MK_NUMERIC_ERROR);
        same_bytes(out,before);
    }
}

TEST(Transform, RangeCancellationAndSubnormal)
{
    auto a=identity(), b=identity(), out=identity();
    a.m[0][3]=std::numeric_limits<mk_real>::max(); b.m[0][3]=-a.m[0][3];
    ASSERT_EQ(mk_transform_compose(&a, &b, &out), MK_OK);
    EXPECT_EQ(out.m[0][3],0);
    a=identity(); b=identity(); b.m[0][3]=std::numeric_limits<mk_real>::denorm_min();
    ASSERT_EQ(mk_transform_compose(&a,&b,&out),MK_OK);
    EXPECT_EQ(out.m[0][3],b.m[0][3]);
    EXPECT_GT(out.m[0][3],0);
}

TEST(Transform, MultiplicationOverflowIsControlled)
{
    auto a=identity(), b=identity(), out=sentinel(), before=out;
    mk_real c=(mk_real)std::sqrt(0.5L);
    a.m[0][0]=c; a.m[0][1]=c; a.m[1][0]=-c; a.m[1][1]=c;
    b.m[0][3]=std::numeric_limits<mk_real>::max(); b.m[1][3]=b.m[0][3];
    EXPECT_EQ(mk_transform_compose(&a,&b,&out),MK_NUMERIC_ERROR);
    same_bytes(out,before);
    a=identity(); a.m[0][0]=1+(mk_real)(rotation_tolerance/8);
    ASSERT_EQ(mk_transform_validate(&a),MK_OK);
    b=identity(); b.m[0][3]=std::numeric_limits<mk_real>::max();
    EXPECT_EQ(mk_transform_compose(&a,&b,&out),MK_NUMERIC_ERROR);
    same_bytes(out,before);
}

TEST(Transform, ResultDriftIsNumericErrorAndOutputIsUnchanged)
{
    auto a=identity(),out=sentinel(),before=out;
    a.m[0][0]+= (mk_real)(rotation_tolerance*0.4);
    ASSERT_EQ(mk_transform_validate(&a),MK_OK);
    EXPECT_EQ(mk_transform_compose(&a,&a,&out),MK_NUMERIC_ERROR);
    same_bytes(out,before);
}
