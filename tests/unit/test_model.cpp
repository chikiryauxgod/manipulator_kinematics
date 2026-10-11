#include "support.h"
#include <cstdint>

TEST(Model, RequiresEveryExplicitSettingAndResetsAllFlags)
{
    k_model model{};
    k_joint_config storage[2]{};
    EXPECT_EQ(k_model_validate(&model), K_NOT_READY);
    const auto transform = identity();
    const auto geometry = joint();
    const k_joint_limits limits{K_UNBOUNDED,0,0};
    for(int missing=0;missing<6;++missing)
    {
        ASSERT_EQ(k_model_begin(&model,storage,2,2),K_OK);
        EXPECT_EQ(k_model_validate(&model),K_NOT_READY);
        for(size_t i=0;i<2;++i)
        {
            if(missing!=(int)(2*i)) ASSERT_EQ(k_model_set_joint(&model,i,&geometry),K_OK);
            if(missing!=(int)(2*i+1)) ASSERT_EQ(k_model_set_limits(&model,i,&limits),K_OK);
        }
        if(missing!=4) ASSERT_EQ(k_model_set_base(&model,&transform),K_OK);
        if(missing!=5) ASSERT_EQ(k_model_set_tool(&model,&transform),K_OK);
        EXPECT_EQ(k_model_finalize(&model),K_INCOMPLETE_CONFIGURATION);
        EXPECT_EQ(model.state,K_CONFIGURING);
    }
    ASSERT_EQ(k_model_set_tool(&model,&transform),K_OK);
    ASSERT_EQ(k_model_finalize(&model),K_OK);
    EXPECT_EQ(k_model_validate(&model),K_OK);
    EXPECT_EQ(k_model_set_joint(&model,0,&geometry),K_INVALID_STATE);
    ASSERT_EQ(k_model_begin(&model,storage,2,1),K_OK);
    EXPECT_EQ(model.configured,0u);
    EXPECT_EQ(storage[0].configured,0u);
    EXPECT_EQ(k_model_finalize(&model),K_INCOMPLETE_CONFIGURATION);
}

TEST(Model, InvalidBeginPreservesModelStorageAndTail)
{
    TestModel model;
    auto geometry=joint();
    ASSERT_EQ(test_model_init(&model,&geometry,1),K_OK);
    const k_model before=model;
    auto storage=model.storage;
    EXPECT_EQ(k_model_begin(nullptr,model.storage.data(),1,1),K_INVALID_ARGUMENT);
    EXPECT_EQ(k_model_begin(&model,nullptr,1,1),K_INVALID_ARGUMENT);
    EXPECT_EQ(k_model_begin(&model,model.storage.data(),1,0),K_INVALID_MODEL);
    EXPECT_EQ(k_model_begin(&model,model.storage.data(),1,SIZE_MAX),K_INVALID_MODEL);
    EXPECT_EQ(k_model_begin(&model,model.storage.data(),0,1),K_BUFFER_TOO_SMALL);
    EXPECT_EQ(std::memcmp(&model,&before,sizeof(before)),0);
    EXPECT_EQ(std::memcmp(model.storage.data(),storage.data(),sizeof(k_joint_config)),0);
    k_joint_config separate[2]{}; separate[1].configured=91;
    ASSERT_EQ(k_model_begin(&model,separate,2,1),K_OK);
    EXPECT_EQ(separate[1].configured,91u);
}

TEST(Model, InvalidSettersAreAtomic)
{
    k_joint_config storage[1]{}; k_model model{};
    ASSERT_EQ(k_model_begin(&model,storage,1,1),K_OK);
    const auto before=storage[0];
    auto geometry=joint();
    geometry.a=std::numeric_limits<k_real>::quiet_NaN();
    EXPECT_EQ(k_model_set_joint(&model,0,&geometry),K_NONFINITE_INPUT);
    EXPECT_EQ(k_model_set_joint(&model,0,nullptr),K_INVALID_ARGUMENT);
    EXPECT_EQ(k_model_set_joint(&model,1,&geometry),K_SIZE_MISMATCH);
    geometry=joint(); geometry.type=(k_joint_type)99;
    EXPECT_EQ(k_model_set_joint(&model,0,&geometry),K_INVALID_MODEL);
    k_joint_limits limits{K_BOUNDED,2,1};
    EXPECT_EQ(k_model_set_limits(&model,0,&limits),K_INVALID_MODEL);
    limits.max=std::numeric_limits<k_real>::infinity();
    EXPECT_EQ(k_model_set_limits(&model,0,&limits),K_NONFINITE_INPUT);
    limits.mode=(k_limit_mode)99;
    EXPECT_EQ(k_model_set_limits(&model,0,&limits),K_INVALID_MODEL);
    EXPECT_EQ(std::memcmp(storage,&before,sizeof(before)),0);
    auto transform=identity(); transform.m[0][0]=2;
    EXPECT_EQ(k_model_set_base(&model,&transform),K_INVALID_TRANSFORM);
    EXPECT_EQ(model.configured,0u);
}

TEST(Model, CoordinateLimitsAndZeroValues)
{
    auto geometry=joint(); TestModel model;
    ASSERT_EQ(test_model_init(&model,&geometry,1),K_OK);
    model.joints[0].limits={K_BOUNDED,0,0};
    k_real position=0; auto out=sentinel();
    EXPECT_EQ(k_forward(&model,&position,1,&out),K_OK);
    const auto before=out;
    position=1;
    EXPECT_EQ(k_forward(&model,&position,1,&out),K_JOINT_LIMIT);
    same_bytes(out,before);
    model.joints[0].limits={K_BOUNDED,-1,1};
    EXPECT_EQ(k_forward(&model,&position,1,&out),K_OK);
    model.storage_capacity=0;
    EXPECT_EQ(k_model_validate(&model),K_INVALID_MODEL);
}

TEST(Model, ValidatesBaseToolAndFiniteGeometry)
{
    auto geometry=joint(); TestModel model;
    ASSERT_EQ(test_model_init(&model,&geometry,1),K_OK);
    model.world_base.m[0][0]=2;
    EXPECT_EQ(k_model_validate(&model),K_INVALID_TRANSFORM);
    model.world_base=identity(); model.last_tool.m[3][0]=(k_real)1e-10;
    EXPECT_EQ(k_model_validate(&model),K_INVALID_TRANSFORM);
    model.last_tool=identity(); model.joints[0].geometry.alpha=std::numeric_limits<k_real>::infinity();
    EXPECT_EQ(k_model_validate(&model),K_NONFINITE_INPUT);
}
