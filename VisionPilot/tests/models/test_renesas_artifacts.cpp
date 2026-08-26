#include <gtest/gtest.h>
#include <engine/onnx_engine.hpp>

#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;
using visionpilot::engine::resolve_renesas_artifacts;

namespace {

class ArtifactsDir : public ::testing::Test {
protected:
    void SetUp() override
    {
        root_ = fs::temp_directory_path() / "vp_artifacts_test";
        fs::remove_all(root_);
        dir_ = root_ / "v7_frozen_s3cal_artifacts";
        fs::create_directories(dir_);
    }
    void TearDown() override { fs::remove_all(root_); }

    void touch(const fs::path& p)
    {
        fs::create_directories(p.parent_path());
        std::ofstream(p) << "x";
    }
    void complete_set()
    {
        fs::create_directories(dir_ / "nnx");
        fs::create_directories(dir_ / "fused_subgraphs");
        touch(dir_ / "nnx" / "manifest.json");
        touch(dir_ / "legalized_model.onnx");
    }

    fs::path root_, dir_;
};

}  // namespace

TEST_F(ArtifactsDir, ResolvesCompleteSet)
{
    complete_set();
    const auto a = resolve_renesas_artifacts(dir_.string());

    EXPECT_EQ(a.model, (dir_ / "legalized_model.onnx").string());
    EXPECT_EQ(a.manifest, (dir_ / "nnx" / "manifest.json").string());
    EXPECT_FALSE(a.qdq_inserted);
    // base_path is the PARENT of the artifacts directory.
    EXPECT_EQ(a.base, root_.string());
}

TEST_F(ArtifactsDir, PrefersQdqInsertedModel)
{
    complete_set();
    touch(dir_ / "qdq_inserted_legalized_model.onnx");

    const auto a = resolve_renesas_artifacts(dir_.string());
    EXPECT_EQ(a.model, (dir_ / "qdq_inserted_legalized_model.onnx").string());
    EXPECT_TRUE(a.qdq_inserted);
}

TEST_F(ArtifactsDir, RejectsMissingNnxDir)
{
    fs::create_directories(dir_ / "fused_subgraphs");
    touch(dir_ / "legalized_model.onnx");
    EXPECT_THROW(resolve_renesas_artifacts(dir_.string()), std::runtime_error);
}

TEST_F(ArtifactsDir, RejectsMissingFusedSubgraphsDir)
{
    fs::create_directories(dir_ / "nnx");
    touch(dir_ / "legalized_model.onnx");
    EXPECT_THROW(resolve_renesas_artifacts(dir_.string()), std::runtime_error);
}

TEST_F(ArtifactsDir, RejectsMissingLegalizedOnnx)
{
    fs::create_directories(dir_ / "nnx");
    fs::create_directories(dir_ / "fused_subgraphs");
    EXPECT_THROW(resolve_renesas_artifacts(dir_.string()), std::runtime_error);
}

TEST_F(ArtifactsDir, RejectsNonExistentDirectory)
{
    EXPECT_THROW(resolve_renesas_artifacts((root_ / "nope").string()),
                 std::runtime_error);
}
