#include <gtest/gtest.h>
#include <engine/onnx_engine.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

namespace fs = std::filesystem;
using visionpilot::engine::Config;
using visionpilot::engine::OnnxEngine;
using visionpilot::engine::resolve_renesas_artifacts;

namespace {

void touch(const fs::path& p)
{
    fs::create_directories(p.parent_path());
    std::ofstream(p) << "x";
}

// A directory unique to this process/run: two concurrent test runs (or two
// users on a shared host) must never collide or clobber each other's
// fixtures. Mirrors ScopedTempDir in test_merged_contract.cpp.
fs::path make_unique_temp_dir(const std::string& tag)
{
    const auto stamp =
        std::chrono::steady_clock::now().time_since_epoch().count();
    return fs::temp_directory_path() / ("vp_" + tag + "_" + std::to_string(stamp));
}

// True iff resolve_renesas_artifacts's error message lists `path` as its own
// "Missing:" entry (i.e. the entry's exact text, not merely a substring of
// it or of a longer entry). A plain substring search would misfire two
// ways: "legalized_*.onnx" contains "nnx", and the nnx/ entry is itself a
// path-prefix of the nnx/manifest.json entry.
bool has_missing_entry(const std::string& msg, const std::string& path)
{
    const std::string needle = "\n  " + path;
    for (auto pos = msg.find(needle); pos != std::string::npos;
         pos = msg.find(needle, pos + 1)) {
        const auto after = pos + needle.size();
        if (after == msg.size() || msg[after] == '\n') return true;
    }
    return false;
}

class ArtifactsDir : public ::testing::Test {
protected:
    void SetUp() override
    {
        root_ = make_unique_temp_dir("artifacts_test");
        dir_ = root_ / "artifacts_set_a";
        fs::create_directories(dir_);
    }
    void TearDown() override { fs::remove_all(root_); }

    void add_nnx_with_manifest()
    {
        fs::create_directories(dir_ / "nnx");
        touch(dir_ / "nnx" / "manifest.json");
    }
    void add_fused_subgraphs() { fs::create_directories(dir_ / "fused_subgraphs"); }
    void add_legalized() { touch(dir_ / "legalized_model.onnx"); }

    void complete_set()
    {
        add_nnx_with_manifest();
        add_fused_subgraphs();
        add_legalized();
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

TEST_F(ArtifactsDir, PicksLexicographicallyFirstOnMultipleMatches)
{
    add_nnx_with_manifest();
    add_fused_subgraphs();
    touch(dir_ / "legalized_b.onnx");
    touch(dir_ / "legalized_a.onnx");

    // directory_iterator's enumeration order is unspecified; the resolver
    // must sort so the pick does not depend on it.
    const auto a = resolve_renesas_artifacts(dir_.string());
    EXPECT_EQ(a.model, (dir_ / "legalized_a.onnx").string());
}

TEST_F(ArtifactsDir, RejectsMissingNnxDir)
{
    add_fused_subgraphs();
    add_legalized();

    try {
        resolve_renesas_artifacts(dir_.string());
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        EXPECT_TRUE(has_missing_entry(msg, (dir_ / "nnx").string()));
        EXPECT_FALSE(has_missing_entry(msg, (dir_ / "fused_subgraphs").string()));
        EXPECT_FALSE(has_missing_entry(msg, (dir_ / "legalized_*.onnx").string()));
    }
}

TEST_F(ArtifactsDir, RejectsMissingManifestJson)
{
    fs::create_directories(dir_ / "nnx");  // nnx/ exists, manifest.json does not
    add_fused_subgraphs();
    add_legalized();

    try {
        resolve_renesas_artifacts(dir_.string());
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        EXPECT_TRUE(has_missing_entry(msg, (dir_ / "nnx" / "manifest.json").string()));
        // The nnx/ entry itself must NOT appear: nnx/ exists, only the file
        // inside it is missing, and that distinction must be visible in the
        // message (not collapsed into "nnx/ is missing").
        EXPECT_FALSE(has_missing_entry(msg, (dir_ / "nnx").string()));
        EXPECT_FALSE(has_missing_entry(msg, (dir_ / "fused_subgraphs").string()));
    }
}

TEST_F(ArtifactsDir, RejectsMissingFusedSubgraphsDir)
{
    add_nnx_with_manifest();
    add_legalized();

    try {
        resolve_renesas_artifacts(dir_.string());
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        EXPECT_TRUE(has_missing_entry(msg, (dir_ / "fused_subgraphs").string()));
        EXPECT_FALSE(has_missing_entry(msg, (dir_ / "nnx").string()));
        EXPECT_FALSE(has_missing_entry(msg, (dir_ / "nnx" / "manifest.json").string()));
    }
}

TEST_F(ArtifactsDir, RejectsMissingLegalizedOnnx)
{
    add_nnx_with_manifest();
    add_fused_subgraphs();

    try {
        resolve_renesas_artifacts(dir_.string());
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        EXPECT_TRUE(has_missing_entry(msg, (dir_ / "legalized_*.onnx").string()));
        EXPECT_FALSE(has_missing_entry(msg, (dir_ / "fused_subgraphs").string()));
        EXPECT_FALSE(has_missing_entry(msg, (dir_ / "nnx").string()));
    }
}

TEST_F(ArtifactsDir, RejectsNonExistentDirectory)
{
    EXPECT_THROW(resolve_renesas_artifacts((root_ / "nope").string()),
                 std::runtime_error);
}

TEST_F(ArtifactsDir, RejectsEmptyArtifactsDir)
{
    EXPECT_THROW(resolve_renesas_artifacts(""), std::runtime_error);
}

TEST_F(ArtifactsDir, RejectsArtifactsDirThatIsARegularFile)
{
    const auto file = root_ / "not_a_directory";
    touch(file);
    EXPECT_THROW(resolve_renesas_artifacts(file.string()), std::runtime_error);
}

// ─── OnnxEngine-level guards ─────────────────────────────────────────────────
//
// These run before resolve_renesas_artifacts and before
// AppendExecutionProvider, so they are reachable on any host -- no Renesas
// execution provider required.

namespace {

class OnnxEngineRenesasGuards : public ::testing::Test {
protected:
    void SetUp() override { root_ = make_unique_temp_dir("engine_guards_test"); }
    void TearDown() override { fs::remove_all(root_); }

    fs::path root_;
};

}  // namespace

TEST_F(OnnxEngineRenesasGuards, RejectsEmptyArcProgPath)
{
    Config cfg;
    cfg.provider = "renesas";
    cfg.arc_prog_path.clear();
    OnnxEngine engine(cfg);

    try {
        engine.create_session((root_ / "artifacts").string());
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        EXPECT_NE(std::string(e.what()).find("arc_prog_path"), std::string::npos);
    }
}

TEST_F(OnnxEngineRenesasGuards, RejectsArcProgPathThatIsARegularFile)
{
    const auto file = root_ / "arc_prog_as_a_file";
    touch(file);

    Config cfg;
    cfg.provider = "renesas";
    cfg.arc_prog_path = file.string();
    OnnxEngine engine(cfg);

    try {
        engine.create_session((root_ / "artifacts").string());
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        EXPECT_NE(std::string(e.what()).find("arc_prog_path"), std::string::npos);
    }
}

TEST_F(OnnxEngineRenesasGuards, RejectsUnknownProvider)
{
    Config cfg;
    cfg.provider = "not_a_real_provider";
    OnnxEngine engine(cfg);

    try {
        engine.create_session((root_ / "model.onnx").string());
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        EXPECT_NE(std::string(e.what()).find("Unknown provider"), std::string::npos);
    }
}
