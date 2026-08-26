#include <gtest/gtest.h>

#include <models/merged_backend.hpp>
#include <models/merged_contract.hpp>
#include <onnxruntime_cxx_api.h>
#include <string>
#include <unordered_map>
#include <vector>

using visionpilot::models::DeclaredOutput;
using visionpilot::models::MergedContract;
using visionpilot::models::validate_contract_names;
using visionpilot::models::validate_output_shapes;
using visionpilot::models::validate_plain_merged_names;

namespace {

// The v7 contract shape: no steer_lane_value passthrough, ego path recovered
// on the host via steer_xp.
const char* kV7Contract = R"({
 "attn_mode": "frozen",
 "passthrough": ["steer_height"],
 "head": {
  "output": "drive_head_raw",
  "alpha": [8.0, 512.0, 0.5],
  "map": [["drive_distance", "relu"],
          ["drive_curvature", "tanh"],
          ["drive_flag_logit", "none"]]
 },
 "speed": {
  "levels": [
   {"box": "speed_l15_box", "cls": "speed_l15_cls", "stride": 8,  "hw": [64, 128]}
  ]
 },
 "steer_xp": {
  "logits": "steer_silu_41",
  "positions": 256,
  "div": 256.0
 }
})";

const std::vector<std::string> kV7Outputs = {
    "steer_height", "drive_head_raw", "steer_silu_41",
    "speed_l15_box", "speed_l15_cls"};

// The v6 contract shape: lane comes through passthrough instead of steer_xp.
const char* kV6Contract = R"({
 "attn_mode": "frozen",
 "passthrough": ["steer_lane_value", "steer_height"],
 "head": {
  "output": "drive_head_raw",
  "alpha": [8.0, 512.0, 0.5],
  "map": [["drive_distance", "relu"],
          ["drive_curvature", "tanh"],
          ["drive_flag_logit", "none"]]
 },
 "speed": {
  "levels": [
   {"box": "speed_l15_box", "cls": "speed_l15_cls", "stride": 8, "hw": [64, 128]}
  ]
 }
})";

const std::vector<std::string> kV6Outputs = {
    "steer_lane_value", "steer_height", "drive_head_raw",
    "speed_l15_box", "speed_l15_cls"};

bool contains(const std::string& haystack, const std::string& needle)
{
    return haystack.find(needle) != std::string::npos;
}

}  // namespace

// ─── validate_contract_names ────────────────────────────────────────────────

TEST(ValidateContractNames, AcceptsMatchingV7Contract)
{
    const auto c = MergedContract::from_json_string(kV7Contract);
    EXPECT_NO_THROW(validate_contract_names(c, kV7Outputs));
}

TEST(ValidateContractNames, AcceptsMatchingV6Contract)
{
    const auto c = MergedContract::from_json_string(kV6Contract);
    EXPECT_NO_THROW(validate_contract_names(c, kV6Outputs));
}

TEST(ValidateContractNames, RejectsUnknownOutput)
{
    const auto c = MergedContract::from_json_string(kV7Contract);
    // Missing speed_l15_cls entirely.
    const std::vector<std::string> outputs = {
        "steer_height", "drive_head_raw", "steer_silu_41", "speed_l15_box"};
    try {
        validate_contract_names(c, outputs);
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        EXPECT_TRUE(contains(msg, "Missing:\n    speed_l15_cls"));
        // Every other contract-named output is present; only speed_l15_cls
        // may be reported missing.
        EXPECT_FALSE(contains(msg, "Missing:\n    steer_height"));
        EXPECT_FALSE(contains(msg, "Missing:\n    drive_head_raw"));
        EXPECT_FALSE(contains(msg, "Missing:\n    steer_silu_41"));
        EXPECT_FALSE(contains(msg, "Missing:\n    speed_l15_box"));
    }
}

TEST(ValidateContractNames, RejectsV6ContractAgainstV7OutputList)
{
    // v6 needs steer_lane_value, which the v7 model does not expose.
    const auto c = MergedContract::from_json_string(kV6Contract);
    try {
        validate_contract_names(c, kV7Outputs);
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        EXPECT_TRUE(contains(msg, "Missing:\n    steer_lane_value"));
        // steer_height is common to both contracts and is in kV7Outputs; it
        // must not be reported missing.
        EXPECT_FALSE(contains(msg, "Missing:\n    steer_height"));
    }
}

TEST(ValidateContractNames, RejectsV7ContractAgainstV6OutputList)
{
    // v7 needs steer_silu_41, which the v6 model does not expose.
    const auto c = MergedContract::from_json_string(kV7Contract);
    try {
        validate_contract_names(c, kV6Outputs);
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        EXPECT_TRUE(contains(msg, "Missing:\n    steer_silu_41"));
        // drive_head_raw is common to both contracts and is in kV6Outputs;
        // it must not be reported missing.
        EXPECT_FALSE(contains(msg, "Missing:\n    drive_head_raw"));
    }
}

TEST(ValidateContractNames, RejectsEgoPathSuppliedZeroTimes)
{
    const char* json = R"({"passthrough": ["steer_height"]})";
    const auto  c = MergedContract::from_json_string(json);
    try {
        validate_contract_names(c, {"steer_height"});
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        EXPECT_TRUE(contains(msg, "supplies no ego path"));
        EXPECT_FALSE(contains(msg, "supplies the ego path twice"));
    }
}

TEST(ValidateContractNames, RejectsEgoPathSuppliedTwice)
{
    const char* json = R"({
     "passthrough": ["steer_lane_value", "steer_height"],
     "steer_xp": {"logits": "steer_silu_41", "positions": 256, "div": 256.0}
    })";
    const auto c = MergedContract::from_json_string(json);
    const std::vector<std::string> outputs = {
        "steer_lane_value", "steer_height", "steer_silu_41"};
    try {
        validate_contract_names(c, outputs);
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        EXPECT_TRUE(contains(msg, "supplies the ego path twice"));
        EXPECT_FALSE(contains(msg, "supplies no ego path"));
    }
}

TEST(ValidateContractNames, RejectsMissingSteerHeightPassthrough)
{
    const char* json = R"({"passthrough": ["steer_lane_value"]})";
    const auto  c = MergedContract::from_json_string(json);
    EXPECT_THROW(validate_contract_names(c, {"steer_lane_value"}),
                std::runtime_error);
}

TEST(ValidateContractNames, RejectsUnroutablePassthroughEntry)
{
    // steer_lane_value supplies the ego path so that only the whitelist
    // check (block 2) can fire here. Without it, deleting block 2 entirely
    // would leave this test green: it would fail later anyway, on block 4's
    // "supplies no ego path", which proves nothing about the whitelist.
    const char* json = R"({
     "passthrough": ["steer_height", "steer_lane_value", "mystery_output"]
    })";
    const auto c = MergedContract::from_json_string(json);
    const std::vector<std::string> outputs = {"steer_height",
                                              "steer_lane_value",
                                              "mystery_output"};
    try {
        validate_contract_names(c, outputs);
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        EXPECT_TRUE(contains(msg, "'mystery_output'"));
        EXPECT_FALSE(contains(msg, "supplies no ego path"));
        EXPECT_FALSE(contains(msg, "supplies the ego path twice"));
    }
}

// ─── validate_output_shapes ─────────────────────────────────────────────────

TEST(ValidateOutputShapes, AcceptsMatchingSpeedGeometry)
{
    const char* json = R"({
     "speed": {"levels": [
        {"box": "speed_l15_box", "cls": "speed_l15_cls", "stride": 8, "hw": [64, 128]}
     ]}
    })";
    const auto c = MergedContract::from_json_string(json);

    std::unordered_map<std::string, DeclaredOutput> declared;
    declared["speed_l15_box"] =
        DeclaredOutput{{1, 4, 64, 128}, ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT};
    declared["speed_l15_cls"] =
        DeclaredOutput{{1, 5, 64, 128}, ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT};

    EXPECT_NO_THROW(validate_output_shapes(c, declared));
}

TEST(ValidateOutputShapes, RejectsSpeedGeometryMismatch)
{
    const char* json = R"({
     "speed": {"levels": [
        {"box": "speed_l15_box", "cls": "speed_l15_cls", "stride": 8, "hw": [64, 128]}
     ]}
    })";
    const auto c = MergedContract::from_json_string(json);

    std::unordered_map<std::string, DeclaredOutput> declared;
    declared["speed_l15_box"] =
        DeclaredOutput{{1, 4, 64, 128}, ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT};
    // The graph actually emits a 32x64 grid; the contract wrongly claims
    // 64x128 -- exactly the mismatch assemble_speed()'s box_count/cls_count
    // guard exists to catch, but here at startup instead of frame 1.
    declared["speed_l15_cls"] =
        DeclaredOutput{{1, 5, 32, 64}, ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT};

    try {
        validate_output_shapes(c, declared);
        FAIL() << "expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        const std::string msg = e.what();
        // cls_count = 5*32*64 = 10240; expected = num_classes(5)*h(64)*w(128)
        // = 40960. box (4*64*128 = 32768, matching) must not be implicated.
        EXPECT_TRUE(contains(msg, "speed_l15_cls"));
        EXPECT_TRUE(contains(msg, "10240"));
        EXPECT_TRUE(contains(msg, "40960"));
        EXPECT_FALSE(contains(msg, "speed box output"));
    }
}

TEST(ValidateOutputShapes, RejectsNonFloatOutput)
{
    const char* json = R"({"passthrough": ["steer_height"]})";
    const auto  c = MergedContract::from_json_string(json);

    std::unordered_map<std::string, DeclaredOutput> declared;
    declared["steer_height"] =
        DeclaredOutput{{1, 64}, ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64};

    EXPECT_THROW(validate_output_shapes(c, declared), std::runtime_error);
}

TEST(ValidateOutputShapes, RejectsWrongPassthroughElementCount)
{
    const char* json = R"({"passthrough": ["steer_height"]})";
    const auto  c = MergedContract::from_json_string(json);

    std::unordered_map<std::string, DeclaredOutput> declared;
    declared["steer_height"] =
        DeclaredOutput{{1, 32}, ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT};

    EXPECT_THROW(validate_output_shapes(c, declared), std::runtime_error);
}

TEST(ValidateOutputShapes, RejectsSymbolicTrailingDimension)
{
    const char* json = R"({"passthrough": ["steer_height"]})";
    const auto  c = MergedContract::from_json_string(json);

    std::unordered_map<std::string, DeclaredOutput> declared;
    declared["steer_height"] =
        DeclaredOutput{{1, -1}, ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT};

    EXPECT_THROW(validate_output_shapes(c, declared), std::runtime_error);
}

TEST(ValidateOutputShapes, ToleratesSymbolicBatchDimension)
{
    const char* json = R"({"passthrough": ["steer_height"]})";
    const auto  c = MergedContract::from_json_string(json);

    std::unordered_map<std::string, DeclaredOutput> declared;
    declared["steer_height"] =
        DeclaredOutput{{-1, 64}, ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT};

    EXPECT_NO_THROW(validate_output_shapes(c, declared));
}

TEST(ValidateOutputShapes, AcceptsRank1PassthroughShape)
{
    // No explicit batch axis at all -- shape.size() < 2, so trailing_count()
    // must validate the whole shape rather than skipping index 0 (which
    // would leave nothing to check on a shape this short).
    const char* json = R"({"passthrough": ["steer_height"]})";
    const auto  c = MergedContract::from_json_string(json);

    std::unordered_map<std::string, DeclaredOutput> declared;
    declared["steer_height"] =
        DeclaredOutput{{64}, ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT};

    EXPECT_NO_THROW(validate_output_shapes(c, declared));
}

TEST(ValidateOutputShapes, RejectsRank0ScalarShape)
{
    // Rank 0: no batch axis to skip and no other dimension either. The
    // product-of-remaining-dims is the empty product (1), which must still
    // be compared against the required count (64) and rejected -- not
    // treated as "no dimensions to check, so pass".
    const char* json = R"({"passthrough": ["steer_height"]})";
    const auto  c = MergedContract::from_json_string(json);

    std::unordered_map<std::string, DeclaredOutput> declared;
    declared["steer_height"] =
        DeclaredOutput{{}, ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT};

    EXPECT_THROW(validate_output_shapes(c, declared), std::runtime_error);
}

TEST(ValidateOutputShapes, RejectsHeadRowCountMismatch)
{
    const char* json = R"({
     "head": {
      "output": "drive_head_raw",
      "alpha": [8.0, 512.0, 0.5],
      "map": [["drive_distance", "relu"],
              ["drive_curvature", "tanh"],
              ["drive_flag_logit", "none"]]
     }
    })";
    const auto c = MergedContract::from_json_string(json);

    std::unordered_map<std::string, DeclaredOutput> declared;
    // Only 2 elements declared; the contract's head.map describes 3 rows.
    declared["drive_head_raw"] =
        DeclaredOutput{{1, 2, 1, 1}, ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT};

    EXPECT_THROW(validate_output_shapes(c, declared), std::runtime_error);
}

TEST(ValidateOutputShapes, RejectsSteerXpRowCountMismatch)
{
    const char* json = R"({
     "steer_xp": {"logits": "steer_silu_41", "positions": 256, "div": 256.0}
    })";
    const auto c = MergedContract::from_json_string(json);

    std::unordered_map<std::string, DeclaredOutput> declared;
    // 32 rows instead of the 64 AutoSteerOutput::xp requires.
    declared["steer_silu_41"] =
        DeclaredOutput{{1, 1, 32, 256}, ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT};

    EXPECT_THROW(validate_output_shapes(c, declared), std::runtime_error);
}

// ─── validate_plain_merged_names ────────────────────────────────────────────

TEST(ValidatePlainMergedNames, AcceptsCompleteOutputList)
{
    EXPECT_NO_THROW(validate_plain_merged_names(
        {"steer_xp", "steer_h_vector", "speed_output"}));
}

TEST(ValidatePlainMergedNames, RejectsMissingSteerHVector)
{
    EXPECT_THROW(validate_plain_merged_names({"steer_xp", "speed_output"}),
                std::runtime_error);
}

TEST(ValidatePlainMergedNames, RejectsMissingSteerXp)
{
    EXPECT_THROW(
        validate_plain_merged_names({"steer_h_vector", "speed_output"}),
        std::runtime_error);
}

TEST(ValidatePlainMergedNames, RejectsBothMissing)
{
    EXPECT_THROW(validate_plain_merged_names({"speed_output"}),
                std::runtime_error);
}
