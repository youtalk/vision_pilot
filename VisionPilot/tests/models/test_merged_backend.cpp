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
    EXPECT_THROW(validate_contract_names(c, outputs), std::runtime_error);
}

TEST(ValidateContractNames, RejectsV6ContractAgainstV7OutputList)
{
    // v6 needs steer_lane_value, which the v7 model does not expose.
    const auto c = MergedContract::from_json_string(kV6Contract);
    EXPECT_THROW(validate_contract_names(c, kV7Outputs), std::runtime_error);
}

TEST(ValidateContractNames, RejectsV7ContractAgainstV6OutputList)
{
    // v7 needs steer_silu_41, which the v6 model does not expose.
    const auto c = MergedContract::from_json_string(kV7Contract);
    EXPECT_THROW(validate_contract_names(c, kV6Outputs), std::runtime_error);
}

TEST(ValidateContractNames, RejectsEgoPathSuppliedZeroTimes)
{
    const char* json = R"({"passthrough": ["steer_height"]})";
    const auto  c = MergedContract::from_json_string(json);
    EXPECT_THROW(validate_contract_names(c, {"steer_height"}),
                std::runtime_error);
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
    EXPECT_THROW(validate_contract_names(c, outputs), std::runtime_error);
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
    const char* json =
        R"({"passthrough": ["steer_height", "mystery_output"]})";
    const auto c = MergedContract::from_json_string(json);
    const std::vector<std::string> outputs = {"steer_height",
                                              "mystery_output"};
    EXPECT_THROW(validate_contract_names(c, outputs), std::runtime_error);
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

    EXPECT_THROW(validate_output_shapes(c, declared), std::runtime_error);
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
