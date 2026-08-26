#include <gtest/gtest.h>

#include <cmath>
#include <models/auto_drive.hpp>
#include <models/auto_steer.hpp>
#include <models/merged_contract.hpp>

using visionpilot::models::MergedContract;

namespace {

// The v7 contract emitted by openadkit's rewrite pipeline, verbatim from
// x5h-work/npu/merged/v7_frozen.onnx.contract.json.
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
   {"box": "speed_l15_box", "cls": "speed_l15_cls", "stride": 8,  "hw": [64, 128]},
   {"box": "speed_l16_box", "cls": "speed_l16_cls", "stride": 16, "hw": [32, 64]},
   {"box": "speed_l17_box", "cls": "speed_l17_cls", "stride": 32, "hw": [16, 32]}
  ],
  "note": "boxes_l = box*stride"
 },
 "steer_xp": {
  "logits": "steer_silu_41",
  "positions": 256,
  "div": 256.0,
  "note": "softmax(logits [64,256], axis=-1) @ arange(positions) / div"
 }
})";

// The v6 contract, still supported: lane comes through passthrough instead.
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

}  // namespace

TEST(MergedContract, ParsesV7Contract)
{
    const auto c = MergedContract::from_json_string(kV7Contract);

    EXPECT_EQ(c.attn_mode, "frozen");

    // v7 passes only the height vector through; lane is computed on the host.
    ASSERT_EQ(c.passthrough.size(), 1u);
    EXPECT_EQ(c.passthrough[0], "steer_height");

    ASSERT_TRUE(c.head.has_value());
    EXPECT_EQ(c.head->output, "drive_head_raw");
    ASSERT_EQ(c.head->alpha.size(), 3u);
    EXPECT_FLOAT_EQ(c.head->alpha[0], 8.0f);
    EXPECT_FLOAT_EQ(c.head->alpha[1], 512.0f);
    EXPECT_FLOAT_EQ(c.head->alpha[2], 0.5f);
    ASSERT_EQ(c.head->map.size(), 3u);
    EXPECT_EQ(c.head->map[0].output, "drive_distance");
    EXPECT_EQ(c.head->map[0].activation, "relu");
    EXPECT_EQ(c.head->map[1].activation, "tanh");
    EXPECT_EQ(c.head->map[2].activation, "none");

    ASSERT_TRUE(c.steer_xp.has_value());
    EXPECT_EQ(c.steer_xp->logits, "steer_silu_41");
    EXPECT_EQ(c.steer_xp->positions, 256);
    EXPECT_FLOAT_EQ(c.steer_xp->div, 256.0f);

    ASSERT_TRUE(c.speed.has_value());
    ASSERT_EQ(c.speed->levels.size(), 3u);
    EXPECT_EQ(c.speed->levels[0].box, "speed_l15_box");
    EXPECT_EQ(c.speed->levels[0].cls, "speed_l15_cls");
    EXPECT_EQ(c.speed->levels[0].stride, 8);
    EXPECT_EQ(c.speed->levels[0].h, 64);
    EXPECT_EQ(c.speed->levels[0].w, 128);
    EXPECT_EQ(c.speed->levels[2].stride, 32);
    EXPECT_EQ(c.speed->levels[2].h, 16);
    EXPECT_EQ(c.speed->levels[2].w, 32);
}

TEST(MergedContract, ParsesV6ContractWithoutSteerXp)
{
    const auto c = MergedContract::from_json_string(kV6Contract);
    ASSERT_EQ(c.passthrough.size(), 2u);
    EXPECT_EQ(c.passthrough[0], "steer_lane_value");
    EXPECT_FALSE(c.steer_xp.has_value());
    ASSERT_TRUE(c.head.has_value());
}

TEST(MergedContract, MinimalContractLeavesOptionalsEmpty)
{
    const auto c = MergedContract::from_json_string(R"({"attn_mode": "keep"})");
    EXPECT_EQ(c.attn_mode, "keep");
    EXPECT_TRUE(c.passthrough.empty());
    EXPECT_FALSE(c.head.has_value());
    EXPECT_FALSE(c.speed.has_value());
    EXPECT_FALSE(c.steer_xp.has_value());
}

TEST(MergedContract, RejectsMalformedJson)
{
    EXPECT_THROW(MergedContract::from_json_string("{ not json"),
                 std::runtime_error);
}

TEST(MergedContract, RejectsAlphaMapLengthMismatch)
{
    EXPECT_THROW(MergedContract::from_json_string(R"({
      "head": {"output": "drive_head_raw", "alpha": [1.0, 2.0],
               "map": [["drive_distance", "relu"]]}
    })"), std::runtime_error);
}

TEST(MergedContract, RejectsNonPositiveSteerXpPositions)
{
    EXPECT_THROW(MergedContract::from_json_string(R"({
      "steer_xp": {"logits": "x", "positions": 0, "div": 256.0}
    })"), std::runtime_error);
}

TEST(MergedContract, RejectsZeroSteerXpDivisor)
{
    EXPECT_THROW(MergedContract::from_json_string(R"({
      "steer_xp": {"logits": "x", "positions": 256, "div": 0.0}
    })"), std::runtime_error);
}

TEST(MergedContract, DetectsRewriteSignatureOutputs)
{
    using visionpilot::models::has_rewrite_signature_output;
    EXPECT_TRUE(has_rewrite_signature_output({"steer_height", "drive_head_raw"}));
    EXPECT_TRUE(has_rewrite_signature_output({"speed_l15_box", "speed_l15_cls"}));
    EXPECT_FALSE(has_rewrite_signature_output(
        {"steer_xp", "speed_output", "drive_distance"}));
}
