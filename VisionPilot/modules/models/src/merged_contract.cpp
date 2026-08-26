#include "models/merged_contract.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace visionpilot::models {

namespace {

ContractHead parse_head(const nlohmann::json& j)
{
    ContractHead h;
    h.output = j.at("output").get<std::string>();
    h.alpha  = j.at("alpha").get<std::vector<float>>();

    for (const auto& row : j.at("map")) {
        if (!row.is_array() || row.size() != 2) {
            throw std::runtime_error(
                "[MergedContract] head.map rows must be [output, activation]");
        }
        ContractHeadMap m;
        m.output     = row[0].get<std::string>();
        m.activation = row[1].get<std::string>();
        if (m.activation != "relu" && m.activation != "tanh" &&
            m.activation != "none") {
            throw std::runtime_error(
                "[MergedContract] unknown activation '" + m.activation +
                "' for head output '" + m.output + "'");
        }
        h.map.push_back(std::move(m));
    }

    if (h.alpha.size() != h.map.size()) {
        throw std::runtime_error(
            "[MergedContract] head.alpha has " + std::to_string(h.alpha.size()) +
            " entries but head.map has " + std::to_string(h.map.size()));
    }
    if (h.alpha.empty()) {
        throw std::runtime_error("[MergedContract] head.alpha is empty");
    }
    for (size_t i = 0; i < h.alpha.size(); ++i) {
        if (h.alpha[i] == 0.f) {
            throw std::runtime_error(
                "[MergedContract] head.alpha[" + std::to_string(i) + "] is zero");
        }
    }
    return h;
}

ContractSteerXp parse_steer_xp(const nlohmann::json& j)
{
    ContractSteerXp s;
    s.logits    = j.at("logits").get<std::string>();
    s.positions = j.at("positions").get<int>();
    s.div       = j.at("div").get<float>();

    if (s.positions <= 0) {
        throw std::runtime_error(
            "[MergedContract] steer_xp.positions must be positive, got " +
            std::to_string(s.positions));
    }
    if (s.div == 0.f) {
        throw std::runtime_error("[MergedContract] steer_xp.div is zero");
    }
    return s;
}

ContractSpeed parse_speed(const nlohmann::json& j)
{
    ContractSpeed s;
    for (const auto& lv : j.at("levels")) {
        ContractSpeedLevel l;
        l.box    = lv.at("box").get<std::string>();
        l.cls    = lv.at("cls").get<std::string>();
        l.stride = lv.at("stride").get<int>();

        const auto hw = lv.at("hw");
        if (!hw.is_array() || hw.size() != 2) {
            throw std::runtime_error("[MergedContract] speed level hw must be [h, w]");
        }
        l.h = hw[0].get<int>();
        l.w = hw[1].get<int>();

        if (l.stride <= 0 || l.h <= 0 || l.w <= 0) {
            throw std::runtime_error(
                "[MergedContract] speed level '" + l.box +
                "' has non-positive stride or extent");
        }
        s.levels.push_back(std::move(l));
    }
    if (s.levels.empty()) {
        throw std::runtime_error("[MergedContract] speed.levels is empty");
    }
    return s;
}

}  // namespace

namespace {

float apply_activation(const std::string& kind, float v)
{
    if (kind == "relu") return v > 0.f ? v : 0.f;
    if (kind == "tanh") return std::tanh(v);
    return v;  // "none" — validated at parse time
}

}  // namespace

void apply_head(const ContractHead& head, const float* raw, size_t raw_count,
                AutoDriveOutput& out)
{
    if (raw_count != head.map.size()) {
        throw std::runtime_error(
            "[MergedContract] head tensor has " + std::to_string(raw_count) +
            " elements but the contract's head.map describes " +
            std::to_string(head.map.size()) + " rows");
    }

    for (size_t i = 0; i < head.map.size(); ++i) {
        const auto& m = head.map[i];
        const float v = apply_activation(m.activation, raw[i] / head.alpha[i]);

        if (m.output == "drive_distance") {
            out.dist_normalized = v;
        } else if (m.output == "drive_curvature") {
            out.curvature_raw = v;
        } else if (m.output == "drive_flag_logit") {
            out.flag_prob = 1.f / (1.f + std::exp(-v));
        } else {
            throw std::runtime_error(
                "[MergedContract] unknown head output '" + m.output +
                "'. Expected drive_distance, drive_curvature, or "
                "drive_flag_logit");
        }
    }
    out.valid = true;
}

void apply_steer_xp(const ContractSteerXp& rule, const float* logits,
                    size_t count, AutoSteerOutput& out)
{
    if (rule.positions <= 0) {
        throw std::runtime_error(
            "[MergedContract] steer_xp.positions must be positive, got " +
            std::to_string(rule.positions));
    }
    if (rule.div == 0.f) {
        throw std::runtime_error("[MergedContract] steer_xp.div is zero");
    }

    const size_t positions = static_cast<size_t>(rule.positions);

    if (count == 0 || count % positions != 0) {
        throw std::runtime_error(
            "[MergedContract] steer_xp logits hold " + std::to_string(count) +
            " elements, which is not a positive multiple of positions=" +
            std::to_string(positions));
    }

    const size_t rows = count / positions;
    if (rows != out.xp.size()) {
        throw std::runtime_error(
            "[MergedContract] steer_xp logits describe " +
            std::to_string(rows) + " rows but AutoSteerOutput::xp holds " +
            std::to_string(out.xp.size()));
    }

    for (size_t r = 0; r < rows; ++r) {
        const float* row = logits + r * positions;

        // Subtract the row max before exponentiating, as the reference does;
        // the raw logits are unbounded and expf would overflow.
        double max_v = row[0];
        for (size_t i = 1; i < positions; ++i) {
            if (static_cast<double>(row[i]) > max_v) max_v = row[i];
        }

        double sum = 0.0;
        double acc = 0.0;
        for (size_t i = 0; i < positions; ++i) {
            const double e = std::exp(static_cast<double>(row[i]) - max_v);
            sum += e;
            acc += e * static_cast<double>(i);
        }

        // sum >= 1 because the max element contributes exp(0) = 1.
        out.xp[r] = static_cast<float>((acc / sum) / rule.div);
    }
    out.valid = true;
}

MergedContract MergedContract::from_json_string(const std::string& text)
{
    nlohmann::json j;
    try {
        j = nlohmann::json::parse(text);
    } catch (const nlohmann::json::exception& e) {
        throw std::runtime_error(
            std::string("[MergedContract] malformed JSON: ") + e.what());
    }

    MergedContract c;
    try {
        if (j.contains("attn_mode")) {
            c.attn_mode = j.at("attn_mode").get<std::string>();
        }
        if (j.contains("passthrough")) {
            c.passthrough = j.at("passthrough").get<std::vector<std::string>>();
        }
        if (j.contains("head")) {
            c.head = parse_head(j.at("head"));
        }
        if (j.contains("steer_xp")) {
            c.steer_xp = parse_steer_xp(j.at("steer_xp"));
        }
        if (j.contains("speed")) {
            c.speed = parse_speed(j.at("speed"));
        }
    } catch (const nlohmann::json::exception& e) {
        throw std::runtime_error(
            std::string("[MergedContract] invalid contract: ") + e.what());
    }
    return c;
}

MergedContract MergedContract::from_file(const std::string& path)
{
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("[MergedContract] cannot open " + path);
    }
    const std::string text((std::istreambuf_iterator<char>(in)),
                            std::istreambuf_iterator<char>());
    return from_json_string(text);
}

std::string resolve_contract_path(const std::string& model_path,
                                  const std::string& artifacts_dir)
{
    if (!model_path.empty()) {
        const std::string sidecar = model_path + ".contract.json";
        if (std::filesystem::exists(sidecar)) return sidecar;
    }
    if (!artifacts_dir.empty()) {
        const auto in_dir =
            std::filesystem::path(artifacts_dir) / "contract.json";
        if (std::filesystem::exists(in_dir)) return in_dir.string();
    }
    return {};
}

bool has_rewrite_signature_output(const std::vector<std::string>& output_names)
{
    for (const auto& n : output_names) {
        if (n == "drive_head_raw") return true;
        // speed_l<N>_box exists only after the R7 per-level DFL rewrite.
        if (n.rfind("speed_l", 0) == 0 &&
            n.size() > 4 && n.compare(n.size() - 4, 4, "_box") == 0) {
            return true;
        }
    }
    return false;
}

}  // namespace visionpilot::models
