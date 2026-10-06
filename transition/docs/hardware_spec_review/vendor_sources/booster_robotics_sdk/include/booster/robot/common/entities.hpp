#pragma once
#include <string>
#include <booster/third_party/nlohmann_json/json.hpp>

namespace booster {
namespace robot {

/**
 * @brief 3D position in metres.
 * @note Supported model: K1 | T1 | T2
 */
class Position {
public:
    Position() = default;
    Position(float x, float y, float z) :
        x_(x), y_(y), z_(z) {
    }

    /** @brief Loads x, y and z from JSON. */
    void FromJson(nlohmann::json &json) {
        x_ = json["x"];
        y_ = json["y"];
        z_ = json["z"];
    }

    /** @brief Serializes this position as `{x,y,z}`. */
    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["x"] = x_;
        json["y"] = y_;
        json["z"] = z_;
        return json;
    }

public:
    float x_ = 0.; // unit : m
    float y_ = 0.; // unit : m
    float z_ = 0.; // unit : m
};

/** @brief Roll/pitch/yaw orientation in radians. */
class Orientation {
public:
    Orientation() = default;
    Orientation(float roll, float pitch, float yaw) :
        roll_(roll), pitch_(pitch), yaw_(yaw) {
    }

    void FromJson(nlohmann::json &json) {
        roll_ = json["roll"];
        pitch_ = json["pitch"];
        yaw_ = json["yaw"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["roll"] = roll_;
        json["pitch"] = pitch_;
        json["yaw"] = yaw_;
        return json;
    }

public:
    float roll_ = 0.;  // unit : rad
    float pitch_ = 0.; // unit : rad
    float yaw_ = 0.;   // unit : rad
};

/** @brief Pose combining a Position and Orientation. */
class Posture {
public:
    Posture() = default;
    Posture(const Position &pos, const Orientation &orient) :
        position_(pos), orientation_(orient) {
    }

    void FromJson(nlohmann::json &json) {
        position_.FromJson(json["position"]);
        orientation_.FromJson(json["orientation"]);
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["position"] = position_.ToJson();
        json["orientation"] = orientation_.ToJson();
        return json;
    }

public:
    Position position_;
    Orientation orientation_;
};

/** @brief Quaternion in x, y, z, w order. */
class Quaternion {
public:
    Quaternion() = default;
    Quaternion(float x, float y, float z, float w) :
        x_(x), y_(y), z_(z), w_(w) {}
    
    void FromJson(nlohmann::json &json) {
        x_ = json["x"];
        y_ = json["y"];
        z_ = json["z"];
        w_ = json["w"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["x"] = x_;
        json["y"] = y_;
        json["z"] = z_;
        json["w"] = w_;
        return json;
    }

public:
    float x_ = 0.;
    float y_ = 0.;
    float z_ = 0.;
    float w_ = 0.;
};

/** @brief Rigid transform combining translation and quaternion orientation. */
class Transform {
public:
    Transform() = default;
    Transform(const Position &position, const Quaternion &orientation) :
        position_(position), orientation_(orientation) {
    }

    void FromJson(nlohmann::json &json) {
        position_.FromJson(json["position"]);
        orientation_.FromJson(json["orientation"]);
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["position"] = position_.ToJson();
        json["orientation"] = orientation_.ToJson();
        return json;
    }

public:
    Position position_;
    Quaternion orientation_;

};

}
} // namespace booster::robot
