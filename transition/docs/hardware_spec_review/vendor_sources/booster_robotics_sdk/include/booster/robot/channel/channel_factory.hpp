#ifndef __BOOSTER_ROBOTICS_SDK_CHANNEL_FACTORY_HPP__
#define __BOOSTER_ROBOTICS_SDK_CHANNEL_FACTORY_HPP__

#include <iostream>
#include <mutex>

#include <booster/common/dds/dds_factory_model.hpp>
#include <booster/third_party/nlohmann_json/json.hpp>

namespace booster {
namespace robot {

/** @brief DDS topic channel type used by the SDK. */
template <typename MSG>
using Channel = booster::common::DdsTopicChannel<MSG>;

template <typename MSG>
using ChannelPtr = booster::common::DdsTopicChannelPtr<MSG>;

using TopicDataTypePtr = booster::common::DdsTopicDataTypePtr;
using TopicPtr = booster::common::DdsTopicPtr;

/**
 * @brief Process-wide factory for DDS publishers and subscribers.
 * @note Supported model: K1 | T1 | T2
 * @note Endpoint availability depends on the deployed DDS services and network
 * configuration.
 */
class ChannelFactory {
public:
    /** @brief Returns the process-wide factory instance. */
    static ChannelFactory *Instance() {
        static ChannelFactory instance;
        return &instance;
    }

    /** @brief Initializes DDS with a domain and optional network interface. */
    void Init(int32_t domain_id, const std::string &network_interface = "");
    /** @brief Initializes DDS from a JSON configuration object. */
    void Init(const nlohmann::json &config);
    /** @brief Initializes DDS with SDK default settings. */
    void InitDefault(int32_t domain_id);
    /** @brief Loads DDS settings from a configuration file. */
    void InitWithConfigPath(int32_t domain_id, const std::string &config_file_path);

    /** @brief Closes a writer associated with a topic name. */
    void CloseWriter(const std::string &channel_name);
    /** @brief Closes a reader associated with a topic name. */
    void CloseReader(const std::string &channel_name);
    /** @brief Closes and releases a topic object. */
    void CloseTopic(TopicPtr topic);

    template <typename MSG>
    /** @brief Creates a DDS writer channel. Returns null if the factory is uninitialized. */
    ChannelPtr<MSG> CreateSendChannel(const std::string &name, bool reliable = false) {
        if (dds_factory_model_ == nullptr) {
            std::cerr << "ChannelFactory is not initialized before creating writer for " << name << std::endl;
            return nullptr;
        }
        ChannelPtr<MSG> channel_ptr = dds_factory_model_->CreateTopicChannel<MSG>(name);
        dds_factory_model_->SetWriter(channel_ptr, reliable);
        return channel_ptr;
    }

    template <typename MSG>
    /** @brief Creates a DDS reader channel and installs @p handler. */
    ChannelPtr<MSG> CreateRecvChannel(
        const std::string &name,
        std::function<void(const void *)> handler,
        bool reliable = false,
        const common::DdsReaderExecutorOptions &executor_options = {}) {
        if (dds_factory_model_ == nullptr) {
            std::cerr << "ChannelFactory is not initialized before creating reader for " << name << std::endl;
            return nullptr;
        }
        ChannelPtr<MSG> channel_ptr = dds_factory_model_->CreateTopicChannel<MSG>(name);
        dds_factory_model_->SetReader(channel_ptr, handler, reliable, executor_options);
        return channel_ptr;
    }

private:
    bool initialized_ = false;
    std::mutex mutex_;
    common::DdsFactoryModelPtr dds_factory_model_;
};

}
} // namespace booster::robot

#endif // __BOOSTER_ROBOTICS_SDK_CHANNEL_FACTORY_HPP__
