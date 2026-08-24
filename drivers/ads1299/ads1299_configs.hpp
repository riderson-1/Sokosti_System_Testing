#pragma once

#include "ads1299_driver.hpp"

enum class AdsPreset {
    SingleChannelTest,   // only uses single channel at 1000 sps, gain 8
    DefaultSettings,
    AllChannelsMeasurement,
    AllChannelsLowSpeed
};

/**
 * @brief Returns a fully-populated settings struct for a known configuration.
 */
ADS1299Settings makeAdsSettings(AdsPreset preset);