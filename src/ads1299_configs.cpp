#include "ads1299_configs.hpp"

/**
 * @brief 
 * 
 * @return ADS1299Settings 
 */

static ADS1299Settings DefaultSettings()
{
    ADS1299Settings cfg;                 // start from struct defaults
    return cfg;
}

static ADS1299Settings SingleChannelTest()
{
    ADS1299Settings cfg;

    cfg.channel[0].powerDown = 0;
    cfg.channel[0].gain = 8;
    cfg.channel[0].mux = 0;
    cfg.channel[0].srb2 = 0;
    
    
    for (size_t i = 1; i < 16; i++)
    {
        cfg.channel[1].powerDown = 1;
    }
    
    cfg.device.samplingRate = 1000;
    cfg.device.nPdRefBuf = 1;

    return cfg;
}

ADS1299Settings makeAdsSettings(AdsPreset preset)
{
    switch (preset) {
        case AdsPreset::DefaultSettings:    return DefaultSettings();
        case AdsPreset::SingleChannelTest:  return SingleChannelTest();
    }
    return defaultSettings();  // unreachable, keeps compiler happy without -Wswitch complaints masking a real bug
}