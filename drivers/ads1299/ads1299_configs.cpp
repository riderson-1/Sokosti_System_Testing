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
    
    
    for (size_t i = 1; i < ADS_NUM_CHANNELS; i++)
    {
        cfg.channel[i].powerDown = 1;
    }
    
    cfg.device.nDaisyChain = 0;
    cfg.device.samplingRate = 1000;
    cfg.device.nPdBias = 1;

    return cfg;
}

static ADS1299Settings AllChannelsMeasurement()
{
    ADS1299Settings cfg;
    
    for (size_t i = 0; i < ADS_NUM_CHANNELS; i++)
    {
        cfg.channel[i].powerDown = 0;
        cfg.channel[i].gain = 1;
        cfg.channel[i].mux = 0;
        cfg.channel[i].srb2 = 0;
        
    }
    
    cfg.device.nDaisyChain = 0;
    cfg.device.clkEn = 0;

    cfg.device.samplingRate = 1000;
    cfg.device.nPdBias = 1;
    cfg.device.nPdRefBuf = 1;
    cfg.device.biasRefInt = 1;

    cfg.device.intCal = 0;

    return cfg;
}

static ADS1299Settings AllChannelsLowSpeed()
{
    ADS1299Settings cfg;
    
    for (size_t i = 0; i < ADS_NUM_CHANNELS; i++)
    {
        cfg.channel[i].powerDown = 0;
        cfg.channel[i].gain = 1;
        cfg.channel[i].mux = 0;
        cfg.channel[i].srb2 = 0;
        
    }
    
    cfg.device.nDaisyChain = 0;
    cfg.device.clkEn = 0;

    cfg.device.samplingRate = 500;
    cfg.device.nPdBias = 1;
    cfg.device.nPdRefBuf = 1;
    cfg.device.biasRefInt = 1;

    cfg.device.intCal = 0;

    return cfg;
}

static ADS1299Settings AllChannelsShorted()
{
    ADS1299Settings cfg;

    for (size_t i = 0; i < ADS_NUM_CHANNELS; i++)
    {
        cfg.channel[i].powerDown = 0;   // PDn = 0: channel powered up
        cfg.channel[i].gain = 1;        // keep gain 1 for baseline characterization
        cfg.channel[i].mux = 1;         // MUXn[2:0] = 001: input shorted (internal, bypasses pins)
        cfg.channel[i].srb2 = 0;
    }

    cfg.device.nDaisyChain = 0;
    cfg.device.clkEn = 0;

    cfg.device.samplingRate = 1000;
    cfg.device.nPdBias = 1;
    cfg.device.nPdRefBuf = 1;
    cfg.device.biasRefInt = 1;

    cfg.device.intCal = 0;

    return cfg;
}

ADS1299Settings makeAdsSettings(AdsPreset preset)
{
    switch (preset) {
        case AdsPreset::DefaultSettings:        return DefaultSettings();
        case AdsPreset::SingleChannelTest:      return SingleChannelTest();
        case AdsPreset::AllChannelsMeasurement: return AllChannelsMeasurement();
        case AdsPreset::AllChannelsLowSpeed:    return AllChannelsLowSpeed();
        case AdsPreset::AllChannelsShorted:     return AllChannelsShorted();
    }
    return DefaultSettings();  // unreachable, keeps compiler happy without -Wswitch complaints masking a real bug
}