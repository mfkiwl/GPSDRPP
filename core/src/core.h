#pragma once
#include <config.h>
#include <module.h>
#include <module_com.h>
#include "command_args.h"
#include "up_converter.h"
#include "si5351.h"
#include "encoder.h"
#include "gps.h"

// Pin 40 (4B2 on Vivid Unit) controls the power of VU GPSDR board (active low)
#define CORE_CMD_POWER_ALT_0        "vgp alt 40 0"
#define CORE_CMD_POWER_MODE_OUT     "vgp mode 40 OUT"
#define CORE_CMD_POWER_SET_HIGH     "vgp set 40 1"
#define CORE_CMD_POWER_SET_LOW      "vgp set 40 0"

// Pins for Encoder A and B (physical header positions)
#define CORE_CMD_ENCODER_AA_ALT_0   "vgp alt 37 0"
#define CORE_CMD_ENCODER_AB_ALT_0   "vgp alt 33 0"
#define CORE_CMD_ENCODER_AC_ALT_0   "vgp alt 35 0"
#define CORE_CMD_ENCODER_BA_ALT_0   "vgp alt 19 0"
#define CORE_CMD_ENCODER_BB_ALT_0   "vgp alt 13 0"
#define CORE_CMD_ENCODER_BC_ALT_0   "vgp alt 15 0"

#define CORE_PIN_ENCODER_AA 37
#define CORE_PIN_ENCODER_AB 33
#define CORE_PIN_ENCODER_AC 35
#define CORE_PIN_ENCODER_BA 19
#define CORE_PIN_ENCODER_BB 13
#define CORE_PIN_ENCODER_BC 15

namespace core {
	
    struct VUGPSDR_Initializer {
        VUGPSDR_Initializer();
    };

    SDRPP_EXPORT ConfigManager configManager;
    SDRPP_EXPORT ModuleManager moduleManager;
    SDRPP_EXPORT ModuleComManager modComManager;
    SDRPP_EXPORT CommandArgsParser args;
    SDRPP_EXPORT Gps gps;
    SDRPP_EXPORT Si5351 si5351;
    SDRPP_EXPORT UpConverter upConverter;

    void setInputSampleRate(double samplerate);
    
    namespace encoders {
        void on_encoder_a_cw(Encoder *enc);
        void on_encoder_a_ccw(Encoder *enc);
        void on_encoder_a_pressed(Encoder *enc);
        void on_encoder_a_released(Encoder *enc);
        
        void on_encoder_b_cw(Encoder *enc);
        void on_encoder_b_ccw(Encoder *enc);
        void on_encoder_b_pressed(Encoder *enc);
        void on_encoder_b_released(Encoder *enc);
    }
};

int gpsdrpp_main(int argc, char* argv[]);