/*
 * compressor.h
 *
 *  Created on: Jul 3, 2019
 *      Author: joaoantoniocardoso
 */

#ifndef COMPRESSOR_H_
#define COMPRESSOR_H_

#include <arm_math.h>
#include "audio_config.h"
#include <leds_and_buttons.h>

#define COMPRESSOR_SETUP_DEFAULT
//#define COMPRESSOR_SETUP_ILHA_DIGITAL_1
//#define COMPRESSOR_SETUP_ILHA_DIGITAL_2
//#define COMPRESSOR_SETUP_ILHA_DIGITAL_3
#define AUTO_MAKEUP_ON

/**
 * @brief   Compressor Parameters
 * @param   sample is the signal
 * @param   gain_db is the pre-gain or input-gain level (dB)
 * @param   ratio is the compression ratio
 * @param   th_db is the threshold level (dB)
 * @param   w_db is the knee-width (dB)
 * @param   att is the attack time (ms)
 * @param   rlt is the release time (ms)
 * @param   makeup_db is the makeup gain level (dB)
 * @param   clip_db is the clip level (dB)
 */
typedef struct compressor_parameter_t{
    float32_t gain_db;
    float32_t ratio;
    float32_t th_db;
    float32_t w_db;
    float32_t w_db_x2_inv;
    float32_t k1_db;
    float32_t k2_db;
    float32_t att;
    float32_t rlt;
    float32_t tau_attack;
    float32_t tau_release;
    float32_t clip_db;
    float32_t alpha_attack;
    float32_t alpha_release;
}compressor_parameter_t;

typedef struct compressor_t{
    compressor_parameter_t *param;
    float32_t sample;
    int32_t sign;
}compressor_t;

#define int_to_float(x)  ((float32_t)(x/40313.44708198012f))
#define float_to_int(x)  ((int16_t)(x*40313.44708198012f))

float32_t linear_to_db(float32_t linear_value);
float32_t db_to_linear(float32_t db_value);
void compressor_parameter_init(compressor_parameter_t *p);
void compressor_init(compressor_parameter_t *p, compressor_t *c);
float pre_gain_db(compressor_t *c);
void sign_detection(compressor_t *c);
void limitter_stage_db(compressor_t *c);
float gain_stage_db(compressor_t *c, float sample_db);
float ballistics_stage_db(compressor_t *c, float gain_compressor_stage_db);
int16_t compressor_stage(int16_t b, compressor_t *c);
float auto_makeup_stage_db(compressor_t *c);

// Compressor setups
#ifdef COMPRESSOR_SETUP_DEFAULT
	#define GAIN_DB		0			// pre-gain level (dB)
	#define RATIO		10			// compress ratio
	#define TH_DB		-9			// threshold level (dB)
	#define W_DB		3			// knee-width (dB)
	#define ATT			0			// attack time (ms)
	#define RLT			0			// release time (ms)
	#define CLIP		-2			// clipping level (dB)
#endif
#ifdef COMPRESSOR_SETUP_ILHA_DIGITAL_1
	#define GAIN_DB		0			// pre-gain level (dB)
	#define RATIO		10			// compress ratio
	#define TH_DB		-25.4		// threshold level (dB)
	#define W_DB		1			// knee-width (dB)
	#define ATT			8.8			// attack time (ms)
	#define RLT			135.0		// release time (ms)
	#define CLIP		-2			// clipping level (dB)
#endif
#ifdef COMPRESSOR_SETUP_ILHA_DIGITAL_2
	#define GAIN_DB		0			// pre-gain level (dB)
	#define RATIO		13.4		// compress ratio
	#define TH_DB		-21.4		// threshold level (dB)
	#define W_DB		3			// knee-width (dB)
	#define ATT			0.08		// attack time (ms)
	#define RLT			0			// release time (ms)
	#define CLIP		-2			// clipping level (dB)
#endif
#ifdef COMPRESSOR_SETUP_ILHA_DIGITAL_3
	#define GAIN_DB		0			// pre-gain level (dB)
	#define RATIO		19.7		// compress ratio
	#define TH_DB		-21.1		// threshold level (dB)
	#define W_DB		5.9			// knee-width (dB)
	#define ATT			5.6		// attack time (ms)
	#define RLT			44			// release time (ms)
	#define CLIP		-2			// clipping level (dB)
#endif


#endif /* COMPRESSOR_H_ */
