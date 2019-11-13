/*
 * compressor.c
 *
 *  Created on: Jul 3, 2019
 *      Author: joaoantoniocardoso
 */


#include "compressor.h"


float32_t linear_to_db(float32_t linear_value)
{
	return 20.0f * log10f(linear_value);
}
float32_t db_to_linear(float32_t db_value)
{
	return powf(10, 0.05f * db_value);
}

void compressor_parameter_init(compressor_parameter_t *p)
{
    // Apply parameters limits
    if(p->gain_db > 20)          	p->gain_db = 20;
    else if(p->gain_db < -10)    	p->gain_db = -10;
    else if(p->ratio < 1)      		p->ratio = 1;
    if(p->th_db > 0)          		p->th_db = 0;
    if(p->w_db < 0)           		p->w_db = 0;
    else if(p->w_db > -p->th_db) 	p->w_db = -p->th_db;
    if(p->att > 500)       			p->att = 500;
    if(p->att < 0)         			p->att = 0;
    if(p->rlt > 1000)      			p->rlt = 1000;
    if(p->rlt < 0)         			p->rlt = 0;
    if(p->clip_db > -2)           	p->clip_db = -2;
    else if(p->clip_db < -20)    	p->clip_db = -20;

    p->w_db_x2_inv = 1.f / (4.f * p->w_db);
    p->k1_db = p->th_db -p->w_db;              			// knee minimum level (dB)
    p->k2_db = p->th_db +p->w_db;              			// knee maximum level (dB)
    p->ratio = 1 / p->ratio;                  			// ratio to multiply instead divide
    p->tau_attack = p->att * 0.001f;   					// attack time
    p->tau_release = p->rlt * 0.001f;  					// release time
    p->alpha_attack = expf(-1 / (p->tau_attack * AUDIO_FREQUENCY));
    p->alpha_release = expf(-1 / (p->tau_release * AUDIO_FREQUENCY));
}

void compressor_init(compressor_parameter_t *p, compressor_t *c)
{
    p->gain_db = GAIN_DB;
    p->ratio = RATIO;
    p->th_db = TH_DB;
    p->w_db = W_DB;
    p->att = ATT;
    p->rlt = RLT;
    p->clip_db = CLIP;

    compressor_parameter_init(p);
    c->param = p;
    c->sign = 1;
}

float pre_gain_db(compressor_t *c)
{
	float pre_gain_db = c->param->gain_db;
	return pre_gain_db;
}

void sign_detection(compressor_t *c)
{
    if(c->sample < 0)   c->sign = -1;
    else                c->sign = 1;
}

void limitter_stage_db(compressor_t *c)
{
    if(c->sample > c->param->clip_db){
        LED2_on();
        c->sample = c->param->clip_db;
    }else{
        LED2_off();
    }
}

float gain_stage_db(compressor_t *c, float sample_db)
{
	float gain_compressor_stage_db;

	if(sample_db < c->param->k1_db){
		LED0_off();
		gain_compressor_stage_db = 0;
	}else if(sample_db <= c->param->k2_db){
		LED0_on();
		gain_compressor_stage_db = (c->param->ratio -1) *
				(sample_db -c->param->k1_db) *
				(sample_db -c->param->k1_db) *
				c->param->w_db_x2_inv;
	}else{
		LED0_on();
		gain_compressor_stage_db =
				(c->param->ratio -1) *
				(sample_db -c->param->th_db);
	}

	return gain_compressor_stage_db;
}

float ballistics_stage_db(compressor_t *c, float gain_compressor_stage_db)
{
	static float gain_ballisctics_stage_db = 0;

	if(gain_compressor_stage_db <= gain_ballisctics_stage_db){
		gain_ballisctics_stage_db =
				c->param->alpha_attack * gain_ballisctics_stage_db +
				(1 - c->param->alpha_attack) * gain_compressor_stage_db;
	}else{
		gain_ballisctics_stage_db =
				c->param->alpha_release * gain_ballisctics_stage_db +
				(1 - c->param->alpha_release) * gain_compressor_stage_db;
	}

	return gain_ballisctics_stage_db;
}

float auto_makeup_stage_db(compressor_t *c)
{
	float gain_makeup_stage_db;
	float sample_db = 0;

	gain_makeup_stage_db = -gain_stage_db(c, sample_db)/2;

	return gain_makeup_stage_db;
}

int16_t compressor_stage(int16_t b, compressor_t *c)
{
    c->sample = int_to_float(b);
    sign_detection(c);

    // COMPRESSOR STAGE
    c->sample = linear_to_db(c->sign * c->sample);
    static float gain_db = 0;

    gain_db = pre_gain_db(c);
    c->sample += gain_db;

    gain_db = gain_stage_db(c, c->sample);
    gain_db = ballistics_stage_db(c, gain_db);
#ifdef AUTO_MAKEUP_ON
    gain_db += auto_makeup_stage_db(c);
#endif

	if(BT0_on()){
		LED3_on();
	}else{
		LED3_off();
		c->sample += gain_db;
	}

	limitter_stage_db(c);

    c->sample = c->sign * db_to_linear(c->sample);

    return float_to_int(c->sample);
}
