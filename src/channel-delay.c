/*

Channel Delay
LV2 plugin. Signal delay by specific channel.

License: GPLv3 (https://github.com/unclechu/lv2-channel-delay/blob/master/LICENSE)
Author: Viacheslav Lotsmanov (unclechu)

*/

#ifdef DEBUG
#include <stdio.h>
#endif

#include "lv2/lv2plug.in/ns/lv2core/lv2.h"
#include "lv2/lv2plug.in/ns/ext/atom/atom.h"

#define URI_PREFIX "https://github.com/unclechu/lv2-channel-delay/"
#define URI_STEREO URI_PREFIX "channel-delay-stereo"

/** Define a macro for converting a gain in dB to a coefficient */
#define DB_CO(g) ((g) > -90.0f ? powf(10.0f, (g) * 0.05f) : 0.0f)
double round(double d) { return floor(d + 0.5); }

#define MAX_DELAY_MS 2000 // maximum milliseconds of delay

double samplerate;
float *buffer_l;
float *buffer_r;
float last_delay_ms_l = -1;
float last_delay_ms_r = -1;
int32_t last_delay_samples_l = -1;
int32_t last_delay_samples_r = -1;
int32_t sample_i_l = 0;
int32_t sample_i_r = 0;

typedef enum {
	stereo_input_l = 0,
	stereo_input_r = 1,
	stereo_output_l = 2,
	stereo_output_r = 3,
	stereo_gain_l = 4,
	stereo_delay_l = 5,
	stereo_gain_r = 6,
	stereo_delay_r = 7
} PortIndex;

typedef struct {
	const float* input_l;
	const float* input_r;
	float* output_l;
	float* output_r;
	const float* gain_l;
	const float* delay_l;
	const float* gain_r;
	const float* delay_r;
} Stereo;

static LV2_Handle instantiate ( // {{{1
	const LV2_Descriptor *descriptor,
	double rate,
	const char *bundle_path,
	const LV2_Feature* const* features
) {
	Stereo *plugin = (Stereo *)malloc(sizeof(Stereo));

	samplerate = rate;
	buffer_l = (float *)malloc( round((samplerate / 1000.0) * MAX_DELAY_MS) * sizeof(float) );
	buffer_r = (float *)malloc( round((samplerate / 1000.0) * MAX_DELAY_MS) * sizeof(float) );

#ifdef DEBUG
	printf("Sample rate: %d\n", (int)samplerate);
#endif

	return (LV2_Handle)plugin;
} // instantiate }}}1

static void connect_port ( // {{{1
	LV2_Handle instance,
	uint32_t port,
	void* data
) {
	Stereo *plugin = (Stereo *)instance;

	switch((PortIndex)port) {
	case stereo_input_l:
		plugin->input_l = (const float *)data;
		break;
	case stereo_input_r:
		plugin->input_r = (const float *)data;
		break;
	case stereo_output_l:
		plugin->output_l = (float *)data;
		break;
	case stereo_output_r:
		plugin->output_r = (float *)data;
		break;
	case stereo_gain_l:
		plugin->gain_l = (const float *)data;
		break;
	case stereo_delay_l:
		plugin->delay_l = (const float *)data;
		break;
	case stereo_gain_r:
		plugin->gain_r = (const float *)data;
		break;
	case stereo_delay_r:
		plugin->delay_r = (const float *)data;
		break;
	}
} // connect_port() }}}1

static void run ( // {{{1
	LV2_Handle instance,
	uint32_t n_samples
) {
	const Stereo *plugin = (const Stereo *)instance;

	const float* const input_l = plugin->input_l;
	const float* const input_r = plugin->input_r;
	float* const output_l = plugin->output_l;
	float* const output_r = plugin->output_r;
	const float gain_l = *(plugin->gain_l);
	const float delay_l = *(plugin->delay_l);
	const float gain_r = *(plugin->gain_r);
	const float delay_r = *(plugin->delay_r);

	float gain_l_val = DB_CO( gain_l );
	float gain_r_val = DB_CO( gain_r );

	inline float prepareDelay(float delay) {
		if (delay < 0) return 0;
		else if (delay > MAX_DELAY_MS) return MAX_DELAY_MS;
		else return delay;
	}

	float delay_l_val = prepareDelay(delay_l);
	float delay_r_val = prepareDelay(delay_r);

	inline float initDelay(
		float delay,
		float *last_delay_ms,
		int32_t *last_delay_samples,
		int32_t *sample_i,
		char *channelName
	) {
		if (*last_delay_ms != delay) {
			*last_delay_ms = delay;
			*last_delay_samples = round((samplerate / 1000.0) * (*last_delay_ms));
			*sample_i = -(*last_delay_samples);
#ifdef DEBUG
			printf("New %s delay in samples: %d\n", channelName, *last_delay_samples);
#endif
		}
	}

	initDelay(delay_l_val, &last_delay_ms_l, &last_delay_samples_l, &sample_i_l, "L");
	initDelay(delay_r_val, &last_delay_ms_r, &last_delay_samples_r, &sample_i_r, "R");

	uint32_t i;
	for (i=0; i<n_samples; i++) {

		void chProc(
			float input,
			float *output,
			float *buffer,
			int32_t *sample_i,
			int32_t last_delay_samples,
			float gain
		) {
			if (*sample_i >= last_delay_samples) *sample_i = 0;

			if (*sample_i < 0) {
				//printf("%d %f\n", *sample_i, input);
				buffer[ (*sample_i) + last_delay_samples ] = input;
				*output = 0;
			} else {
				if (last_delay_samples == 0) {
					*output = input * gain;
				} else {
					if ((*sample_i) - 1 < 0) {
						buffer[ last_delay_samples - 1 ] = input;
					} else {
						buffer[ (*sample_i) - 1 ] = input;
					}
					*output = buffer[ *sample_i ] * gain;
				}
			}

			(*sample_i)++;
		}

		chProc(input_l[i], &output_l[i], buffer_l, &sample_i_l, last_delay_samples_l, gain_l_val);
		chProc(input_r[i], &output_r[i], buffer_r, &sample_i_r, last_delay_samples_r, gain_r_val);

	}
} // run() }}}1

// destroy
static void cleanup ( LV2_Handle instance ) {
	free( instance );
	free( buffer_l );
	free( buffer_r );
}

static const LV2_Descriptor stereo_descriptor = {
	URI_STEREO,
	instantiate,
	connect_port,
	NULL,
	run,
	NULL,
	cleanup
};

LV2_SYMBOL_EXPORT const LV2_Descriptor* lv2_descriptor ( uint32_t index ) {
	switch (index) {
	case 0:
		return &stereo_descriptor;
	default:
		return NULL;
	}
}
