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

static LV2_Handle instantiate (
	const LV2_Descriptor *descriptor,
	double rate,
	const char *bundle_path,
	const LV2_Feature* const* features
) {
	Stereo *plugin = (Stereo *)malloc(sizeof(Stereo));

	samplerate = rate;
	buffer_l = malloc( round((samplerate / 1000.0) * MAX_DELAY_MS) * sizeof(float) );
	buffer_r = malloc( round((samplerate / 1000.0) * MAX_DELAY_MS) * sizeof(float) );

#ifdef DEBUG
	printf("Sample rate: %d\n", (int)samplerate);
#endif

	return (LV2_Handle)plugin;
}

static void connect_port (
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
}

static void run (
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

	float delay_l_val = delay_l;
	if (delay_l_val < 0) delay_l_val = 0;
	if (delay_l_val > MAX_DELAY_MS) delay_l_val = MAX_DELAY_MS;
	float delay_r_val = delay_r;
	if (delay_r_val < 0) delay_r_val = 0;
	if (delay_r_val > MAX_DELAY_MS) delay_r_val = MAX_DELAY_MS;

	if (last_delay_ms_l != delay_l_val) {
		last_delay_ms_l = delay_l_val;
		last_delay_samples_l = round((samplerate / 1000.0) * last_delay_ms_l);
		sample_i_l = -last_delay_samples_l;
#ifdef DEBUG
		printf("New L delay in samples: %d\n", last_delay_samples_l);
#endif
	}

	if (last_delay_ms_r != delay_r_val) {
		last_delay_ms_r = delay_r_val;
		last_delay_samples_r = round((samplerate / 1000.0) * last_delay_ms_r);
		sample_i_r = -last_delay_samples_r;
#ifdef DEBUG
		printf("New R delay in samples: %d\n", last_delay_samples_r);
#endif
	}

	uint32_t i;
	for (i=0; i<n_samples; i++) {

		// left

		if (sample_i_l >= last_delay_samples_l) sample_i_l = 0;

		if (sample_i_l < 0) {
			output_l[i] = 0;
			buffer_l[ sample_i_l + last_delay_samples_l ] = input_l[i];
		} else {
			if (last_delay_samples_l == 0) {
				output_l[i] = input_l[i] * gain_l_val;
			} else {
				if (sample_i_l - 1 < 0) {
					buffer_l[ last_delay_samples_l - 1 ] = input_l[i];
				} else {
					buffer_l[ sample_i_l - 1 ] = input_l[i];
				}
				output_l[i] = buffer_l[ sample_i_l ] * gain_l_val;
			}
		}

		sample_i_l++;

		// right

		if (sample_i_r >= last_delay_samples_r) sample_i_r = 0;

		if (sample_i_r < 0) {
			output_r[i] = 0;
			buffer_r[ sample_i_r + last_delay_samples_r ] = input_r[i];
		} else {
			if (last_delay_samples_r == 0) {
				output_r[i] = input_r[i] * gain_r_val;
			} else {
				if (sample_i_r - 1 < 0) {
					buffer_r[ last_delay_samples_r - 1 ] = input_r[i];
				} else {
					buffer_r[ sample_i_r - 1 ] = input_r[i];
				}
				output_r[i] = buffer_r[ sample_i_r ] * gain_r_val;
			}
		}

		sample_i_r++;

	}
}

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
