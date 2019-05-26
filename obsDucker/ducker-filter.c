#include <stdint.h>
#include <inttypes.h>
#include <math.h>

#include <obs-module.h>
#include <media-io/audio-math.h>
#include <util/platform.h>
#include <util/circlebuf.h>
#include <util/threading.h>

#pragma region Defines

#define do_log(level, format, ...) \
	blog(level, "[ducker: '%s'] " format, \
			obs_source_get_name(cd->context), ##__VA_ARGS__)

#define warn(format, ...)  do_log(LOG_WARNING, format, ##__VA_ARGS__)
#define info(format, ...)  do_log(LOG_INFO,    format, ##__VA_ARGS__)

#ifdef _DEBUG
#define debug(format, ...) do_log(LOG_DEBUG,   format, ##__VA_ARGS__)
#else
#define debug(format, ...)
#endif

#define S_RATIO				"ratio"
#define S_THRESHOLD			"threshold"
#define S_OPEN_THRESHOLD                "open_threshold"
#define S_CLOSE_THRESHOLD               "close_threshold"
#define S_ATTACK_TIME                   "attack_time"
#define S_RELEASE_TIME                  "release_time"
#define S_HOLD_TIME			"hold_time"
#define S_DUCKING_SOURCE		"ducking_source"
#define S_LIMITER_THRESHOLD		"limiter_threshold"

#define MT_ obs_module_text
#define TEXT_RATIO			"Ratio (X:1)"
#define TEXT_THRESHOLD                  "Threshold (dB)"
#define TEXT_OPEN_THRESHOLD             "Open Threshold (dB)"
#define TEXT_CLOSE_THRESHOLD            "Close Threshold (dB)"
#define TEXT_ATTACK_TIME                "Attack (ms)"
#define TEXT_RELEASE_TIME               "Release (ms)"
#define TEXT_HOLD_TIME			"Hold (ms)"
#define TEXT_DUCKING_SOURCE		"Ducking Source"
#define TEXT_LIMITER_THRESHOLD		"Limiter Threshold (dB)"

#define MIN_RATIO                       1.0
#define MAX_RATIO                       32.0
#define MIN_THRESHOLD_DB                -60.0
#define MAX_THRESHOLD_DB                0.0f
#define MIN_ATK_HLD_RLS_MS              1
#define MAX_RLS_HLD_MS                  10000
#define MAX_ATK_MS                      500

#define DEFAULT_AUDIO_BUF_MS            10

#define MS_IN_S                         1000
#define MS_IN_S_F                       ((float)MS_IN_S)
#pragma endregion

#pragma region Structs

struct ducker_data
{
	obs_source_t *context;

	size_t audio_buf_len;

	float ratio;
	float threshold_mul;
	float attack_rate;
	float release_rate;
	float open_threshold_mul;
	float close_threshold_mul;
	float hold_time_s;
	float held_time_s;
	float limiter_threshold_mul;

	float sample_period_s;
	
	float gate;
	bool isOpen;

	size_t num_channels;
	uint32_t sample_rate;

	pthread_mutex_t sidechain_update_mutex;
	uint64_t sidechain_check_time;
	obs_weak_source_t *weak_sidechain;
	char *sidechain_name;

	pthread_mutex_t sidechain_mutex;
	struct circlebuf sidechain_data[MAX_AUDIO_CHANNELS];
	float *sidechain_buf[MAX_AUDIO_CHANNELS];
	size_t max_sidechain_frames;
};

struct sidechain_prop_info
{
	obs_property_t *sources;
	obs_source_t *parent;
};

#pragma endregion

#pragma region Helpers

static inline obs_source_t *get_sidechain(struct ducker_data *cd)
{
	if (cd->weak_sidechain)
		return obs_weak_source_get_source(cd->weak_sidechain);
	return NULL;
}

static inline void get_sidechain_data(struct ducker_data *cd,
	const uint32_t num_samples)
{
	size_t data_size = cd->audio_buf_len * sizeof(float);
	if (!data_size)
		return;

	pthread_mutex_lock(&cd->sidechain_mutex);
	if (cd->max_sidechain_frames < num_samples)
		cd->max_sidechain_frames = num_samples;

	if (cd->sidechain_data[0].size < data_size)
	{
		pthread_mutex_unlock(&cd->sidechain_mutex);
		goto clear;
	}

	for (size_t i = 0; i < cd->num_channels; i++)
		circlebuf_pop_front(&cd->sidechain_data[i],
			cd->sidechain_buf[i], data_size);

	pthread_mutex_unlock(&cd->sidechain_mutex);
	return;

clear:
	for (size_t i = 0; i < cd->num_channels; i++)
		memset(cd->sidechain_buf[i], 0, data_size);
}

static void resize_env_buffer(struct ducker_data *cd, size_t len)
{
	cd->audio_buf_len = len;
	//cd->envelope_buf = brealloc(cd->envelope_buf, len * sizeof(float));

	for (size_t i = 0; i < cd->num_channels; i++)
		cd->sidechain_buf[i] = brealloc(cd->sidechain_buf[i],
			len * sizeof(float));
}


static inline float ms_to_secf(int ms)
{
	return (float)ms / 1000.0f;
}


#pragma endregion

#pragma region Callbacks

static bool add_sources(void *data, obs_source_t *source)
{
	struct sidechain_prop_info *info = data;
	uint32_t caps = obs_source_get_output_flags(source);

	if (source == info->parent)
		return true;
	if ((caps & OBS_SOURCE_AUDIO) == 0)
		return true;

	const char *name = obs_source_get_name(source);
	obs_property_list_add_string(info->sources, name, name);
	return true;
}

static void sidechain_capture(void *param, obs_source_t *source,
	const struct audio_data *audio_data, bool muted)
{
	struct ducker_data *cd = param;

	UNUSED_PARAMETER(source);

	pthread_mutex_lock(&cd->sidechain_mutex);

	if (cd->max_sidechain_frames < audio_data->frames)
		cd->max_sidechain_frames = audio_data->frames;

	size_t expected_size = cd->max_sidechain_frames * sizeof(float);

	if (!expected_size)
		goto unlock;

	if (cd->sidechain_data[0].size > expected_size * 2)
	{
		for (size_t i = 0; i < cd->num_channels; i++)
		{
			circlebuf_pop_front(&cd->sidechain_data[i], NULL,
				expected_size);
		}
	}

	if (muted)
	{
		for (size_t i = 0; i < cd->num_channels; i++)
		{
			circlebuf_push_back_zero(&cd->sidechain_data[i],
				audio_data->frames * sizeof(float));
		}
	}
	else
	{
		for (size_t i = 0; i < cd->num_channels; i++)
		{
			circlebuf_push_back(&cd->sidechain_data[i],
				audio_data->data[i],
				audio_data->frames * sizeof(float));
		}
	}

unlock:
	pthread_mutex_unlock(&cd->sidechain_mutex);
}

#pragma endregion

#pragma region SourceCallbacks
static const char *ducker_name(void *unused)
{
	UNUSED_PARAMETER(unused);
	return "Ducker";
}

static void ducker_destroy(void *data)
{
	struct ducker_data *cd = data;

	if (cd->weak_sidechain)
	{
		obs_source_t *sidechain = get_sidechain(cd);
		if (sidechain)
		{
			obs_source_remove_audio_capture_callback(sidechain,
				sidechain_capture, cd);
			obs_source_release(sidechain);
		}

		obs_weak_source_release(cd->weak_sidechain);
	}

	for (size_t i = 0; i < MAX_AUDIO_CHANNELS; i++)
	{
		circlebuf_free(&cd->sidechain_data[i]);
		bfree(cd->sidechain_buf[i]);
	}
	pthread_mutex_destroy(&cd->sidechain_mutex);
	pthread_mutex_destroy(&cd->sidechain_update_mutex);

	bfree(cd->sidechain_name);
	//bfree(cd->envelope_buf);
	bfree(cd);
}

static void ducker_update(void *data, obs_data_t *s)
{
	struct ducker_data *cd = data;

	const uint32_t sample_rate =
		audio_output_get_sample_rate(obs_get_audio());
	const size_t num_channels =
		audio_output_get_channels(obs_get_audio());
	const float attack_time_ms =
		(float)obs_data_get_int(s, S_ATTACK_TIME);
	const float release_time_ms =
		(float)obs_data_get_int(s, S_RELEASE_TIME);
	const float hold_time =
		(float)obs_data_get_int(s, S_HOLD_TIME);

	const char *sidechain_name =
		obs_data_get_string(s, S_DUCKING_SOURCE);

	cd->ratio = (float)obs_data_get_double(s, S_RATIO);
	cd->threshold_mul = db_to_mul((float)obs_data_get_double(s, S_THRESHOLD));
	cd->open_threshold_mul = db_to_mul((float)obs_data_get_double(s, S_OPEN_THRESHOLD));
	cd->close_threshold_mul = db_to_mul((float)obs_data_get_double(s, S_CLOSE_THRESHOLD));
	cd->limiter_threshold_mul = db_to_mul((float)obs_data_get_double(s, S_LIMITER_THRESHOLD));	

	cd->attack_rate = 1.0f / (ms_to_secf(attack_time_ms) * (float)sample_rate);
	cd->release_rate = 1.0f / (ms_to_secf(release_time_ms) * (float)sample_rate);

	cd->hold_time_s = ms_to_secf(hold_time);

	cd->num_channels = num_channels;
	cd->sample_rate = sample_rate;
	cd->sample_period_s = 1.0f / sample_rate;
	
	bool valid_sidechain =
		*sidechain_name && strcmp(sidechain_name, "none") != 0;
	obs_weak_source_t *old_weak_sidechain = NULL;

	pthread_mutex_lock(&cd->sidechain_update_mutex);

	if (!valid_sidechain)
	{
		if (cd->weak_sidechain)
		{
			old_weak_sidechain = cd->weak_sidechain;
			cd->weak_sidechain = NULL;
		}

		bfree(cd->sidechain_name);
		cd->sidechain_name = NULL;

	}
	else
	{
		if (!cd->sidechain_name ||
			strcmp(cd->sidechain_name, sidechain_name) != 0)
		{
			if (cd->weak_sidechain)
			{
				old_weak_sidechain = cd->weak_sidechain;
				cd->weak_sidechain = NULL;
			}

			bfree(cd->sidechain_name);
			cd->sidechain_name = bstrdup(sidechain_name);
			cd->sidechain_check_time = os_gettime_ns() - 3000000000;
		}
	}

	pthread_mutex_unlock(&cd->sidechain_update_mutex);

	if (old_weak_sidechain)
	{
		obs_source_t *old_sidechain =
			obs_weak_source_get_source(old_weak_sidechain);

		if (old_sidechain)
		{
			obs_source_remove_audio_capture_callback(old_sidechain,
				sidechain_capture, cd);
			obs_source_release(old_sidechain);
		}

		obs_weak_source_release(old_weak_sidechain);
	}

	size_t sample_len = sample_rate * DEFAULT_AUDIO_BUF_MS / MS_IN_S;
	if (cd->audio_buf_len == 0)
		resize_env_buffer(cd, sample_len);
}

static void *ducker_create(obs_data_t *settings, obs_source_t *filter)
{
	struct ducker_data *cd = bzalloc(sizeof(struct ducker_data));
	cd->context = filter;
	cd->isOpen = false;
	cd->held_time_s = 0.0f;
	cd->audio_buf_len = 0;
	cd->level = 0.0f;

	if (pthread_mutex_init(&cd->sidechain_mutex, NULL) != 0)
	{
		blog(LOG_ERROR, "Failed to create mutex");
		bfree(cd);
		return NULL;
	}

	if (pthread_mutex_init(&cd->sidechain_update_mutex, NULL) != 0)
	{
		pthread_mutex_destroy(&cd->sidechain_mutex);
		blog(LOG_ERROR, "Failed to create mutex");
		bfree(cd);
		return NULL;
	}

	ducker_update(cd, settings);
	return cd;
}

static struct obs_audio_data *ducker_filter_audio(void *data,
	struct obs_audio_data *audio)
{
	struct ducker_data *cd = data;

	const float ratio = cd->ratio;
	const float threshold = cd->threshold_mul;
	const float close_threshold = cd->close_threshold_mul;
	const float open_threshold = cd->open_threshold_mul;
	const uint32_t sample_rate = cd->sample_rate;
	const float release_rate = cd->release_rate;
	const float attack_rate = cd->attack_rate;
	const float hold_time = cd->hold_time_s;	
	const float sample_period_s = cd->sample_period_s;
	const float limiter_threshold = cd->limiter_threshold_mul;

	const uint32_t num_samples = audio->frames;
	if (num_samples == 0)
		return audio;

	if (cd->audio_buf_len < num_samples)
	{
		resize_env_buffer(cd, num_samples);
	}

	pthread_mutex_lock(&cd->sidechain_update_mutex);
	obs_weak_source_t *weak_sidechain = cd->weak_sidechain;
	pthread_mutex_unlock(&cd->sidechain_update_mutex);

	float **adata = (float**)audio->data;
	float **sidechain_buf = cd->sidechain_buf;

	if (weak_sidechain)
	{
		get_sidechain_data(cd, num_samples);

		const size_t channels = cd->num_channels;
		for (uint32_t i = 0; i < num_samples; ++i)
		{
			float cur_level = fabsf(adata[0][i]);
			float cur_sc_level = fabsf(sidechain_buf[0][i]);

			for (size_t j = 0; j < channels; j++)
			{
				cur_level = fmaxf(cur_level, fabsf(adata[j][i]));
				cur_sc_level = fmaxf(cur_sc_level, fabsf(sidechain_buf[j][i]));
			}

			

			if (cur_sc_level > open_threshold)
			{
				cd->isOpen = true;
				cd->held_time_s = 0.0f;

			} else if (cur_sc_level < close_threshold)
			{
				cd->isOpen = false;
				
			}

			//cd->level = fmaxf(cd->level, cur_sc_level) - decay_rate;

			if (!cd->isOpen)
			{

				if (cd->held_time_s < hold_time)
				{
					cd->held_time_s += sample_period_s;
					cd->gate = fminf(1.0f,
						cd->gate + attack_rate);
				}
				else
				{
					cd->gate = fmaxf(0.0f,
						cd->gate - release_rate);
				}
			}
			else
			{
				cd->gate = fminf(1.0f,
					cd->gate + attack_rate);
			}
			float cur_level_db = mul_to_db(cur_level);
			float db_Delta = fmaxf(0.0f, cur_level_db - mul_to_db(threshold));
			float gain_reduction = fmaxf( db_Delta - ( db_Delta / ratio), cur_level_db - mul_to_db(limiter_threshold));
			gain_reduction *= cd->gate;
			
			gain_reduction = gain_reduction > 0 ? (1.0f / db_to_mul(gain_reduction)) : 1.0f;
			
			for (size_t c = 0; c < channels; c++)
				adata[c][i] *= gain_reduction;

		}

	}
	return audio;
}

static void ducker_tick(void *data, float seconds)
{
	struct ducker_data *cd = data;
	char *new_name = NULL;

	pthread_mutex_lock(&cd->sidechain_update_mutex);

	if (cd->sidechain_name && !cd->weak_sidechain)
	{
		uint64_t t = os_gettime_ns();

		if (t - cd->sidechain_check_time > 3000000000)
		{
			new_name = bstrdup(cd->sidechain_name);
			cd->sidechain_check_time = t;
		}
	}

	pthread_mutex_unlock(&cd->sidechain_update_mutex);

	if (new_name)
	{
		obs_source_t *sidechain = new_name && *new_name ?
			obs_get_source_by_name(new_name) : NULL;
		obs_weak_source_t *weak_sidechain = sidechain ?
			obs_source_get_weak_source(sidechain) : NULL;

		pthread_mutex_lock(&cd->sidechain_update_mutex);

		if (cd->sidechain_name &&
			strcmp(cd->sidechain_name, new_name) == 0)
		{
			cd->weak_sidechain = weak_sidechain;
			weak_sidechain = NULL;
		}

		pthread_mutex_unlock(&cd->sidechain_update_mutex);

		if (sidechain)
		{
			obs_source_add_audio_capture_callback(sidechain,
				sidechain_capture, cd);

			obs_weak_source_release(weak_sidechain);
			obs_source_release(sidechain);
		}

		bfree(new_name);
	}

	UNUSED_PARAMETER(seconds);
}

static void ducker_defaults(obs_data_t *s)
{
	obs_data_set_default_double(s, S_RATIO, 2.0f);
	obs_data_set_default_double(s, S_THRESHOLD, -18.0f);
	obs_data_set_default_double(s, S_LIMITER_THRESHOLD, 0.0f);
	obs_data_set_default_int(s, S_ATTACK_TIME, 6);
	obs_data_set_default_int(s, S_HOLD_TIME, 200);
	obs_data_set_default_int(s, S_RELEASE_TIME, 60);
	obs_data_set_default_string(s, S_DUCKING_SOURCE, "none");
	obs_data_set_default_double(s, S_OPEN_THRESHOLD, -30.0f);
	obs_data_set_default_double(s, S_CLOSE_THRESHOLD, -30.0f);
}

static obs_properties_t *ducker_properties(void *data)
{
	struct ducker_data *cd = data;
	obs_properties_t *props = obs_properties_create();
	obs_source_t *parent = NULL;


	if (cd)
		parent = obs_filter_get_parent(cd->context);

	obs_properties_add_float_slider(props, S_RATIO,
		TEXT_RATIO, MIN_RATIO, MAX_RATIO, 0.10);
	obs_properties_add_float_slider(props, S_THRESHOLD,
		TEXT_THRESHOLD, MIN_THRESHOLD_DB, MAX_THRESHOLD_DB, 0.1);
	obs_properties_add_float_slider(props, S_LIMITER_THRESHOLD,
		TEXT_LIMITER_THRESHOLD, MIN_THRESHOLD_DB, MAX_THRESHOLD_DB, 0.1);
	obs_properties_add_int_slider(props, S_ATTACK_TIME,
		TEXT_ATTACK_TIME, MIN_ATK_HLD_RLS_MS, MAX_ATK_MS, 1);
	obs_properties_add_int_slider(props, S_HOLD_TIME,
		TEXT_HOLD_TIME, MIN_ATK_HLD_RLS_MS, MAX_RLS_HLD_MS, 1);
	obs_properties_add_int_slider(props, S_RELEASE_TIME,
		TEXT_RELEASE_TIME, MIN_ATK_HLD_RLS_MS, MAX_RLS_HLD_MS, 1);

	obs_property_t *sources = obs_properties_add_list(props,
		S_DUCKING_SOURCE, TEXT_DUCKING_SOURCE,
		OBS_COMBO_TYPE_LIST, OBS_COMBO_FORMAT_STRING);

	obs_property_list_add_string(sources, obs_module_text("None"), "none");

	struct sidechain_prop_info info = { sources, parent };
	obs_enum_sources(add_sources, &info);

	obs_properties_add_float_slider(props, S_OPEN_THRESHOLD,
		TEXT_OPEN_THRESHOLD, MIN_THRESHOLD_DB, MAX_THRESHOLD_DB, 0.1);
	obs_properties_add_float_slider(props, S_CLOSE_THRESHOLD,
		TEXT_CLOSE_THRESHOLD, MIN_THRESHOLD_DB, MAX_THRESHOLD_DB, 0.1);

	UNUSED_PARAMETER(data);
	return props;
}

#pragma endregion

struct obs_source_info ducker_filter = {
	.id = "hpducker_filter",
	.type = OBS_SOURCE_TYPE_FILTER,
	.output_flags = OBS_SOURCE_AUDIO,
	.get_name = ducker_name,
	.create = ducker_create,
	.destroy = ducker_destroy,
	.update = ducker_update,
	.filter_audio = ducker_filter_audio,
	.video_tick = ducker_tick,
	.get_defaults = ducker_defaults,
	.get_properties = ducker_properties,
};
