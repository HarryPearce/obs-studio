#include <obs-module.h>
#include "obs-filters-config.h"

OBS_DECLARE_MODULE()
OBS_MODULE_USE_DEFAULT_LOCALE("obs-ducker", "en-US")
MODULE_EXPORT const char *obs_module_description(void)
{
	return "HP's Audio Ducker";
}

extern struct obs_source_info ducker_filter;
bool obs_module_load(void)
{
	obs_register_source(&ducker_filter);
	return true;
}
