#include "settings_interface.h"


SETTINGS_INTERFACE settings;

bool ( * notify_callbacks[ SETTING_NOTIFY_TOTAL ] )( setget_param_enum param_id, settings_container  data ) = { nullptr };
bool ( * load_callbacks[   SETTING_NOTIFY_TOTAL ] )( setget_param_enum param_id, settings_container *data ) = { nullptr }; // called before a setting is loaded
