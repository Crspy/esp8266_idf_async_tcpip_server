#include "Util.h"
#include "esp_timer.h"

unsigned long IRAM_ATTR millis()
{
	return (unsigned long)(esp_timer_get_time() / 1000ULL);
}

unsigned long IRAM_ATTR micros()
{
	return (unsigned long)(esp_timer_get_time());
}