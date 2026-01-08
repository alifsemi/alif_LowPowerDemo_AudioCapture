#include <stdint.h>
int32_t playback_init(void);
int32_t playback_audio(const void *buf, uint32_t len);
int32_t playback_deinit(void);
