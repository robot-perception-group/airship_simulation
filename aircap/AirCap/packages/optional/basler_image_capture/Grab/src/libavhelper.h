/**
 * @file
 * libavformat API example.
 *
 * Output a media file in any supported libavformat format. The default
 * codecs are used.
 * @example muxing.c
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <libavutil/avassert.h>
#include <libavutil/channel_layout.h>
#include <libavutil/opt.h>
#include <libavutil/mathematics.h>
#include <libavutil/timestamp.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libswresample/swresample.h>

int libavhelper_init_write(const char *filename, const char* format, const char* codec, int width, int height, int framerate, enum AVPixelFormat pixelformatout, unsigned int threads);
int libavhelper_init_read(const char *filename, int* width, int *height, enum AVPixelFormat *pixelformatin, unsigned int threads);
void libavhelper_close_read(void);
void libavhelper_close_write(void);
void libavhelper_save_frame(uint8_t* buffer, unsigned long long timestamp);
int libavhelper_read_frame(uint8_t* buffer, unsigned long long *timestamp);
