/*
 * Copyright (c) 2003 Fabrice Bellard
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/**
 * @file
 * libavformat API example.
 *
 * Output a media file in any supported libavformat format. The default
 * codecs are used.
 * @example muxing.c
 */

#include "libavhelper.h"

#define STREAM_PIX_FMT    AV_PIX_FMT_BGR24 /* default pix_fmt */
#define FRAME_BUFFER 20

#define SCALE_FLAGS SWS_BICUBIC

// a wrapper around a single output AVStream
typedef struct OutputStream {
    AVStream *st;
    AVCodecContext *enc;

    /* pts of the next frame that will be generated */
    int64_t next_pts;
    int samples_count;

    AVFrame *frame[FRAME_BUFFER];

    AVFrame *tmp_frame[FRAME_BUFFER];

    float t, tincr, tincr2;

    struct SwsContext *sws_ctx;
} OutputStream;

typedef struct VideoContext {
   int width;
   int height;
   enum AVPixelFormat pixelformatout;
   AVFormatContext *oc;
   OutputStream video_st;
   AVCodec *video_codec;
   AVOutputFormat *fmt;
	
} VideoContext;

static VideoContext VC = {0};

static void log_packet(const AVFormatContext *fmt_ctx, const AVPacket *pkt)
{
    AVRational *time_base = &fmt_ctx->streams[pkt->stream_index]->time_base;

    printf("pts:%s pts_time:%s dts:%s dts_time:%s duration:%s duration_time:%s stream_index:%d\n",
           av_ts2str(pkt->pts), av_ts2timestr(pkt->pts, time_base),
           av_ts2str(pkt->dts), av_ts2timestr(pkt->dts, time_base),
           av_ts2str(pkt->duration), av_ts2timestr(pkt->duration, time_base),
           pkt->stream_index);
}

static int write_frame(AVFormatContext *fmt_ctx, const AVRational *time_base, AVStream *st, AVPacket *pkt)
{
    /* rescale output packet timestamp values from codec to stream timebase */
    av_packet_rescale_ts(pkt, *time_base, st->time_base);
    pkt->stream_index = st->index;

    /* Write the compressed frame to the media file. */
    //log_packet(fmt_ctx, pkt);
    return av_interleaved_write_frame(fmt_ctx, pkt);
}

/* Add an output stream. */
static void add_stream(OutputStream *ost, AVFormatContext *oc,
                       AVCodec **codec,
                       const char * codecname,
                       int framerate,
                       enum AVCodecID codec_id)
{
    AVCodecContext *c;
    int i;

    /* find the encoder */
if (codecname) {
    *codec = avcodec_find_encoder_by_name(codecname);
    if (!(*codec)) {
        fprintf(stderr, "Could not find encoder named '%s'\n",
                codecname);
        exit(1);
    }
} else {
    *codec = avcodec_find_encoder(codec_id);
    if (!(*codec)) {
        fprintf(stderr, "Could not find encoder for '%s'\n",
                avcodec_get_name(codec_id));
        exit(1);
    }
}

    ost->st = avformat_new_stream(oc, NULL);
    if (!ost->st) {
        fprintf(stderr, "Could not allocate stream\n");
        exit(1);
    }
    ost->st->id = oc->nb_streams-1;
    c = avcodec_alloc_context3(*codec);
    if (!c) {
        fprintf(stderr, "Could not alloc an encoding context\n");
        exit(1);
    }
    ost->enc = c;

    switch ((*codec)->type) {
    case AVMEDIA_TYPE_VIDEO:
        c->codec_id = (*codec)->id;

        c->bit_rate = 400000;
        /* Resolution must be a multiple of two. */
        //c->width    = 352;
        c->width    = VC.width;
        //c->height   = 288;
        c->height   = VC.height;
        /* timebase: This is the fundamental unit of time (in seconds) in terms
         * of which frame timestamps are represented. For fixed-fps content,
         * timebase should be 1/framerate and timestamp increments should be
         * identical to 1. */
        ost->st->time_base = (AVRational){ 1, framerate };
        c->time_base       = ost->st->time_base;

        c->gop_size      = 12; /* emit one intra frame every twelve frames at most */
        c->pix_fmt       = VC.pixelformatout;
        if (c->codec_id == AV_CODEC_ID_MPEG2VIDEO) {
            /* just for testing, we also add B-frames */
            c->max_b_frames = 2;
        }
        if (c->codec_id == AV_CODEC_ID_MPEG1VIDEO) {
            /* Needed to avoid using macroblocks in which some coeffs overflow.
             * This does not happen with normal video, it just happens here as
             * the motion of the chroma plane does not match the luma plane. */
            c->mb_decision = 2;
        }
    break;

    default:
        break;
    }

    /* Some formats want stream headers to be separate. */
    if (oc->oformat->flags & AVFMT_GLOBALHEADER)
        c->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
}


/**************************************************************/
/* video output */

static AVFrame *alloc_picture(enum AVPixelFormat pix_fmt, int width, int height)
{
    AVFrame *picture;
    int ret;

    picture = av_frame_alloc();
    if (!picture)
        return NULL;

    picture->format = pix_fmt;
    picture->width  = width;
    picture->height = height;

    /* allocate the buffers for the frame data */
    ret = av_frame_get_buffer(picture, 32);
    if (ret < 0) {
        fprintf(stderr, "Could not allocate frame data.\n");
        exit(1);
    }

    return picture;
}

static void open_video(AVFormatContext *oc, AVCodec *codec, OutputStream *ost, AVDictionary *opt_arg)
{
    int ret;
    AVCodecContext *c = ost->enc;
    AVDictionary *opt = NULL;

    av_dict_copy(&opt, opt_arg, 0);

    /* open the codec */
    ret = avcodec_open2(c, codec, &opt);
    av_dict_free(&opt);
    if (ret < 0) {
        fprintf(stderr, "Could not open video codec: %s\n", av_err2str(ret));
        exit(1);
    }

    /* allocate and init a re-usable frame */
    int t;
    for (t=0;t<FRAME_BUFFER;t++) {
    ost->frame[t] = alloc_picture(c->pix_fmt, c->width, c->height);
    if (!ost->frame[t]) {
        fprintf(stderr, "Could not allocate video frame\n");
        exit(1);
    }

    /* If the output format is not STREAM_PIX_FMT, then a temporary
     * picture is needed too. It is then converted to the required
     * output format. */
        ost->tmp_frame[t] = NULL;
        if (c->pix_fmt != STREAM_PIX_FMT) {
            ost->tmp_frame[t] = alloc_picture(STREAM_PIX_FMT, c->width, c->height);
            if (!ost->tmp_frame[t]) {
                fprintf(stderr, "Could not allocate temporary picture\n");
                exit(1);
            }
        }
    }

    /* copy the stream parameters to the muxer */
    ret = avcodec_parameters_from_context(ost->st->codecpar, c);
    if (ret < 0) {
        fprintf(stderr, "Could not copy the stream parameters\n");
        exit(1);
    }
}

/* Prepare a dummy image. */
static void fill_rgb_image(uint8_t * source,AVFrame *pict, int width, int height)
{
    int x, y, i, ret;

    /* when we pass a frame to the encoder, it may keep a reference to it
     * internally;
     * make sure we do not overwrite it here
     */
    ret = av_frame_make_writable(pict);
    if (ret < 0)
        exit(1);

    //pict->linesize[0]=width*3;
    //pict->data[0]=source;
    for (y=0;y<height;y++) {
           memcpy(&pict->data[0][y*pict->linesize[0]],&source[width*3*y],width*3);
    }

}

/* retrieve rgb image. */
static void get_rgb_image(uint8_t * dest,AVFrame *pict, int width, int height)
{
    int x, y, i, ret;

    for (y=0;y<height;y++) {
           memcpy(&dest[width*3*y],&pict->data[0][y*pict->linesize[0]],width*3);
    }

}



/* Prepare a dummy image. */
static void fill_yuv_image(AVFrame *pict, int frame_index,
                           int width, int height)
{
    int x, y, i, ret;

    /* when we pass a frame to the encoder, it may keep a reference to it
     * internally;
     * make sure we do not overwrite it here
     */
    ret = av_frame_make_writable(pict);
    if (ret < 0)
        exit(1);

    i = frame_index;

    /* Y */
    for (y = 0; y < height; y++)
        for (x = 0; x < width; x++)
            pict->data[0][y * pict->linesize[0] + x] = x + y + i * 3;

    /* Cb and Cr */
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            pict->data[1][y * pict->linesize[1] + x] = 128 + y + i * 2;
            pict->data[2][y * pict->linesize[2] + x] = 64 + x + i * 5;
        }
    }
}

int libavhelper_read_frame(uint8_t* buffer, unsigned long long *timestamp)
{
    AVFormatContext *oc=VC.oc;
    OutputStream *ist=&VC.video_st;
    int ret;
    AVCodecContext *c;
    AVFrame *frame;
    int got_packet = 0;
    AVPacket pkt = { 0 };

    ret = av_read_frame(oc,&pkt);
    //log_packet(oc, &pkt);

    ist->next_pts=(ist->next_pts+1)%FRAME_BUFFER;
    c = ist->enc;
    if (c->pix_fmt != STREAM_PIX_FMT) {
        while (!got_packet) {
	        ret = avcodec_decode_video2(c,ist->tmp_frame[ist->next_pts],&got_packet,&pkt);
		if (!ret) {
    			av_free_packet(&pkt);
			return 0;
		}
        }
        if (!ist->sws_ctx) {
            ist->sws_ctx = sws_getContext(c->width, c->height,
                                          c->pix_fmt,
                                          c->width, c->height,
                                          STREAM_PIX_FMT,
                                          SCALE_FLAGS, NULL, NULL, NULL);
            if (!ist->sws_ctx) {
                fprintf(stderr,
                        "Could not initialize the conversion context\n");
                exit(1);
            }
        }
        sws_scale(ist->sws_ctx,
                  (const uint8_t * const *)ist->tmp_frame[ist->next_pts]->data, ist->tmp_frame[ist->next_pts]->linesize,
                  0, c->height, ist->frame[ist->next_pts]->data, ist->frame[ist->next_pts]->linesize);
    } else {
        while (!got_packet) {
            ret = avcodec_decode_video2(c,ist->frame[ist->next_pts],&got_packet,&pkt);
	    if (!ret) {
		av_free_packet(&pkt);
		return 0;
	    }
        }
    }
    frame = ist->frame[ist->next_pts];
    *timestamp=pkt.pts;
//frame->pts;
    get_rgb_image(buffer,frame, c->width, c->height);
    av_free_packet(&pkt);
    return 1;
}

/*
 * encode one video frame and send it to the muxer
 * return 1 when encoding is finished, 0 otherwise
 */
void libavhelper_save_frame(uint8_t* buffer, unsigned long long timestamp)
//static int write_video_frame(AVFormatContext *oc, OutputStream *ost)
{
    AVFormatContext *oc=VC.oc;
    OutputStream *ost=&VC.video_st;
    int ret;
    AVCodecContext *c;
    AVFrame *frame;
    int got_packet = 0;
    AVPacket pkt = { 0 };

    ost->next_pts=(ost->next_pts+1)%FRAME_BUFFER;
    c = ost->enc;
    if (c->pix_fmt != STREAM_PIX_FMT) {
        if (!ost->sws_ctx) {
            ost->sws_ctx = sws_getContext(c->width, c->height,
                                          STREAM_PIX_FMT,
                                          c->width, c->height,
                                          c->pix_fmt,
                                          SCALE_FLAGS, NULL, NULL, NULL);
            if (!ost->sws_ctx) {
                fprintf(stderr,
                        "Could not initialize the conversion context\n");
                exit(1);
            }
        }
        fill_rgb_image(buffer,ost->tmp_frame[ost->next_pts], c->width, c->height);
        sws_scale(ost->sws_ctx,
                  (const uint8_t * const *)ost->tmp_frame[ost->next_pts]->data, ost->tmp_frame[ost->next_pts]->linesize,
                  0, c->height, ost->frame[ost->next_pts]->data, ost->frame[ost->next_pts]->linesize);
    } else {
        fill_rgb_image(buffer,ost->frame[ost->next_pts], c->width, c->height);
    }

    ost->frame[ost->next_pts]->pts = timestamp;

    frame = ost->frame[ost->next_pts];

    av_init_packet(&pkt);

    /* encode the image */
    ret = avcodec_encode_video2(c, &pkt, frame, &got_packet);
    if (ret < 0) {
        fprintf(stderr, "Error encoding video frame: %s\n", av_err2str(ret));
        exit(1);
    }

    if (got_packet) {
        ret = write_frame(oc, &c->time_base, ost->st, &pkt);
    } else {
        ret = 0;
    }

    if (ret < 0) {
        fprintf(stderr, "Error while writing video frame: %s\n", av_err2str(ret));
        exit(1);
    }

    //return (frame || got_packet) ? 0 : 1;
    return;
}

static void close_stream(AVFormatContext *oc, OutputStream *ost)
{
    //avcodec_free_context(&ost->enc);
    int t;
    for (t=0;t<FRAME_BUFFER;t++) {
    av_frame_free(&ost->frame[t]);
    av_frame_free(&ost->tmp_frame[t]);
    }
    if(ost->sws_ctx) sws_freeContext(ost->sws_ctx);
}

int libavhelper_init_read(const char *filename, int *width, int *height, enum AVPixelFormat *pixelformatin, unsigned int threads)
{
  
    int ret;
    //VC.width=width; 
    //VC.height=height; 
    //VC.pixelformatout=pixelformatout; 
    AVDictionary *opt = NULL;
    //int i;
    VC.oc = 0;
    
    av_dict_set_int(&opt, "threads", (int64_t)threads ,0);


    /* Initialize libavcodec, and register all codecs and formats. */
    av_register_all();

    if ((ret = avformat_open_input(&VC.oc, filename, 0, 0)) < 0) {
        fprintf(stderr, "Could not open input file '%s'", filename);
        exit(1);
    }

//    if (!VC.oc)
//        exit(1);

    if ((ret = avformat_find_stream_info(VC.oc, 0)) < 0) {
        fprintf(stderr, "Failed to retrieve input stream information");
        exit(1);
    }

    av_dump_format(VC.oc, 0, filename, 0);
    VC.video_st.st=VC.oc->streams[0];
    VC.video_st.enc=VC.video_st.st->codec;
    VC.width=VC.video_st.enc->width;
    VC.height=VC.video_st.enc->height;
    VC.pixelformatout=VC.video_st.enc->pix_fmt;
    VC.video_codec= avcodec_find_decoder(VC.video_st.enc->codec_id);
    //if (VC.pixelformatout==AV_PIX_FMT_BGR0) VC.pixelformatout=AV_PIX_FMT_RGB24;
    //VC.video_st.enc->pix_fmt=VC.pixelformatout;

    *width=VC.width;
    *height=VC.height;
    *pixelformatin=VC.pixelformatout;

    /* Now that all the parameters are set, we can open the audio and
     * video codecs and allocate the necessary encode buffers. */
    open_video(VC.oc, VC.video_codec, &VC.video_st, opt);
    

}
/**************************************************************/
/* media file output */

int libavhelper_init_write(const char *filename, const char* format, const char* codec, int width, int height, int framerate, enum AVPixelFormat pixelformatout, unsigned int threads)
{
  
    VC.width=width; 
    VC.height=height; 
    VC.pixelformatout=pixelformatout; 
    int ret;
    AVDictionary *opt = NULL;
    int i;
    VC.oc = 0;

    av_dict_set_int(&opt, "threads", (int64_t)threads ,0);

    /* Initialize libavcodec, and register all codecs and formats. */
    av_register_all();

    /* allocate the output media context */
    avformat_alloc_output_context2(&VC.oc, NULL, format, NULL);
    if (!VC.oc)
        exit(1);

    VC.fmt = VC.oc->oformat;

    add_stream(&VC.video_st, VC.oc, &VC.video_codec, codec, framerate, AV_CODEC_ID_RAWVIDEO);

    /* Now that all the parameters are set, we can open the audio and
     * video codecs and allocate the necessary encode buffers. */
    open_video(VC.oc, VC.video_codec, &VC.video_st, opt);


    av_dump_format(VC.oc, 0, filename, 1);

    /* open the output file, if needed */
    if (!(VC.fmt->flags & AVFMT_NOFILE)) {
        ret = avio_open(&VC.oc->pb, filename, AVIO_FLAG_WRITE);
        if (ret < 0) {
            fprintf(stderr, "Could not open '%s': %s\n", filename,
                    av_err2str(ret));
            exit(1);
        }
    }

    /* Write the stream header, if any. */
    ret = avformat_write_header(VC.oc, &opt);
    if (ret < 0) {
        fprintf(stderr, "Error occurred when opening output file: %s\n",
                av_err2str(ret));
        exit(1);
    }
}


void libavhelper_close_read(void) {
    close_stream(VC.oc, &VC.video_st);
    avformat_close_input(&VC.oc);
}

void libavhelper_close_write(void) {
    /* Write the trailer, if any. The trailer must be written before you
     * close the CodecContexts open when you wrote the header; otherwise
     * av_write_trailer() may try to use memory that was freed on
     * av_codec_close(). */
    av_write_trailer(VC.oc);

    /* Close each codec. */
        close_stream(VC.oc, &VC.video_st);

    if (!(VC.fmt->flags & AVFMT_NOFILE))
        /* Close the output file. */
        avio_closep(&VC.oc->pb);

    /* free the stream */
    avformat_free_context(VC.oc);

}
