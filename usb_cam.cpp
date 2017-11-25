/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#define __STDC_CONSTANT_MACROS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <vector>
#include <iostream>
#include <iterator>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <asm/types.h>          /* for videodev2.h */

extern "C" {
#include <stdint.h>
#include <linux/videodev2.h>
}

#include "usb_cam.h"

#define CLEAR(x) memset (&(x), 0, sizeof (x))

struct buffer {
  void * start;
  size_t length;
};

static char *camera_dev;
static unsigned int pixelformat;
static usb_cam_io_method io = IO_METHOD_MMAP;
static int fd = -1;
struct buffer * buffers = NULL;
static unsigned int n_buffers = 0;

static void errno_exit(const char * s)
{
  exit(EXIT_FAILURE);
}

static int xioctl(int fd, int request, void * arg)
{
  int r;

  do
    r = ioctl(fd, request, arg); while (-1==r&&EINTR==errno);

  return r;
}

static void process_image(const void * src, int len, std::vector<unsigned char>* dest)
{
  dest->clear();
  for (int i = 0; i < len; i++) {
    dest->push_back(((char *) src)[i]);
  }
}

static void stop_capturing(void)
{
  enum v4l2_buf_type type;

  switch (io) {
  case IO_METHOD_READ:
    /* Nothing to do. */
    break;

  case IO_METHOD_MMAP:
  case IO_METHOD_USERPTR:
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (-1==xioctl(fd, VIDIOC_STREAMOFF, &type))
      errno_exit("VIDIOC_STREAMOFF");

    break;
  }
}

static void start_capturing(void)
{
  unsigned int i;
  enum v4l2_buf_type type;

  switch (io) {
  case IO_METHOD_MMAP:
    for(i = 0; i<n_buffers; ++i) {
      struct v4l2_buffer buf;

      CLEAR (buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;

      if (-1==xioctl(fd, VIDIOC_QBUF, &buf))
        errno_exit("VIDIOC_QBUF");
    }

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (-1==xioctl(fd, VIDIOC_STREAMON, &type))
      errno_exit("VIDIOC_STREAMON");

    break;
  }
}

static void uninit_device(void)
{
  unsigned int i;

  switch (io) {
  case IO_METHOD_MMAP:
    for(i = 0; i<n_buffers; ++i)
      if (-1==munmap(buffers[i].start, buffers[i].length))
        errno_exit("munmap");
    break;
  }

  free(buffers);
}

static void init_mmap(void)
{
  struct v4l2_requestbuffers req;

  CLEAR (req);

  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (-1==xioctl(fd, VIDIOC_REQBUFS, &req)) {
    if (EINVAL==errno) {
      exit(EXIT_FAILURE);
    } else {
      errno_exit("VIDIOC_REQBUFS");
    }
  }

  if (req.count<2) {
    exit(EXIT_FAILURE);
  }

  buffers = (buffer*)calloc(req.count, sizeof(*buffers));

  if (!buffers) {
    exit(EXIT_FAILURE);
  }

  for(n_buffers = 0; n_buffers<req.count; ++n_buffers) {
    struct v4l2_buffer buf;

    CLEAR (buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = n_buffers;

    if (-1==xioctl(fd, VIDIOC_QUERYBUF, &buf))
      errno_exit("VIDIOC_QUERYBUF");

    buffers[n_buffers].length = buf.length;
    buffers[n_buffers].start = mmap(NULL /* start anywhere */, buf.length, PROT_READ|PROT_WRITE /* required */, MAP_SHARED /* recommended */, fd, buf.m.offset);

    if (MAP_FAILED==buffers[n_buffers].start)
      errno_exit("mmap");
  }
}

static void init_device(int image_width, int image_height, int framerate, bool compressed)
{
  struct v4l2_capability cap;
  struct v4l2_cropcap cropcap;
  struct v4l2_crop crop;
  struct v4l2_format fmt;
  unsigned int min;

  if (-1==xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
    if (EINVAL==errno) {
      exit(EXIT_FAILURE);
    } else {
      errno_exit("VIDIOC_QUERYCAP");
    }
  }

  if (!(cap.capabilities&V4L2_CAP_VIDEO_CAPTURE)) {
    exit(EXIT_FAILURE);
  }

  switch (io) {
  case IO_METHOD_READ:
    if (!(cap.capabilities&V4L2_CAP_READWRITE)) {
      exit(EXIT_FAILURE);
    }

    break;

  case IO_METHOD_MMAP:
  case IO_METHOD_USERPTR:
    if (!(cap.capabilities&V4L2_CAP_STREAMING)) {
      exit(EXIT_FAILURE);
    }

    break;
  }

  /* Select video input, video standard and tune here. */

  CLEAR (cropcap);

  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (0==xioctl(fd, VIDIOC_CROPCAP, &cropcap)) {
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect; /* reset to default */

    if (-1==xioctl(fd, VIDIOC_S_CROP, &crop)) {
      switch (errno) {
      case EINVAL:
        /* Cropping not supported. */
        break;
      default:
        /* Errors ignored. */
        break;
      }
    }
  } else {
    /* Errors ignored. */
  }

  CLEAR (fmt);

//  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
//  fmt.fmt.pix.width = 640;
//  fmt.fmt.pix.height = 480;
//  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
//  fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = image_width;
  fmt.fmt.pix.height = image_height;
  fmt.fmt.pix.pixelformat = pixelformat;
  fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;


  if (-1==xioctl(fd, VIDIOC_S_FMT, &fmt))
    errno_exit("VIDIOC_S_FMT");

  /* Note VIDIOC_S_FMT may change width and height. */

  /* Buggy driver paranoia. */
  min = fmt.fmt.pix.width*2;
  if (fmt.fmt.pix.bytesperline<min)
    fmt.fmt.pix.bytesperline = min;
  min = fmt.fmt.pix.bytesperline*fmt.fmt.pix.height;
  if (fmt.fmt.pix.sizeimage<min)
    fmt.fmt.pix.sizeimage = min;
  if (compressed) {
    // https://linuxtv.org/downloads/v4l-dvb-apis/uapi/v4l/pixfmt-v4l2.html#c.v4l2_pix_format
    fmt.fmt.pix.bytesperline = 0;
  }

  image_width = fmt.fmt.pix.width;
  image_height = fmt.fmt.pix.height;

  struct v4l2_streamparm stream_params;
  memset(&stream_params, 0, sizeof(stream_params));
  stream_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if(xioctl(fd, VIDIOC_G_PARM, &stream_params) < 0) {
    errno_exit("Couldn't query v4l fps!\n");
  }
  stream_params.parm.capture.timeperframe.numerator = 1;
  stream_params.parm.capture.timeperframe.denominator = framerate;
  if (xioctl(fd, VIDIOC_S_PARM, &stream_params) < 0) {
    errno_exit("Couldn't set camera framerate\n");
  }

  switch (io) {
  case IO_METHOD_MMAP:
    init_mmap();
    break;
  }
}

static void close_device(void)
{
  if (-1==close(fd))
    errno_exit("close");

  fd = -1;
}

static void open_device(void)
{
  struct stat st;

  if (-1==stat(camera_dev, &st)) {
    fd = -1;
    return;
  }

  if (!S_ISCHR (st.st_mode)) {
    fd = -1;
    return;
  }

  fd = open(camera_dev, O_RDWR /* required */|O_NONBLOCK, 0);

  if (-1==fd) {
    return;
  }
}

usb_cam_camera_image_t *usb_cam_camera_start(
  const char* dev, usb_cam_io_method io_method,
  uint32_t pixel_format, int image_width, int image_height, int framerate)
{
  camera_dev = (char*)calloc(1,strlen(dev)+1);
  strcpy(camera_dev,dev);

  usb_cam_camera_image_t *image;
  io = io_method;
  bool compressed = false;
  pixelformat = pixel_format;
  if(pixel_format == V4L2_PIX_FMT_YUYV);
    // no-op
  else if(pixel_format == V4L2_PIX_FMT_UYVY);
    // no-op
  else if(pixel_format == V4L2_PIX_FMT_H264) {
    compressed = true;
  }
  else {
    return NULL;
  }

  open_device();
  if (fd == -1) {
      return NULL;
  }
  init_device(image_width, image_height, framerate, compressed);
  start_capturing();

  image = (usb_cam_camera_image_t *) calloc(1, sizeof(usb_cam_camera_image_t));

  image->width = image_width;
  image->height = image_height;
  image->bytes_per_pixel = 24;

  image->image_size = image->width*image->height*image->bytes_per_pixel;
  image->is_new = 0;
  image->image = (char *) calloc(image->image_size, sizeof(char));
  memset(image->image, 0, image->image_size*sizeof(char));

  return image;
}

void usb_cam_camera_shutdown(void)
{
  stop_capturing();
  uninit_device();
  close_device();
}

void usb_cam_camera_grab_raw(std::vector<unsigned char>* image, bool *keyframe)
{
  fd_set fds;
  struct timeval tv;
  int r;

  FD_ZERO (&fds);
  FD_SET (fd, &fds);

  /* Timeout. */
  tv.tv_sec = 5;
  tv.tv_usec = 0;

  r = select(fd+1, &fds, NULL, NULL, &tv);

  if (-1==r) {
    if (EINTR==errno)
      return;

    errno_exit("select");
  }

  if (0==r) {
    exit(EXIT_FAILURE);
  }

  struct v4l2_buffer buf;
  unsigned int i;
  int len;

  switch (io) {
  case IO_METHOD_MMAP:
    CLEAR (buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (-1==xioctl(fd, VIDIOC_DQBUF, &buf)) {
      switch (errno) {
      case EAGAIN:
        return;

      case EIO:
        /* Could ignore EIO, see spec. */

        /* fall through */

      default:
        errno_exit("VIDIOC_DQBUF");
      }
    }

    assert (buf.index < n_buffers);
    len = buf.bytesused;
    process_image(buffers[buf.index].start, len, image);
    *keyframe = buf.flags & V4L2_BUF_FLAG_KEYFRAME;

    if (-1==xioctl(fd, VIDIOC_QBUF, &buf))
      errno_exit("VIDIOC_QBUF");

    break;
  }
}

void usb_cam_camera_grab_h264(std::vector<unsigned char>* image, bool *keyframe)
{
  usb_cam_camera_grab_raw(image, keyframe);
}

int main()
{
    usb_cam_camera_start("/dev/video0", IO_METHOD_MMAP, V4L2_PIX_FMT_H264, 640, 480, 30);
    std::vector<unsigned char> data;
    bool keyframe = false;
    for (int frame = 0; frame < 100; frame++) {
      data.clear();
      usb_cam_camera_grab_h264(&data, &keyframe);
      for (int i = 3; i < data.size(); i++) {
        // NAL header is 00 00 01 0b0xxyyyyy
        // Where xx is the 2-bit nal_ref_idc and yyyyy is the 5-bit nal_type
        // nal_type 5 is a IDR frame
        if (data[i - 3] == 0 &&
            data[i - 2] == 0 &&
            data[i - 1] == 1) {
          if ((data[i] & 0x1f) == 5) {
              // See: https://linuxtv.org/downloads/v4l-dvb-apis/uapi/v4l/buffer.html#buffer-flags
              std::cout << "BUG! NAL type: " << (data[i] & 0x1f) << " keyframe flag: " << keyframe << std::endl;
              return 1;
          }
        }
      }
    }

    return 0;
}
