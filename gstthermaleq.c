/*
 * thermaleq — plateau histogram equalization for 16-bit thermal video.
 *
 * Converts GRAY16 to GRAY8. Instead of a linear stretch, output gray levels
 * are handed out in proportion to how many pixels use each part of the input
 * range, with a per-value cap (the "plateau") so large uniform regions such
 * as sky cannot take over the output range.
 *
 * On top of that:
 *  - the mapping is smoothed over time (adapt-time), and speeds up
 *    automatically when the scene changes a lot (scene-threshold,
 *    fast-adapt-time);
 *  - it can be mixed with a plain linear stretch (linear-blend);
 *  - the stretch can be gain-limited (max-gain) and held while the scene is
 *    near-uniform, e.g. during a shutter calibration (shutter-span);
 *  - fine detail can be boosted independently of the tone curve (detail);
 *  - the histogram can be taken from a region of interest and subsampled.
 *
 * Example:
 *   gst-launch-1.0 pleorasrc ! video/x-raw,format=GRAY16_LE ! thermaleq ! \
 *       videoconvert ! autovideosink
 */

#include <gst/gst.h>
#include <gst/video/video.h>
#include <gst/video/gstvideofilter.h>
#include <math.h>
#include <string.h>

GST_DEBUG_CATEGORY_STATIC (gst_thermal_eq_debug);
#define GST_CAT_DEFAULT gst_thermal_eq_debug

#define NBINS 65536

/* Fraction of pixels ignored at each end when measuring the input range. */
#define RANGE_TAIL 0.01
/* Half-width, in input counts, of the window used to measure curve slope. */
#define SLOPE_WINDOW 8
/* Detail boost peaks around this many output levels and fades out beyond it,
 * so strong edges are left alone instead of growing halos. */
#define DETAIL_KNEE 48.0
#define DETAIL_TBL_HALF 65535

#define DEFAULT_PLATEAU            0.002
#define DEFAULT_ADAPT_TIME         300
#define DEFAULT_FAST_ADAPT_TIME    50
#define DEFAULT_SCENE_THRESHOLD    20.0
#define DEFAULT_LINEAR_BLEND       0.1
#define DEFAULT_MAX_GAIN           0.0
#define DEFAULT_SHUTTER_SPAN       0
#define DEFAULT_SHUTTER_HOLD_TIME  1000
#define DEFAULT_DETAIL             0.0
#define DEFAULT_DETAIL_RADIUS      8
#define DEFAULT_ROI_X              0
#define DEFAULT_ROI_Y              0
#define DEFAULT_ROI_WIDTH          0
#define DEFAULT_ROI_HEIGHT         0
#define DEFAULT_INVERT             FALSE
#define DEFAULT_HISTOGRAM_STEP     2

enum
{
  PROP_0,
  PROP_PLATEAU,
  PROP_ADAPT_TIME,
  PROP_FAST_ADAPT_TIME,
  PROP_SCENE_THRESHOLD,
  PROP_LINEAR_BLEND,
  PROP_MAX_GAIN,
  PROP_SHUTTER_SPAN,
  PROP_SHUTTER_HOLD_TIME,
  PROP_DETAIL,
  PROP_DETAIL_RADIUS,
  PROP_ROI_X,
  PROP_ROI_Y,
  PROP_ROI_WIDTH,
  PROP_ROI_HEIGHT,
  PROP_INVERT,
  PROP_HISTOGRAM_STEP,
  /* read-only statistics */
  PROP_INPUT_LOW,
  PROP_INPUT_HIGH,
  PROP_SCENE_CHANGE,
  PROP_FROZEN,
};

#define GST_TYPE_THERMAL_EQ (gst_thermal_eq_get_type ())
G_DECLARE_FINAL_TYPE (GstThermalEq, gst_thermal_eq, GST, THERMAL_EQ,
    GstVideoFilter)

struct _GstThermalEq
{
  GstVideoFilter parent;

  /* properties (protected by the object lock) */
  gdouble plateau;
  guint adapt_time;
  guint fast_adapt_time;
  gdouble scene_threshold;
  gdouble linear_blend;
  gdouble max_gain;
  guint shutter_span;
  guint shutter_hold_time;
  gdouble detail;
  guint detail_radius;
  gint roi_x, roi_y, roi_width, roi_height;
  gboolean invert;
  guint histogram_step;

  /* statistics of the last frame (protected by the object lock) */
  gint input_low;
  gint input_high;
  gdouble scene_change;
  gboolean frozen;

  /* streaming state */
  gboolean swap;                /* input endianness differs from the host */
  guint32 *hist;                /* NBINS sampled pixel counts */
  gfloat *target;               /* NBINS mapping wanted for this frame */
  gfloat *lut;                  /* NBINS smoothed mapping, 0..255 */
  guint8 *lut8;                 /* NBINS rounded (and inverted) copy of lut */
  gboolean have_lut;            /* FALSE until the first frame is processed */
  GstClockTime last_pts;
  GstClockTime hold_elapsed;    /* how long the scene has been near-uniform */
  gdouble detail_gain;          /* smoothed typical curve slope, levels/count */

  /* detail enhancement scratch space, sized for the current frame */
  gint16 *detail_tbl;           /* 2 * DETAIL_TBL_HALF + 1 entries */
  guint16 *blur_tmp;
  guint16 *base;
  guint32 *colsum;
  guint64 *prefix;
};

G_DEFINE_TYPE (GstThermalEq, gst_thermal_eq, GST_TYPE_VIDEO_FILTER);

static GstStaticPadTemplate sink_template = GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK, GST_PAD_ALWAYS,
    GST_STATIC_CAPS (GST_VIDEO_CAPS_MAKE ("{ GRAY16_LE, GRAY16_BE, GRAY8 }")));

static GstStaticPadTemplate src_template = GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC, GST_PAD_ALWAYS,
    GST_STATIC_CAPS (GST_VIDEO_CAPS_MAKE ("GRAY8")));

#define PIX(p) (swap ? GUINT16_SWAP_LE_BE (p) : (p))

static void
gst_thermal_eq_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec)
{
  GstThermalEq *self = GST_THERMAL_EQ (object);

  GST_OBJECT_LOCK (self);
  switch (prop_id) {
    case PROP_PLATEAU:
      self->plateau = g_value_get_double (value);
      break;
    case PROP_ADAPT_TIME:
      self->adapt_time = g_value_get_uint (value);
      break;
    case PROP_FAST_ADAPT_TIME:
      self->fast_adapt_time = g_value_get_uint (value);
      break;
    case PROP_SCENE_THRESHOLD:
      self->scene_threshold = g_value_get_double (value);
      break;
    case PROP_LINEAR_BLEND:
      self->linear_blend = g_value_get_double (value);
      break;
    case PROP_MAX_GAIN:
      self->max_gain = g_value_get_double (value);
      break;
    case PROP_SHUTTER_SPAN:
      self->shutter_span = g_value_get_uint (value);
      break;
    case PROP_SHUTTER_HOLD_TIME:
      self->shutter_hold_time = g_value_get_uint (value);
      break;
    case PROP_DETAIL:
      self->detail = g_value_get_double (value);
      break;
    case PROP_DETAIL_RADIUS:
      self->detail_radius = g_value_get_uint (value);
      break;
    case PROP_ROI_X:
      self->roi_x = g_value_get_int (value);
      break;
    case PROP_ROI_Y:
      self->roi_y = g_value_get_int (value);
      break;
    case PROP_ROI_WIDTH:
      self->roi_width = g_value_get_int (value);
      break;
    case PROP_ROI_HEIGHT:
      self->roi_height = g_value_get_int (value);
      break;
    case PROP_INVERT:
      self->invert = g_value_get_boolean (value);
      break;
    case PROP_HISTOGRAM_STEP:
      self->histogram_step = g_value_get_uint (value);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
  GST_OBJECT_UNLOCK (self);
}

static void
gst_thermal_eq_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec)
{
  GstThermalEq *self = GST_THERMAL_EQ (object);

  GST_OBJECT_LOCK (self);
  switch (prop_id) {
    case PROP_PLATEAU:
      g_value_set_double (value, self->plateau);
      break;
    case PROP_ADAPT_TIME:
      g_value_set_uint (value, self->adapt_time);
      break;
    case PROP_FAST_ADAPT_TIME:
      g_value_set_uint (value, self->fast_adapt_time);
      break;
    case PROP_SCENE_THRESHOLD:
      g_value_set_double (value, self->scene_threshold);
      break;
    case PROP_LINEAR_BLEND:
      g_value_set_double (value, self->linear_blend);
      break;
    case PROP_MAX_GAIN:
      g_value_set_double (value, self->max_gain);
      break;
    case PROP_SHUTTER_SPAN:
      g_value_set_uint (value, self->shutter_span);
      break;
    case PROP_SHUTTER_HOLD_TIME:
      g_value_set_uint (value, self->shutter_hold_time);
      break;
    case PROP_DETAIL:
      g_value_set_double (value, self->detail);
      break;
    case PROP_DETAIL_RADIUS:
      g_value_set_uint (value, self->detail_radius);
      break;
    case PROP_ROI_X:
      g_value_set_int (value, self->roi_x);
      break;
    case PROP_ROI_Y:
      g_value_set_int (value, self->roi_y);
      break;
    case PROP_ROI_WIDTH:
      g_value_set_int (value, self->roi_width);
      break;
    case PROP_ROI_HEIGHT:
      g_value_set_int (value, self->roi_height);
      break;
    case PROP_INVERT:
      g_value_set_boolean (value, self->invert);
      break;
    case PROP_HISTOGRAM_STEP:
      g_value_set_uint (value, self->histogram_step);
      break;
    case PROP_INPUT_LOW:
      g_value_set_int (value, self->input_low);
      break;
    case PROP_INPUT_HIGH:
      g_value_set_int (value, self->input_high);
      break;
    case PROP_SCENE_CHANGE:
      g_value_set_double (value, self->scene_change);
      break;
    case PROP_FROZEN:
      g_value_set_boolean (value, self->frozen);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
  GST_OBJECT_UNLOCK (self);
}

static void
gst_thermal_eq_free_frame_buffers (GstThermalEq * self)
{
  g_clear_pointer (&self->blur_tmp, g_free);
  g_clear_pointer (&self->base, g_free);
  g_clear_pointer (&self->colsum, g_free);
  g_clear_pointer (&self->prefix, g_free);
}

static void
gst_thermal_eq_finalize (GObject * object)
{
  GstThermalEq *self = GST_THERMAL_EQ (object);

  gst_thermal_eq_free_frame_buffers (self);
  g_free (self->hist);
  g_free (self->target);
  g_free (self->lut);
  g_free (self->lut8);
  g_free (self->detail_tbl);

  G_OBJECT_CLASS (gst_thermal_eq_parent_class)->finalize (object);
}

/* The pads differ only in pixel format: the source pad is always GRAY8, the
 * sink pad takes 16-bit gray (equalized) or GRAY8 (passed through). */
static GstCaps *
gst_thermal_eq_transform_caps (GstBaseTransform * trans,
    GstPadDirection direction, GstCaps * caps, GstCaps * filter)
{
  static const gchar *const sink_formats[] =
      { "GRAY16_LE", "GRAY16_BE", "GRAY8" };
  static const gchar *const src_formats[] = { "GRAY8" };
  const gchar *const *formats;
  guint n_formats;
  GstCaps *res = gst_caps_new_empty ();
  guint i, f, n = gst_caps_get_size (caps);

  if (direction == GST_PAD_SINK) {
    formats = src_formats;
    n_formats = G_N_ELEMENTS (src_formats);
  } else {
    formats = sink_formats;
    n_formats = G_N_ELEMENTS (sink_formats);
  }

  for (i = 0; i < n; i++) {
    for (f = 0; f < n_formats; f++) {
      GstStructure *st = gst_structure_copy (gst_caps_get_structure (caps, i));

      gst_structure_set (st, "format", G_TYPE_STRING, formats[f], NULL);
      gst_structure_remove_fields (st, "colorimetry", "chroma-site", NULL);
      gst_caps_append_structure_full (res, st,
          gst_caps_features_copy (gst_caps_get_features (caps, i)));
    }
  }

  if (filter) {
    GstCaps *tmp =
        gst_caps_intersect_full (filter, res, GST_CAPS_INTERSECT_FIRST);
    gst_caps_unref (res);
    res = tmp;
  }

  GST_DEBUG_OBJECT (trans, "transformed %" GST_PTR_FORMAT " into %"
      GST_PTR_FORMAT, caps, res);

  return res;
}

static void
gst_thermal_eq_reset (GstThermalEq * self)
{
  self->have_lut = FALSE;
  self->last_pts = GST_CLOCK_TIME_NONE;
  self->hold_elapsed = 0;
  self->detail_gain = 0.0;
}

static gboolean
gst_thermal_eq_start (GstBaseTransform * trans)
{
  gst_thermal_eq_reset (GST_THERMAL_EQ (trans));

  return TRUE;
}

static gboolean
gst_thermal_eq_stop (GstBaseTransform * trans)
{
  gst_thermal_eq_free_frame_buffers (GST_THERMAL_EQ (trans));

  return TRUE;
}

static gboolean
gst_thermal_eq_set_info (GstVideoFilter * filter,
    G_GNUC_UNUSED GstCaps * incaps, GstVideoInfo * in_info,
    G_GNUC_UNUSED GstCaps * outcaps, G_GNUC_UNUSED GstVideoInfo * out_info)
{
  GstThermalEq *self = GST_THERMAL_EQ (filter);
  const GstVideoFormat format = GST_VIDEO_INFO_FORMAT (in_info);
  const gsize width = GST_VIDEO_INFO_WIDTH (in_info);
  const gsize height = GST_VIDEO_INFO_HEIGHT (in_info);

  /* 8-bit input has nothing left to equalize from: hand it on untouched */
  gst_base_transform_set_passthrough (GST_BASE_TRANSFORM (filter),
      format == GST_VIDEO_FORMAT_GRAY8);

  self->swap = (format == GST_VIDEO_FORMAT_GRAY16_BE)
      != (G_BYTE_ORDER == G_BIG_ENDIAN);

  gst_thermal_eq_free_frame_buffers (self);
  self->blur_tmp = g_new (guint16, width * height);
  self->base = g_new (guint16, width * height);
  self->colsum = g_new (guint32, width);
  self->prefix = g_new (guint64, width + 1);

  /* new stream format: start the temporal smoothing from scratch */
  gst_thermal_eq_reset (self);

  return TRUE;
}

/* Convert a smoothing time constant into a per-frame blend factor. */
static gdouble
time_to_alpha (guint time_ms, gdouble dt_ms)
{
  if (time_ms == 0)
    return 1.0;

  return 1.0 - exp (-dt_ms / (gdouble) time_ms);
}

/* Box blur of the input into self->base (host byte order). Windows shrink at
 * the frame edges instead of reading outside the image. */
static void
gst_thermal_eq_box_blur (GstThermalEq * self, const guint8 * in_data,
    gint in_stride, gint width, gint height, gint radius)
{
  const gboolean swap = self->swap;
  guint16 *tmp = self->blur_tmp;
  guint16 *base = self->base;
  guint32 *colsum = self->colsum;
  guint64 *prefix = self->prefix;
  gint x, y;

  /* horizontal pass */
  for (y = 0; y < height; y++) {
    const guint16 *src = (const guint16 *) (in_data + (gsize) y * in_stride);
    guint16 *dst = tmp + (gsize) y * width;

    prefix[0] = 0;
    for (x = 0; x < width; x++)
      prefix[x + 1] = prefix[x] + PIX (src[x]);

    for (x = 0; x < width; x++) {
      const gint x0 = MAX (x - radius, 0);
      const gint x1 = MIN (x + radius, width - 1) + 1;
      const guint32 count = x1 - x0;

      dst[x] = (guint16) ((prefix[x1] - prefix[x0] + count / 2) / count);
    }
  }

  /* vertical pass, keeping a running sum per column */
  memset (colsum, 0, width * sizeof (guint32));
  for (y = 0; y <= MIN (radius, height - 1); y++) {
    const guint16 *row = tmp + (gsize) y * width;

    for (x = 0; x < width; x++)
      colsum[x] += row[x];
  }

  for (y = 0; y < height; y++) {
    const gint y0 = MAX (y - radius, 0);
    const gint y1 = MIN (y + radius, height - 1);
    const guint32 count = y1 - y0 + 1;
    guint16 *dst = base + (gsize) y * width;

    for (x = 0; x < width; x++)
      dst[x] = (guint16) ((colsum[x] + count / 2) / count);

    if (y + radius + 1 < height) {
      const guint16 *row = tmp + (gsize) (y + radius + 1) * width;

      for (x = 0; x < width; x++)
        colsum[x] += row[x];
    }
    if (y - radius >= 0) {
      const guint16 *row = tmp + (gsize) (y - radius) * width;

      for (x = 0; x < width; x++)
        colsum[x] -= row[x];
    }
  }
}

static GstFlowReturn
gst_thermal_eq_transform_frame (GstVideoFilter * filter,
    GstVideoFrame * in_frame, GstVideoFrame * out_frame)
{
  GstThermalEq *self = GST_THERMAL_EQ (filter);
  const gboolean swap = self->swap;
  const gint width = GST_VIDEO_FRAME_WIDTH (in_frame);
  const gint height = GST_VIDEO_FRAME_HEIGHT (in_frame);
  const gint in_stride = GST_VIDEO_FRAME_PLANE_STRIDE (in_frame, 0);
  const gint out_stride = GST_VIDEO_FRAME_PLANE_STRIDE (out_frame, 0);
  const guint8 *in_data = GST_VIDEO_FRAME_PLANE_DATA (in_frame, 0);
  guint8 *out_data = GST_VIDEO_FRAME_PLANE_DATA (out_frame, 0);
  const GstClockTime pts = GST_BUFFER_PTS (in_frame->buffer);
  guint32 *hist = self->hist;
  gfloat *target = self->target;
  gfloat *lut = self->lut;
  guint8 *lut8 = self->lut8;
  gdouble plateau, scene_threshold, linear_blend, max_gain, detail;
  guint adapt_time, fast_adapt_time, shutter_span, shutter_hold_time;
  guint detail_radius, hist_step;
  gint roi_x, roi_y, roi_w, roi_h;
  gboolean invert, frozen;
  GstClockTime dt;
  gdouble dt_ms, nsamp, tail, acc, scene_change = 0.0;
  gint low = 0, high = NBINS - 1;
  gint x, y, v;

  GST_OBJECT_LOCK (self);
  plateau = self->plateau;
  adapt_time = self->adapt_time;
  fast_adapt_time = self->fast_adapt_time;
  scene_threshold = self->scene_threshold;
  linear_blend = self->linear_blend;
  max_gain = self->max_gain;
  shutter_span = self->shutter_span;
  shutter_hold_time = self->shutter_hold_time;
  detail = self->detail;
  detail_radius = self->detail_radius;
  roi_x = self->roi_x;
  roi_y = self->roi_y;
  roi_w = self->roi_width;
  roi_h = self->roi_height;
  invert = self->invert;
  hist_step = self->histogram_step;
  GST_OBJECT_UNLOCK (self);

  /* Time since the previous frame, so smoothing is independent of fps. */
  if (GST_CLOCK_TIME_IS_VALID (pts) && GST_CLOCK_TIME_IS_VALID (self->last_pts)
      && pts > self->last_pts && pts - self->last_pts < GST_SECOND) {
    dt = pts - self->last_pts;
  } else if (GST_VIDEO_INFO_FPS_N (&in_frame->info) > 0) {
    dt = gst_util_uint64_scale (GST_SECOND,
        GST_VIDEO_INFO_FPS_D (&in_frame->info),
        GST_VIDEO_INFO_FPS_N (&in_frame->info));
  } else {
    dt = GST_SECOND / 30;
  }
  self->last_pts = pts;
  dt_ms = (gdouble) dt / GST_MSECOND;

  /* Region the histogram is taken from (the mapping applies everywhere). */
  roi_x = CLAMP (roi_x, 0, width - 1);
  roi_y = CLAMP (roi_y, 0, height - 1);
  roi_w = (roi_w <= 0) ? width - roi_x : MIN (roi_w, width - roi_x);
  roi_h = (roi_h <= 0) ? height - roi_y : MIN (roi_h, height - roi_y);
  hist_step = MAX (hist_step, 1);

  /* 1. Histogram: how many sampled pixels have each of the 65536 values. */
  memset (hist, 0, NBINS * sizeof (guint32));
  for (y = roi_y; y < roi_y + roi_h; y += hist_step) {
    const guint16 *src = (const guint16 *) (in_data + (gsize) y * in_stride);

    for (x = roi_x; x < roi_x + roi_w; x += hist_step)
      hist[PIX (src[x])]++;
  }
  nsamp = (gdouble) ((roi_w + hist_step - 1) / hist_step)
      * ((roi_h + hist_step - 1) / hist_step);

  /* Input range, ignoring a small tail of outliers at each end. */
  tail = nsamp * RANGE_TAIL;
  acc = 0.0;
  for (v = 0; v < NBINS; v++) {
    acc += hist[v];
    if (acc > tail) {
      low = v;
      break;
    }
  }
  acc = 0.0;
  for (v = NBINS - 1; v >= 0; v--) {
    acc += hist[v];
    if (acc > tail) {
      high = v;
      break;
    }
  }
  if (high < low)
    high = low;

  /* A near-uniform scene (shutter closed) says nothing useful about the
   * real one: keep the current mapping, but only for so long in case the
   * camera really is looking at something flat. */
  if (self->have_lut && shutter_span > 0
      && (guint) (high - low) < shutter_span) {
    self->hold_elapsed += dt;
    frozen = self->hold_elapsed <= shutter_hold_time * GST_MSECOND;
  } else {
    self->hold_elapsed = 0;
    frozen = FALSE;
  }

  if (!frozen) {
    const guint32 cap = (guint32) MAX (1.0, plateau * nsamp);
    const gdouble mid = (low + high) / 2.0;
    gdouble total = 0.0, scale, range = 255.0, cum, slope = 0.0;
    gdouble lin_slope = 0.0;
    gdouble alpha;

    /* 2. Plateau: cap each bin so big flat regions can't hog the output.
     * Each bin then claims a share of the 255 output levels; with a gain
     * limit a bin may claim at most max_gain levels, so a low-contrast scene
     * no longer fills 0..255 and is centred around mid-gray instead. */
    for (v = 0; v < NBINS; v++)
      total += MIN (hist[v], cap);
    scale = 255.0 / total;
    if (max_gain > 0.0) {
      range = 0.0;
      for (v = 0; v < NBINS; v++)
        range += MIN (MIN (hist[v], cap) * scale, max_gain);
    }

    /* Plain linear stretch of the input range, for blending. */
    if (high > low)
      lin_slope = 255.0 / (high - low);
    if (max_gain > 0.0 && lin_slope > max_gain)
      lin_slope = max_gain;

    /* 3. The running total of the claims is the equalized mapping. Mix in
     * the linear one, and measure how far the mapping in use is from it. */
    cum = (255.0 - range) / 2.0;
    for (v = 0; v < NBINS; v++) {
      gdouble step = MIN (hist[v], cap) * scale;
      gdouble eq, lin;

      if (max_gain > 0.0 && step > max_gain)
        step = max_gain;
      eq = cum + step / 2.0;
      cum += step;

      lin = 127.5 + lin_slope * (v - mid);
      lin = CLAMP (lin, 0.0, 255.0);

      target[v] = (gfloat) ((1.0 - linear_blend) * eq + linear_blend * lin);

      if (hist[v])
        scene_change += hist[v] * fabs (target[v] - lut[v]);
    }
    scene_change /= nsamp;

    /* Typical slope of the mapping (output levels per input count), which
     * sets how strongly fine detail is boosted. */
    if (detail > 0.0) {
      for (v = 0; v < NBINS; v++) {
        if (hist[v]) {
          const gint v0 = MAX (v - SLOPE_WINDOW, 0);
          const gint v1 = MIN (v + SLOPE_WINDOW, NBINS - 1);

          slope += hist[v] * (target[v1] - target[v0]) / (v1 - v0);
        }
      }
      slope /= nsamp;
    }

    /* 4. Blend towards the new mapping: gently while the scene is stable,
     * quickly when it has changed a lot (e.g. panning onto sky). */
    if (!self->have_lut) {
      alpha = 1.0;
      scene_change = 0.0;
    } else {
      const gdouble slow = time_to_alpha (adapt_time, dt_ms);
      const gdouble fast = MAX (slow, time_to_alpha (fast_adapt_time, dt_ms));
      gdouble urgency = 0.0;

      if (scene_threshold > 0.0) {
        urgency = MIN (scene_change / scene_threshold, 1.0);
        urgency *= urgency;
      }
      alpha = slow + (fast - slow) * urgency;
    }

    for (v = 0; v < NBINS; v++) {
      guint8 level;

      lut[v] += (gfloat) alpha *(target[v] - lut[v]);
      level = (guint8) CLAMP (lut[v] + 0.5f, 0.0f, 255.0f);
      lut8[v] = invert ? 255 - level : level;
    }
    self->detail_gain += alpha * (slope - self->detail_gain);
    self->have_lut = TRUE;
  }

  GST_OBJECT_LOCK (self);
  self->input_low = low;
  self->input_high = high;
  self->scene_change = scene_change;
  self->frozen = frozen;
  GST_OBJECT_UNLOCK (self);

  GST_LOG_OBJECT (self, "input %d..%d, scene change %.1f%s", low, high,
      scene_change, frozen ? ", holding mapping" : "");

  /* 5. Apply the mapping, optionally adding boosted fine detail. */
  if (detail > 0.0 && self->detail_gain > 0.0) {
    const gdouble gain = detail * self->detail_gain;
    const gint dmax =
        (gint) MIN ((gdouble) DETAIL_TBL_HALF, ceil (4.0 * DETAIL_KNEE / gain));
    gint16 *tbl = self->detail_tbl + DETAIL_TBL_HALF;
    gint d;

    /* Detail is the pixel minus its blurred neighbourhood. Small differences
     * are amplified; large ones are edges and are left alone. */
    for (d = -dmax; d <= dmax; d++) {
      const gdouble boost = gain * d;
      const gdouble shaped =
          boost * exp (-0.5 * (boost / DETAIL_KNEE) * (boost / DETAIL_KNEE));

      tbl[d] = (gint16) lrint (invert ? -shaped : shaped);
    }

    gst_thermal_eq_box_blur (self, in_data, in_stride, width, height,
        (gint) detail_radius);

    for (y = 0; y < height; y++) {
      const guint16 *src = (const guint16 *) (in_data + (gsize) y * in_stride);
      const guint16 *base = self->base + (gsize) y * width;
      guint8 *dst = out_data + (gsize) y * out_stride;

      for (x = 0; x < width; x++) {
        const guint16 value = PIX (src[x]);
        const gint diff = (gint) value - (gint) base[x];
        gint level = lut8[value];

        if (diff >= -dmax && diff <= dmax)
          level += tbl[diff];
        dst[x] = (guint8) CLAMP (level, 0, 255);
      }
    }
  } else {
    for (y = 0; y < height; y++) {
      const guint16 *src = (const guint16 *) (in_data + (gsize) y * in_stride);
      guint8 *dst = out_data + (gsize) y * out_stride;

      for (x = 0; x < width; x++)
        dst[x] = lut8[PIX (src[x])];
    }
  }

  return GST_FLOW_OK;
}

static void
gst_thermal_eq_class_init (GstThermalEqClass * klass)
{
  GObjectClass *gobject_class = G_OBJECT_CLASS (klass);
  GstElementClass *element_class = GST_ELEMENT_CLASS (klass);
  GstBaseTransformClass *trans_class = GST_BASE_TRANSFORM_CLASS (klass);
  GstVideoFilterClass *filter_class = GST_VIDEO_FILTER_CLASS (klass);
  const GParamFlags rw = G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS;
  const GParamFlags ro = G_PARAM_READABLE | G_PARAM_STATIC_STRINGS;

  gobject_class->set_property = gst_thermal_eq_set_property;
  gobject_class->get_property = gst_thermal_eq_get_property;
  gobject_class->finalize = gst_thermal_eq_finalize;

  g_object_class_install_property (gobject_class, PROP_PLATEAU,
      g_param_spec_double ("plateau", "Plateau",
          "Most pixels a single input value may count for, as a fraction of "
          "the sampled pixels. Lower acts more like a linear stretch; higher "
          "gives more contrast but amplifies noise in flat regions (1 = "
          "plain histogram equalization)", 0.0, 1.0, DEFAULT_PLATEAU, rw));

  g_object_class_install_property (gobject_class, PROP_ADAPT_TIME,
      g_param_spec_uint ("adapt-time", "Adapt time",
          "Time constant in milliseconds for following a stable scene. "
          "Higher is steadier (0 = no smoothing)",
          0, 60000, DEFAULT_ADAPT_TIME, rw));

  g_object_class_install_property (gobject_class, PROP_FAST_ADAPT_TIME,
      g_param_spec_uint ("fast-adapt-time", "Fast adapt time",
          "Time constant in milliseconds used when the scene has changed by "
          "scene-threshold or more", 0, 60000, DEFAULT_FAST_ADAPT_TIME, rw));

  g_object_class_install_property (gobject_class, PROP_SCENE_THRESHOLD,
      g_param_spec_double ("scene-threshold", "Scene change threshold",
          "Scene change, in output gray levels, at which adaptation runs at "
          "full fast-adapt-time speed; smaller changes speed it up "
          "proportionally less (0 = always use adapt-time)",
          0.0, 255.0, DEFAULT_SCENE_THRESHOLD, rw));

  g_object_class_install_property (gobject_class, PROP_LINEAR_BLEND,
      g_param_spec_double ("linear-blend", "Linear blend",
          "Share of a plain linear stretch mixed into the equalized mapping, "
          "for a more natural look (0 = equalization only, 1 = linear only)",
          0.0, 1.0, DEFAULT_LINEAR_BLEND, rw));

  g_object_class_install_property (gobject_class, PROP_MAX_GAIN,
      g_param_spec_double ("max-gain", "Maximum gain",
          "Most output gray levels one input count may be stretched over, "
          "which stops near-uniform scenes turning into amplified noise "
          "(0 = no limit)", 0.0, 255.0, DEFAULT_MAX_GAIN, rw));

  g_object_class_install_property (gobject_class, PROP_SHUTTER_SPAN,
      g_param_spec_uint ("shutter-span", "Shutter span",
          "Hold the current mapping while the input range (input-high minus "
          "input-low) is below this many counts, as it is while the camera's "
          "calibration shutter is closed (0 = never hold)",
          0, 65535, DEFAULT_SHUTTER_SPAN, rw));

  g_object_class_install_property (gobject_class, PROP_SHUTTER_HOLD_TIME,
      g_param_spec_uint ("shutter-hold-time", "Shutter hold time",
          "Longest time in milliseconds the mapping is held for shutter-span "
          "before adapting to the flat scene anyway",
          0, 60000, DEFAULT_SHUTTER_HOLD_TIME, rw));

  g_object_class_install_property (gobject_class, PROP_DETAIL,
      g_param_spec_double ("detail", "Detail boost",
          "Extra contrast given to fine detail, relative to the contrast the "
          "mapping gives a typical pixel. Costs a blur per frame and also "
          "amplifies noise (0 = off)", 0.0, 8.0, DEFAULT_DETAIL, rw));

  g_object_class_install_property (gobject_class, PROP_DETAIL_RADIUS,
      g_param_spec_uint ("detail-radius", "Detail radius",
          "Radius in pixels of the neighbourhood that fine detail is "
          "measured against", 1, 64, DEFAULT_DETAIL_RADIUS, rw));

  g_object_class_install_property (gobject_class, PROP_ROI_X,
      g_param_spec_int ("roi-x", "ROI x",
          "Left edge of the region the histogram is taken from",
          0, G_MAXINT, DEFAULT_ROI_X, rw));

  g_object_class_install_property (gobject_class, PROP_ROI_Y,
      g_param_spec_int ("roi-y", "ROI y",
          "Top edge of the region the histogram is taken from",
          0, G_MAXINT, DEFAULT_ROI_Y, rw));

  g_object_class_install_property (gobject_class, PROP_ROI_WIDTH,
      g_param_spec_int ("roi-width", "ROI width",
          "Width of the region the histogram is taken from (0 = to the "
          "right edge)", 0, G_MAXINT, DEFAULT_ROI_WIDTH, rw));

  g_object_class_install_property (gobject_class, PROP_ROI_HEIGHT,
      g_param_spec_int ("roi-height", "ROI height",
          "Height of the region the histogram is taken from (0 = to the "
          "bottom edge)", 0, G_MAXINT, DEFAULT_ROI_HEIGHT, rw));

  g_object_class_install_property (gobject_class, PROP_INVERT,
      g_param_spec_boolean ("invert", "Invert",
          "Output black-hot (hot is dark) instead of white-hot",
          DEFAULT_INVERT, rw));

  g_object_class_install_property (gobject_class, PROP_HISTOGRAM_STEP,
      g_param_spec_uint ("histogram-step", "Histogram step",
          "Sample every Nth pixel in each direction when building the "
          "histogram (1 = every pixel)", 1, 16, DEFAULT_HISTOGRAM_STEP, rw));

  g_object_class_install_property (gobject_class, PROP_INPUT_LOW,
      g_param_spec_int ("input-low", "Input low",
          "1st percentile of the last frame's input values, in counts",
          0, NBINS - 1, 0, ro));

  g_object_class_install_property (gobject_class, PROP_INPUT_HIGH,
      g_param_spec_int ("input-high", "Input high",
          "99th percentile of the last frame's input values, in counts",
          0, NBINS - 1, 0, ro));

  g_object_class_install_property (gobject_class, PROP_SCENE_CHANGE,
      g_param_spec_double ("scene-change", "Scene change",
          "How far the mapping in use was from the one the last frame "
          "wanted, in output gray levels averaged over its pixels",
          0.0, 255.0, 0.0, ro));

  g_object_class_install_property (gobject_class, PROP_FROZEN,
      g_param_spec_boolean ("frozen", "Frozen",
          "Whether the mapping was being held for shutter-span on the last "
          "frame", FALSE, ro));

  gst_element_class_set_static_metadata (element_class,
      "Thermal plateau equalizer", "Filter/Effect/Video",
      "Converts 16-bit thermal video to 8-bit using temporally smoothed "
      "plateau histogram equalization", "Allen");

  gst_element_class_add_static_pad_template (element_class, &sink_template);
  gst_element_class_add_static_pad_template (element_class, &src_template);

  trans_class->transform_caps =
      GST_DEBUG_FUNCPTR (gst_thermal_eq_transform_caps);
  trans_class->start = GST_DEBUG_FUNCPTR (gst_thermal_eq_start);
  trans_class->stop = GST_DEBUG_FUNCPTR (gst_thermal_eq_stop);

  filter_class->set_info = GST_DEBUG_FUNCPTR (gst_thermal_eq_set_info);
  filter_class->transform_frame =
      GST_DEBUG_FUNCPTR (gst_thermal_eq_transform_frame);
}

static void
gst_thermal_eq_init (GstThermalEq * self)
{
  self->plateau = DEFAULT_PLATEAU;
  self->adapt_time = DEFAULT_ADAPT_TIME;
  self->fast_adapt_time = DEFAULT_FAST_ADAPT_TIME;
  self->scene_threshold = DEFAULT_SCENE_THRESHOLD;
  self->linear_blend = DEFAULT_LINEAR_BLEND;
  self->max_gain = DEFAULT_MAX_GAIN;
  self->shutter_span = DEFAULT_SHUTTER_SPAN;
  self->shutter_hold_time = DEFAULT_SHUTTER_HOLD_TIME;
  self->detail = DEFAULT_DETAIL;
  self->detail_radius = DEFAULT_DETAIL_RADIUS;
  self->roi_x = DEFAULT_ROI_X;
  self->roi_y = DEFAULT_ROI_Y;
  self->roi_width = DEFAULT_ROI_WIDTH;
  self->roi_height = DEFAULT_ROI_HEIGHT;
  self->invert = DEFAULT_INVERT;
  self->histogram_step = DEFAULT_HISTOGRAM_STEP;

  self->hist = g_new0 (guint32, NBINS);
  self->target = g_new0 (gfloat, NBINS);
  self->lut = g_new0 (gfloat, NBINS);
  self->lut8 = g_new0 (guint8, NBINS);
  self->detail_tbl = g_new0 (gint16, 2 * DETAIL_TBL_HALF + 1);

  gst_thermal_eq_reset (self);
}

static gboolean
plugin_init (GstPlugin * plugin)
{
  GST_DEBUG_CATEGORY_INIT (gst_thermal_eq_debug, "thermaleq", 0,
      "Thermal plateau equalizer");

  return gst_element_register (plugin, "thermaleq", GST_RANK_NONE,
      GST_TYPE_THERMAL_EQ);
}

GST_PLUGIN_DEFINE (
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    thermaleq,
    "Plateau histogram equalization for 16-bit thermal video",
    plugin_init,
    "0.1.0",
    "LGPL",
    "GstTemplate",
    "https://example.com"
)
