/*
 * thermaleq — plateau histogram equalization for 16-bit thermal video.
 *
 * Converts GRAY16_LE to GRAY8. Instead of a linear stretch, output gray
 * levels are handed out in proportion to how many pixels use each part of
 * the input range, with a per-value cap (the "plateau") so large uniform
 * regions such as sky cannot take over the output range. The mapping is
 * blended over time so it follows scene changes without flicker.
 *
 * Example:
 *   gst-launch-1.0 pleorasrc ! video/x-raw,format=GRAY16_LE ! thermaleq ! \
 *       videoconvert ! autovideosink
 */

#include <gst/gst.h>
#include <gst/video/video.h>
#include <gst/video/gstvideofilter.h>
#include <string.h>

GST_DEBUG_CATEGORY_STATIC (gst_thermal_eq_debug);
#define GST_CAT_DEFAULT gst_thermal_eq_debug

#define NBINS 65536

#define DEFAULT_PLATEAU     0.002
#define DEFAULT_ADAPT_RATE  0.1
#define DEFAULT_MAX_GAIN    0.0

enum
{
  PROP_0,
  PROP_PLATEAU,
  PROP_ADAPT_RATE,
  PROP_MAX_GAIN,
};

#define GST_TYPE_THERMAL_EQ (gst_thermal_eq_get_type ())
G_DECLARE_FINAL_TYPE (GstThermalEq, gst_thermal_eq, GST, THERMAL_EQ,
    GstVideoFilter)

struct _GstThermalEq
{
  GstVideoFilter parent;

  /* properties (protected by the object lock) */
  gdouble plateau;
  gdouble adapt_rate;
  gdouble max_gain;

  /* streaming state */
  guint32 *hist;                /* NBINS pixel counts for the current frame */
  gfloat *lut;                  /* NBINS smoothed output levels, 0..255 */
  guint8 *lut8;                 /* NBINS rounded copy of lut */
  gboolean have_lut;            /* FALSE until the first frame is processed */
};

G_DEFINE_TYPE (GstThermalEq, gst_thermal_eq, GST_TYPE_VIDEO_FILTER);

static GstStaticPadTemplate sink_template = GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK, GST_PAD_ALWAYS,
    GST_STATIC_CAPS (GST_VIDEO_CAPS_MAKE ("GRAY16_LE")));

static GstStaticPadTemplate src_template = GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC, GST_PAD_ALWAYS,
    GST_STATIC_CAPS (GST_VIDEO_CAPS_MAKE ("GRAY8")));

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
    case PROP_ADAPT_RATE:
      self->adapt_rate = g_value_get_double (value);
      break;
    case PROP_MAX_GAIN:
      self->max_gain = g_value_get_double (value);
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
    case PROP_ADAPT_RATE:
      g_value_set_double (value, self->adapt_rate);
      break;
    case PROP_MAX_GAIN:
      g_value_set_double (value, self->max_gain);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
  GST_OBJECT_UNLOCK (self);
}

static void
gst_thermal_eq_finalize (GObject * object)
{
  GstThermalEq *self = GST_THERMAL_EQ (object);

  g_free (self->hist);
  g_free (self->lut);
  g_free (self->lut8);

  G_OBJECT_CLASS (gst_thermal_eq_parent_class)->finalize (object);
}

/* The two pads differ only in pixel format: swap it and keep the rest. */
static GstCaps *
gst_thermal_eq_transform_caps (GstBaseTransform * trans,
    GstPadDirection direction, GstCaps * caps, GstCaps * filter)
{
  const gchar *format = (direction == GST_PAD_SINK) ? "GRAY8" : "GRAY16_LE";
  GstCaps *res = gst_caps_new_empty ();
  guint i, n = gst_caps_get_size (caps);

  for (i = 0; i < n; i++) {
    GstStructure *st = gst_structure_copy (gst_caps_get_structure (caps, i));

    gst_structure_set (st, "format", G_TYPE_STRING, format, NULL);
    gst_structure_remove_fields (st, "colorimetry", "chroma-site", NULL);
    gst_caps_append_structure_full (res, st,
        gst_caps_features_copy (gst_caps_get_features (caps, i)));
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

static gboolean
gst_thermal_eq_start (GstBaseTransform * trans)
{
  GstThermalEq *self = GST_THERMAL_EQ (trans);

  self->have_lut = FALSE;

  return TRUE;
}

static gboolean
gst_thermal_eq_set_info (GstVideoFilter * filter,
    G_GNUC_UNUSED GstCaps * incaps, G_GNUC_UNUSED GstVideoInfo * in_info,
    G_GNUC_UNUSED GstCaps * outcaps, G_GNUC_UNUSED GstVideoInfo * out_info)
{
  GstThermalEq *self = GST_THERMAL_EQ (filter);

  /* new stream format: start the temporal smoothing from scratch */
  self->have_lut = FALSE;

  return TRUE;
}

static GstFlowReturn
gst_thermal_eq_transform_frame (GstVideoFilter * filter,
    GstVideoFrame * in_frame, GstVideoFrame * out_frame)
{
  GstThermalEq *self = GST_THERMAL_EQ (filter);
  const gint width = GST_VIDEO_FRAME_WIDTH (in_frame);
  const gint height = GST_VIDEO_FRAME_HEIGHT (in_frame);
  const gint in_stride = GST_VIDEO_FRAME_PLANE_STRIDE (in_frame, 0);
  const gint out_stride = GST_VIDEO_FRAME_PLANE_STRIDE (out_frame, 0);
  const guint8 *in_data = GST_VIDEO_FRAME_PLANE_DATA (in_frame, 0);
  guint8 *out_data = GST_VIDEO_FRAME_PLANE_DATA (out_frame, 0);
  const gdouble npixels = (gdouble) width * height;
  guint32 *hist = self->hist;
  gfloat *lut = self->lut;
  guint8 *lut8 = self->lut8;
  gdouble plateau, adapt_rate, max_gain;
  gdouble total, scale, range, cum;
  guint32 cap;
  gfloat alpha;
  gint x, y, v;

  GST_OBJECT_LOCK (self);
  plateau = self->plateau;
  adapt_rate = self->adapt_rate;
  max_gain = self->max_gain;
  GST_OBJECT_UNLOCK (self);

  /* 1. Histogram: how many pixels have each of the 65536 possible values. */
  memset (hist, 0, NBINS * sizeof (guint32));
  for (y = 0; y < height; y++) {
    const guint16 *src = (const guint16 *) (in_data + (gsize) y * in_stride);

    for (x = 0; x < width; x++)
      hist[GUINT16_FROM_LE (src[x])]++;
  }

  /* 2. Plateau: cap each bin so big flat regions can't hog the output. */
  cap = (guint32) MAX (1.0, plateau * npixels);
  total = 0.0;
  for (v = 0; v < NBINS; v++) {
    if (hist[v] > cap)
      hist[v] = cap;
    total += hist[v];
  }

  /* Each bin now claims hist[v] * scale output levels. With a gain limit a
   * bin may claim at most max_gain levels, so a low-contrast scene no longer
   * fills 0..255; centre whatever range is left around mid-gray. */
  scale = 255.0 / total;
  range = 255.0;
  if (max_gain > 0.0) {
    range = 0.0;
    for (v = 0; v < NBINS; v++)
      range += MIN (hist[v] * scale, max_gain);
  }

  /* 3. Running total of the claims is the mapping; blend it into the
   * previous frame's mapping so brightness changes gradually. */
  alpha = self->have_lut ? (gfloat) adapt_rate : 1.0f;
  cum = (255.0 - range) / 2.0;
  for (v = 0; v < NBINS; v++) {
    gdouble step = hist[v] * scale;
    gfloat target;

    if (max_gain > 0.0 && step > max_gain)
      step = max_gain;

    target = (gfloat) (cum + step / 2.0);
    cum += step;

    lut[v] += alpha * (target - lut[v]);
    lut8[v] = (guint8) CLAMP (lut[v] + 0.5f, 0.0f, 255.0f);
  }
  self->have_lut = TRUE;

  /* 4. Apply the mapping. */
  for (y = 0; y < height; y++) {
    const guint16 *src = (const guint16 *) (in_data + (gsize) y * in_stride);
    guint8 *dst = out_data + (gsize) y * out_stride;

    for (x = 0; x < width; x++)
      dst[x] = lut8[GUINT16_FROM_LE (src[x])];
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

  gobject_class->set_property = gst_thermal_eq_set_property;
  gobject_class->get_property = gst_thermal_eq_get_property;
  gobject_class->finalize = gst_thermal_eq_finalize;

  g_object_class_install_property (gobject_class, PROP_PLATEAU,
      g_param_spec_double ("plateau", "Plateau",
          "Most pixels a single input value may count for, as a fraction of "
          "the frame. Lower acts more like a linear stretch; higher gives "
          "more contrast but amplifies noise in flat regions (1 = plain "
          "histogram equalization)",
          0.0, 1.0, DEFAULT_PLATEAU,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS |
          GST_PARAM_CONTROLLABLE));

  g_object_class_install_property (gobject_class, PROP_ADAPT_RATE,
      g_param_spec_double ("adapt-rate", "Adapt rate",
          "How much of each new frame's mapping is blended in. Lower is "
          "steadier but slower to follow scene changes (1 = no smoothing)",
          0.001, 1.0, DEFAULT_ADAPT_RATE,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS |
          GST_PARAM_CONTROLLABLE));

  g_object_class_install_property (gobject_class, PROP_MAX_GAIN,
      g_param_spec_double ("max-gain", "Maximum gain",
          "Most output gray levels one input count may be stretched over, "
          "which stops near-uniform scenes turning into amplified noise "
          "(0 = no limit)",
          0.0, 255.0, DEFAULT_MAX_GAIN,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS |
          GST_PARAM_CONTROLLABLE));

  gst_element_class_set_static_metadata (element_class,
      "Thermal plateau equalizer", "Filter/Effect/Video",
      "Converts 16-bit thermal video to 8-bit using temporally smoothed "
      "plateau histogram equalization", "Allen");

  gst_element_class_add_static_pad_template (element_class, &sink_template);
  gst_element_class_add_static_pad_template (element_class, &src_template);

  trans_class->transform_caps =
      GST_DEBUG_FUNCPTR (gst_thermal_eq_transform_caps);
  trans_class->start = GST_DEBUG_FUNCPTR (gst_thermal_eq_start);

  filter_class->set_info = GST_DEBUG_FUNCPTR (gst_thermal_eq_set_info);
  filter_class->transform_frame =
      GST_DEBUG_FUNCPTR (gst_thermal_eq_transform_frame);
}

static void
gst_thermal_eq_init (GstThermalEq * self)
{
  self->plateau = DEFAULT_PLATEAU;
  self->adapt_rate = DEFAULT_ADAPT_RATE;
  self->max_gain = DEFAULT_MAX_GAIN;

  self->hist = g_new0 (guint32, NBINS);
  self->lut = g_new0 (gfloat, NBINS);
  self->lut8 = g_new0 (guint8, NBINS);
  self->have_lut = FALSE;
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
