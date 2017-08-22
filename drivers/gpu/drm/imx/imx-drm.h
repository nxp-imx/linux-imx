#ifndef _IMX_DRM_H_
#define _IMX_DRM_H_

#define MAX_CRTC	4
#define MAX_DPU		2

struct device_node;
struct drm_crtc;
struct drm_connector;
struct drm_device;
struct drm_display_mode;
struct drm_encoder;
struct drm_fbdev_cma;
struct drm_framebuffer;
struct drm_plane;
struct imx_drm_crtc;
struct platform_device;

struct imx_drm_device {
	struct drm_device			*drm;
	struct imx_drm_crtc			*crtc[MAX_CRTC];
	unsigned int				pipes;
	struct drm_fbdev_cma			*fbhelper;
	struct drm_atomic_state			*state;
};

struct imx_crtc_state {
	struct drm_crtc_state			base;
	u32					bus_format;
	u32					bus_flags;
	int					di_hsync_pin;
	int					di_vsync_pin;
};

/* useful structure to embedded a drm_ioctl_desc */
struct imx_drm_ioctl {
       struct drm_ioctl_desc *ioctl;
       struct list_head next;
};

static inline struct imx_crtc_state *to_imx_crtc_state(struct drm_crtc_state *s)
{
	return container_of(s, struct imx_crtc_state, base);
}

struct imx_drm_crtc_helper_funcs {
	int (*enable_vblank)(struct drm_crtc *crtc);
	void (*disable_vblank)(struct drm_crtc *crtc);
	const struct drm_crtc_helper_funcs *crtc_helper_funcs;
	const struct drm_crtc_funcs *crtc_funcs;
};

int imx_drm_add_crtc(struct drm_device *drm, struct drm_crtc *crtc,
		struct imx_drm_crtc **new_crtc, struct drm_plane *primary_plane,
		const struct imx_drm_crtc_helper_funcs *imx_helper_funcs,
		struct device_node *port);
int imx_drm_remove_crtc(struct imx_drm_crtc *);
int imx_drm_init_drm(struct platform_device *pdev,
		int preferred_bpp);
int imx_drm_exit_drm(void);

void imx_drm_mode_config_init(struct drm_device *drm);

struct drm_gem_cma_object *imx_drm_fb_get_obj(struct drm_framebuffer *fb);

int imx_drm_encoder_parse_of(struct drm_device *drm,
	struct drm_encoder *encoder, struct device_node *np);

void imx_drm_connector_destroy(struct drm_connector *connector);
void imx_drm_encoder_destroy(struct drm_encoder *encoder);

int imx_drm_register_ioctl(struct drm_ioctl_desc *ioctl);
int imx_drm_unregister_ioctl(struct drm_ioctl_desc *ioctl);

#endif /* _IMX_DRM_H_ */
