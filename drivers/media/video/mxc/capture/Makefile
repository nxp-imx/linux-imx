ifeq ($(CONFIG_VIDEO_MXC_IPU_CAMERA),y)
	obj-$(CONFIG_VIDEO_MXC_CAMERA) += mxc_v4l2_capture.o
	obj-$(CONFIG_MXC_IPU_PRP_VF_SDC) += ipu_prp_vf_sdc.o ipu_prp_vf_sdc_bg.o
	obj-$(CONFIG_MXC_IPU_PRP_ENC) += ipu_prp_enc.o ipu_still.o
	obj-$(CONFIG_MXC_IPU_CSI_ENC) += ipu_csi_enc.o ipu_still.o
endif

obj-$(CONFIG_VIDEO_MXC_CSI_CAMERA) += fsl_csi.o csi_v4l2_capture.o

mx27_capture-objs := mx27_prphw.o mx27_prpsw.o emma_v4l2_capture.o
obj-$(CONFIG_VIDEO_MXC_EMMA_CAMERA) += mx27_csi.o mx27_capture.o

mc521da_camera-objs := mc521da.o sensor_clock.o
obj-$(CONFIG_MXC_CAMERA_MC521DA) += mc521da_camera.o

emma_mt9v111_camera-objs := emma_mt9v111.o sensor_clock.o
obj-$(CONFIG_MXC_EMMA_CAMERA_MICRON111) += emma_mt9v111_camera.o

mt9v111_camera-objs := mt9v111.o sensor_clock.o
obj-$(CONFIG_MXC_CAMERA_MICRON111) += mt9v111_camera.o

hv7161_camera-objs := hv7161.o sensor_clock.o
obj-$(CONFIG_MXC_CAMERA_HV7161) += hv7161_camera.o

s5k3aaex_camera-objs := s5k3aaex.o sensor_clock.o
obj-$(CONFIG_MXC_CAMERA_S5K3AAEX) += s5k3aaex_camera.o

emma_ov2640_camera-objs := emma_ov2640.o sensor_clock.o
obj-$(CONFIG_MXC_CAMERA_OV2640_EMMA) += emma_ov2640_camera.o

ov2640_camera-objs := ov2640.o sensor_clock.o
obj-$(CONFIG_MXC_CAMERA_OV2640) += ov2640_camera.o

ov3640_camera-objs := ov3640.o sensor_clock.o
obj-$(CONFIG_MXC_CAMERA_OV3640) += ov3640_camera.o

ov5640_camera-objs := ov5640.o sensor_clock.o
obj-$(CONFIG_MXC_CAMERA_OV5640) += ov5640_camera.o

ov5642_camera-objs := ov5642.o sensor_clock.o
obj-$(CONFIG_MXC_CAMERA_OV5642) += ov5642_camera.o

adv7180_tvin-objs := adv7180.o
obj-$(CONFIG_MXC_TVIN_ADV7180) += adv7180_tvin.o
