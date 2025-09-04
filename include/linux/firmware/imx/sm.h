/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2024 NXP
 */

#ifndef _SCMI_IMX_H
#define _SCMI_IMX_H

#include <linux/bitfield.h>
#include <linux/errno.h>
#include <linux/scmi_imx_protocol.h>
#include <linux/types.h>

#define SCMI_IMX95_CTRL_PDM_CLK_SEL	0	/* AON PDM clock sel */
#define SCMI_IMX95_CTRL_MQS1_SETTINGS	1	/* AON MQS settings */
#define SCMI_IMX95_CTRL_SAI1_MCLK	2	/* AON SAI1 MCLK */
#define SCMI_IMX95_CTRL_SAI3_MCLK	3	/* WAKE SAI3 MCLK */
#define SCMI_IMX95_CTRL_SAI4_MCLK	4	/* WAKE SAI4 MCLK */
#define SCMI_IMX95_CTRL_SAI5_MCLK	5	/* WAKE SAI5 MCLK */

#define SCMI_IMX94_CTRL_PDM_CLK_SEL	0U	/*!< AON PDM clock sel */
#define SCMI_IMX94_CTRL_MQS1_SETTINGS	1U	/*!< AON MQS settings */
#define SCMI_IMX94_CTRL_MQS2_SETTINGS	2U	/*!< WAKE MQS settings */
#define SCMI_IMX94_CTRL_SAI1_MCLK	3U	/*!< AON SAI1 MCLK */
#define SCMI_IMX94_CTRL_SAI2_MCLK	4U	/*!< WAKE SAI2 MCLK */
#define SCMI_IMX94_CTRL_SAI3_MCLK	5U	/*!< WAKE SAI3 MCLK */
#define SCMI_IMX94_CTRL_SAI4_MCLK	6U	/*!< WAKE SAI4 MCLK */

#if IS_ENABLED(CONFIG_IMX_SCMI_MISC_EXT)
int scmi_imx_misc_ctrl_get(u32 id, u32 *num, u32 *val);
int scmi_imx_misc_ctrl_set(u32 id, u32 val);
#else
static inline int scmi_imx_misc_ctrl_get(u32 id, u32 *num, u32 *val)
{
	return -EOPNOTSUPP;
}

static inline int scmi_imx_misc_ctrl_set(u32 id, u32 val)
{
	return -EOPNOTSUPP;
}
#endif

#if IS_ENABLED(CONFIG_IMX_SCMI_CPU_EXT)
extern int scmi_imx_cpu_start(u32 cpuid);
extern int scmi_imx_cpu_started(u32 cpuid, bool *started);
extern int scmi_imx_cpu_stop(u32 cpuid);
extern int scmi_imx_cpu_reset_vector_set(u32 cpuid, u64 vector, bool start,
					 bool boot, bool resume);
#else
static inline int scmi_imx_cpu_start(u32 cpuid)
{
	return -EOPNOTSUPP;
}

static inline int scmi_imx_cpu_started(u32 cpuid, bool *started)
{
	return -EOPNOTSUPP;
}

static inline int scmi_imx_cpu_stop(u32 cpuid)
{
	return -EOPNOTSUPP;
}

static inline int scmi_imx_cpu_reset_vector_set(u32 cpuid, u64 vector,
						bool start, bool boot, bool resume)
{
	return -EOPNOTSUPP;
}
#endif

#if IS_ENABLED(CONFIG_IMX_SCMI_LMM_EXT)
extern int scmi_imx_lmm_boot(u32 lmid);
extern int scmi_imx_lmm_info(u32 lmid, struct scmi_imx_lmm_info *info);
extern int scmi_imx_lmm_reset_vector_set(u32 lmid, u32 cpuid, u64 vector);
extern int scmi_imx_lmm_power_on(u32 lmid);
extern int scmi_imx_lmm_shutdown(u32 lmid, u32 flags);
#else
static inline int scmi_imx_lmm_boot(u32 lmid)
{
	return -EOPNOTSUPP;
}

static inline int scmi_imx_lmm_info(u32 lmid, struct scmi_imx_lmm_info *info)
{
	return -EOPNOTSUPP;
}

static inline int scmi_imx_lmm_reset_vector_set(u32 lmid, u32 cpuid, u64 vector)
{
	return -EOPNOTSUPP;
}

static inline int scmi_imx_lmm_power_on(u32 lmid)
{
	return -EOPNOTSUPP;
}

static inline int scmi_imx_lmm_shutdown(u32 lmid, u32 flags)
{
	return -EOPNOTSUPP;
}
#endif
#endif
