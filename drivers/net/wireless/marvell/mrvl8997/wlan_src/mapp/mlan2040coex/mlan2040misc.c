/** @file  mlan2040misc.c
  *
  * @brief This file contains helper functions for coex application
  *
  *
  * Copyright 2014-2020 NXP
  *
  * This software file (the File) is distributed by NXP
  * under the terms of the GNU General Public License Version 2, June 1991
  * (the License).  You may use, redistribute and/or modify the File in
  * accordance with the terms and conditions of the License, a copy of which
  * is available by writing to the Free Software Foundation, Inc.,
  * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
  * worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
  *
  * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
  * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
  * this warranty disclaimer.
  *
  */
/************************************************************************
Change log:
     06/24/2009: initial version
************************************************************************/
#include    <stdio.h>
#include    <ctype.h>
#include    <string.h>
#include    <stdlib.h>
#include    "mlan2040coex.h"
#include    "mlan2040misc.h"

/********************************************************
		Local Variables
********************************************************/
/** Regulatory class and Channel mapping for various regions */
static class_chan_t us_class_chan_t[] = {
	{32, {1, 2, 3, 4, 5, 6, 7}, 7},
	{33, {5, 6, 7, 8, 9, 10, 11}, 7}
};

static class_chan_t europe_class_chan_t[] = {
	{11, {1, 2, 3, 4, 5, 6, 7, 8, 9}, 9},
	{12, {5, 6, 7, 8, 9, 10, 11, 12, 13}, 9}
};

static class_chan_t japan_class_chan_t[] = {
	{56, {1, 2, 3, 4, 5, 6, 7, 8, 9}, 9},
	{57, {5, 6, 7, 8, 9, 10, 11, 12, 13}, 9},
	{58, {14}, 1}
};

/** Region-code(Regulatory domain) and Class-channel table mapping */
static region_class_chan_t region_class_chan_table[] = {
	{0x10, us_class_chan_t, sizeof(us_class_chan_t) / sizeof(class_chan_t)}	/* US */
	,
	{0x20, us_class_chan_t, sizeof(us_class_chan_t) / sizeof(class_chan_t)}	/* CANADA */
	,
	{0x30, europe_class_chan_t, sizeof(europe_class_chan_t) / sizeof(class_chan_t)}	/* EUROPE */
	,
	{0x32, europe_class_chan_t, sizeof(europe_class_chan_t) / sizeof(class_chan_t)}	/* FRANCE */
	,
	{0x40, japan_class_chan_t, sizeof(japan_class_chan_t) / sizeof(class_chan_t)}	/* JAPAN */
	,
	{0x41, japan_class_chan_t, sizeof(japan_class_chan_t) / sizeof(class_chan_t)}	/* JAPAN */
	,
	{0x50, europe_class_chan_t, sizeof(europe_class_chan_t) / sizeof(class_chan_t)}	/* CHINA */
};

/********************************************************
		Global Variables
********************************************************/

/********************************************************
		Local Functions
********************************************************/
/**
 *  @brief  This function prepares the channel list for a particular
 *  		regulatory class from channel number for legacy AP
 *  @param cur_class_chan_table  A pointer to the class_chan_t
 *  @param num_entry             Number of entry in cur_class_chan_table table
 *  @param chan_list             A pointer to the output channel list
 *  @param chan_num	             total number of channel in output channel list
 *  @param reg_domain            regulatory domain
 *  @param reg_class             regulatory class
 *  @param is_intol_ap_present   It sets TRUE when 40MHz intolerant AP is found
 *  						     otherwise FALSE
 *  @return      	  None
 */
static void
get_channels_for_specified_reg_class(class_chan_t *cur_class_chan_table,
				     int num_entry, t_u8 *chan_list,
				     t_u8 *chan_num, t_u8 reg_domain,
				     t_u8 reg_class, t_u8 *is_intol_ap_present)
{
	int i, j, k, idx = 0;

	*is_intol_ap_present = FALSE;

	/* For each regulatory class */
	for (i = 0; i < num_entry; i++) {
		if (cur_class_chan_table[i].reg_class == reg_class) {
			/* For each channel of the regulatory class */
			for (j = 0; j < cur_class_chan_table[i].total_chan; j++) {
				for (k = 0; k < num_leg_ap_chan; k++) {

					if (cur_class_chan_table[i].
					    channels[j] ==
					    leg_ap_chan_list[k].chan_num) {
						*(chan_list + idx) =
							leg_ap_chan_list[k].
							chan_num;
						idx++;
						if (leg_ap_chan_list[k].
						    is_intol_set)
							*is_intol_ap_present =
								TRUE;
					}
				}
			}
			break;
		}
	}
	/* update the total number of channel */
	*chan_num = idx--;
	return;
}

/********************************************************
		Global Functions
********************************************************/
/**
 *  @brief Prepare 2040 coex command buffer
 *  @param buf		   A pointer to the command buffer
 *  @param chan_list   Channel list
 *  @param num_of_chan Number of channel present in channel list
 *  @param reg_class   Regulatory class
 *  @param is_intol_ap_present   Flag: is any 40 MHz intolerant AP
 *  				   is present in these chaanel set
 *  @return      	MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
void
prepare_coex_cmd_buff(t_u8 *buf, t_u8 *chan_list, t_u8 num_of_chan,
		      t_u8 reg_class, t_u8 is_intol_ap_present)
{
	HostCmd_DS_GEN *hostcmd;
	MrvlIETypes_2040COEX_t *coex_ie = NULL;
	MrvlIETypes_2040BssIntolerantChannelReport_t *bss_intol_ie = NULL;
	t_u8 *pos = NULL;
	int intol;

	hostcmd = (HostCmd_DS_GEN *)(buf + sizeof(t_u32));
	hostcmd->command = cpu_to_le16(HostCmd_CMD_11N_2040COEX);
	hostcmd->size = S_DS_GEN;
	pos = buf + sizeof(t_u32) + S_DS_GEN;
	{
		coex_ie = (MrvlIETypes_2040COEX_t *)pos;
		coex_ie->header.element_id = TLV_ID_2040COEX;
		coex_ie->header.len = sizeof(coex_ie->coex_elem);
		/* Check STA is 40 MHz intolerant or not */
		is_intolerant_sta(&intol);
		if (intol)
			coex_ie->coex_elem |= MBIT(1);

		if (is_intol_ap_present)
			coex_ie->coex_elem |= MBIT(2);
		pos += sizeof(MrvlIETypes_2040COEX_t);
		hostcmd->size += sizeof(MrvlIETypes_2040COEX_t);
	}
	{
		bss_intol_ie =
			(MrvlIETypes_2040BssIntolerantChannelReport_t *)pos;
		bss_intol_ie->header.element_id =
			TLV_ID_2040BSS_INTOL_CHAN_REPORT;
		hostcmd->size +=
			sizeof(MrvlIETypes_2040BssIntolerantChannelReport_t) -
			sizeof(bss_intol_ie->chan_num);
		bss_intol_ie->reg_class = reg_class;
		memcpy(bss_intol_ie->chan_num, chan_list, num_of_chan);
		bss_intol_ie->header.len =
			sizeof(bss_intol_ie->reg_class) + num_of_chan;
		hostcmd->size += num_of_chan;
	}
	hostcmd->size = cpu_to_le16(hostcmd->size);
	return;
}

/**
 *  @brief Invoke multiple 2040Coex commands for multiple regulatory classes
 *
 *  @return      	MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
int
invoke_coex_command(void)
{
	int cur_reg_domain;
	t_u8 chan_list[MAX_CHAN], is_intol_ap_present;
	t_u8 num_of_chan;
	int i, num_entry, ret = MLAN_STATUS_SUCCESS;
	class_chan_t *cur_class_chan_table = NULL;

    /** get region code */
	ret = get_region_code(&cur_reg_domain);
	if (ret != MLAN_STATUS_SUCCESS)
		return ret;
    /** Find region_class_chan_table for this region */
	for (i = 0;
	     (unsigned int)i <
	     (sizeof(region_class_chan_table) / sizeof(region_class_chan_t));
	     i++) {
		if (region_class_chan_table[i].reg_domain == cur_reg_domain) {
			cur_class_chan_table =
				region_class_chan_table[i].class_chan_list;
			num_entry =
				region_class_chan_table[i].num_class_chan_entry;
			break;
		}
	}
	if (cur_class_chan_table == NULL) {
		printf("No region_class_chan table found for this region\n");
		return MLAN_STATUS_FAILURE;
	}

	for (i = 0; i < num_entry; i++) {
	/** Get channels for the specified regulatory class */
		get_channels_for_specified_reg_class(cur_class_chan_table,
						     num_entry, chan_list,
						     &num_of_chan,
						     cur_reg_domain,
						     cur_class_chan_table[i].
						     reg_class,
						     &is_intol_ap_present);

	/** If any channel found for this regulatory class, then invoke the 2040coex command */
		if (num_of_chan > 0) {
			ret = process_host_cmd(CMD_2040COEX, chan_list,
					       num_of_chan,
					       cur_class_chan_table[i].
					       reg_class, is_intol_ap_present);
			if (ret)
				break;
		}
	}
	return ret;
}

/**
 *  @brief Process host_cmd response
 *
 *  @param cmd_name	The command string
 *  @param buf		A pointer to the response buffer
 *
 *  @return      	MLAN_STATUS_SUCCESS--success, otherwise--fail
 */
int
process_host_cmd_resp(char *cmd_name, t_u8 *buf)
{
	t_u32 hostcmd_size = 0;
	HostCmd_DS_GEN *hostcmd = NULL;
	int ret = MLAN_STATUS_SUCCESS;

	buf += strlen(CMD_NXP) + strlen(cmd_name);
	memcpy((t_u8 *)&hostcmd_size, buf, sizeof(t_u32));
	buf += sizeof(t_u32);

	hostcmd = (HostCmd_DS_GEN *)buf;
	hostcmd->command = le16_to_cpu(hostcmd->command);
	hostcmd->size = le16_to_cpu(hostcmd->size);

	hostcmd->command &= ~HostCmd_RET_BIT;
	if (!le16_to_cpu(hostcmd->result)) {
		switch (hostcmd->command) {
		}
	} else {
		printf("HOSTCMD failed: ReturnCode=%#04x, Result=%#04x\n",
		       le16_to_cpu(hostcmd->command),
		       le16_to_cpu(hostcmd->result));
		ret = MLAN_STATUS_FAILURE;
	}
	return ret;
}
