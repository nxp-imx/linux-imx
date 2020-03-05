/** @file  timestamp.h
 *
 * @brief This file contains definitions used for application
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

#ifndef _TIMESTAMP_H_
#define _TIMESTAMP_H_

#include  "mlanutl.h"
#include  <linux/if_packet.h>
#include  <netinet/ether.h>
#include  <linux/net_tstamp.h>

#define BUF_SIZ         1024
#define ARP_FORMAT      "%s %*s %*s %s %*s %s"
#define ARP_FILE_BUFFER_LEN  (1024)

struct interface_data {
	char ip[20];
	int mac[20];
	char interface[20];
};

/**
 * 802.1 Local Experimental 1.
 */
#ifndef ETH_P_802_EX1
#define ETH_P_802_EX1   0x88B5
#endif

void receive_timestamp(int argc, char *argv[]);
int send_timestamp(int argc, char *argv[]);
void get_mac(char *ifc, char *ip);
void recvpacket(int sock, int recvmsg_flags, int siocgstamp, int siocgstampns);
void printpacket(struct msghdr *msg, int res, int sock,
		 int recvmsg_flags, int siocgstamp, int siocgstampns);

#endif //_TIMESTAMP_H_
