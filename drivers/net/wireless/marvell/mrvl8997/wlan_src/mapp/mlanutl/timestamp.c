/** @file  timestamp.c
 *
 * @brief Functions for timestamping feature
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

#include "timestamp.h"
#include "time.h"

/* GLobal Declarations */
struct timespec send_time;
struct interface_data inter;

/**
 *@brief      Receive Timestamps
 *
 *@param argc Number of arguments
 *@param argv Pointer to the arguments array
 *
 * @return     MLAN_STATUS_SUCCESS/MLAN_STATUS_FAILURE
 **/
void
receive_timestamp(int argc, char *argv[])
{
	int sockfd;
	int sockopt;
	char ifName[IFNAMSIZ];
	struct ifreq if_ip;	/* get ip addr */
	int so_timestamping_flags = 0;
	int siocgstamp = 0;
	int siocgstampns = 0;
	struct timeval now;
	int res;
	struct ifreq if_idx;
	struct ifreq if_mac;
	fd_set readfs, errorfs;

	/* Get interface name */
	if (argc > 2)
		strcpy(ifName, argv[1]);
	else {
		fprintf(stderr,
			"invalid no. of arguments to receive_timestamp \n");
		exit(1);
	}

	/* Header structures */
	so_timestamping_flags |= SOF_TIMESTAMPING_SOFTWARE;
	so_timestamping_flags |= SOF_TIMESTAMPING_RAW_HARDWARE;
	memset(&if_ip, 0, sizeof(struct ifreq));

	/* Open PF_PACKET socket, listening for EtherType ETHER_TYPE */
	if ((sockfd = socket(PF_PACKET, SOCK_RAW, htons(ETH_P_802_EX1))) == -1) {
		perror("listener: socket");
	}

	/* Get the index of the interface to receive on */
	memset(&if_idx, 0, sizeof(struct ifreq));
	strncpy(if_idx.ifr_name, ifName, IFNAMSIZ - 1);
	if (ioctl(sockfd, SIOCGIFINDEX, &if_idx) < 0)
		perror("SIOCGIFINDEX");

	/* Get the MAC address of the interface to receive on */
	memset(&if_mac, 0, sizeof(struct ifreq));
	strncpy(if_mac.ifr_name, ifName, IFNAMSIZ - 1);
	if (ioctl(sockfd, SIOCGIFHWADDR, &if_mac) < 0)
		perror("SIOCGIFHWADDR");

	/* Allow the socket to be reused - incase connection is closed prematurely */
	if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &sockopt,
		       sizeof sockopt) == -1) {
		perror("setsockopt");
		close(sockfd);
		exit(1);
	}

	/* Bind to device */
	if (setsockopt(sockfd, SOL_SOCKET, SO_BINDTODEVICE,
		       ifName, IFNAMSIZ - 1) == -1) {
		perror("SO_BINDTODEVICE");
		close(sockfd);
		exit(1);
	}

	if (so_timestamping_flags &&
	    setsockopt(sockfd, SOL_SOCKET, SO_TIMESTAMPING,
		       &so_timestamping_flags,
		       sizeof(so_timestamping_flags)) < 0)
		perror("setsockopt SO_TIMESTAMPING");

	while (1) {
		FD_ZERO(&readfs);
		FD_ZERO(&errorfs);
		FD_SET(sockfd, &readfs);
		FD_SET(sockfd, &errorfs);
		gettimeofday(&now, NULL);
		res = select(sockfd + 1, &readfs, 0, &errorfs, NULL);
		if (res > 0) {
			recvpacket(sockfd, 0, siocgstamp, siocgstampns);
		}
	}
}

/**
 *@brief      Send Timestamps
 *
 *@param argc Number of arguments
 *@param argv Pointer to the arguments array
 *
 *@return     MLAN_STATUS_SUCCESS/MLAN_STATUS_FAILURE
 **/
int
send_timestamp(int argc, char *argv[])
{
	int sockfd;
	struct ifreq if_idx;
	struct ifreq if_mac;
	int tx_len = 0, i;
	char sendbuf[BUF_SIZ];
	char buff[BUF_SIZ];
	struct ether_header *eh = (struct ether_header *)sendbuf;
	struct sockaddr_ll socket_address;
	char ifName[IFNAMSIZ], ip[50];
	struct timeval delta;
	fd_set readfs, errorfs;
	int res, siocgstamp = 1, siocgstampns = 1;
	int so_timestamping_flags = SOF_TIMESTAMPING_TX_HARDWARE;
	struct ifreq hwtstamp;
	struct hwtstamp_config hwconfig;

	so_timestamping_flags |= SOF_TIMESTAMPING_RAW_HARDWARE;
	so_timestamping_flags |= SOF_TIMESTAMPING_TX_HARDWARE;
	so_timestamping_flags |= SOF_TIMESTAMPING_SYS_HARDWARE;

	/* Get interface name */
	if (argc > 4) {
		strcpy(ifName, argv[1]);
		strcpy(ip, argv[4]);
	} else {
		fprintf(stderr, "invalid no. of args for send_timestamp\n");
		exit(1);
	}

	/* Open RAW socket to send on */
	if ((sockfd = socket(PF_PACKET, SOCK_RAW, ETH_P_802_EX1)) == -1) {
		perror("socket");
	}

	memset(&hwtstamp, 0, sizeof(hwtstamp));
	strncpy(hwtstamp.ifr_name, ifName, sizeof(hwtstamp.ifr_name));
	hwtstamp.ifr_data = (void *)&hwconfig;
	memset(&hwconfig, 0, sizeof(hwconfig));

	hwconfig.tx_type =
		(so_timestamping_flags & SOF_TIMESTAMPING_TX_HARDWARE) ?
		HWTSTAMP_TX_ON : HWTSTAMP_TX_OFF;

	hwconfig.rx_filter =
		(so_timestamping_flags & SOF_TIMESTAMPING_RX_HARDWARE) ?
		HWTSTAMP_FILTER_PTP_V1_L4_SYNC : HWTSTAMP_FILTER_NONE;

	/* Get the index of the interface to send on */
	memset(&if_idx, 0, sizeof(struct ifreq));
	strncpy(if_idx.ifr_name, ifName, IFNAMSIZ - 1);
	if (ioctl(sockfd, SIOCGIFINDEX, &if_idx) < 0)
		perror("SIOCGIFINDEX");

	/* Get the MAC address of the interface to send on */
	memset(&if_mac, 0, sizeof(struct ifreq));
	strncpy(if_mac.ifr_name, ifName, IFNAMSIZ - 1);
	if (ioctl(sockfd, SIOCGIFHWADDR, &if_mac) < 0)
		perror("SIOCGIFHWADDR");

	if (so_timestamping_flags &&
	    setsockopt(sockfd, SOL_SOCKET, SO_TIMESTAMPING,
		       &so_timestamping_flags,
		       sizeof(so_timestamping_flags)) < 0)
		perror("setsockopt SO_TIMESTAMPING");

	if (setsockopt(sockfd, SOL_SOCKET, SO_BINDTODEVICE,
		       ifName, IFNAMSIZ - 1) == -1) {
		perror("bind");
		exit(1);
	}

	/* Construct the Ethernet header */
	memset(sendbuf, 0, BUF_SIZ);

	/* Ethernet header */
	memcpy(eh->ether_shost, (u_int8_t *) & if_mac.ifr_hwaddr.sa_data,
	       MLAN_MAC_ADDR_LENGTH);

	eh->ether_type = htons(ETH_P_802_EX1);

	tx_len += sizeof(struct ether_header);

	get_mac(ifName, ip);
	for (i = 0; i < MLAN_MAC_ADDR_LENGTH; i++) {
		eh->ether_dhost[i] = (uint8_t) inter.mac[i];
	}
	/* Index of the network device */
	socket_address.sll_ifindex = if_idx.ifr_ifindex;

	/* Address length */
	socket_address.sll_halen = ETH_ALEN;
	memcpy(&socket_address.sll_addr, (uint8_t *) & inter.mac,
	       MLAN_MAC_ADDR_LENGTH);

	clock_gettime(CLOCK_REALTIME, &send_time);
	sprintf(buff, "%lld.%lld", (long long)send_time.tv_sec,
		(long long)send_time.tv_nsec);
	strcpy((sendbuf + tx_len), buff);

	/* Send packet */
	res = sendto(sockfd, sendbuf, tx_len + strlen(buff), 0,
		     (struct sockaddr *)&socket_address,
		     sizeof(struct sockaddr_ll));
	if (res < 0)
		perror("Send ");

	fprintf(stdout, "Application time : %lld.%09lld (sent)\n",
		(long long)send_time.tv_sec, (long long)send_time.tv_nsec);

	delta.tv_sec = 5;
	delta.tv_usec = 0;

	FD_ZERO(&readfs);
	FD_ZERO(&errorfs);
	FD_SET(sockfd, &readfs);
	FD_SET(sockfd, &errorfs);

	res = select(sockfd + 1, &readfs, 0, &errorfs, &delta);
	if (res > 0) {
		recvpacket(sockfd, MSG_ERRQUEUE, siocgstamp, siocgstampns);
	}
	return MLAN_STATUS_SUCCESS;
}

/**
 *@brief    get destination mac address
 *
 *@param    ifc interface from which packet has to be sent
 *@param    ip IP Address of destination
 *
 *@return   N/A
 **/
void
get_mac(char *ifc, char *ip)
{
	char ipAddr[20];
	char hwAddr[20];
	char device[10], temp[3], in[50];
	int i = 0, j = 0, k = 0, res = 0, retry = 0;
	FILE *arpCache = fopen("/proc/net/arp", "r");
	if (!arpCache) {
		fprintf(stderr,
			"Arp Cache: Failed to open file \"/proc/net/arp\"");
	}

	/* Ignore the first line, which contains the header */
	char header[ARP_FILE_BUFFER_LEN];

retry_again:

	if (!fgets(header, sizeof(header), arpCache))
		fprintf(stderr, "error getting mac from proc files");
	while (3 == fscanf(arpCache, ARP_FORMAT, ipAddr, hwAddr, device)) {
		if ((!strcmp(ipAddr, ip)) && (!strcmp(ifc, device))) {
			printf("Sending Packet to Peer : %s\n", hwAddr);
			strcpy(inter.ip, ipAddr);
			strcpy(inter.interface, device);
			while (hwAddr[i] != '\0') {
				if (hwAddr[i] == ':') {
					inter.mac[j++] = strtol(temp, NULL, 16);
					i++;
					k = 0;
				} else
					temp[k++] = hwAddr[i++];
			}
			inter.mac[j] = strtol(temp, NULL, 16);
			res = 1;
		}
	}
	if (res != 1 && retry == 0) {
		sprintf(in, "ping -c 2 %s > /dev/null", ip);
		system(in);
		retry = 1;
		rewind(arpCache);
		goto retry_again;
	} else if (res != 1 && retry == 1) {
		printf("cannot find mac address for the specified ip\n");
		fclose(arpCache);
		exit(1);
	}
	fclose(arpCache);
}

/*
 *@brief    Receive Sync Packets
 *
 *@param sock socket from which packet must be recieved
 *@param recvmsg_flags flags for recvmsg
 *@param siocgstamp timestamp flag
 *@param siocgstampns timestamp flag for nano secs
 *
 *@return     N/A
 **/
void
recvpacket(int sock, int recvmsg_flags, int siocgstamp, int siocgstampns)
{
	unsigned char data[256];
	struct msghdr msg;
	struct iovec entry;
	struct sockaddr_in from_addr;
	struct {
		struct cmsghdr cm;
		char control[512];
	} control;
	int res, i;

	memset(&msg, 0, sizeof(msg));
	msg.msg_iov = &entry;
	msg.msg_iovlen = 1;
	entry.iov_base = data;
	entry.iov_len = sizeof(data);
	msg.msg_name = (caddr_t) & from_addr;
	msg.msg_namelen = sizeof(from_addr);
	msg.msg_control = &control;
	msg.msg_controllen = sizeof(control);

	res = recvmsg(sock, &msg, recvmsg_flags | MSG_DONTWAIT);
	if (res < 0) {
		fprintf(stderr, "%s %s: %s\n",
			"recvmsg",
			(recvmsg_flags & MSG_ERRQUEUE) ? "error" : "regular",
			strerror(errno));
	} else {
		if (!(recvmsg_flags & MSG_ERRQUEUE)) {
			printf("Received Packet from Peer : ");
			for (i = 6; i < 12; i++)
				printf("%02x:", data[i]);
			printf("\n");
		}
		printpacket(&msg, res, sock, recvmsg_flags, siocgstamp,
			    siocgstampns);
	}
}

/**
 * @brief      Prints Sent/Received Sync packets
 *
 * @param      msg msghdr structure variable
 * @param      res result of recvmsg call
 * @param      sock socket variable
 * @param      recvmsg_flags  flags for receive message
 * @param      siocgstamp timestamp flag
 * @param      siocgstampns timestamp flag for nano secs
 *
 * @return     N/A
 **/
void
printpacket(struct msghdr *msg, int res,
	    int sock, int recvmsg_flags, int siocgstamp, int siocgstampns)
{
	struct cmsghdr *cmsg;
	struct timespec now;
	struct timespec *stamp;
	clock_gettime(CLOCK_REALTIME, &now);
	if (!(recvmsg_flags & MSG_ERRQUEUE)) {
		printf("Application time : %ld.%09ld (received)\n",
		       (long)now.tv_sec, (long)now.tv_nsec);
	}

	for (cmsg = CMSG_FIRSTHDR(msg); cmsg; cmsg = CMSG_NXTHDR(msg, cmsg)) {
		if (cmsg->cmsg_level == SOL_SOCKET &&
		    cmsg->cmsg_type == SO_TIMESTAMPING) {
			stamp = (struct timespec *)CMSG_DATA(cmsg);
			stamp++;
			/* skip deprecated HW transformed */
			stamp++;
			fprintf(stdout, "HW time : %ld.%09ld\n",
				(long)stamp->tv_sec, (long)stamp->tv_nsec);
			if (!(recvmsg_flags & MSG_ERRQUEUE))
				fprintf(stdout, "Delta in nsecs= %lld\n",
					((long long)(now.tv_sec -
						     stamp->tv_sec) *
					 1000000000L + now.tv_nsec) -
					(stamp->tv_nsec));
			else
				fprintf(stdout, "Delta in nsecs= %lld",
					((long long)(stamp->tv_sec -
						     send_time.tv_sec) *
					 1000000000L + (stamp->tv_nsec) -
					 send_time.tv_nsec));
		}
	}
	fprintf(stdout, "\n");
}
