/*    This is a component of LinuxCNC
 *    Copyright 2013,2014 Michael Geszkiewicz <micges@wp.pl>,
 *    Jeff Epler <jepler@unpythonic.net>
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/sockios.h>
#include <net/if_arp.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <ifaddrs.h>
#include <unistd.h>
#include <spawn.h>
#include <sys/types.h>
#include <sys/wait.h>

#include <rtapi_slab.h>
#include <rtapi_ctype.h>
#include <rtapi_list.h>
#include <rtapi_math64.h>

#include "rtapi.h"
#include "rtapi_app.h"
#include "rtapi_string.h"

#include "hal.h"

#include "hostmot2-lowlevel.h"
#include "hostmot2.h"
#include "hm2_eth.h"
#include "eshellf.h"

struct kvlist {
    struct rtapi_list_head list;
    char key[16];
    int value;
};

static struct rtapi_list_head board_num;
static struct rtapi_list_head ifnames;

static int *kvlist_lookup(struct rtapi_list_head *head, const char *name) {
    struct rtapi_list_head *ptr;
    rtapi_list_for_each(ptr, head) {
        struct kvlist *ent = rtapi_list_entry(ptr, struct kvlist, list);
        if(strncmp(name, ent->key, sizeof(ent->key)) == 0) return &ent->value;
    }
    struct kvlist *ent = rtapi_kzalloc(sizeof(struct kvlist), RTAPI_GPF_KERNEL);
    strncpy(ent->key, name, sizeof(ent->key));
    rtapi_list_add(&ent->list, head);
    return &ent->value;
}

static void kvlist_free(struct rtapi_list_head *head) {
    struct rtapi_list_head *orig_head = head;
    for(head = head->next; head != orig_head;) {
        struct rtapi_list_head *ptr = head;
        head = head->next;
        struct kvlist *ent = rtapi_list_entry(ptr, struct kvlist, list);
        rtapi_list_del(ptr);
        rtapi_kfree(ent);
    }
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Michael Geszkiewicz");
MODULE_DESCRIPTION("Driver for HostMot2 on the 7i80 Anything I/O board from Mesa Electronics");
#ifdef MODULE_SUPPORTED_DEVICE
MODULE_SUPPORTED_DEVICE("Mesa-AnythingIO-7i80");
#endif

static char *board_ip[MAX_ETH_BOARDS];
RTAPI_MP_ARRAY_STRING(board_ip, MAX_ETH_BOARDS, "ip address of ethernet board(s)");

static char *config[MAX_ETH_BOARDS];
RTAPI_MP_ARRAY_STRING(config, MAX_ETH_BOARDS, "config string for the AnyIO boards (see hostmot2(9) manpage)")

int debug = 0;
RTAPI_MP_INT(debug, "Developer/debug use only!  Enable debug logging.");

static int boards_count = 0;

int comm_active = 0;

static int comp_id;

static const char *hm2_7i96_pin_names[] = {
    "TB3-01",
    "TB3-02",
    "TB3-03",
    "TB3-04",
    "TB3-05",
    "TB3-06",
    "TB3-07",
    "TB3-08",
    "TB3-09",
    "TB3-10",
    "TB3-11",
    "TB3-13/TB3-14",
    "TB3-15/TB3-16",
    "TB3-17/TB3-18",
    "TB3-19/TB3-20",
    "TB3-21/TB3-22",
    "TB3-23/TB3-24",

    "TB1-02/TB1-03",
    "TB1-04/TB1-05",
    "TB1-08/TB1-09",
    "TB1-10/TB1-11",
    "TB1-14/TB1-15",
    "TB1-16/TB1-17",
    "TB1-20/TB1-21",
    "TB1-22-TB1-23",

    "TB2-02/TB2-03",
    "TB2-04/TB2-05",
    "TB2-07/TB2-08",
    "TB2-10/TB2-11",
    "TB2-13/TB2-14",
    "TB2-16/TB2-17",
    "TB2-18/TB2-19",

    "internal",  /* SSerial TXEN */
    "internal",  /* SSR AC Reference pin */

    "P1-01/DB25-01", /* P1 parallel expansion */
    "P1-02/DB25-14",
    "P1-03/DB25-02",
    "P1-04/DB25-15",
    "P1-05/DB25-03",
    "P1-06/DB25-16",
    "P1-07/DB25-04",
    "P1-08/DB25-17",
    "P1-09/DB25-05",
    "P1-11/DB25-06",
    "P1-13/DB25-07",
    "P1-15/DB25-08",
    "P1-17/DB25-09",
    "P1-19/DB25-10",
    "P1-21/DB25-11",
    "P1-23/DB25-12",
    "P1-25/DB25-13",
};

static const char *hm2_7i94_pin_names[] = {
    "P2-01/DB25-01", /* P2 parallel expansion */
    "P2-02/DB25-14",
    "P2-03/DB25-02",
    "P2-04/DB25-15",
    "P2-05/DB25-03",
    "P2-06/DB25-16",
    "P2-07/DB25-04",
    "P2-08/DB25-17",
    "P2-09/DB25-05",
    "P2-11/DB25-06",
    "P2-13/DB25-07",
    "P2-15/DB25-08",
    "P2-17/DB25-09",
    "P2-19/DB25-10",
    "P2-21/DB25-11",
    "P2-23/DB25-12",
    "P2-25/DB25-13",
    "J6-RX0",
    "J6-TX0",
    "J6-TXEN0",
    "J7-RX1",

    "J7-TX1",
    "J7-TXEN1",
    "J8-RX2",
    "J8-TX2",
    "J8-TXEN2",
    "J9-RX3",
    "J9-TX3",
    "J9-TXEN3",
    "J4-RX4",
    "J4-TX4",
    "J4-TXEN4",
    "J3-RX5",
    "J3-TX5",
    "J3-TXEN5",
    "J2-RX6",
    "J2-TX6",
    "J2-TXEN6",
    "J1-RX7",
    "J1-TX7",
    "J1-TXEN7",
    "P2-/IOENA"
};

static const char *hm2_7i95_pin_names[] = {
    "TB3-02/TB3-03", /* Step/Dir/Misc 5V out */
    "TB3-04/TB3-05",
    "TB3-08/TB3-09",
    "TB3-10/TB3-11",
    "TB3-14/TB3-15",
    "TB3-16/TB3-17",
    "TB3-20/TB3-21",
    "TB3-22/TB3-23",
    "TB4-02/TB4-03",
    "TB4-04/TB4-05",
    "TB4-08/TB4-09",
    "TB4-10/TB4-11",
    "TB4-14/TB4-15", /* RS-422/RS-485 interface */
    "TB4-16/TB4-17",
    "TB3-16/TB3-17",
    "TB4-20/TB4-21",
    "TB4-22/TB4-23",
    "TB4-22/TB4-23",
    "TB1-01/TB1-09", /* Muxed encoders */
    "TB1-04/TB1-12",
    "TB1-07/TB1-15",
    "TB1-17/TB2-01",
    "TB1-20/TB2-04",
    "TB1-23/TB2-07",
    "TB2-09/TB2-17",
    "TB2-12/TB2-20",
    "TB2-15/TB2-23",
    "Internal EncMux",
    "Internal InMux0", /* InMux */
    "Internal InMux1",
    "Internal InMux2",
    "Internal InMux3",
    "Internal InMux4",
    "Internal InMuxData",
    "TB5-13/TB5-14", /* SSR */
    "TB5-15/TB5-16",
    "TB5-17/TB5-18",
    "TB5-19/TB5-20",
    "TB5-21/TB5-22",
    "TB5-23/TB5-24",
    "internal",

    "P1-01/DB25-01",  /* P1 parallel expansion */
    "P1-02/DB25-14",
    "P1-03/DB25-02",
    "P1-04/DB25-15",
    "P1-05/DB25-03",
    "P1-06/DB25-16",
    "P1-07/DB25-04",
    "P1-08/DB25-17",
    "P1-09/DB25-05",
    "P1-11/DB25-06",
    "P1-13/DB25-07",
    "P1-15/DB25-08",
    "P1-17/DB25-09",
    "P1-19/DB25-10",
    "P1-21/DB25-11",
    "P1-23/DB25-12",
    "P1-25/DB25-13",
};

static const char *hm2_7i97_pin_names[] = {
    "TB3-04", 		/* Analog out */
    "TB3-08",
    "TB3-12",
    "TB3-16",
    "TB3-20",    
    "AN_ENA4",
    "TB3-24",
    "AN_ENA5",
    "AN-ENA0..3",
    "TB1-01/TB1-09", /* Muxed encoders */
    "TB1-04/TB1-12",
    "TB1-07/TB1-15",
    "TB1-17/TB2-01",
    "TB1-20/TB2-04",
    "TB1-23/TB2-07",
    "TB2-09/TB2-17",

    "TB2-12/TB2-20",
    "TB2-15/TB2-23",
    "Internal EncMux",
    "TB5-13/TB5-14", /* SSR */
    "TB5-15/TB5-16",
    "TB5-17/TB5-18",
    "TB5-19/TB5-20",
    "TB5-21/TB5-22",
    "TB5-23/TB5-24",
    "Internal InMux0", /* InMux */
    "Internal InMux1",
    "Internal InMux2",
    "Internal InMux3",
    "Internal InMuxData",
    "TB4-15/TB4-16", /* RS-422/RS-485 interface */
    "TB4-17/TB4-18",
    "TB4-17/TB4-18",

    "P1-01/DB25-01",  /* P1 parallel expansion */
    "P1-02/DB25-14",
    "P1-03/DB25-02",
    "P1-04/DB25-15",
    "P1-05/DB25-03",
    "P1-06/DB25-16",
    "P1-07/DB25-04",
    "P1-08/DB25-17",
    "P1-09/DB25-05",
    "P1-11/DB25-06",
    "P1-13/DB25-07",
    "P1-15/DB25-08",
    "P1-17/DB25-09",
    "P1-19/DB25-10",
    "P1-21/DB25-11",
    "P1-23/DB25-12",
    "P1-25/DB25-13",
};

static const char *hm2_Mc04_pin_names[] = {
    "Relay2",
    "Relay3",
    "Relay4",
    "Relay5",
    "Relay1",
    "Single led",
    "Encoder #0, pin B (Input)",
    "Encoder #0, pin A (Input)",
    "Encoder #1, pin A (Input)",
    "Encoder #0, pin Index (Input)",
    "Servo0VRDY",
    "StepGen #0, pin Step (Output)",
    "StepGen #0, pin Direction (Output)",
    "StepGen #1, pin Step (Output)",
    "StepGen #1, pin Direction (Output)",
    "StepGen #2, pin Step (Output)",
    "StepGen #2, pin Direction (Output)",
    "StepGen #3, pin Step (Output)",
    "StepGen #3, pin Direction (Output)",
    "Encoder #2, pin B (Input)",
    "Encoder #2, pin A (Input)",
    "Encoder #1, pin Index (Input)",
    "Encoder #1, pin B (Input)",
    "Servo1VRDY",
    "Servo2VRDY",
    "Encoder #3, pin Index (Input)",
    "Encoder #3, pin B (Input)",
    "Encoder #3, pin A (Input)",
    "Encoder #2, pin Index (Input)",
    "Servo3VRDY",
    "Servo's on",
    "Output0",
    "Output1",
    "Output2",
    "Output3",
    "Output4",
    "Output5",
    "Output6",
    "Output7",
    "Input7",
    "Input6",
    "Input5",
    "Input4",
    "Input3",
    "Input2",
    "Input1",
    "Input0",
    "PWMGen #0, pin Out0 (PWM or Up) (Output)",
    "PWMGen #0, pin Not-Enable (Output)",
    "Spindle-on",
    "Spindle-ccw",
    "Smart Serial Interface #0, pin txen2 (Output)",
    "Smart Serial Interface #0, pin tx2 (Output)",
    "Smart Serial Interface #0, pin rx2 (Input)",
    "Smart Serial Interface #0, pin txen1 (Output)",
    "Smart Serial Interface #0, pin tx1 (Output)",
    "Smart Serial Interface #0, pin rx1 (Input)",
    "Smart Serial Interface #0, pin txen0 (Output)",
    "Smart Serial Interface #0, pin tx0 (Output)",
    "Smart Serial Interface #0, pin rx0 (Input)"
};

static const char *hm2_8cSS_pin_names[] = {
    "Not used",
    "Not used",
    "Not used",
    "Not used",
    "Not used",
    "Not used",
    "Smart Serial Interface #0, pin tx1 (Output)",
    "Smart Serial Interface #0, pin rx1 (Input)",
    "Smart Serial Interface #0, pin txen1 (Output)",
    "Smart Serial Interface #0, pin tx2 (Output)",
    "Smart Serial Interface #0, pin rx2 (Input)",
    "Smart Serial Interface #0, pin txen2 (Output)",
    "Smart Serial Interface #0, pin tx3 (Output)",
    "Smart Serial Interface #0, pin rx3 (Input)",
    "Smart Serial Interface #0, pin txen3 (Output)",
    "Smart Serial Interface #0, pin tx4 (Output)",
    "Smart Serial Interface #0, pin rx4 (Input)",
    "Smart Serial Interface #0, pin txen4 (Output)",
    "Input0",
    "Input1",
    "Input2",
    "Input3",
    "Input4",
    "Input5",
    "Input6",
    "Input7",
    "Not used",
    "Not used",
    "Not used",
    "Not used",
    "Not used",
    "Not used",
    "Output2",
    "Output1",
    "Output0",
    "Output5",
    "Output4",
    "Output3",
    "Output7",
    "Output6",   
    "Not used",
    "Not used",
    "Not used",
    "Not used",
    "Spindle CW",
    "Spindle PWM",
    "Spindle enable",
    "Spindle CCW",
    "Smart Serial Interface #0, pin tx8 (Output)", 
    "Smart Serial Interface #0, pin rx8 (Input)",
    "Smart Serial Interface #0, pin txen8 (Output)",
    "Smart Serial Interface #0, pin rx7 (Input)",
    "Smart Serial Interface #0, pin tx7 (Output)",
    "Smart Serial Interface #0, pin txen5 (Output)",   
    "Smart Serial Interface #0, pin rx5 (Input)",
    "Smart Serial Interface #0, pin tx5 (Output)",
    "Smart Serial Interface #0, pin txen6 (Output)",
    "Smart Serial Interface #0, pin rx6 (Input)",   
    "Smart Serial Interface #0, pin tx6 (Output)",
    "Smart Serial Interface #0, pin txen7 (Output)",   

};

#define UDP_PORT 27181
#define SEND_TIMEOUT_US 10
#define RECV_TIMEOUT_US 10
#define READ_PCK_DELAY_NS 10000

static hm2_eth_t boards[MAX_ETH_BOARDS];

/// ethernet io functions

static int eth_socket_send(int sockfd, const void *buffer, int len, int flags);
static int eth_socket_recv(int sockfd, void *buffer, int len, int flags);

#define IPTABLES "env \"PATH=/usr/sbin:/sbin:${PATH}\" iptables"
#define CHAIN "hm2-eth-rules-output"

static bool chain_exists() {
    int result =
        shell(IPTABLES" -n -L "CHAIN" > /dev/null 2>&1");
    return result == EXIT_SUCCESS;
}

static int iptables_state = -1;
static bool use_iptables() {
    if(iptables_state == -1) {
        if(!chain_exists()) {
            int res = shell(IPTABLES " -N " CHAIN);
            if(res != EXIT_SUCCESS) {
                LL_PRINT("ERROR: Failed to create iptables chain "CHAIN);
                return (iptables_state = 0);
            }
        }
        // now add a jump to our chain at the start of the OUTPUT chain if it isn't in the chain already
        int res = shell(IPTABLES "-C OUTPUT -j " CHAIN " 2>/dev/null || /sbin/iptables -I OUTPUT 1 -j " CHAIN);
        if(res != EXIT_SUCCESS) {
            LL_PRINT("ERROR: Failed to insert rule in OUTPUT chain");
            return (iptables_state = 0);
        }
        return (iptables_state = 1);
    }
    return iptables_state;
}

static void clear_iptables() {
    shell(IPTABLES" -F "CHAIN" > /dev/null 2>&1");
}

static char* inet_ntoa_buf(struct in_addr in, char *buf, size_t n) {
    const char *addr = inet_ntoa(in);
    snprintf(buf, n, "%s", addr);
    return buf;
}

static char* fetch_ifname(int sockfd, char *buf, size_t n) {
    struct sockaddr_in srcaddr;
    struct ifaddrs *ifa, *it;

    socklen_t addrlen = sizeof(srcaddr);
    int res = getsockname(sockfd, &srcaddr, &addrlen);
    if(res < 0) return NULL;

    if(getifaddrs(&ifa) < 0) {
        perror("getifaddrs");
        return NULL;
    }

    for(it=ifa; it; it=it->ifa_next) {
        struct sockaddr_in *ifaddr = (struct sockaddr_in*)it->ifa_addr;
        if (ifaddr == NULL) continue;
        if(ifaddr->sin_family != srcaddr.sin_family) continue;
        if(ifaddr->sin_addr.s_addr != srcaddr.sin_addr.s_addr) continue;
        snprintf(buf, n, "%s", it->ifa_name);
        freeifaddrs(ifa);
        return buf;
    }

    freeifaddrs(ifa);
    return NULL;
}

static char *vseprintf(char *buf, char *ebuf, const char *fmt, va_list ap) {
    int result = vsnprintf(buf, ebuf-buf, fmt, ap);
    if(result < 0) return ebuf;
    else if(buf + result > ebuf) return ebuf;
    else return buf + result;
}

static char *seprintf(char *buf, char *ebuf, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    char *result = vseprintf(buf, ebuf, fmt, ap);
    va_end(ap);
    return result;
}

static int install_iptables_rule(const char *fmt, ...) {
    char commandbuf[1024], *ptr = commandbuf,
        *ebuf = commandbuf + sizeof(commandbuf);
    ptr = seprintf(ptr, ebuf, IPTABLES" -A "CHAIN" ");
    va_list ap;
    va_start(ap, fmt);
    ptr = vseprintf(ptr, ebuf, fmt, ap);
    va_end(ap);

    if(ptr == ebuf)
    {
        LL_PRINT("ERROR: commandbuf too small\n");
        return -ENOSPC;
    }

    int res = shell(commandbuf);
    if(res == EXIT_SUCCESS) return 0;

    LL_PRINT("ERROR: Failed to execute '%s'\n", commandbuf);
    return -EINVAL;
}

static int install_iptables_board(int sockfd) {
    struct sockaddr_in srcaddr, dstaddr;
    char srchost[16], dsthost[16]; // enough for 255.255.255.255\0

    socklen_t addrlen = sizeof(srcaddr);
    int res = getsockname(sockfd, &srcaddr, &addrlen);
    if(res < 0) return -errno;

    addrlen = sizeof(dstaddr);
    res = getpeername(sockfd, &dstaddr, &addrlen);
    if(res < 0) return -errno;

    res = install_iptables_rule(
        "-p udp -m udp -d %s --dport %d -s %s --sport %d -j ACCEPT",
        inet_ntoa_buf(dstaddr.sin_addr, dsthost, sizeof(dsthost)),
        ntohs(dstaddr.sin_port),
        inet_ntoa_buf(srcaddr.sin_addr, srchost, sizeof(srchost)),
        ntohs(srcaddr.sin_port));
    return res;
}

static int install_iptables_perinterface(const char *ifbuf) {
    // without this rule, 'ping' spews a lot of messages like
    //    From 192.168.1.1 icmp_seq=5 Packet filtered
    // many times for each ping packet sent.  With this rule,
    // ping prints 'ping: sendmsg: Operation not permitted' once
    // per second.
    int res = install_iptables_rule(
        "-o %s -p icmp -j DROP",
        ifbuf);
    if(res < 0) return res;

    res = install_iptables_rule(
        "-o %s -j REJECT --reject-with icmp-admin-prohibited",
        ifbuf);
    if(res < 0) return res;

    res = eshellf(HM2_LLIO_NAME, "/sbin/sysctl -q net.ipv6.conf.%s.disable_ipv6=1", ifbuf);
    if(res < 0) return res;

    return 0;
}

static int fetch_hwaddr(const char *board_ip, int sockfd, unsigned char buf[6]) {
    lbp16_cmd_addr packet;
    unsigned char response[6];
    LBP16_INIT_PACKET4(packet, 0x4983, 0x0002);
    int res = eth_socket_send(sockfd, &packet, sizeof(packet), 0);
    if(res < 0) return -errno;

    int i=0;
    do {
        res = eth_socket_recv(sockfd, &response, sizeof(response), 0);
    } while(++i < 10 && res < 0 && errno == EAGAIN);
    if(res < 0) return -errno;

    // eeprom order is backwards from arp AF_LOCAL order
    for(i=0; i<6; i++) buf[i] = response[5-i];

    LL_PRINT("%s: INFO: Hardware address (MAC): %02x:%02x:%02x:%02x:%02x:%02x\n",
        board_ip, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

    return 0;
}

int ioctl_siocsarp(void *arg) {
    hm2_eth_t *board = (hm2_eth_t *)arg;
    return ioctl(board->sockfd, SIOCSARP, &board->req);
}

int ioctl_siocdarp(void *arg) {
    hm2_eth_t *board = (hm2_eth_t *)arg;
    return ioctl(board->sockfd, SIOCDARP, &board->req);
}

static int init_board(hm2_eth_t *board, const char *board_ip) {
    int ret;

    board->sockfd = socket(PF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (board->sockfd < 0) {
        LL_PRINT("ERROR: can't open socket: %s\n", strerror(errno));
        return -errno;
    }
    board->server_addr.sin_family = AF_INET;
    board->server_addr.sin_port = htons(LBP16_UDP_PORT);
    board->server_addr.sin_addr.s_addr = inet_addr(board_ip);

    board->local_addr.sin_family      = AF_INET;
    board->local_addr.sin_addr.s_addr = INADDR_ANY;

    ret = connect(board->sockfd, (struct sockaddr *) &board->server_addr, sizeof(struct sockaddr_in));
    if (ret < 0) {
        LL_PRINT("ERROR: can't connect: %s\n", strerror(errno));
        return -errno;
    }

    if(!use_iptables()) {
        LL_PRINT(\
"WARNING: Unable to restrict other access to the hm2-eth device.\n"
"This means that other software using the same network interface can violate\n"
"realtime guarantees.  See hm2_eth(9) for more information.\n");
    }

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = RECV_TIMEOUT_US;

    ret = setsockopt(board->sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout));
    if (ret < 0) {
        LL_PRINT("ERROR: can't set receive timeout socket option: %s\n", strerror(errno));
        return -errno;
    }

    timeout.tv_usec = SEND_TIMEOUT_US;
    ret = setsockopt(board->sockfd, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout, sizeof(timeout));
    if (ret < 0) {
        LL_PRINT("ERROR: can't set send timeout socket option: %s\n", strerror(errno));
        return -errno;
    }

    memset(&board->req, 0, sizeof(board->req));
    struct sockaddr_in *sin;

    sin = (struct sockaddr_in *) &board->req.arp_pa;
    sin->sin_family = AF_INET;
    sin->sin_addr.s_addr = inet_addr(board_ip);

    board->req.arp_ha.sa_family = AF_LOCAL;
    board->req.arp_flags = ATF_PERM | ATF_COM;
    ret = fetch_hwaddr( board_ip, board->sockfd, (void*)&board->req.arp_ha.sa_data );
    if(ret < 0) {
        LL_PRINT("ERROR: Could not retrieve hardware address (MAC) of %s: %s\n", board_ip, strerror(-ret));
        return ret;
    }

    ret = ioctl_siocsarp(board);
    if(ret < 0) {
        LL_PRINT("ERROR: ioctl SIOCSARP failed: %s\n", strerror(errno));
        board->req.arp_flags &= ~ATF_PERM;
        return -errno;
    }

    if(use_iptables())
    {
        ret = install_iptables_board(board->sockfd);
        if(ret < 0) return ret;
    }

    board->write_packet_ptr = board->write_packet;
    board->read_packet_ptr = board->read_packet;

    return 0;
}

static int close_board(hm2_eth_t *board) {
    int ret;
    board->llio.reset(&board->llio);

    if(use_iptables()) clear_iptables();

    if(board->req.arp_flags & ATF_PERM) {
        ret = ioctl_siocdarp(board);
        if(ret < 0) perror("ioctl SIOCDARP");
    }
    ret = shutdown(board->sockfd, SHUT_RDWR);
    if (ret == -1)
        LL_PRINT("ERROR: can't shutdown socket: %s\n", strerror(errno));
    
    ret = close(board->sockfd);
    if (ret == -1)
        LL_PRINT("ERROR: can't close socket: %s\n", strerror(errno));
    
    return ret < 0 ? -errno : 0;
}

static int eth_socket_send(int sockfd, const void *buffer, int len, int flags) {
    return send(sockfd, buffer, len, flags);
}

static int eth_socket_recv(int sockfd, void *buffer, int len, int flags) {
    return recv(sockfd, buffer, len, flags);
}

static int eth_socket_recv_loop(int sockfd, void *buffer, int len, int flags, long timeout) {
    long long end = rtapi_get_clocks() + timeout;
    int result;
    do {
        result = eth_socket_recv(sockfd, buffer, len, flags);
    } while(result < 0 && rtapi_get_clocks() < end);
    return result;
}

/// hm2_eth io functions

static int hm2_eth_read(hm2_lowlevel_io_t *this, rtapi_u32 addr, void *buffer, int size) {
    hm2_eth_t *board = this->private;
    int send, recv, i = 0;
    rtapi_u8 tmp_buffer[size + 4];
    long long t1, t2;

    if (comm_active == 0) return 1;
    if (size == 0) return 1;
    board->read_cnt++;

    if(rtapi_task_self() >= 0) {
        static bool printed = false;
        if(!printed) {
            LL_PRINT("ERROR: used llio->read in realtime task (addr=0x%04x)\n", addr);
            LL_PRINT("This causes additional network packets which hurts performance\n");
            printed = true;
        }
    }

    lbp16_cmd_addr read_packet;

    LBP16_INIT_PACKET4(read_packet, CMD_READ_HOSTMOT2_ADDR32_INCR(size/4), addr & 0xFFFF);

    send = eth_socket_send(board->sockfd, (void*) &read_packet, sizeof(read_packet), 0);
    if(send < 0)
        LL_PRINT("ERROR: sending packet: %s\n", strerror(errno));
    LL_PRINT_IF(debug, "read(%d) : PACKET SENT [CMD:%02X%02X | ADDR: %02X%02X | SIZE: %d]\n", board->read_cnt, read_packet.cmd_hi, read_packet.cmd_lo,
      read_packet.addr_lo, read_packet.addr_hi, size);
    t1 = rtapi_get_time();
    do {
        errno = 0;
        recv = eth_socket_recv(board->sockfd, (void*) &tmp_buffer, size, 0);
        if(recv < 0) rtapi_delay(READ_PCK_DELAY_NS);
        t2 = rtapi_get_time();
        i++;
    } while ((recv < 0) && ((t2 - t1) < 200*1000*1000));

    if (recv == 4) {
        LL_PRINT_IF(debug, "read(%d) : PACKET RECV [DATA: %08X | SIZE: %d | TRIES: %d | TIME: %llu]\n", board->read_cnt, *tmp_buffer, recv, i, t2 - t1);
    } else {
        LL_PRINT_IF(debug, "read(%d) : PACKET RECV [SIZE: %d | TRIES: %d | TIME: %llu]\n", board->read_cnt, recv, i, t2 - t1);
    }
    if (recv < 0)
        return 0;
    memcpy(buffer, tmp_buffer, size);
    return 1;  // success
}

static int hm2_eth_send_queued_reads(hm2_lowlevel_io_t *this) {
    hm2_eth_t *board = this->private;
    int send;

    // read (low 16 bits of) last write number from space 4 address 0010
    LBP16_INIT_PACKET4(*(lbp16_cmd_addr*)(board->read_packet_ptr), CMD_READ_COMM_CTRL_ADDR16(1), 0x8);
    board->read_packet_ptr += sizeof(lbp16_cmd_addr);
    board->queue_reads[board->queue_reads_count].buffer = &board->rxudpcount;
    board->queue_reads[board->queue_reads_count].size = 2;
    board->queue_reads[board->queue_reads_count].from = board->queue_buff_size;
    board->queue_reads_count++;
    board->queue_buff_size += 2;
    
    board->read_cnt++;
    // write then read back space 4 scratch register at 0010 to verify we got the right receive packet
    LBP16_INIT_PACKET4(*(lbp16_cmd_addr*)(board->read_packet_ptr), CMD_WRITE_TIMER_ADDR16_INCR(2), 0x10);
    board->read_packet_ptr += sizeof(lbp16_cmd_addr);
    *(uint32_t*)board->read_packet_ptr = board->read_cnt;
    board->read_packet_ptr += sizeof(uint32_t);

    LBP16_INIT_PACKET4(*(lbp16_cmd_addr*)(board->read_packet_ptr), CMD_READ_TIMER_ADDR16_INCR(4), 0x10);
    board->read_packet_ptr += sizeof(lbp16_cmd_addr);
    board->queue_reads[board->queue_reads_count].buffer = &board->confirm_read_cnt;
    board->queue_reads[board->queue_reads_count].size = 8;
    board->queue_reads[board->queue_reads_count].from = board->queue_buff_size;
    board->queue_reads_count++;
    board->queue_buff_size += 8;

    send = eth_socket_send(board->sockfd, (void*) &board->read_packet, board->read_packet_ptr - board->read_packet, 0);
    if(send < 0) {
        LL_PRINT("ERROR: sending packet: %s\n", strerror(errno));
        return 0;
    }
    return 1;
}

static bool record_soft_error(hm2_eth_t *board) {
    if(!board->hal) return 1; // still early in hm2_eth_probe
    board->llio.needs_soft_reset = 1;
    *board->hal->packet_error = 1;
    *board->hal->packet_error_total += 1;
    int32_t increment = board->hal->packet_error_increment;
    if(increment < 1) increment = 1;
    board->comm_error_counter += increment;
    if(board->comm_error_counter < 0 || board->comm_error_counter > board->hal->packet_error_limit)
        board->comm_error_counter = board->hal->packet_error_limit;
    *board->hal->packet_error_level = board->comm_error_counter;
    bool result = board->comm_error_counter < board->hal->packet_error_limit;
    if(!result) *board->llio.io_error = true;
    *board->hal->packet_error_exceeded = !result;
    return result;
}

static void decrement_soft_error(hm2_eth_t *board) {
    if(!board->hal) return; // still early in hm2_eth_probe
    int32_t decrement = board->hal->packet_error_decrement;
    if(decrement < 1) decrement = 1;
    board->comm_error_counter -= decrement;
    if(board->comm_error_counter < 0) board->comm_error_counter = 0;
    *board->hal->packet_error_level = board->comm_error_counter;
    *board->hal->packet_error = 0;
    *board->hal->packet_error_exceeded = 0;
}

static int hm2_eth_receive_queued_reads(hm2_lowlevel_io_t *this) {
    hm2_eth_t *board = this->private;
    int recv, i = 0;
    rtapi_u8 tmp_buffer[board->queue_buff_size];
    unsigned long long t1, t2;
    t1 = rtapi_get_time();
    
    // an error occurred in the past but the user has reset the io_error
    // pin (or they did something else like fiddle with the error limit
    // during a run, in which case we don't care if we reset the counter
    // or not)
    if(board->hal && board->comm_error_counter == board->hal->packet_error_limit && !*board->llio.io_error) {
        board->comm_error_counter = 0;
    }

    long read_timeout = board->hal ? board->hal->read_timeout : 1600000;
    if(read_timeout <= 0)//less than or equal to 0, use 80% of the thread period.
        read_timeout = 80;
    if(read_timeout < 100)//less than 100 is interpreted as a percentage of the thread period.
        read_timeout = rtapi_div_s64(read_timeout * (unsigned long long)board->llio.period, 100);
    if(read_timeout < 100000)//Interpret as nanoseconds
        read_timeout = 100000;
 
    if(!board->hal) this->read_time = t1;
    unsigned long long read_deadline = this->read_time + read_timeout;
    do {
do_recv_packet:
        errno = 0;
        recv = eth_socket_recv(board->sockfd, (void*) &tmp_buffer, board->queue_buff_size, MSG_DONTWAIT);
        if(recv < 0) rtapi_delay(READ_PCK_DELAY_NS);
        t2 = rtapi_get_time();
        i++;
    } while (recv != board->queue_buff_size && t2 < read_deadline);
    if(recv != board->queue_buff_size) {
        board->read_packet_ptr = board->read_packet;
        board->queue_reads_count = 0;
        board->queue_buff_size = 0;
        if(!record_soft_error(board)) return 0;
        return -EAGAIN;
    }

    LL_PRINT_IF(debug, "enqueue_read(%d) : PACKET RECV [SIZE: %d | TRIES: %d | TIME: %llu]\n", board->read_cnt, recv, i, t2 - t1);

    for (i = 0; i < board->queue_reads_count; i++) {
        memcpy(board->queue_reads[i].buffer, &tmp_buffer[board->queue_reads[i].from], board->queue_reads[i].size);
    }

    if(board->confirm_read_cnt != board->read_cnt && t2 < read_deadline)
        goto do_recv_packet;

    board->read_packet_ptr = board->read_packet;
    board->queue_reads_count = 0;
    board->queue_buff_size = 0;

    int result = 1;
    // (this means that one in 2^32 lost writes will not be diagnosed,
    // each time board->write_cnt overflows)
    if(board->write_cnt && board->write_cnt != board->confirm_write_cnt) {
        result = record_soft_error(board);
    } else {
        decrement_soft_error(board);
    }
    return result;
}

static int hm2_eth_reset(hm2_lowlevel_io_t *this) {
    LL_PRINT("in hm2_eth_reset\n");

    hm2_eth_t *board = this->private;

    // Make the watchdog timer bite in 1ns from now
    lbp16_cmd_addr_data32 bite_packet;
    LBP16_INIT_PACKET8(bite_packet, CMD_WRITE_HOSTMOT2_ADDR32_INCR(1), HM2_ADDR_WATCHDOG, 0x0001);
    int ret = eth_socket_send(board->sockfd, (void*) &bite_packet, sizeof(bite_packet), 0);
    if(ret < 0) perror("eth_socket_send(bite_packet)");
    return ret < 0 ? -errno : 0;
}

static int hm2_eth_enqueue_read(hm2_lowlevel_io_t *this, rtapi_u32 addr, void *buffer, int size) {
    hm2_eth_t *board = this->private;
    if (comm_active == 0) return 1;
    if (size == 0) return 1;
    // XXX this is missing a check for exceeding the maximum packet size!
    LBP16_INIT_PACKET4(*(lbp16_cmd_addr*)board->read_packet_ptr, CMD_READ_HOSTMOT2_ADDR32_INCR(size/4), addr);
    board->read_packet_ptr += sizeof(lbp16_cmd_addr);
    board->queue_reads[board->queue_reads_count].buffer = buffer;
    board->queue_reads[board->queue_reads_count].size = size;
    board->queue_reads[board->queue_reads_count].from = board->queue_buff_size;
    board->queue_reads_count++;
    board->queue_buff_size += size;
    return 1;
}

static int hm2_eth_enqueue_write(hm2_lowlevel_io_t *this, rtapi_u32 addr, const void *buffer, int size);

static int hm2_eth_write(hm2_lowlevel_io_t *this, rtapi_u32 addr, const void *buffer, int size) {
    if(rtapi_task_self() >= 0 || this->force_enqueue)
        return hm2_eth_enqueue_write(this, addr, buffer, size);

    int send;
    static struct {
        lbp16_cmd_addr wr_packet;
        rtapi_u8 tmp_buffer[127*8];
    } packet;

    if (comm_active == 0) return 1;
    if (size == 0) return 1;
    hm2_eth_t *board = this->private;
    board->write_cnt++;

    memcpy(packet.tmp_buffer, buffer, size);
    LBP16_INIT_PACKET4(packet.wr_packet, CMD_WRITE_HOSTMOT2_ADDR32_INCR(size/4), addr & 0xFFFF);

    send = eth_socket_send(board->sockfd, (void*) &packet, sizeof(lbp16_cmd_addr) + size, 0);
    if(send < 0)
        LL_PRINT("ERROR: sending packet: %s\n", strerror(errno));
    LL_PRINT_IF(debug, "write(%d): PACKET SENT [CMD:%02X%02X | ADDR: %02X%02X | SIZE: %d]\n", board->write_cnt, packet.wr_packet.cmd_hi, packet.wr_packet.cmd_lo,
      packet.wr_packet.addr_lo, packet.wr_packet.addr_hi, size);

    return 1;  // success
}

static int hm2_eth_send_queued_writes(hm2_lowlevel_io_t *this) {
    int send;
    long long t0, t1;
    hm2_eth_t *board = this->private;

    board->write_cnt++;
    // XXX this is missing a check for exceeding the maximum packet size!
    lbp16_cmd_addr *packet = (lbp16_cmd_addr *) board->write_packet_ptr;
    LBP16_INIT_PACKET4(*packet, CMD_WRITE_TIMER_ADDR16_INCR(2), 0x14);
    board->write_packet_ptr += sizeof(*packet);
    memcpy(board->write_packet_ptr, &board->write_cnt, 4);
    board->write_packet_ptr += 4;
    board->write_packet_size += (sizeof(*packet) + 4);
    
    t0 = rtapi_get_time();
    send = eth_socket_send(board->sockfd, (void*) &board->write_packet, board->write_packet_size, 0);
    if(send < 0) {
        LL_PRINT("ERROR: sending packet: %s\n", strerror(errno));
        return 0;
    }
    t1 = rtapi_get_time();
    LL_PRINT_IF(debug, "enqueue_write(%d) : PACKET SEND [SIZE: %d | TIME: %llu]\n", board->write_cnt, send, t1 - t0);
    board->write_packet_ptr = board->write_packet;
    board->write_packet_size = 0;
    return 1;
}

static int hm2_eth_enqueue_write(hm2_lowlevel_io_t *this, rtapi_u32 addr, const void *buffer, int size) {
    hm2_eth_t *board = this->private;
    if (comm_active == 0) return 1;
    if (size == 0) return 1;
    lbp16_cmd_addr *packet = (lbp16_cmd_addr *) board->write_packet_ptr;

    // XXX this is missing a check for exceeding the maximum packet size!
    LBP16_INIT_PACKET4_PTR(packet, CMD_WRITE_HOSTMOT2_ADDR32_INCR(size/4), addr);
    board->write_packet_ptr += sizeof(*packet);
    memcpy(board->write_packet_ptr, buffer, size);
    board->write_packet_ptr += size;
    board->write_packet_size += (sizeof(*packet) + size);
    return 1;
}

static int hm2_eth_set_force_enqueue(hm2_lowlevel_io_t *this, int do_enqueue) {
    if (do_enqueue) {
        this->force_enqueue = 1;
        return 1;
    } else {
        this->force_enqueue = 0;
        return hm2_eth_send_queued_writes(this);
    }
}

static int llio_idx(const char *llio_name) {
    int *idx = kvlist_lookup(&board_num, llio_name);
    return (*idx)++;
}

static int hm2_eth_probe(hm2_eth_t *board) {
    lbp16_cmd_addr read_packet;

    int ret, send, recv;
    char board_name[16] = {0, };
    char llio_name[16] = {0, };

    LBP16_INIT_PACKET4(read_packet, CMD_READ_BOARD_INFO_ADDR16_INCR(16/2), 0);
    send = eth_socket_send(board->sockfd, (void*) &read_packet, sizeof(read_packet), 0);
    if(send < 0) {
        LL_PRINT("ERROR: sending packet: %s\n", strerror(errno));
        return -errno;
    }
    recv = eth_socket_recv_loop(board->sockfd, (void*) &board_name, 16, 0,
                200 * 1000 * 1000);
    if(recv < 0) {
        LL_PRINT("ERROR: receiving packet: %s\n", strerror(errno));
        return -errno;
    }

    board = &boards[boards_count];
    board->llio.private = board;
    board->llio.split_read = true;

    if (strncmp(board_name, "7I80DB-16", 9) == 0) {
        strncpy(llio_name, board_name, 4);
        llio_name[1] = tolower(llio_name[1]);

        board->llio.num_ioport_connectors = 4;
        board->llio.pins_per_connector = 17;
        board->llio.ioport_connector_name[0] = "J2";
        board->llio.ioport_connector_name[1] = "J3";
        board->llio.ioport_connector_name[2] = "J4";
        board->llio.ioport_connector_name[3] = "J5";
        board->llio.fpga_part_number = "XC6SLX16";
        board->llio.num_leds = 4;
    } else if (strncmp(board_name, "7I80DB-25", 9) == 0) {
        strncpy(llio_name, board_name, 4);
        llio_name[1] = tolower(llio_name[1]);

        board->llio.num_ioport_connectors = 4;
        board->llio.pins_per_connector = 17;
        board->llio.ioport_connector_name[0] = "J2";
        board->llio.ioport_connector_name[1] = "J3";
        board->llio.ioport_connector_name[2] = "J4";
        board->llio.ioport_connector_name[3] = "J5";
        board->llio.fpga_part_number = "XC6SLX25";
        board->llio.num_leds = 4;
    } else if (strncmp(board_name, "7I80HD-16", 9) == 0) {
        strncpy(llio_name, board_name, 4);
        llio_name[1] = tolower(llio_name[1]);

        board->llio.num_ioport_connectors = 3;
        board->llio.pins_per_connector = 24;
        board->llio.ioport_connector_name[0] = "P1";
        board->llio.ioport_connector_name[1] = "P2";
        board->llio.ioport_connector_name[2] = "P3";
        board->llio.fpga_part_number = "XC6SLX16";
        board->llio.num_leds = 4;
    } else if (strncmp(board_name, "7I80HD-25", 9) == 0) {
        strncpy(llio_name, board_name, 4);
        llio_name[1] = tolower(llio_name[1]);

        board->llio.num_ioport_connectors = 3;
        board->llio.pins_per_connector = 24;
        board->llio.ioport_connector_name[0] = "P1";
        board->llio.ioport_connector_name[1] = "P2";
        board->llio.ioport_connector_name[2] = "P3";
        board->llio.fpga_part_number = "XC6SLX25";
        board->llio.num_leds = 4;
    } else if (strncmp(board_name, "7I80HDT", 7) == 0) {
        strncpy(llio_name, board_name, 4);
        llio_name[1] = tolower(llio_name[1]);

        board->llio.num_ioport_connectors = 3;
        board->llio.pins_per_connector = 24;
        board->llio.ioport_connector_name[0] = "P1";
        board->llio.ioport_connector_name[1] = "P2";
        board->llio.ioport_connector_name[2] = "P3";
        board->llio.fpga_part_number = "T20F256";
        board->llio.num_leds = 4;
    } else if (strncmp(board_name, "7I76E-16", 8) == 0) {
        strncpy(llio_name, board_name, 5);
        llio_name[1] = tolower(llio_name[1]);
        llio_name[4] = tolower(llio_name[4]);

        board->llio.num_ioport_connectors = 3;
        board->llio.pins_per_connector = 17;
        board->llio.ioport_connector_name[0] = "P1";
        board->llio.ioport_connector_name[1] = "P2";
        board->llio.ioport_connector_name[2] = "P3";
        board->llio.fpga_part_number = "XC6SLX16";
        board->llio.num_leds = 4;
    } else if (strncmp(board_name, "7I76EU", 8) == 0) {
        strncpy(llio_name, board_name, 5);
        llio_name[1] = tolower(llio_name[1]);
        llio_name[4] = tolower(llio_name[4]);

        board->llio.num_ioport_connectors = 3;
        board->llio.pins_per_connector = 17;
        board->llio.ioport_connector_name[0] = "P1";
        board->llio.ioport_connector_name[1] = "P2";
        board->llio.ioport_connector_name[2] = "P3";
        board->llio.fpga_part_number = "T20F256";
        board->llio.num_leds = 4;
    } else if (strncmp(board_name, "7I92", 4) == 0) {
        strncpy(llio_name, board_name, 4);
        llio_name[1] = tolower(llio_name[1]);

        board->llio.num_ioport_connectors = 2;
        board->llio.pins_per_connector = 17;
        board->llio.ioport_connector_name[0] = "P2";
        board->llio.ioport_connector_name[1] = "P1";
        board->llio.fpga_part_number = "XC6SLX9";
        board->llio.num_leds = 4;
    } else if (strncmp(board_name, "7I92T", 5) == 0) {
        strncpy(llio_name, board_name, 4);
        llio_name[1] = tolower(llio_name[1]);

        board->llio.num_ioport_connectors = 2;
        board->llio.pins_per_connector = 17;
        board->llio.ioport_connector_name[0] = "P2";
        board->llio.ioport_connector_name[1] = "P1";
        board->llio.fpga_part_number = "T20F256";
        board->llio.num_leds = 4;

    } else if (strncmp(board_name, "7I93", 4) == 0) {
        strncpy(llio_name, board_name, 4);
        llio_name[1] = tolower(llio_name[1]);
        board->llio.num_ioport_connectors = 2;
        board->llio.pins_per_connector = 24;
        board->llio.ioport_connector_name[0] = "P2";
        board->llio.ioport_connector_name[1] = "P1";
        board->llio.fpga_part_number = "6slx9tqg144";
        board->llio.num_leds = 4;

    } else if (strncmp(board_name, "7I94", 8) == 0) {
        strncpy(llio_name, board_name, 8);
        llio_name[1] = tolower(llio_name[1]);
        board->llio.num_ioport_connectors = 2;
        board->llio.pins_per_connector = 21;
        board->llio.io_connector_pin_names = hm2_7i94_pin_names;

        // DB25, 17 pins used, IO 0 to IO 16
        board->llio.ioport_connector_name[0] = "P2";

        // Serial 0..3 IO 17 to IO 28
        board->llio.ioport_connector_name[1] = "J1,J4";

        // Serial 4..7 IO 29 to IO 41
        board->llio.ioport_connector_name[2] = "J6,J9";

        // P2 /IOENA
        board->llio.ioport_connector_name[3] = "P2-ENA";

        board->llio.fpga_part_number = "6slx9tqg144";
        board->llio.num_leds = 4;
    } else if (strncmp(board_name, "7I94T", 8) == 0) {
        strncpy(llio_name, board_name, 4);
        llio_name[1] = tolower(llio_name[1]);
        board->llio.num_ioport_connectors = 2;
        board->llio.pins_per_connector = 21;
        board->llio.io_connector_pin_names = hm2_7i94_pin_names;

        // DB25, 17 pins used, IO 0 to IO 16
        board->llio.ioport_connector_name[0] = "P2";

        // Serial 0..3 IO 17 to IO 28
        board->llio.ioport_connector_name[1] = "J1,J4";

        // Serial 4..7 IO 29 to IO 41
        board->llio.ioport_connector_name[2] = "J6,J9";

        // P2 /IOENA
        board->llio.ioport_connector_name[3] = "P2-ENA";

        board->llio.fpga_part_number = "T20F256";
        board->llio.num_leds = 4;

    } else if (strncmp(board_name, "7I95", 8) == 0) {
        strncpy(llio_name, board_name, 4);
        llio_name[1] = tolower(llio_name[1]);
        board->llio.num_ioport_connectors = 2;
        board->llio.pins_per_connector = 29;
        board->llio.io_connector_pin_names = hm2_7i95_pin_names;

        // DB25, 17 pins used, IO 34 to IO 50
        board->llio.ioport_connector_name[0] = "P1";

        // terminal block, 10 pins used, enc 0-2
        board->llio.ioport_connector_name[1] = "TB1";
  
        // terminal block, 10 pins used, enc 3-5
        board->llio.ioport_connector_name[2] = "TB2";

        // terminal block, 8 pins used, Step & Dir 0-3
        board->llio.ioport_connector_name[3] = "TB3";
 
        // terminal block, 10 pins used, Step & Dir 4,5, serial Rx/Tx/Txen 0,1
        board->llio.ioport_connector_name[2] = "TB4";

        // terminal block, 8 inputs, 6 SSR outputs
        board->llio.ioport_connector_name[3] = "TB5";

        // terminal block, 16 inputs
        board->llio.ioport_connector_name[2] = "TB6";

        board->llio.fpga_part_number = "6slx9tqg144";
        board->llio.num_leds = 4;

    } else if (strncmp(board_name, "7I95T", 8) == 0) {
        strncpy(llio_name, board_name, 4); 
        llio_name[1] = tolower(llio_name[1]);
        board->llio.num_ioport_connectors = 2;
        board->llio.pins_per_connector = 29;
        board->llio.io_connector_pin_names = hm2_7i95_pin_names;

        // DB25, 17 pins used, IO 34 to IO 50
        board->llio.ioport_connector_name[0] = "P1";

        // terminal block, 10 pins used, enc 0-2
        board->llio.ioport_connector_name[1] = "TB1";
  
        // terminal block, 10 pins used, enc 3-5
        board->llio.ioport_connector_name[2] = "TB2";

        // terminal block, 8 pins used, Step & Dir 0-3
        board->llio.ioport_connector_name[3] = "TB3";
 
        // terminal block, 10 pins used, Step & Dir 4,5, serial Rx/Tx/Txen 0,1
        board->llio.ioport_connector_name[2] = "TB4";

        // terminal block, 8 inputs, 6 SSR outputs
        board->llio.ioport_connector_name[3] = "TB5";

        // terminal block, 16 inputs
        board->llio.ioport_connector_name[2] = "TB6";

        board->llio.fpga_part_number = "T20F256";
        board->llio.num_leds = 4;

    } else if (strncmp(board_name, "7I96", 8) == 0) {
        strncpy(llio_name, board_name, 8);
        llio_name[1] = tolower(llio_name[1]);
        board->llio.num_ioport_connectors = 3;
        board->llio.pins_per_connector = 17;
        board->llio.io_connector_pin_names = hm2_7i96_pin_names;

        // DB25, 17 pins used, IO 34 to IO 50
        board->llio.ioport_connector_name[0] = "P1";

        // terminal block, 8 pins used, Step & Dir 0-3
        board->llio.ioport_connector_name[1] = "TB1";

        // terminal block, 7 pins used, Step & Dir 4, Enc A, B, Z, serial Rx/Tx
        board->llio.ioport_connector_name[2] = "TB2";

        // terminal block, 11 inputs, 6 SSR outputs
        board->llio.ioport_connector_name[3] = "TB3";

        board->llio.fpga_part_number = "6slx9tqg144";
        board->llio.num_leds = 4;

    } else if (strncmp(board_name, "7I96S", 8) == 0) {
        strncpy(llio_name, board_name, 8);
        llio_name[1] = tolower(llio_name[1]);
        llio_name[4] = tolower(llio_name[4]);
       board->llio.num_ioport_connectors = 3;
        board->llio.pins_per_connector = 17;
        board->llio.io_connector_pin_names = hm2_7i96_pin_names;

        // DB25, 17 pins used, IO 34 to IO 50
        board->llio.ioport_connector_name[0] = "P1";

        // terminal block, 8 pins used, Step & Dir 0-3
        board->llio.ioport_connector_name[1] = "TB1";

        // terminal block, 7 pins used, Step & Dir 4, Enc A, B, Z, serial Rx/Tx, Spindle Analog
        board->llio.ioport_connector_name[2] = "TB2";

        // terminal block, 11 inputs, 6 SSR outputs
        board->llio.ioport_connector_name[3] = "TB3";

        board->llio.fpga_part_number = "T20F256";
        board->llio.num_leds = 4;

    } else if (strncmp(board_name, "7I97", 8) == 0) {
        strncpy(llio_name, board_name, 8);
        llio_name[1] = tolower(llio_name[1]);
        board->llio.num_ioport_connectors = 3;
        board->llio.pins_per_connector = 17;
        board->llio.io_connector_pin_names = hm2_7i97_pin_names;

        // DB25, 17 pins used, IO 34 to IO 50
        board->llio.ioport_connector_name[0] = "P1";

        // terminal block, 10 pins used, enc 0-2
        board->llio.ioport_connector_name[1] = "TB1";
  
        // terminal block, 10 pins used, enc 3-5
        board->llio.ioport_connector_name[2] = "TB2";

        // terminal block, Analog
        board->llio.ioport_connector_name[3] = "TB3";
 
        // terminal block, 8 inputs + Serial
        board->llio.ioport_connector_name[4] = "TB4";

        // terminal block, 8 inputs, 6 SSR outputs
        board->llio.ioport_connector_name[5] = "TB5";

        board->llio.fpga_part_number = "6slx9tqg144";
        board->llio.num_leds = 4;

    } else if (strncmp(board_name, "7I97T", 8) == 0) {
        strncpy(llio_name, board_name, 4);
        llio_name[1] = tolower(llio_name[1]);
        board->llio.num_ioport_connectors = 3;
        board->llio.pins_per_connector = 17;
        board->llio.io_connector_pin_names = hm2_7i97_pin_names;

        // DB25, 17 pins used, IO 34 to IO 50
        board->llio.ioport_connector_name[0] = "P1";

        // terminal block, 10 pins used, enc 0-2
        board->llio.ioport_connector_name[1] = "TB1";
  
        // terminal block, 10 pins used, enc 3-5
        board->llio.ioport_connector_name[2] = "TB2";

        // terminal block, Analog
        board->llio.ioport_connector_name[3] = "TB3";
 
        // terminal block, 8 inputs + Serial
        board->llio.ioport_connector_name[4] = "TB4";

        // terminal block, 8 inputs, 6 SSR outputs
        board->llio.ioport_connector_name[5] = "TB5";

        board->llio.fpga_part_number = "T20F256";
        board->llio.num_leds = 4;


    } else if (strncmp(board_name, "7I98", 4) == 0) {
        strncpy(llio_name, board_name, 4);
        llio_name[1] = tolower(llio_name[1]);
        board->llio.num_ioport_connectors = 3;
        board->llio.pins_per_connector = 17;
        board->llio.ioport_connector_name[0] = "P1";
        board->llio.ioport_connector_name[1] = "P2";
        board->llio.ioport_connector_name[2] = "P3";
        board->llio.fpga_part_number = "6slx9tqg144";
        board->llio.num_leds = 4;


    } else if (strncmp(board_name, "MC04", 4) == 0) {
        strncpy(llio_name, board_name, 4);
        llio_name[1] = tolower(llio_name[1]);
        board->llio.num_ioport_connectors = 2;
        board->llio.pins_per_connector = 30;
        board->llio.ioport_connector_name[0] = "P1";
        board->llio.ioport_connector_name[1] = "P2";
        board->llio.fpga_part_number = "6slx9tqg144";
        board->llio.num_leds = 4;
        board->llio.io_connector_pin_names = hm2_Mc04_pin_names;
        board->llio.ioport_connector_name[0] = "P1";
        board->llio.ioport_connector_name[1] = "P2";


     } else if (strncmp(board_name, "8CSS", 4) == 0) {
        strncpy(llio_name, board_name, 4);
        llio_name[1] = tolower(llio_name[1]);
        board->llio.num_ioport_connectors = 2;
        board->llio.pins_per_connector = 30;
        board->llio.ioport_connector_name[0] = "P1";
        board->llio.ioport_connector_name[1] = "P2";
        board->llio.fpga_part_number = "6slx9tqg144";
        board->llio.num_leds = 4;
        board->llio.io_connector_pin_names = hm2_8cSS_pin_names;
        board->llio.ioport_connector_name[0] = "P1";
        board->llio.ioport_connector_name[1] = "P2";

    } else {
        LL_PRINT("Unrecognized ethernet board found: %.16s -- port names will be wrong\n", board_name);
        strncpy(llio_name, board_name, 4);
        llio_name[1] = tolower(llio_name[1]);

        // this is a layering violation.  it would be nice if special values
        // (such as 0 or -1) could be passed here and the layer which can
        // legitimately read idroms would read the values and store them, but
        // that wasn't trivial to do.
        rtapi_u32 read_data;
        hm2_eth_read(&board->llio, HM2_ADDR_IDROM_OFFSET, &read_data, 4);
        unsigned int idrom_address = read_data & 0xffff;
        hm2_idrom_t idrom;
        memset(&idrom, 0, sizeof(idrom));
        hm2_eth_read(&board->llio, idrom_address, &idrom, sizeof(idrom));

        board->llio.num_ioport_connectors = idrom.io_ports;
        board->llio.pins_per_connector = idrom.port_width;
        unsigned i;
        for(i=0; i<board->llio.num_ioport_connectors; i++)
            board->llio.ioport_connector_name[i] = "??";
        board->llio.fpga_part_number = "??";
        board->llio.num_leds = 0;
    }

    LL_PRINT("discovered %.*s\n", 16, board_name);

    rtapi_snprintf(board->llio.name, sizeof(board->llio.name), "hm2_%.*s.%d", (int)strlen(llio_name), llio_name, llio_idx(llio_name));

    board->llio.comp_id = comp_id;

    board->llio.read = hm2_eth_read;
    board->llio.write = hm2_eth_write;
    board->llio.queue_read = hm2_eth_enqueue_read;
    board->llio.send_queued_reads = hm2_eth_send_queued_reads;
    board->llio.receive_queued_reads = hm2_eth_receive_queued_reads;
    board->llio.queue_write = hm2_eth_enqueue_write;
    board->llio.send_queued_writes = hm2_eth_send_queued_writes;
    if (strncmp(board_name, "litehm2", 7) == 0)
	    board->llio.set_force_enqueue = hm2_eth_set_force_enqueue;
    board->llio.reset = hm2_eth_reset;

    ret = hm2_register(&board->llio, config[boards_count]);
    if (ret != 0) {
        rtapi_print("board fails HM2 registration\n");
        return ret;
    }
    boards_count++;

    return 0;
}

static int hm2_eth_items(hm2_eth_t *board) {
    int r;

    board->hal = hal_malloc(sizeof(*board->hal));
    if(!board->hal) return -ENOMEM;

    if((r = hal_param_s32_newf(HAL_RW,
            &board->hal->read_timeout,
            board->llio.comp_id,
            "%s.packet-read-timeout",
            board->llio.name)) < 0)
        return r;
    board->hal->read_timeout = 80;

    if((r = hal_param_s32_newf(HAL_RW,
            &board->hal->packet_error_limit,
            board->llio.comp_id,
            "%s.packet-error-limit",
            board->llio.name)) < 0)
        return r;
    board->hal->packet_error_limit = 10;

    if((r = hal_param_s32_newf(HAL_RW,
            &board->hal->packet_error_increment,
            board->llio.comp_id,
            "%s.packet-error-increment",
            board->llio.name)) < 0)
        return r;
    board->hal->packet_error_increment = 2;

    if((r = hal_param_s32_newf(HAL_RO,
            &board->hal->packet_error_decrement,
            board->llio.comp_id,
            "%s.packet-error-decrement",
            board->llio.name)) < 0)
        return r;
    board->hal->packet_error_decrement = 1;

    if((r = hal_pin_bit_newf(HAL_OUT,
            &board->hal->packet_error,
            board->llio.comp_id,
            "%s.packet-error",
            board->llio.name)) < 0)
        return r;
    *board->hal->packet_error = 0;

    if((r = hal_pin_u32_newf(HAL_IO,
            &board->hal->packet_error_total,
            board->llio.comp_id,
            "%s.packet-error-total",
            board->llio.name)) < 0)
        return r;
    *board->hal->packet_error_total = 0;

    if((r = hal_pin_s32_newf(HAL_OUT,
            &board->hal->packet_error_level,
            board->llio.comp_id,
            "%s.packet-error-level",
            board->llio.name)) < 0)
        return r;
    *board->hal->packet_error_level = 0;

    if((r = hal_pin_bit_newf(HAL_OUT,
            &board->hal->packet_error_exceeded,
            board->llio.comp_id,
            "%s.packet-error-exceeded",
            board->llio.name)) < 0)
        return r;
    *board->hal->packet_error_exceeded = 0;

    return 0;
}

int rtapi_app_main(void) {
    RTAPI_INIT_LIST_HEAD(&ifnames);
    RTAPI_INIT_LIST_HEAD(&board_num);

    int ret, i;

    LL_PRINT("loading Mesa AnyIO HostMot2 ethernet driver version " HM2_ETH_VERSION "\n");

    ret = hal_init(HM2_LLIO_NAME);
    if (ret < 0)
        return ret;
    comp_id = ret;

    if(use_iptables()) clear_iptables();

    for(i = 0, ret = 0; ret == 0 && i<MAX_ETH_BOARDS && board_ip[i] && *board_ip[i]; i++) {
        ret = init_board(&boards[i], board_ip[i]);
        if(ret < 0) board_ip[i] = 0;
    }

    if (ret < 0)
        goto error;

    int num_boards = i;
    comm_active = 1;

    for(i = 0; i<num_boards; i++)
    {
        ret = hm2_eth_probe(&boards[i]);

        if (ret < 0)
            goto error;

        ret = hm2_eth_items(&boards[i]);

        if (ret < 0)
            goto error;
    }

    for(i = 0; i<num_boards; i++) {
        char ifbuf[64]; // more than enough for eth0
        char *ifptr = fetch_ifname(boards[i].sockfd, ifbuf, sizeof(ifbuf));
        if(!ifptr) {
            LL_PRINT("failed to retrieve interface name for board");
            continue;
        } 
        boards[i].read_cnt = boards[i].write_cnt = 0;
        int *added = kvlist_lookup(&ifnames, ifptr);
        if(*added) continue;
        install_iptables_perinterface(ifptr);
        *added = 1;
    }

    hal_ready(comp_id);

    return 0;

error:
    for(i = 0; i<MAX_ETH_BOARDS && board_ip[i] && board_ip[i][0]; i++)
        close_board(&boards[i]);
    if(use_iptables()) clear_iptables();
    kvlist_free(&board_num);
    kvlist_free(&ifnames);
    hal_exit(comp_id);
    return ret;
}

void rtapi_app_exit(void) {
    int i;
    comm_active = 0;
    for(i = 0; i<MAX_ETH_BOARDS && board_ip[i] && board_ip[i][0]; i++)
        close_board(&boards[i]);

    if(use_iptables()) clear_iptables();

    kvlist_free(&board_num);
    kvlist_free(&ifnames);

    hal_exit(comp_id);
    LL_PRINT("HostMot2 ethernet driver unloaded\n");
}
