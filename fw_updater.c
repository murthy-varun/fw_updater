/*
MIT License
Copyright (c) 2019 Varun Murthy (varun.tk@gmail.com)
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>

/*! XMODEM protocol definitions */
#define XMDM_SOH             0x01 /* Start of Header           */
#define XMDM_EOT             0x04 /* End of Transmission       */
#define XMDM_ACK             0x06 /* Acknowledge               */
#define XMDM_NACK            0x15 /* Not Acknowledge           */
#define XMDM_ETB             0x17 /* End of Transmission Block */
#define XMDM_CAN             0x18 /* Cancel                    */
#define XMDM_C               0x43 /* ASCII "C"                 */

#define XMDM_PKT_DATA_LEN    128  /* Packet length             */

#define CRC_POLY             0x1021

#define BUFFER_SIZE 1024

/*! commands */
#define VERSION_CMD  "version"
#define UPDATEFW_CMD "dload fw"
#define SWAPBANK_CMD "swapbank"

struct xmodem_pkt {
    uint8_t  type;
    uint8_t  seq_num;
    uint8_t  seq_neg;
    uint8_t  payload[128];
    uint8_t  crc[2];
} __attribute__((packed));

static void dump_pkt(uint8_t* buffer, size_t size)
{
    int indx = 0;
    for(indx = 0; indx < size; indx++) {
        printf("0x%x\t", buffer[indx]);
        if(indx%8 == 7) {
            printf("\n");
        }
    }
    printf("\n");
}

static int get_baud_rate(uint32_t baud_rate)
{
    int ret = B115200;

    switch (baud_rate) {
        case 0:
            ret = B0;
            break;
        case 50:
            ret = B50;
            break;
        case 75:
            ret = B75;
            break;
        case 110:
            ret = B110;
            break;
        case 134:
            ret = B134;
            break;
        case 150:
            ret = B150;
            break;
        case 200:
            ret = B200;
            break;
        case 300:
            ret = B300;
            break;
        case 600:
            ret = B600;
            break;
        case 1200:
            ret = B1200;
            break;
        case 1800:
            ret = B1800;
            break;
        case 2400:
            ret = B2400;
            break;
        case 4800:
            ret = B4800;
            break;
        case 9600:
            ret = B9600;
            break;
        case 19200:
            ret = B19200;
            break;
        case 38400:
            ret = B38400;
            break;
        case 57600:
            ret = B57600;
            break;
        case 115200:
            ret = B115200;
            break;
        case 230400:
            ret = B230400;
            break;
        default:
            break;
    }
    return ret;
}

static void read_all_serial(int serial_fd, uint8_t* buf, uint16_t buf_len)
{
    ssize_t  len = 0;
    uint16_t indx = 0;

    len = read (serial_fd, buf, buf_len);
    if (len > 0) {
        for(indx = 0; indx < len; indx++)
            printf("%c", buf[indx]);
        printf("\n");
    }
    return;
}

static void send_cmd(int serial_fd, const char* cmd, uint8_t len)
{
    int     ret = 0;
    uint8_t indx = 0;
    uint8_t cr = 0x0d;
    uint8_t byte = 0;

    usleep(100);
    for(indx=0; indx<len; indx++) {
        ret = write(serial_fd, &cmd[indx], 1);
        if(ret != 1) {
            return;
        }
        usleep(100);
        ret = read(serial_fd, &byte, 1);
    }
    ret = write(serial_fd, &cr, 1);
    usleep(100);
    return;
}

static uint8_t xmodem_send_packet(int serial_fd, uint8_t *buf, uint8_t seq_num)
{
    uint32_t           i;
    uint32_t           j;
    uint16_t           checksum = 0;
    int8_t	       c_data;
    uint8_t	       ack;
    struct xmodem_pkt  pkt = {0};
    int                ret;
    uint8_t            byte;

    memset(&pkt, 0x1A, sizeof(struct xmodem_pkt));

    pkt.type    = XMDM_SOH;
    pkt.seq_num = seq_num;
    pkt.seq_neg = (uint8_t)(~seq_num);

    for(i = 0; i < XMDM_PKT_DATA_LEN; i++) {
        c_data = *buf++;
        pkt.payload[i] = c_data;

        checksum = checksum ^ (int32_t) c_data << 8;
        for (j = 0; j < 8; j++) {
            if (checksum & 0x8000) {
                checksum = checksum << 1 ^ CRC_POLY;
            } else {
                checksum = checksum << 1;
            }
        }
        checksum = checksum & 0xFFFF;
    }

    /* An "endian independent way to extract the CRC bytes. */
    pkt.crc[0] = (uint8_t)(checksum >> 8);
    pkt.crc[1] = (uint8_t)(checksum);

    ret = write(serial_fd, &pkt, sizeof(pkt));
    if (ret != sizeof(pkt)) {
        return -1;
    }
    usleep(100);

    /* Wait for ack */
    ret = read(serial_fd, &ack, sizeof(ack));
    if (ret != sizeof(ack)) {
        return -1;
    }
    return ack;
}

static int xmodem_send(int serial_fd, const char *filename)
{
    size_t         len;
    size_t         file_len;
    int            ret;
    int            fd;
    uint8_t        c_char;
    struct stat    stat;
    const uint8_t *buf;
    uint8_t        eof = XMDM_EOT;
    int            ul_done = 0;
    uint16_t       seq_num = 1;

    fd = open(filename, O_RDONLY);
    if (fd < 0) {
        printf("Error opening file %s: %s\n", filename, strerror(errno));
        return -1;
    }

    fstat(fd, &stat);
    len = stat.st_size;
    buf = mmap(NULL, len, PROT_READ, MAP_PRIVATE, fd, 0);
    if (!buf) {
        printf("Error during mmap: %s\n", strerror(errno));
        return -1;
    }


    if (len & (XMDM_PKT_DATA_LEN-1)) {
        len += XMDM_PKT_DATA_LEN;
        len &= ~(XMDM_PKT_DATA_LEN-1);
    }
    file_len = len;

    printf("Waiting for receiver ...\n");
    fflush(stdout);

    do {
        ret = read(serial_fd, &c_char, sizeof(c_char));
        if (ret != sizeof(c_char)) {
            printf("Error during read:%s ret:%d\n", strerror(errno), ret);
            return -1;
        }
    } while (c_char != 'C');
    printf("Received receiver ping\n");

    printf("Sending %s\n", filename);
    ul_done = 0;
    seq_num = 1;
    while (!ul_done) {
        printf("Sending xmodem packet:%u/%u\r",
               (unsigned int)seq_num,
               (unsigned int)(file_len/XMDM_PKT_DATA_LEN));
        fflush(stdout);
        c_char = xmodem_send_packet(serial_fd, (uint8_t *)buf, seq_num);
        switch(c_char) {
            case XMDM_ACK:
                ++seq_num;
                len -= XMDM_PKT_DATA_LEN;
                buf += XMDM_PKT_DATA_LEN;
                /* printf("Received ACK pending length:%lu/%lu\n", len, file_len); */
                break;
            case XMDM_NACK:
                printf("\nReceived NACK\n");
                break;
            case XMDM_CAN:
                printf("\nReceived CAN\n");
                ul_done = -1;
                break;
            case XMDM_EOT:
                printf("\nReceived EOT\n");
                ul_done = -1;
                break;
            default:
                printf("\nReceived unknown reply :%u\n", c_char);
                ul_done = -1;
                break;
        }

        if (!len) {
            ret = write(serial_fd, &eof, sizeof(eof));
            if (ret != sizeof(eof)) {
                return -1;
            }

            /* Flush the ACK */
            ret = read(serial_fd, &c_char, sizeof(c_char));
            if (ret != sizeof(c_char)) {
                printf("Error during read:%s ret:%d\n", strerror(errno), ret);
                return -1;
            }
            break;
        }
    }

    printf("\nFile transfer complete.\n");
    close(fd);
    return 0;
}

static int open_serial(const char *path, int baud_rate)
{
    int             fd;
    struct termios  tty = {0};

    fd = open(path, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        printf("Error opening %s: %s\n", path, strerror(errno));
        return -1;
    }

	if (ioctl(fd, TIOCEXCL)) {
		printf("locking %s failed\n", path);
		close(fd);
		return -1;
	}

    if (tcgetattr(fd, &tty) != 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, baud_rate);
    cfsetispeed(&tty, baud_rate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                         // disable break processing
    tty.c_lflag = 0;                                // no signaling chars, no echo,
                                                    // no canonical processing
    tty.c_oflag = 0;                                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;                            // read doesn't block
    tty.c_cc[VTIME] = 5;                            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);                // ignore modem controls,
                                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);              // shut off parity
    tty.c_cflag |= 0;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }

    return fd;
}

void set_blocking (int serial_fd, int should_block)
{
    struct termios  tty;

    memset (&tty, 0, sizeof tty);
    if (tcgetattr (serial_fd, &tty) != 0) {
        printf ("error %d from tggetattr", errno);
        return;
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr (serial_fd, TCSANOW, &tty) != 0) {
        printf ("error %d setting term attributes", errno);
    }
    return;
}

void usage(char *command) {
    printf("Usage:\n%s -d <serial_device> -b <baud_rate> "
           "-f <fw_file_path> -v <version>\n",
           command);
}

int main(int argc, char **argv)
{
    int          ret = 0;
    int          serial_fd;
    int          opt;
    char        *version = NULL;
    int          fw_version = 0;
    char        *serial_dev = NULL;
    uint32_t     baud_rate = 115200;
    char        *fw_filename = NULL;
    char         fw_update_cmd[32];
    size_t       fw_update_cmd_len = 0;
    uint8_t      buffer[BUFFER_SIZE];
    int          indx = 0;
    int          fd = 0;
    uint32_t     fw_img_size = 0;
    char         update_cmd[32];
    struct stat  stat;

    if (argc != 9) {
        usage(argv[0]);
        return 0;
    }

    while ((opt = getopt(argc, argv, "f:d:v:b:")) != -1) {
        switch (opt) {
            case 'f':
                fw_filename = optarg;
                break;
            case 'd':
                serial_dev = optarg;
                break;
            case 'b':
                baud_rate = atoi(optarg);
                break;
            case 'v':
                version = optarg;
                break;
            case '?':
                printf("unknown option: %c\n", optopt);
                usage(argv[0]);
                return 0;
                break;
        }
    }


    serial_fd = open_serial(serial_dev, get_baud_rate(baud_rate));
    if (serial_fd < 0) {
        printf("unable to open serial device %s: %s\n",
               serial_dev, strerror(errno));
        return -1;
    }

    fd = open(fw_filename, O_RDONLY);
    if (fd < 0) {
        printf("Error opening file %s: %s\n", fw_filename, strerror(errno));
        return -1;
    }
    fstat(fd, &stat);
    fw_img_size = stat.st_size;
    close(fd);

    /* read all available serial buffer */
    read_all_serial(serial_fd, buffer, BUFFER_SIZE);

    /*-------------------------------------------------------------------------
           Issuing Version command before FW upload
    -------------------------------------------------------------------------*/
    printf("\n\nGetting Version number before FW upload\n");
    send_cmd(serial_fd, VERSION_CMD, strlen(VERSION_CMD));
    usleep ((1000) * 100);
    read_all_serial(serial_fd, buffer, BUFFER_SIZE);

    /*-------------------------------------------------------------------------
           Issuing Update FW command before FW upload
    -------------------------------------------------------------------------*/
	snprintf(update_cmd,
             32,
             "%s %u %s",
             UPDATEFW_CMD,
             fw_img_size,
             version);

    send_cmd(serial_fd, update_cmd, strlen(update_cmd));
    usleep ((100) * 100);

    /*-------------------------------------------------------------------------
           Send file using xmodem
    -------------------------------------------------------------------------*/
    printf("\nSending file:%s using xmodem\n", fw_filename);
    set_blocking(serial_fd, 1);
    ret = xmodem_send(serial_fd, fw_filename);
    set_blocking(serial_fd, 0);
    if (ret < 0) {
        printf("\nunable to send file %s: %s\n",
               fw_filename, strerror(errno));
        ret = -1;
    }
    usleep(10000*100);
    /* read all available serial buffer */
    read_all_serial(serial_fd, buffer, BUFFER_SIZE);


	/*-------------------------------------------------------------------------
           Issuing Swapbank command before FW upload
    -------------------------------------------------------------------------*/
    printf("\n\nIssuing Swapbank command\n");
    send_cmd(serial_fd, SWAPBANK_CMD, strlen(SWAPBANK_CMD));
    usleep ((1000) * 100);
    read_all_serial(serial_fd, buffer, BUFFER_SIZE);
	sleep(3);


    /*-------------------------------------------------------------------------
           Issuing Version command after FW upload
    -------------------------------------------------------------------------*/
    printf("\n\nGetting Version number after FW upload\n");
    send_cmd(serial_fd, VERSION_CMD, strlen(VERSION_CMD));
    /* sleep enough to transmit and receive approx 100 uS per char transmit */
    usleep ((10000) * 100);
    read_all_serial(serial_fd, buffer, BUFFER_SIZE);

    close(serial_fd);

    return ret;
}

