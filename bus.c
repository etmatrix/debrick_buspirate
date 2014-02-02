#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>

int buspirate_serial_open(char *port);
void buspirate_serial_close(int fd);
int buspirate_serial_setspeed(int fd, char speed);
void buspirate_binary_protocol(int fd);
int buspirate_serial_write(int fd, char *buf, int size);
void buspirate_binary_reset(int fd);
void buspirate_set_pin(int fd);

/*
   BP   - JTAG
   GND  - GND
   MOSI - TDI  => // output BP
   MISO - TDO  <= // input BP
   CLK  - TCK  => // output BP
   CS   - TMS  => // output BP
   N/A  - TRST
   N/A  - RTCK
   AUX  - SRST => // output BP
 */

enum {
        SERIAL_NORMAL = 0,
        SERIAL_FAST = 1
};

int buspirate_serial_open(char *port)
{
        int fd;
        fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
        return fd;
}

int buspirate_serial_read(int fd, char *buf, int size)
{
        int len = 0;
        int ret = 0;
        int timeout = 0;

        while (len < size) {
                ret = read(fd, buf+len, size-len);
                if (ret == -1)
                        return -1;

                if (ret == 0) {
                        timeout++;

                        if (timeout >= 10)
                                break;

                        continue;
                }

                len += ret;
        }

        if (len != size)
                printf("Error reading data");

        return len;
}

void buspirate_serial_close(int fd)
{
        close(fd);
}

int buspirate_serial_setspeed(int fd, char speed)
{
        struct termios t_opt;
        speed_t baud = (speed == SERIAL_FAST) ? B1000000 : B115200;

        /* set the serial port parameters */
        fcntl(fd, F_SETFL, 0);
        tcgetattr(fd, &t_opt);
        cfsetispeed(&t_opt, baud);
        cfsetospeed(&t_opt, baud);
        t_opt.c_cflag |= (CLOCAL | CREAD);
        t_opt.c_cflag &= ~PARENB;
        t_opt.c_cflag &= ~CSTOPB;
        t_opt.c_cflag &= ~CSIZE;
        t_opt.c_cflag |= CS8;
        t_opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        t_opt.c_iflag &= ~(IXON | IXOFF | IXANY);
        t_opt.c_oflag &= ~OPOST;
        t_opt.c_cc[VMIN] = 0;
        t_opt.c_cc[VTIME] = 10;
        tcflush(fd, TCIFLUSH);
        tcsetattr(fd, TCSANOW, &t_opt);

        return 0;
}

void buspirate_binary_protocol(int fd)
{
        int ret;
        char tmp[21] = { [0 ... 20] = 0x00 };
        int done = 0;
        int cmd_sent = 0;

        buspirate_serial_write(fd, tmp, 20);
        usleep(10000);

        /* reads 1 to n "BBIO1"s and one "OCD1" */
        ret = buspirate_serial_read(fd, tmp, 4);
        if (ret != 4) {
                 printf("Buspirate error. Is binary"
                        "/OpenOCD support enabled?");
                 exit(-1);
        }
        if (strncmp(tmp, "BBIO", 4) == 0) {
            ret = buspirate_serial_read(fd, tmp, 1);
            if (ret != 1) {
                printf("Buspirate did not answer correctly! "
                       "Do you have correct firmware?");
                exit(-1);
            }
            if (tmp[0] != '1') {
                printf("Unsupported binary protocol");
                exit(-1);
            }
        }

}

int buspirate_serial_write(int fd, char *buf, int size)
{
        int ret = 0;

        ret = write(fd, buf, size);

        if (ret != size)
                printf("Error sending data");

        return ret;
}

void buspirate_binary_reset(int fd)
{
        char tmp[5];

        tmp[0] = 0x00; /* exit OCD1 mode */
        buspirate_serial_write(fd, tmp, 1);
        usleep(10000);
        /* We ignore the return value here purposly, nothing we can do */
        buspirate_serial_read(fd, tmp, 5);
        if (strncmp(tmp, "BBIO1", 5) == 0) {
                tmp[0] = 0x0F; /*  reset BP */
                buspirate_serial_write(fd, tmp, 1);
        } else
               printf("Unable to restart buspirate!");
}

void buspirate_set_direction(int fd)
{
    unsigned char buf[1] = { 0x40 }; // 010 XXXXX

    // BP     AUX      | MOSI     | CLK      | MISO     | CS
    // JTAG   SRST     | TDI      | TCK      | TDO      | TMS
    buf[0] |= (0 << 4) | (0 << 3) | (0 << 2) | (1 << 1) | (0 << 0); // input for TDO/MISO all others output

    buspirate_serial_write(fd,buf,1);
    buspirate_serial_read(fd,buf,1); // dummy read
}

void buspirate_set_pin(int fd)
{
    unsigned char buf[1] = { 0x80 }; // 1 XXXXXXX
    int ret = 0;

    // POWER|PULLUP|AUX|MOSI|CLK|MISO|CS
    buf[0] |= 0 << 6; // POWER
    buf[0] |= 0 << 5; // PULLUP
    buf[0] |= 0 << 4; // AUX
    buf[0] |= 1 << 3; // MOSI
    buf[0] |= 0 << 2; // CLK
    buf[0] |= 0 << 1; // MISO
    buf[0] |= 0 << 0; // CS

    buspirate_serial_write(fd,buf,1);
    ret = buspirate_serial_read(fd, buf, 1);
    if (ret != 1) {
        printf("error read serial\n");
    }
    printf("%.2X\n",buf[0]);
}



int main()
{
    int buspirate_fd = buspirate_serial_open("/dev/ttyUSB0");

    buspirate_serial_setspeed(buspirate_fd, SERIAL_NORMAL);
    buspirate_binary_protocol(buspirate_fd);
    buspirate_set_direction(buspirate_fd);
    buspirate_set_pin(buspirate_fd);

    sleep(2);

    buspirate_binary_reset(buspirate_fd);

    buspirate_serial_close(buspirate_fd);
}

