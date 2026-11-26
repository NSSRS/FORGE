/* Passive byte capture. The ENCOS guide specifies no serial trace format. */
#include "probe_serial.h"
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <poll.h>
#include <stdlib.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

bool probe_serial_open(probe_serial_t *serial, const char *path, const char *baud)
{
    static const struct { unsigned baud; speed_t setting; } rates[] = {
        {9600, B9600}, {19200, B19200}, {38400, B38400}, {57600, B57600},
        {115200, B115200}, {230400, B230400}, {460800, B460800}, {921600, B921600}
    };
    char *end;
    unsigned long speed = strtoul(baud, &end, 10);
    size_t i = 0;
    while (i < sizeof(rates)/sizeof(rates[0]) && rates[i].baud != speed) ++i;
    if (*end || i == sizeof(rates)/sizeof(rates[0])) {
        fprintf(stderr, "Unsupported debug serial baud rate: %s\n", baud);
        return false;
    }
    int fd = open(path, O_RDONLY | O_NONBLOCK | O_NOCTTY | O_CLOEXEC);
    if (fd < 0) { perror(path); return false; }
    if (flock(fd, LOCK_EX | LOCK_NB) || tcgetattr(fd, &serial->saved) || ioctl(fd, TIOCEXCL)) {
        perror("cannot claim debug tty");
        close(fd);
        return false;
    }
    struct termios config = serial->saved;
    cfmakeraw(&config);
    config.c_iflag &= ~(IXOFF | IXANY);
    config.c_cflag &= ~(CSTOPB | CRTSCTS);
    config.c_cflag |= CLOCAL | CREAD;
    if (cfsetispeed(&config, rates[i].setting) || cfsetospeed(&config, rates[i].setting) ||
        tcsetattr(fd, TCSANOW, &config)) {
        perror("configure debug tty");
        ioctl(fd, TIOCNXCL);
        close(fd);
        return false;
    }
    serial->fd = fd;
    serial->baud = rates[i].baud;
    return true;
}

bool probe_serial_sample(probe_serial_t *serial, FILE *report)
{
    if (serial->fd < 0) return true;
    struct pollfd pfd = {.fd = serial->fd, .events = POLLIN};
    if (poll(&pfd, 1, 0) < 0) {
        if (errno == EINTR) return true;
        perror("poll debug tty");
        return false;
    }
    if (pfd.revents & (POLLHUP | POLLERR | POLLNVAL)) {
        fprintf(stderr, "Debug tty disconnected or failed.\n");
        return false;
    }
    if (!(pfd.revents & POLLIN)) return true;
    uint8_t data[4096];
    ssize_t size = read(serial->fd, data, sizeof(data));
    if (size < 0 && (errno == EAGAIN || errno == EINTR)) return true;
    if (size <= 0) {
        fprintf(stderr, "Debug tty read failed.\n");
        return false;
    }
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    int64_t ns = (int64_t)now.tv_sec * 1000000000 + now.tv_nsec;
    fprintf(report, "{\"event\":\"debug_rx\",\"host_read_mono_ns\":%" PRId64 ",\"hex\":\"", ns);
    for (ssize_t i = 0; i < size; ++i) fprintf(report, "%02x", data[i]);
    fputs("\"}\n", report);
    serial->bytes += (uint64_t)size;
    return !ferror(report);
}

bool probe_serial_close(probe_serial_t *serial)
{
    bool ok = tcsetattr(serial->fd, TCSANOW, &serial->saved) == 0;
    if (!ok) perror("restore debug tty");
    ioctl(serial->fd, TIOCNXCL);
    close(serial->fd);
    serial->fd = -1;
    return ok;
}
