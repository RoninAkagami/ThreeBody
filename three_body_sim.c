#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#ifdef _OPENMP
#include <omp.h>
#endif

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
typedef SOCKET socket_t;
#define CLOSESOCKET closesocket
#else
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
typedef int socket_t;
#define INVALID_SOCKET (-1)
#define SOCKET_ERROR (-1)
#define CLOSESOCKET close
#endif

#define SERVER_PORT        60001
#define G                  0.5
#define DT                 0.25
#define UPDATES_PER_FRAME  2
#define SPEED_MULTIPLIER   1
#define MAX_DISTANCE       5000.0
#define MAX_FRAMES         100000

#define IMAGE_SIZE         1000
#define GRID_HALF          (IMAGE_SIZE / 2)
#define GRID_STEP          1.0

typedef struct {
    double y2;
    double vx2;
    double vy2;
    double vx3;
    double vy3;
} SimParams;

typedef struct {
    uint8_t *data;
    size_t size;
} Buffer;

static double calc_total_distance(const double bodies[3][4]) {
    double d1 = hypot(bodies[0][0] - bodies[1][0], bodies[0][1] - bodies[1][1]);
    double d2 = hypot(bodies[1][0] - bodies[2][0], bodies[1][1] - bodies[2][1]);
    double d3 = hypot(bodies[2][0] - bodies[0][0], bodies[2][1] - bodies[0][1]);
    return d1 + d2 + d3;
}

static void compute_forces(double bodies[3][4]) {
    for (int i = 0; i < 3; ++i) {
        double fx = 0.0, fy = 0.0;
        for (int j = 0; j < 3; ++j) {
            if (i == j) continue;
            double dx = bodies[j][0] - bodies[i][0];
            double dy = bodies[j][1] - bodies[i][1];
            double r = hypot(dx, dy);
            if (r < 1.0) r = 1.0;
            double f = G * 1250.0 * 1250.0 / (r * r);
            fx += f * dx / r;
            fy += f * dy / r;
        }
        bodies[i][2] += fx / 1250.0 * DT;
        bodies[i][3] += fy / 1250.0 * DT;
    }
}

static void update_positions(double bodies[3][4]) {
    for (int i = 0; i < 3; ++i) {
        bodies[i][0] += bodies[i][2] * DT;
        bodies[i][1] += bodies[i][3] * DT;
    }
}

static int run_one_simulation(double x3, double y3, const SimParams *p) {
    double bodies[3][4] = {
        {0.0, 0.0, 0.0, 0.0},
        {0.1, p->y2, p->vx2, p->vy2},
        {x3, y3, p->vx3, p->vy3}
    };

    for (int frame = 0; frame < MAX_FRAMES; ++frame) {
        for (int s = 0; s < UPDATES_PER_FRAME * SPEED_MULTIPLIER; ++s) {
            compute_forces(bodies);
            update_positions(bodies);
        }
        if (calc_total_distance(bodies) > MAX_DISTANCE) {
            return frame + 1;
        }
    }

    return MAX_FRAMES;
}

static int compare_ints(const void *a, const void *b) {
    int ia = *(const int *)a;
    int ib = *(const int *)b;
    return (ia > ib) - (ia < ib);
}

static void percentile_inverted_image(const int *frames, uint8_t *image, int count) {
    int *sorted = (int *)malloc((size_t)count * sizeof(int));
    if (!sorted) return;

    memcpy(sorted, frames, (size_t)count * sizeof(int));
    qsort(sorted, (size_t)count, sizeof(int), compare_ints);

    for (int i = 0; i < count; ++i) {
        int value = frames[i];
        int lo = 0, hi = count;
        while (lo < hi) {
            int mid = lo + (hi - lo) / 2;
            if (sorted[mid] <= value) lo = mid + 1;
            else hi = mid;
        }
        double percentile = (count <= 1) ? 0.0 : (double)(lo - 1) / (double)(count - 1);
        int shade = 255 - (int)lrint(percentile * 255.0);
        if (shade < 0) shade = 0;
        if (shade > 255) shade = 255;
        image[i] = (uint8_t)shade;
    }

    free(sorted);
}

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len) {
    crc = ~crc;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int k = 0; k < 8; ++k) {
            crc = (crc >> 1) ^ (0xedb88320u & (uint32_t)-(int)(crc & 1));
        }
    }
    return ~crc;
}

static uint32_t adler32(const uint8_t *data, size_t len) {
    uint32_t a = 1, b = 0;
    for (size_t i = 0; i < len; ++i) {
        a = (a + data[i]) % 65521u;
        b = (b + a) % 65521u;
    }
    return (b << 16) | a;
}

static void write_be32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

static int append_bytes(Buffer *buf, const void *src, size_t len) {
    uint8_t *next = (uint8_t *)realloc(buf->data, buf->size + len);
    if (!next) return 0;
    buf->data = next;
    memcpy(buf->data + buf->size, src, len);
    buf->size += len;
    return 1;
}

static int append_chunk(Buffer *png, const char type[4], const uint8_t *data, uint32_t len) {
    uint8_t header[8];
    write_be32(header, len);
    memcpy(header + 4, type, 4);
    if (!append_bytes(png, header, sizeof(header))) return 0;
    if (len && !append_bytes(png, data, len)) return 0;

    uint32_t crc = crc32_update(0, (const uint8_t *)type, 4);
    if (len) crc = crc32_update(crc, data, len);
    uint8_t crc_bytes[4];
    write_be32(crc_bytes, crc);
    return append_bytes(png, crc_bytes, sizeof(crc_bytes));
}

static int make_png_gray8(const uint8_t *gray, int width, int height, Buffer *png) {
    size_t stride = (size_t)width + 1;
    size_t raw_size = stride * (size_t)height;
    uint8_t *raw = (uint8_t *)malloc(raw_size);
    if (!raw) return 0;

    for (int y = 0; y < height; ++y) {
        raw[(size_t)y * stride] = 0;
        memcpy(raw + (size_t)y * stride + 1, gray + (size_t)y * width, (size_t)width);
    }

    size_t max_blocks = raw_size / 65535u + 1u;
    size_t zlib_cap = 2u + raw_size + max_blocks * 5u + 4u;
    uint8_t *zlib = (uint8_t *)malloc(zlib_cap);
    if (!zlib) {
        free(raw);
        return 0;
    }

    size_t pos = 0;
    zlib[pos++] = 0x78;
    zlib[pos++] = 0x01;

    size_t offset = 0;
    while (offset < raw_size) {
        uint16_t block_len = (uint16_t)((raw_size - offset > 65535u) ? 65535u : raw_size - offset);
        uint8_t final = (offset + block_len >= raw_size) ? 1u : 0u;
        zlib[pos++] = final;
        zlib[pos++] = (uint8_t)(block_len & 255u);
        zlib[pos++] = (uint8_t)(block_len >> 8);
        uint16_t nlen = (uint16_t)~block_len;
        zlib[pos++] = (uint8_t)(nlen & 255u);
        zlib[pos++] = (uint8_t)(nlen >> 8);
        memcpy(zlib + pos, raw + offset, block_len);
        pos += block_len;
        offset += block_len;
    }

    uint32_t adler = adler32(raw, raw_size);
    write_be32(zlib + pos, adler);
    pos += 4;
    free(raw);

    static const uint8_t sig[8] = {137, 80, 78, 71, 13, 10, 26, 10};
    uint8_t ihdr[13];
    write_be32(ihdr, (uint32_t)width);
    write_be32(ihdr + 4, (uint32_t)height);
    ihdr[8] = 8;
    ihdr[9] = 0;
    ihdr[10] = 0;
    ihdr[11] = 0;
    ihdr[12] = 0;

    int ok = append_bytes(png, sig, sizeof(sig)) &&
             append_chunk(png, "IHDR", ihdr, sizeof(ihdr)) &&
             append_chunk(png, "IDAT", zlib, (uint32_t)pos) &&
             append_chunk(png, "IEND", NULL, 0);

    free(zlib);
    return ok;
}

static int render_png(const SimParams *params, Buffer *png) {
    int count = IMAGE_SIZE * IMAGE_SIZE;
    int *frames = (int *)malloc((size_t)count * sizeof(int));
    uint8_t *image = (uint8_t *)malloc((size_t)count);
    if (!frames || !image) {
        free(frames);
        free(image);
        return 0;
    }

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
    for (int row = 0; row < IMAGE_SIZE; ++row) {
        double y3 = (double)(GRID_HALF - row) * GRID_STEP;
        for (int col = 0; col < IMAGE_SIZE; ++col) {
            double x3 = (double)(col - GRID_HALF) * GRID_STEP;
            frames[row * IMAGE_SIZE + col] = run_one_simulation(x3, y3, params);
        }
#ifndef _OPENMP
        if ((row + 1) % 25 == 0) {
            fprintf(stderr, "render progress: %d/%d rows\n", row + 1, IMAGE_SIZE);
        }
#endif
    }

    percentile_inverted_image(frames, image, count);
    int ok = make_png_gray8(image, IMAGE_SIZE, IMAGE_SIZE, png);

    free(frames);
    free(image);
    return ok;
}

static int query_double(const char *request, const char *name, double *out) {
    char pattern[32];
    snprintf(pattern, sizeof(pattern), "%s=", name);
    const char *p = strstr(request, pattern);
    if (!p) return 0;
    p += strlen(pattern);
    char *end = NULL;
    double value = strtod(p, &end);
    if (end == p || !isfinite(value)) return 0;
    *out = value;
    return 1;
}

static SimParams parse_params(const char *request) {
    SimParams p;
    p.y2 = 100.1;
    p.vx2 = 0.0;
    p.vy2 = 20.0;
    p.vx3 = 1.0;
    p.vy3 = 0.0;

    query_double(request, "y2", &p.y2);
    query_double(request, "vx2", &p.vx2);
    query_double(request, "vy2", &p.vy2);
    query_double(request, "vx3", &p.vx3);
    query_double(request, "vy3", &p.vy3);
    return p;
}

static void send_all(socket_t client, const uint8_t *data, size_t len) {
    while (len > 0) {
        int chunk = (len > 16384u) ? 16384 : (int)len;
        int sent = send(client, (const char *)data, chunk, 0);
        if (sent <= 0) return;
        data += sent;
        len -= (size_t)sent;
    }
}

static void send_text(socket_t client, int status, const char *message) {
    char header[512];
    int body_len = (int)strlen(message);
    int n = snprintf(header, sizeof(header),
                     "HTTP/1.1 %d\r\nContent-Type: text/plain; charset=utf-8\r\nContent-Length: %d\r\nConnection: close\r\n\r\n",
                     status, body_len);
    send_all(client, (const uint8_t *)header, (size_t)n);
    send_all(client, (const uint8_t *)message, (size_t)body_len);
}

static void handle_client(socket_t client) {
    char request[4096];
    int received = recv(client, request, sizeof(request) - 1, 0);
    if (received <= 0) return;
    request[received] = '\0';

    if (strncmp(request, "GET /health", 11) == 0) {
        send_text(client, 200, "ok\n");
        return;
    }

    if (strncmp(request, "GET /", 5) != 0) {
        send_text(client, 405, "Use GET /?y2=100.1&vx2=0&vy2=20&vx3=1&vy3=0\n");
        return;
    }

    SimParams params = parse_params(request);
    fprintf(stderr, "render y2=%g vx2=%g vy2=%g vx3=%g vy3=%g\n",
            params.y2, params.vx2, params.vy2, params.vx3, params.vy3);

    Buffer png = {0};
    if (!render_png(&params, &png)) {
        free(png.data);
        send_text(client, 500, "render failed\n");
        return;
    }

    char header[512];
    int n = snprintf(header, sizeof(header),
                     "HTTP/1.1 200 OK\r\nContent-Type: image/png\r\nContent-Length: %zu\r\nContent-Disposition: inline; filename=\"sim_output_percentile_inverted.png\"\r\nConnection: close\r\n\r\n",
                     png.size);
    send_all(client, (const uint8_t *)header, (size_t)n);
    send_all(client, png.data, png.size);
    free(png.data);
}

int main(void) {
#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        fprintf(stderr, "WSAStartup failed\n");
        return 1;
    }
#endif

    socket_t server = socket(AF_INET, SOCK_STREAM, 0);
    if (server == INVALID_SOCKET) {
        fprintf(stderr, "socket failed\n");
        return 1;
    }

    int opt = 1;
    setsockopt(server, SOL_SOCKET, SO_REUSEADDR, (const char *)&opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(SERVER_PORT);

    if (bind(server, (struct sockaddr *)&addr, sizeof(addr)) == SOCKET_ERROR) {
        fprintf(stderr, "bind failed on port %d\n", SERVER_PORT);
        CLOSESOCKET(server);
        return 1;
    }

    if (listen(server, 8) == SOCKET_ERROR) {
        fprintf(stderr, "listen failed\n");
        CLOSESOCKET(server);
        return 1;
    }

    fprintf(stderr, "three-body render engine listening on http://127.0.0.1:%d/\n", SERVER_PORT);
    fprintf(stderr, "params: y2, vx2, vy2, vx3, vy3\n");

    for (;;) {
        socket_t client = accept(server, NULL, NULL);
        if (client == INVALID_SOCKET) continue;
        handle_client(client);
        CLOSESOCKET(client);
    }

    CLOSESOCKET(server);
#ifdef _WIN32
    WSACleanup();
#endif
    return 0;
}
