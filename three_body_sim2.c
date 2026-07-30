#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
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

#define SERVER_PORT        60002
#define G                  0.5
#define BODY_MASS          1250.0
#define DT                 0.25
#define SUBSTEPS           2
#define MAX_FRAMES         100000

#define DEFAULT_WIDTH      1000
#define DEFAULT_HEIGHT     1000
#define MAX_PIXELS         4000000

typedef enum {
    INTEGRATOR_LEGACY = 0,
    INTEGRATOR_VERLET = 1,
    INTEGRATOR_RK4 = 2
} IntegratorMode;

typedef enum {
    OUTPUT_PNG = 0,
    OUTPUT_DATA = 1
} OutputMode;

typedef struct {
    double y2;
    double vx2;
    double vy2;
    double vx3;
    double vy3;
    double xmin;
    double ymin;
    double xmax;
    double ymax;
    int width;
    int height;
    IntegratorMode integrator;
    OutputMode output;
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

static int has_escaped(const double bodies[3][4]) {
    double dx1 = bodies[2][0] - bodies[0][0];
    double dy1 = bodies[2][1] - bodies[0][1];
    double dx2 = bodies[2][0] - bodies[1][0];
    double dy2 = bodies[2][1] - bodies[1][1];
    double r1 = hypot(dx1, dy1);
    double r2 = hypot(dx2, dy2);
    if (r1 < 1.0) r1 = 1.0;
    if (r2 < 1.0) r2 = 1.0;

    double rel_vx = bodies[2][2] - 0.5 * (bodies[0][2] + bodies[1][2]);
    double rel_vy = bodies[2][3] - 0.5 * (bodies[0][3] + bodies[1][3]);
    double kinetic = 0.5 * (rel_vx * rel_vx + rel_vy * rel_vy);
    double potential = -G * (1.0 / r1 + 1.0 / r2);
    double energy = kinetic + potential;

    double com_x = 0.5 * (bodies[0][0] + bodies[1][0]);
    double com_y = 0.5 * (bodies[0][1] + bodies[1][1]);
    double rel_x = bodies[2][0] - com_x;
    double rel_y = bodies[2][1] - com_y;
    double radial_velocity = rel_x * rel_vx + rel_y * rel_vy;

    return energy > 0.0 && radial_velocity > 0.0;
}

static void accelerations_from_positions(const double pos[3][2], double acc[3][2]) {
    for (int i = 0; i < 3; ++i) {
        acc[i][0] = 0.0;
        acc[i][1] = 0.0;
        for (int j = 0; j < 3; ++j) {
            if (i == j) continue;
            double dx = pos[j][0] - pos[i][0];
            double dy = pos[j][1] - pos[i][1];
            double r = hypot(dx, dy);
            if (r < 1.0) r = 1.0;
            double scale = G * BODY_MASS / (r * r * r);
            acc[i][0] += scale * dx;
            acc[i][1] += scale * dy;
        }
    }
}

static void compute_forces_legacy(double bodies[3][4]) {
    double pos[3][2], acc[3][2];
    for (int i = 0; i < 3; ++i) {
        pos[i][0] = bodies[i][0];
        pos[i][1] = bodies[i][1];
    }
    accelerations_from_positions(pos, acc);
    for (int i = 0; i < 3; ++i) {
        bodies[i][2] += acc[i][0] * DT;
        bodies[i][3] += acc[i][1] * DT;
    }
}

static void update_positions_legacy(double bodies[3][4]) {
    for (int i = 0; i < 3; ++i) {
        bodies[i][0] += bodies[i][2] * DT;
        bodies[i][1] += bodies[i][3] * DT;
    }
}

static void compute_accelerations(const double double_bodies[3][4], double acc[3][2]) {
    double pos[3][2];
    for (int i = 0; i < 3; ++i) {
        pos[i][0] = double_bodies[i][0];
        pos[i][1] = double_bodies[i][1];
    }
    accelerations_from_positions(pos, acc);
}

static void velocity_verlet_step(double bodies[3][4], double dt) {
    double acc[3][2];
    double next_acc[3][2];
    compute_accelerations(bodies, acc);

    for (int i = 0; i < 3; ++i) {
        bodies[i][0] += bodies[i][2] * dt + 0.5 * acc[i][0] * dt * dt;
        bodies[i][1] += bodies[i][3] * dt + 0.5 * acc[i][1] * dt * dt;
    }

    compute_accelerations(bodies, next_acc);

    for (int i = 0; i < 3; ++i) {
        bodies[i][2] += 0.5 * (acc[i][0] + next_acc[i][0]) * dt;
        bodies[i][3] += 0.5 * (acc[i][1] + next_acc[i][1]) * dt;
    }
}

static void derivative(const double state[12], double dst[12]) {
    double pos[3][2], acc[3][2];
    for (int i = 0; i < 3; ++i) {
        pos[i][0] = state[i * 4 + 0];
        pos[i][1] = state[i * 4 + 1];
    }
    accelerations_from_positions(pos, acc);
    for (int i = 0; i < 3; ++i) {
        dst[i * 4 + 0] = state[i * 4 + 2];
        dst[i * 4 + 1] = state[i * 4 + 3];
        dst[i * 4 + 2] = acc[i][0];
        dst[i * 4 + 3] = acc[i][1];
    }
}

static void rk4_step(double bodies[3][4], double dt) {
    double y[12], k1[12], k2[12], k3[12], k4[12], tmp[12];
    for (int i = 0; i < 3; ++i) {
        y[i * 4 + 0] = bodies[i][0];
        y[i * 4 + 1] = bodies[i][1];
        y[i * 4 + 2] = bodies[i][2];
        y[i * 4 + 3] = bodies[i][3];
    }

    derivative(y, k1);
    for (int i = 0; i < 12; ++i) tmp[i] = y[i] + 0.5 * dt * k1[i];
    derivative(tmp, k2);
    for (int i = 0; i < 12; ++i) tmp[i] = y[i] + 0.5 * dt * k2[i];
    derivative(tmp, k3);
    for (int i = 0; i < 12; ++i) tmp[i] = y[i] + dt * k3[i];
    derivative(tmp, k4);

    for (int i = 0; i < 12; ++i) {
        y[i] += dt * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) / 6.0;
    }

    for (int i = 0; i < 3; ++i) {
        bodies[i][0] = y[i * 4 + 0];
        bodies[i][1] = y[i * 4 + 1];
        bodies[i][2] = y[i * 4 + 2];
        bodies[i][3] = y[i * 4 + 3];
    }
}

static int run_one_simulation(double x3, double y3, const SimParams *p) {
    double bodies[3][4] = {
        {0.0, 0.0, 0.0, 0.0},
        {0.1, p->y2, p->vx2, p->vy2},
        {x3, y3, p->vx3, p->vy3}
    };

    for (int frame = 0; frame < MAX_FRAMES; ++frame) {
        for (int s = 0; s < SUBSTEPS; ++s) {
            if (p->integrator == INTEGRATOR_RK4) {
                rk4_step(bodies, DT);
            } else if (p->integrator == INTEGRATOR_VERLET) {
                velocity_verlet_step(bodies, DT);
            } else {
                compute_forces_legacy(bodies);
                update_positions_legacy(bodies);
            }
        }
        if (has_escaped(bodies)) {
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

static int append_printf(Buffer *buf, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    int needed = vsnprintf(NULL, 0, fmt, ap);
    va_end(ap);
    if (needed < 0) return 0;

    size_t old_size = buf->size;
    uint8_t *next = (uint8_t *)realloc(buf->data, old_size + (size_t)needed + 1u);
    if (!next) return 0;
    buf->data = next;

    va_start(ap, fmt);
    vsnprintf((char *)buf->data + old_size, (size_t)needed + 1u, fmt, ap);
    va_end(ap);
    buf->size = old_size + (size_t)needed;
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

static int render_frames(const SimParams *params, int **out_frames) {
    int count = params->width * params->height;
    int *frames = (int *)malloc((size_t)count * sizeof(int));
    if (!frames) return 0;

    double x_step = (params->width <= 1) ? 0.0 : (params->xmax - params->xmin) / (double)(params->width - 1);
    double y_step = (params->height <= 1) ? 0.0 : (params->ymax - params->ymin) / (double)(params->height - 1);

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
    for (int row = 0; row < params->height; ++row) {
        double y3 = params->ymax - row * y_step;
        for (int col = 0; col < params->width; ++col) {
            double x3 = params->xmin + col * x_step;
            frames[row * params->width + col] = run_one_simulation(x3, y3, params);
        }
#ifndef _OPENMP
        if ((row + 1) % 25 == 0) {
            fprintf(stderr, "render progress: %d/%d rows\n", row + 1, params->height);
        }
#endif
    }

    *out_frames = frames;
    return 1;
}

static int render_png(const SimParams *params, Buffer *png) {
    int *frames = NULL;
    int count = params->width * params->height;
    uint8_t *image = (uint8_t *)malloc((size_t)count);
    if (!image) return 0;
    if (!render_frames(params, &frames)) {
        free(image);
        return 0;
    }

    percentile_inverted_image(frames, image, count);
    int ok = make_png_gray8(image, params->width, params->height, png);

    free(frames);
    free(image);
    return ok;
}

static int render_data(const SimParams *params, Buffer *text) {
    int *frames = NULL;
    if (!render_frames(params, &frames)) return 0;

    double x_step = (params->width <= 1) ? 0.0 : (params->xmax - params->xmin) / (double)(params->width - 1);
    double y_step = (params->height <= 1) ? 0.0 : (params->ymax - params->ymin) / (double)(params->height - 1);

    const char *integrator_name = "legacy";
    if (params->integrator == INTEGRATOR_RK4) integrator_name = "rk4";
    else if (params->integrator == INTEGRATOR_VERLET) integrator_name = "verlet";

    int ok = append_printf(text, "# three-body simulation result\n") &&
             append_printf(text, "# format: x/y=survived_frames\n") &&
             append_printf(text, "# range: xmin=%g ymin=%g xmax=%g ymax=%g\n",
                           params->xmin, params->ymin, params->xmax, params->ymax) &&
             append_printf(text, "# size: width=%d height=%d\n", params->width, params->height) &&
             append_printf(text, "# integrator: %s\n\n", integrator_name);

    for (int row = 0; ok && row < params->height; ++row) {
        double y3 = params->ymax - row * y_step;
        for (int col = 0; col < params->width; ++col) {
            double x3 = params->xmin + col * x_step;
            ok = append_printf(text, "%.17g/%.17g=%d\n", x3, y3, frames[row * params->width + col]);
            if (!ok) break;
        }
    }

    free(frames);
    return ok;
}

static const char *find_query_value(const char *request, const char *name) {
    char pattern[64];
    snprintf(pattern, sizeof(pattern), "%s=", name);
    const char *p = request;
    size_t len = strlen(pattern);
    while ((p = strstr(p, pattern)) != NULL) {
        if (p > request && (p[-1] == '?' || p[-1] == '&')) {
            return p + len;
        }
        p += len;
    }
    return NULL;
}

static int query_double(const char *request, const char *name, double *out) {
    const char *p = find_query_value(request, name);
    if (!p) return 0;
    char *end = NULL;
    double value = strtod(p, &end);
    if (end == p || !isfinite(value)) return 0;
    *out = value;
    return 1;
}

static int query_int(const char *request, const char *name, int *out) {
    double value;
    if (!query_double(request, name, &value)) return 0;
    if (value < 1.0 || value > 100000.0) return 0;
    *out = (int)value;
    return 1;
}

static int query_token_equals(const char *request, const char *name, const char *value) {
    const char *p = find_query_value(request, name);
    if (!p) return 0;
    size_t len = strlen(value);
    return strncmp(p, value, len) == 0 &&
           (p[len] == '&' || p[len] == ' ' || p[len] == '\r' || p[len] == '\n' || p[len] == '\0');
}

static SimParams default_params(void) {
    SimParams p;
    p.y2 = 100.1;
    p.vx2 = 0.0;
    p.vy2 = 20.0;
    p.vx3 = 1.0;
    p.vy3 = 0.0;
    p.xmin = -500.0;
    p.ymin = -499.0;
    p.xmax = 499.0;
    p.ymax = 500.0;
    p.width = DEFAULT_WIDTH;
    p.height = DEFAULT_HEIGHT;
    p.integrator = INTEGRATOR_VERLET;
    p.output = OUTPUT_PNG;
    return p;
}

static SimParams parse_params(const char *request) {
    SimParams p = default_params();

    query_double(request, "y2", &p.y2);
    query_double(request, "vx2", &p.vx2);
    query_double(request, "vy2", &p.vy2);
    query_double(request, "vx3", &p.vx3);
    query_double(request, "vy3", &p.vy3);

    query_double(request, "xmin", &p.xmin);
    query_double(request, "ymin", &p.ymin);
    query_double(request, "xmax", &p.xmax);
    query_double(request, "ymax", &p.ymax);
    query_int(request, "width", &p.width);
    query_int(request, "height", &p.height);

    if (query_token_equals(request, "integrator", "rk4")) p.integrator = INTEGRATOR_RK4;
    if (query_token_equals(request, "integrator", "verlet")) p.integrator = INTEGRATOR_VERLET;
    if (query_token_equals(request, "integrator", "legacy")) p.integrator = INTEGRATOR_LEGACY;
    if (query_token_equals(request, "output", "data")) p.output = OUTPUT_DATA;
    if (query_token_equals(request, "output", "png")) p.output = OUTPUT_PNG;

    return p;
}

static int validate_params(const SimParams *p, char *msg, size_t msg_size) {
    if (p->width < 1 || p->height < 1) {
        snprintf(msg, msg_size, "width and height must be positive\n");
        return 0;
    }
    if ((long long)p->width * (long long)p->height > MAX_PIXELS) {
        snprintf(msg, msg_size, "width*height exceeds MAX_PIXELS=%d\n", MAX_PIXELS);
        return 0;
    }
    if (!isfinite(p->xmin) || !isfinite(p->ymin) || !isfinite(p->xmax) || !isfinite(p->ymax) ||
        !isfinite(p->y2) || !isfinite(p->vx2) || !isfinite(p->vy2) || !isfinite(p->vx3) || !isfinite(p->vy3)) {
        snprintf(msg, msg_size, "all numeric parameters must be finite\n");
        return 0;
    }
    if (p->width > 1 && p->xmax == p->xmin) {
        snprintf(msg, msg_size, "xmax must differ from xmin when width > 1\n");
        return 0;
    }
    if (p->height > 1 && p->ymax == p->ymin) {
        snprintf(msg, msg_size, "ymax must differ from ymin when height > 1\n");
        return 0;
    }
    return 1;
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

static void send_buffer(socket_t client, const char *content_type, const char *filename, const Buffer *buf) {
    char header[768];
    int n = snprintf(header, sizeof(header),
                     "HTTP/1.1 200 OK\r\nContent-Type: %s\r\nContent-Length: %zu\r\nContent-Disposition: inline; filename=\"%s\"\r\nConnection: close\r\n\r\n",
                     content_type, buf->size, filename);
    send_all(client, (const uint8_t *)header, (size_t)n);
    send_all(client, buf->data, buf->size);
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
        send_text(client, 405,
                  "Use GET /?y2=100.1&vx2=0&vy2=20&vx3=1&vy3=0&xmin=-500&ymin=-499&xmax=499&ymax=500&width=1000&height=1000&integrator=legacy&output=png\n");
        return;
    }

    SimParams params = parse_params(request);
    char error[256];
    if (!validate_params(&params, error, sizeof(error))) {
        send_text(client, 400, error);
        return;
    }

    const char *integrator_name = "legacy";
    if (params.integrator == INTEGRATOR_RK4) integrator_name = "rk4";
    else if (params.integrator == INTEGRATOR_VERLET) integrator_name = "verlet";

    fprintf(stderr,
            "render y2=%g vx2=%g vy2=%g vx3=%g vy3=%g range=(%g,%g)-(%g,%g) size=%dx%d integrator=%s output=%s\n",
            params.y2, params.vx2, params.vy2, params.vx3, params.vy3,
            params.xmin, params.ymin, params.xmax, params.ymax,
            params.width, params.height,
            integrator_name,
            params.output == OUTPUT_DATA ? "data" : "png");

    Buffer out = {0};
    int ok = (params.output == OUTPUT_DATA) ? render_data(&params, &out) : render_png(&params, &out);
    if (!ok) {
        free(out.data);
        send_text(client, 500, "render failed\n");
        return;
    }

    if (params.output == OUTPUT_DATA) {
        send_buffer(client, "text/plain; charset=utf-8", "sim_result.txt", &out);
    } else {
        send_buffer(client, "image/png", "sim_output_percentile_inverted.png", &out);
    }
    free(out.data);
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

    fprintf(stderr, "three-body render engine v2 listening on http://127.0.0.1:%d/\n", SERVER_PORT);
    fprintf(stderr, "params: y2, vx2, vy2, vx3, vy3, xmin, ymin, xmax, ymax, width, height, integrator, output\n");

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
