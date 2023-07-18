// C library headers
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include "imgui.h"
#include "implot.h"
#include "monitor.h"


template <typename T>
inline T RandomRange(T min, T max) {
    T scale = rand() / (T) RAND_MAX;
    return min + scale * ( max - min );
}

ImVec4 RandomColor() {
    ImVec4 col;
    col.x = RandomRange(0.0f,1.0f);
    col.y = RandomRange(0.0f,1.0f);
    col.z = RandomRange(0.0f,1.0f);
    col.w = 1.0f;
    return col;
}

// utility structure for realtime plot
struct ScrollingBuffer {
    char Idx;
    int Plt;
    char Name[32];
    int MaxSize;
    int Offset;
    ImVec4 Color;
    ImAxis Yax;
    ImVector<ImVec2> Data;
    ScrollingBuffer(int max_size = 2000) {
        Idx = 0;
        Plt = 0;
        Yax = ImAxis_Y1;
        Color = RandomColor();
        memset(Name, 0, sizeof(Name));
        MaxSize = max_size;
        Offset  = 0;
        Data.reserve(MaxSize);
    }
    void SetName(const char *name) {
        snprintf(Name, sizeof(Name), "%s", name);
    }
    void AddPoint(float x, float y) {
        if (Data.size() < MaxSize)
            Data.push_back(ImVec2(x,y));
        else {
            Data[Offset] = ImVec2(x,y);
            Offset =  (Offset + 1) % MaxSize;
        }
    }
    void Reset() {
        Plt = 0;
        Yax = ImAxis_Y1;
    }
    void Erase() {
        if (Data.size() > 0) {
            Data.shrink(0);
            Offset  = 0;
        }
    }
};

#define DATA_MAX_LEN    100
u_int8_t tx_data_buf[DATA_MAX_LEN];
size_t tx_data_len = 0;
char tx_data_hex[3*DATA_MAX_LEN];

u_int8_t rx_data_buf[DATA_MAX_LEN];
char rx_data_hex[3*DATA_MAX_LEN];
char last_line[DATA_MAX_LEN];
size_t rx_data_cnt = 0;
size_t rx_data_len = 0;
u_int16_t rx_data_chksum = 0;

bool full_line = false;
int serial_port = -1;
int data_file = -1;
ScrollingBuffer param_a;
ScrollingBuffer param_b;
ScrollingBuffer param_c;
ScrollingBuffer param_d;
float data_index = 0;
struct timespec t0, t1;

#define NUM_PARAMS 10
ScrollingBuffer params[NUM_PARAMS];

int SerialOpen(const char *port) {
    // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
    int fp = open(port, O_RDWR);
    if (fp < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
        return -1;
    }
    // Create new termios struct, we call it 'tty' for convention
    struct termios tty;

    // Read in existing settings, and handle any error
    if(tcgetattr(fp, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return -1;
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    //tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VTIME] = 0;    // No blocking, return immediately with what is available.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600
    //cfsetispeed(&tty, B9600);
    //cfsetospeed(&tty, B9600);
    if (cfsetispeed(&tty, B57600) != 0) {
        printf("Error %i from cfsetispeed: %s\n", errno, strerror(errno));
        return -1;
    }
    if (cfsetospeed(&tty, B57600) != 0) {
        printf("Error %i from cfsetospeed: %s\n", errno, strerror(errno));
        return -1;
    }

    // Save tty settings, also checking for error
    if (tcsetattr(fp, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return -1;
    }

    // sleep(2); //required to make flush work, for some reason
    tcflush(fp, TCIOFLUSH);

    return fp;
}

int SerialWrite(const int fp, const u_int8_t *write_buf, const size_t len) {
    // Write to serial port
    //unsigned char msg[] = { 'H', 'e', 'l', 'l', 'o', '\r' };
    int num_bytes = write(fp, write_buf, len);
    if (num_bytes < 0) {
        printf("write() error: %s\n", strerror(errno));
        return -1;
    }

    return num_bytes;
}

int SerialRead(const int fp, const char *read_buf, const size_t *len) {
    // Allocate memory for read buffer, set size according to your needs
    // char read_buf [256];

    // Normally you wouldn't do this memset() call, but since we will just receive
    // ASCII data for this example, we'll set everything to 0 so we can
    // call printf() easily.
    // memset(&read_buf, '\0', sizeof(read_buf));

    // Read bytes. The behaviour of read() (e.g. does it block?,
    // how long does it block for?) depends on the configuration
    // settings above, specifically VMIN and VTIME
    int num_bytes = read(fp, &read_buf, *len);
    printf("read() done!\n");

    // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
    if (num_bytes < 0) {
        printf("read() error: %s\n", strerror(errno));
        return -1;
    }
    if (num_bytes == 0) {
        printf("read() timeout..\n");
        return 0;
    }

    // read_buf[num_bytes] = '\0';
    // Here we assume we received ASCII data, but you might be sending raw bytes (in that case, don't try and
    // print it to the screen like this!)
    // printf("read(): [%d] '%s'\n", num_bytes, read_buf);
    printf("read(): [%d] ", num_bytes);
    for (int i = 0; i < num_bytes; i++) {
        printf("%c", read_buf[i]);
    }
    printf("\n");

    return num_bytes;
};

int SerialRead1(const int fp, u_int8_t *read_buf) {
    int num_bytes = read(fp, read_buf, 1);
    // printf("read() done!\n");

    // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
    if (num_bytes < 0) {
        printf("read() error: %s\n", strerror(errno));
        return -1;
    }
    if (num_bytes == 0) {
        // printf("read() timeout..\n");
        return 0;
    }

    // printf("read(): [%d] ", num_bytes);
    // for (int i = 0; i < num_bytes; i++) {
    //     printf("%c", read_buf[i]);
    // }
    // printf("\n");

    return num_bytes;
};

void SerialClose(const int fp) {
    close(fp);
};

int DataOpen(const char *filename) {
    int fp = open(filename, O_WRONLY | O_CREAT | O_APPEND, 0644);
    if (fp < 0) {
        printf("open() error: %s\n", strerror(errno));
        return -1;
    }

    return fp;
}

int DataWrite(const int fp, const char *buf, const size_t len) {
    int num_bytes = write(fp, buf, len);
    if (num_bytes < 0) {
        printf("write() error: %s\n", strerror(errno));
        return -1;
    }
    return num_bytes;
}

void DataClose(const int fp) {
    close(fp);
};

bool DataGetParam(const char param, const char *line, const size_t len, int *value) {
    for (size_t i = 0; i < len-1; i++) {
        if ((line[i] == param) && (line[i+1] == ':')) {
            int ret = sscanf(&line[i+2], "%d", value);
            if (ret == 1) {
                return true;
            } else {
                return false;
            }
        }
    }
    return false;
}

void SerialMonitorInit() {

    for (size_t i = 0; i < NUM_PARAMS; i++) {
        params[i].Idx = 'a'+i;
        sprintf(params[i].Name, "param %c", params[i].Idx);
    }
    params[0].SetName("Bat Cur");
    params[1].SetName("Speed");
    params[2].SetName("Voltage");
    params[3].SetName("KV");
    params[4].SetName("Iq");
    params[5].SetName("Id");
    params[6].SetName("Temp Iq");

    if (serial_port == -1) {
        serial_port = SerialOpen("/dev/ttyUSB0");
        // data_len = 0;
    }

    if (data_file == -1) {
        data_file = DataOpen("testfile.txt");
    }

    clock_gettime(CLOCK_MONOTONIC, &t0);
    clock_gettime(CLOCK_MONOTONIC, &t1);
}

void SerialMonitorDestroy() {
    SerialClose(serial_port);
    DataClose(data_file);
}

void Plots() {
    static float history = 60.0f;
    ImGui::SliderFloat("History", &history, 1, 120, "%.1f s");

    // child window to serve as initial source for our DND items
    ImGui::BeginChild("DND_LEFT",ImVec2(100,400));
    if (ImGui::Button("Erase Data")) {
        for (int k = 0; k < NUM_PARAMS; ++k)
            params[k].Erase();
    }
    if (ImGui::Button("Reset Plot")) {
        for (int k = 0; k < NUM_PARAMS; ++k)
            params[k].Reset();
    }
    for (int k = 0; k < NUM_PARAMS; ++k) {
        if (params[k].Plt > 0)
            continue;
        if (params[k].Data.size() == 0) {
            ImGui::BeginDisabled();
        }
        ImPlot::ItemIcon(params[k].Color); ImGui::SameLine();
        ImGui::Selectable(params[k].Name, false, 0, ImVec2(100, 0));
        if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_None)) {
            ImGui::SetDragDropPayload("MY_DND", &k, sizeof(int));
            ImPlot::ItemIcon(params[k].Color); ImGui::SameLine();
            ImGui::TextUnformatted(params[k].Name);
            ImGui::EndDragDropSource();
        }
        if (params[k].Data.size() == 0) {
            ImGui::EndDisabled();
        }
    }
    ImGui::EndChild();
    if (ImGui::BeginDragDropTarget()) {
        if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("MY_DND")) {
            int i = *(int*)payload->Data; params[i].Reset();
        }
        ImGui::EndDragDropTarget();
    }

    ImGui::SameLine();
    ImGui::BeginChild("DND_RIGHT",ImVec2(-1,400));
    // plot 1 (time series)
    ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels | ImPlotAxisFlags_NoGridLines | ImPlotAxisFlags_NoHighlight;
    if (ImPlot::BeginPlot("##DND1", ImVec2(-1,400))) {
        ImPlot::SetupAxisLimits(ImAxis_X1,data_index - history, data_index, ImGuiCond_Always);
        // ImPlot::SetupAxisLimits(ImAxis_Y1,0,1);
        ImPlot::SetupAxis(ImAxis_X1, "Elapsed [s]", flags|ImPlotAxisFlags_Lock);
        ImPlot::SetupAxis(ImAxis_Y1, "[drop here]", flags|ImPlotAxisFlags_AutoFit);
        ImPlot::SetupAxis(ImAxis_Y2, "[drop here]", flags|ImPlotAxisFlags_AutoFit|ImPlotAxisFlags_Opposite);
        // ImPlot::SetupAxis(ImAxis_Y3, "[drop here]", flags|ImPlotAxisFlags_Opposite);

        for (int k = 0; k < NUM_PARAMS; ++k) {
            if (params[k].Plt == 1 && params[k].Data.size() > 0) {
                ImPlot::SetAxis(params[k].Yax);
                ImPlot::SetNextLineStyle(params[k].Color);
                // ImPlot::PlotLine(params[k].Name, &params[k].Data[0].x, &params[k].Data[0].y, params[k].Data.size(), 0, 0, 2 * sizeof(float));

                // ImPlot::SetupAxisLimits(ImAxis_X1,data_index - history, data_index, ImGuiCond_Always);
                // ImPlot::SetupAxisLimits(ImAxis_Y1,0,1);
                // ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL,0.5f);
                // float fc = 0.01f;
                // ImPlot::PlotLineG("common",SinewaveGetter,&fc,1000);
                ScrollingBuffer *p = &params[k];
                // if (p->Data.size()) {
                    ImPlot::PlotLine(p->Name, &p->Data[0].x, &p->Data[0].y, p->Data.size(), 0, p->Offset, 2*sizeof(float));
                // }

                // allow legend item labels to be DND sources
                if (ImPlot::BeginDragDropSourceItem(params[k].Name)) {
                    ImGui::SetDragDropPayload("MY_DND", &k, sizeof(int));
                    ImPlot::ItemIcon(params[k].Color); ImGui::SameLine();
                    ImGui::TextUnformatted(params[k].Name);
                    ImPlot::EndDragDropSource();
                }
            }
        }
        // allow the main plot area to be a DND target
        if (ImPlot::BeginDragDropTargetPlot()) {
            if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("MY_DND")) {
                int i = *(int*)payload->Data; params[i].Plt = 1; params[i].Yax = ImAxis_Y1;
            }
            ImPlot::EndDragDropTarget();
        }
        // allow each y-axis to be a DND target
        // for (int y = ImAxis_Y1; y <= ImAxis_Y3; ++y) {
        for (int y = ImAxis_Y1; y <= ImAxis_Y2; ++y) {
            if (ImPlot::BeginDragDropTargetAxis(y)) {
                if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("MY_DND")) {
                    int i = *(int*)payload->Data; params[i].Plt = 1; params[i].Yax = y;
                }
                ImPlot::EndDragDropTarget();
            }
        }
        // allow the legend to be a DND target
        if (ImPlot::BeginDragDropTargetLegend()) {
            if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload("MY_DND")) {
                int i = *(int*)payload->Data; params[i].Plt = 1; params[i].Yax = ImAxis_Y1;
            }
            ImPlot::EndDragDropTarget();
        }
        ImPlot::EndPlot();
    }
    ImGui::EndChild();
}

void SerialMonitorWindow(bool* p_open) {
    ImGuiWindowFlags window_flags = 0;

    // Main body of the Demo window starts here.
    if (!ImGui::Begin("Serial Monitor", p_open, window_flags)) {
        // Early out if the window is collapsed, as an optimization.
        ImGui::End();
        return;
    }

    ImGui::Text("This is some useful text.");

    static bool send = false;

    static bool monitor = false;
    if (ImGui::Checkbox("Monitor", &monitor)) {
        tcflush(serial_port, TCIOFLUSH);
        send = true;
    }
    static int min_fw_speed = 20;
    if (ImGui::InputInt("Min FW speed", &min_fw_speed)) {
        send = true;
    }
    static int max_fw_speed = 50;
    if (ImGui::InputInt("max_fw_speed", &max_fw_speed)) {
        send = true;
    }

    if (serial_port > 0) {
        // send settings
        // clock_gettime(CLOCK_MONOTONIC, &t1);
        // if (((t1.tv_sec*1000000000+t1.tv_nsec) - (t0.tv_sec*1000000000+t0.tv_nsec)) > 1000000000/2) {
        //     snprintf(tx_data_buf, sizeof(tx_data_buf), "A:%d,B:%d\n", setting_1, setting_2);
        //     int num_write = SerialWrite(serial_port, tx_data_buf, strlen(tx_data_buf));
        //     clock_gettime(CLOCK_MONOTONIC, &t0);
        // }

        if (send) {
            unsigned o = 0;
            tx_data_buf[o++] = 0x3a;
            tx_data_buf[o++] = 0x1a;
            tx_data_buf[o++] = 0x99;
            tx_data_buf[o++] = 0x03;
            tx_data_buf[o++] = (u_int8_t)monitor & 0xFF;
            tx_data_buf[o++] = min_fw_speed & 0xFF;
            tx_data_buf[o++] = max_fw_speed & 0xFF;
            u_int16_t chksum = 0;
            for (unsigned i = 1; i < o; i++) {
                chksum += tx_data_buf[i];
            }
            tx_data_buf[o++] = chksum & 0xFF;
            tx_data_buf[o++] = (chksum >> 8) & 0xFF;
            tx_data_buf[o++] = 0x0d;
            tx_data_buf[o++] = 0x0a;
            tx_data_len = o;
            o = 0;
            for (unsigned i = 0; i < tx_data_len; i++) {
                o += sprintf(&tx_data_hex[o], "%02x ", tx_data_buf[i]);
            }
            tx_data_hex[o] = '\0';
            printf("TX data [%ld] : %s\n", tx_data_len, tx_data_hex);
            int num_write = SerialWrite(serial_port, tx_data_buf, tx_data_len);
            if (num_write == (int)tx_data_len) {
                send = false;
            }
        }

        // read data if any
        int num_read = 0;
        do {
            num_read = SerialRead1(serial_port, &rx_data_buf[rx_data_cnt]);
            // if (num_read > 0) {
            //     if (rx_data_buf[data_len] == '\n') {
            //         rx_data_buf[data_len] = '\0';
            //         data_len++;
            //         memcpy(last_line, rx_data_buf, data_len);
            //         full_line = true;
            //         last_line[data_len] = '\0';

            //         int value;
            //         DataWrite(data_file, last_line, strlen(last_line));
            //         for (size_t i = 0; i < NUM_PARAMS; i++) {
            //             if (params[i].Idx != 0) {
            //                 if (DataGetParam(params[i].Idx, last_line, data_len, &value)) {
            //                     params[i].AddPoint(data_index, value);
            //                 }
            //             }
            //         }

            //         data_index += .5;
            //         data_len = 0;
            //     } else {
            //         data_len++;
            //     }
            // }

            if (num_read > 0) {
                // TODO: Check for buffer overflow!!!!
                if (rx_data_cnt >= DATA_MAX_LEN) {
                    rx_data_cnt = 0;
                }
                if ((rx_data_cnt > 4) && (rx_data_buf[rx_data_cnt-1] == '\r') && (rx_data_buf[rx_data_cnt] == '\n')) {
                    rx_data_cnt++;
                    u_int16_t chksum = 0;
                    for (unsigned i = 1; i < rx_data_cnt - 4; i++) {
                        chksum += rx_data_buf[i];
                    }
                    u_int16_t rx_chksum = (rx_data_buf[rx_data_cnt-4]) | (rx_data_buf[rx_data_cnt-3] << 8);
                    if (rx_chksum != chksum) {
                        printf("chksum mismatch: %04x %04x\n", rx_chksum, chksum);
                    }
                    rx_data_chksum = chksum;
                    unsigned o = 0;
                    for (unsigned i = 0; i < rx_data_cnt; i++) {
                        o += sprintf(&rx_data_hex[o], "%02x ", rx_data_buf[i]);
                    }
                    rx_data_hex[o] = '\0';
                    printf("RX data [%ld] : %s\n", rx_data_cnt, rx_data_hex);

                    if (rx_data_buf[2] == 0x88) {
                        int value;
                        value = ((int)rx_data_buf[4] << 24) |
                                ((int)rx_data_buf[5] << 16) |
                                ((int)rx_data_buf[6] << 8) |
                                ((int)rx_data_buf[7]);
                        params[0].AddPoint(data_index, value);

                        value = ((int)rx_data_buf[8] << 24) |
                                ((int)rx_data_buf[9] << 16) |
                                ((int)rx_data_buf[10] << 8) |
                                ((int)rx_data_buf[11]);
                        params[1].AddPoint(data_index, value);

                        value = ((int)rx_data_buf[12] << 24) |
                                ((int)rx_data_buf[13] << 16) |
                                ((int)rx_data_buf[14] << 8) |
                                ((int)rx_data_buf[15]);
                        params[2].AddPoint(data_index, value);

                        value = ((int)rx_data_buf[16] << 24) |
                                ((int)rx_data_buf[17] << 16) |
                                ((int)rx_data_buf[18] << 8) |
                                ((int)rx_data_buf[19]);
                        params[3].AddPoint(data_index, value);

                        value = ((int)rx_data_buf[20] << 24) |
                                ((int)rx_data_buf[21] << 16) |
                                ((int)rx_data_buf[22] << 8) |
                                ((int)rx_data_buf[23]);
                        params[4].AddPoint(data_index, value);

                        value = ((int)rx_data_buf[24] << 24) |
                                ((int)rx_data_buf[25] << 16) |
                                ((int)rx_data_buf[26] << 8) |
                                ((int)rx_data_buf[27]);
                        params[5].AddPoint(data_index, value);

                        value = ((int)rx_data_buf[28] << 24) |
                                ((int)rx_data_buf[29] << 16) |
                                ((int)rx_data_buf[30] << 8) |
                                ((int)rx_data_buf[31]);
                        params[6].AddPoint(data_index, value);

                        data_index += .1;
                    }

                    rx_data_len = rx_data_cnt;
                    rx_data_cnt = 0;
                } else {
                    rx_data_cnt++;
                }
            }
        } while (num_read > 0);
    }

    ImGui::Text("TX DATA: [%3ld] '%s'", tx_data_len, tx_data_hex);
    ImGui::Text("RX DATA: [%3ld] '%s'", rx_data_len, rx_data_hex);
    ImGui::Text("RX CHKSUM: %04x", rx_data_chksum);
    // ImGui::Text("RX DATA: '%s'", rx_data_buf);
    // ImGui::Text("RX LINE: [%3ld] '%s'", rx_data_len, last_line);
    // ImGui::Text("LINE: '%s'", last_line);

/*
    static ScrollingBuffer sdata1, sdata2;
    ImVec2 mouse = ImGui::GetMousePos();
    static float t = 0;
    t += ImGui::GetIO().DeltaTime;
    if (full_line) {
        sdata1.AddPoint(t, mouse.x * 0.0005f);
        sdata2.AddPoint(t, mouse.y * 0.0005f);
        full_line = false;
    }
*/
    // static float history = 10.0f;
    // ImGui::SliderFloat("History",&history,1,30,"%.1f s");
    // static ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels;

/*
    ImGui::Text("data size: %d", sdata1.Data.size());
    if (sdata1.Data.size()) {
        if (ImPlot::BeginPlot("##Scrolling", ImVec2(-1,150))) {
            ImPlot::SetupAxes(nullptr, nullptr, flags, flags);
            ImPlot::SetupAxisLimits(ImAxis_X1,t - history, t, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1,0,1);
            ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL,0.5f);
            ImPlot::PlotShaded("Mouse X", &sdata1.Data[0].x, &sdata1.Data[0].y, sdata1.Data.size(), -INFINITY, 0, sdata1.Offset, 2 * sizeof(float));
            ImPlot::PlotLine("Mouse Y", &sdata2.Data[0].x, &sdata2.Data[0].y, sdata2.Data.size(), 0, sdata2.Offset, 2*sizeof(float));
            ImPlot::EndPlot();
        }
    }
*/

    // ImGui::Text("param data size: %d", param_a.Data.size());
    // if (param_a.Data.size()) {
/*
    if (data_index >= 1.0) {
        if (ImPlot::BeginPlot("##Scrolling2", ImVec2(-1,150))) {
            // ImPlot::SetupAxes(nullptr, nullptr, flags, flags);
            ImPlot::SetupAxisLimits(ImAxis_X1,data_index - history, data_index, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1,0,1);
            ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL,0.5f);
            // ImPlot::PlotLine("param A", &param_a.Data[0].x, &param_a.Data[0].y, param_a.Data.size(), 0, param_a.Offset, 2*sizeof(float));
            // ImPlot::PlotLine("param B", &param_b.Data[0].x, &param_b.Data[0].y, param_b.Data.size(), 0, param_b.Offset, 2*sizeof(float));
            for (size_t i = 0; i < NUM_PARAMS; i++) {
                // if (params[i].Idx != 0) {
                ScrollingBuffer *p = &params[i];
                if (p->Data.size()) {
                    ImPlot::PlotLine(p->Name, &p->Data[0].x, &p->Data[0].y, p->Data.size(), 0, p->Offset, 2*sizeof(float));
                }
            }

            ImPlot::EndPlot();
        }
    }
*/
/*
    static ImPlotSubplotFlags flags = ImPlotSubplotFlags_LinkRows | ImPlotSubplotFlags_LinkCols;
    ImGui::CheckboxFlags("ImPlotSubplotFlags_LinkRows", (unsigned int*)&flags, ImPlotSubplotFlags_LinkRows);
    ImGui::CheckboxFlags("ImPlotSubplotFlags_LinkCols", (unsigned int*)&flags, ImPlotSubplotFlags_LinkCols);
    ImGui::CheckboxFlags("ImPlotSubplotFlags_LinkAllX", (unsigned int*)&flags, ImPlotSubplotFlags_LinkAllX);
    ImGui::CheckboxFlags("ImPlotSubplotFlags_LinkAllY", (unsigned int*)&flags, ImPlotSubplotFlags_LinkAllY);

    // static int rows = NUM_PARAMS;
    static int rows = 0;
    for (size_t i = 0; i < NUM_PARAMS; i++) {
        if (params[i].Data.size()) {
            rows++;
        }
    }
    if (rows == 0) {
        rows = 1;
    }
    static int cols = 1;
    if (ImPlot::BeginSubplots("##AxisLinking", rows, cols, ImVec2(-1,800), flags)) {
        for (int i = 0; i < rows*cols; ++i) {
            if (ImPlot::BeginPlot("")) {
                // ImPlot::SetupAxesLimits(0,1000,-1,1);
                ImPlot::SetupAxisLimits(ImAxis_X1,data_index - history, data_index, ImGuiCond_Always);
                ImPlot::SetupAxisLimits(ImAxis_Y1,0,1);
                ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL,0.5f);
                // float fc = 0.01f;
                // ImPlot::PlotLineG("common",SinewaveGetter,&fc,1000);
                ScrollingBuffer *p = &params[i];
                if (p->Data.size()) {
                    ImPlot::PlotLine(p->Name, &p->Data[0].x, &p->Data[0].y, p->Data.size(), 0, p->Offset, 2*sizeof(float));
                }
                ImPlot::EndPlot();
            }
        }
        ImPlot::EndSubplots();
    }
    rows = 0;
*/
    Plots();

    ImGui::End();
}
