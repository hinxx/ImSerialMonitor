// C library headers
#include <stdio.h>
#include <string.h>
#include <math.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include "imgui.h"
#include "implot.h"
#include "monitor.h"

// utility structure for realtime plot
struct ScrollingBuffer {
    char Id;
    char Name[32];
    int MaxSize;
    int Offset;
    ImVector<ImVec2> Data;
    ScrollingBuffer(int max_size = 2000) {
        Id = 0;
        memset(Name, 0, 32);
        MaxSize = max_size;
        Offset  = 0;
        Data.reserve(MaxSize);
    }
    void AddPoint(float x, float y) {
        if (Data.size() < MaxSize)
            Data.push_back(ImVec2(x,y));
        else {
            Data[Offset] = ImVec2(x,y);
            Offset =  (Offset + 1) % MaxSize;
        }
    }
    void Erase() {
        if (Data.size() > 0) {
            Data.shrink(0);
            Offset  = 0;
        }
    }
};

#define DATA_MAX_LEN    100
char data_buf[DATA_MAX_LEN];
char last_line[DATA_MAX_LEN];
size_t data_len = 0;
bool full_line = false;
int serial_port = -1;
int data_file = -1;
ScrollingBuffer param_a;
ScrollingBuffer param_b;
ScrollingBuffer param_c;
ScrollingBuffer param_d;
float data_index = 0;

#define NUM_PARAMS 10
ScrollingBuffer params[NUM_PARAMS];

int SerialOpen(const char *port) {
    // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
    int fp = open(port, O_RDWR);

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
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // Save tty settings, also checking for error
    if (tcsetattr(fp, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return -1;
    }

    return fp;
}

int SerialWrite(const int fp, const char *write_buf, const size_t len) {
    // Write to serial port
    unsigned char msg[] = { 'H', 'e', 'l', 'l', 'o', '\r' };
    int num_bytes = write(fp, msg, sizeof(msg));
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

int SerialRead1(const int fp, char *read_buf) {
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
        params[i].Id = 'a'+i;
        sprintf(params[i].Name, "param %c", params[i].Id);
    }

    if (serial_port == -1) {
        serial_port = SerialOpen("/dev/ttyUSB0");
        data_len = 0;
    }

    if (data_file == -1) {
        data_file = DataOpen("testfile.txt");
    }


}

void SerialMonitorDestroy() {
    SerialClose(serial_port);
    DataClose(data_file);
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
    if (serial_port > 0) {
        // int num_read = SerialRead(serial_port, data_buf, &data_len);
        int num_read = 0;
        do {
            num_read = SerialRead1(serial_port, &data_buf[data_len]);
            if (num_read > 0) {
                if (data_buf[data_len] == '\n') {
                    data_len++;
                    memcpy(last_line, data_buf, data_len);
                    full_line = true;
                    last_line[data_len] = '\0';

                    int value;
                    DataWrite(data_file, last_line, strlen(last_line));
                    // if (DataGetParam('a', last_line, data_len, &value)) {
                    //     param_a.AddPoint(data_index, value);
                    // }
                    // if (DataGetParam('b', last_line, data_len, &value)) {
                    //     param_b.AddPoint(data_index, value);
                    // }
                    for (size_t i = 0; i < NUM_PARAMS; i++) {
                        if (params[i].Id != 0) {
                            if (DataGetParam(params[i].Id, last_line, data_len, &value)) {
                                params[i].AddPoint(data_index, value);
                            }
                        }
                    }

                    data_index += .5;
                    data_len = 0;
                } else {
                    data_len += num_read;
                }
            }
        } while (num_read > 0);
    }

    ImGui::Text("DATA: %s", data_buf);
    ImGui::Text("LEN:  %ld", data_len);
    ImGui::Text("LINE: %s", last_line);

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
    static float history = 10.0f;
    ImGui::SliderFloat("History",&history,1,30,"%.1f s");
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
    if (data_index >= 1.0) {
        if (ImPlot::BeginPlot("##Scrolling2", ImVec2(-1,150))) {
            // ImPlot::SetupAxes(nullptr, nullptr, flags, flags);
            ImPlot::SetupAxisLimits(ImAxis_X1,data_index - history, data_index, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1,0,1);
            ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL,0.5f);
            // ImPlot::PlotLine("param A", &param_a.Data[0].x, &param_a.Data[0].y, param_a.Data.size(), 0, param_a.Offset, 2*sizeof(float));
            // ImPlot::PlotLine("param B", &param_b.Data[0].x, &param_b.Data[0].y, param_b.Data.size(), 0, param_b.Offset, 2*sizeof(float));
            for (size_t i = 0; i < NUM_PARAMS; i++) {
                // if (params[i].Id != 0) {
                ScrollingBuffer *p = &params[i];
                if (p->Data.size()) {
                    ImPlot::PlotLine(p->Name, &p->Data[0].x, &p->Data[0].y, p->Data.size(), 0, p->Offset, 2*sizeof(float));
                }
            }
            
            ImPlot::EndPlot();
        }
    }

    ImGui::End();
}