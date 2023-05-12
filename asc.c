#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#include "asc.h"

#define PROGRAM_NAME "asc"
#define HELP_MSG "Zoom out in the terminal to see all panels. "                                               \
                 "Press 'ctrl+c' to exit, 'ctrl+z' to pause/continue, 's' to step instructions when paused. " \
                 "You may also scroll with 'h', 'j', 'k', 'l' in the memory panel when paused."

#define KEY_CTRL_L 12

#define TOSTR_BUFF_SIZE 1048576
#define DEVS_PAD_WIDTH  45
#define DEVS_PAD_HEIGHT 56
#define DISP_PAD_WIDTH  132
#define DISP_PAD_HEIGHT 74
#define MEM_PAD_WIDTH   128
#define MEM_PAD_HEIGHT  1024
#define FOOT_PAD_HEIGHT 1

#define MEM_PAD_SCRL_X_SPD 1
#define MEM_PAD_SCRL_X_LIM ((int) (MEM_PAD_WIDTH - MEM_PAD_WIDTH * 0.65f))
#define MEM_PAD_SCRL_Y_SPD 3
#define MEM_PAD_SCRL_Y_LIM ((int) (MEM_PAD_HEIGHT - MEM_PAD_HEIGHT * 0.5009f))

static bool should_exit = false;
static bool paused = false;
static bool should_step = true;
static uint8_t last_scancode = 0;

static void signal_handler(int sig)
{
    if (sig == SIGINT || sig == SIGQUIT) {
        should_exit = true;
    } else if (sig == SIGTSTP) {
        paused = !paused;
        should_step = true;
        last_scancode = 0;
    }

    signal(SIGINT,  signal_handler);
    signal(SIGQUIT, signal_handler);
    signal(SIGTSTP, signal_handler);
}

int main(int argc, char *argv[])
{
    int status = EXIT_FAILURE;

    FILE *in_file = NULL;
    firmware_t frm = {
        .image    = NULL,
        .text_sec = NULL,
        .data_sec = NULL
    };

    WINDOW *dev_pad  = NULL;
    WINDOW *mem_pad  = NULL;
    WINDOW *disp_pad = NULL;
    WINDOW *foot_pad = NULL;

    signal(SIGINT,  signal_handler);
    signal(SIGQUIT, signal_handler);
    signal(SIGTSTP, signal_handler);

    if (argc < 2 || argc > 3 || (argc == 3 && (argv[1][0] != '-' || argv[1][1] != 'p'))) {
        fprintf(
            stderr,
            "Usage:\n"
                "\t%s [-p] <path to the compiled binary>\n",
            PROGRAM_NAME
        );
        goto end;
    }
    if (argc == 3) {
        paused = true;
        should_step = false;
    }

    const char *in_file_path = (const char *) argv[argc - 1];
    in_file = fopen(in_file_path, "rb");
    if (in_file == NULL) {
        perror("Failed to open the compiled binary file");
        goto end;
    }

    if (fseek(in_file, 0, SEEK_END) < 0) {
        perror("Failed to find the end of the compiled binary file");
        goto end;
    }

    long file_size = ftell(in_file);
    if (fseek(in_file, 0, SEEK_SET) < 0) {
        perror("Failed to find the beginning of the compiled binary file");
        goto end;
    }

    bin_fmt_header_t bin_fmt_header;
    if (file_size - (long) sizeof(bin_fmt_header) <= 0) {
        fputs("The size of the compiled binary file is incorrect\n", stderr);
        goto end;
    }
    file_size -= (long) sizeof(bin_fmt_header);

    if (fread(&bin_fmt_header, sizeof(bin_fmt_header), 1, in_file) != 1) {
        perror("Failed to read the header of the compiled binary file");
        goto end;
    }

    if (bin_fmt_header.magic != BIN_FMT_MAGIC) {
        fputs("The compiled binary file has an incorrect magic number\n", stderr);
        goto end;
    }

    size_t file_secs_size = bin_fmt_header.data_sec_size + bin_fmt_header.text_sec_size;
    if ((size_t) file_size < file_secs_size) {
        fputs("The size of the data and text sections specified in the file is bigger than the file itself\n", stderr);
        goto end;
    }

    if (file_secs_size > MAX_FIRMWARE_SIZE) {
        fputs("The size of the data and text sections of the compiled binary is too big\n", stderr);
        goto end;
    }

    frm.image_size = file_secs_size;
    frm.image = (uint8_t *) malloc(frm.image_size);
    if (frm.image == NULL) {
        perror("Not enough memory to load the text and data sections from the compiled binary file");
        goto end;
    }

    if (bin_fmt_header.text_sec_size % SECTION_STEP != 0) {
        fprintf(stderr, "The size of the text section is not a multiple of %d\n", SECTION_STEP);
        goto end;
    }

    if (bin_fmt_header.data_sec_size % SECTION_STEP != 0) {
        fprintf(stderr, "The size of the data section is not a multiple of %d\n", SECTION_STEP);
        goto end;
    }

    frm.text_sec = frm.image;
    frm.text_sec_size = bin_fmt_header.text_sec_size;

    frm.data_sec = frm.image + frm.text_sec_size;
    frm.data_sec_size = bin_fmt_header.data_sec_size;

    if (fread(frm.text_sec, frm.text_sec_size, 1, in_file) != 1) {
        perror("Failed to read the text section from the compiled binary file");
        goto end;
    }

    if (fread(frm.data_sec, frm.data_sec_size, 1, in_file) != 1) {
        perror("Failed to read the data section from the compiled binary file");
        goto end;
    }

    char tostr_buff[TOSTR_BUFF_SIZE + 1];
    tostr_buff[TOSTR_BUFF_SIZE] = '\0';

    if (!initscr()) {
        fprintf(stderr, "Failed to create the program's UI\n");
        goto end;
    }
    if (has_colors()) {
        start_color();
        use_default_colors();
        init_pair(1, COLOR_BLACK, COLOR_WHITE);
    }
    noecho();

    dev_pad = newpad(DEVS_PAD_HEIGHT, DEVS_PAD_WIDTH);
    if (!dev_pad) {
        fprintf(stderr, "Failed to create a UI panel for general device info.\n");
        goto end;
    }

    mem_pad = newpad(MEM_PAD_HEIGHT, MEM_PAD_WIDTH);
    if (!mem_pad) {
        fprintf(stderr, "Failed to create a UI panel for memory info.\n");
        goto end;
    }
    int mem_pad_y = 0;
    int mem_pad_x = 0;
    nodelay(mem_pad, true);
    keypad(mem_pad, true);

    disp_pad = newpad(DISP_PAD_HEIGHT, DISP_PAD_WIDTH);
    if (!disp_pad) {
        fprintf(stderr, "Failed to create a UI panel for display info.\n");
        goto end;
    }

    foot_pad = newpad(
        FOOT_PAD_HEIGHT,
        snprintf(
            tostr_buff, TOSTR_BUFF_SIZE,
            "Almurut I (Paused). " HELP_MSG
        )
    );
    if (!foot_pad) {
        fprintf(stderr, "Failed to create a UI panel for the footer.\n");
        goto end;
    }
    wattron(dev_pad, A_BOLD);
    waddstr(foot_pad, tostr_buff);
    wattroff(dev_pad, A_BOLD);

    device_t dev;
    dev_init(&dev, &frm);

    while (!should_exit) {
        if (should_step) {
            dev_step(&dev, last_scancode);
            if (paused) {
                should_step = false;
            }
        }

        werase(dev_pad);
        wattron(dev_pad, A_BOLD);
        cpu_to_string(&dev.cpu, &dev.mem, tostr_buff, TOSTR_BUFF_SIZE);
        waddstr(dev_pad, tostr_buff); waddstr(dev_pad, "\n\n");
        pic_to_string(&dev.pic, tostr_buff, TOSTR_BUFF_SIZE);
        waddstr(dev_pad, tostr_buff); waddstr(dev_pad, "\n\n");
        pit_to_string(&dev.pit, tostr_buff, TOSTR_BUFF_SIZE);
        waddstr(dev_pad, tostr_buff); waddstr(dev_pad, "\n\n");
        kbdc_to_string(&dev.kbdc, tostr_buff, TOSTR_BUFF_SIZE);
        waddstr(dev_pad, tostr_buff); waddstr(dev_pad, "\n\n");
        dispc_to_string(&dev.dispc, tostr_buff, TOSTR_BUFF_SIZE);
        waddstr(dev_pad, tostr_buff);
        wattroff(dev_pad, A_BOLD);

        werase(disp_pad);
        dispc_front_buff_to_string(&dev.dispc, &dev.mem, tostr_buff, TOSTR_BUFF_SIZE);
        waddstr(disp_pad, tostr_buff);

        werase(mem_pad);
        mem_to_string(&dev.mem, tostr_buff, TOSTR_BUFF_SIZE);
        waddstr(mem_pad, tostr_buff);

        werase(foot_pad);
        if (paused) {
            waddstr(foot_pad, "Almurut I (Paused). " HELP_MSG);
        } else {
            waddstr(foot_pad, "Almurut I. " HELP_MSG);
        }

        int win_height, win_width;
        getmaxyx(stdscr, win_height, win_width);
        wnoutrefresh(stdscr);
        prefresh(
            dev_pad,
            0, 0,
            0, 0,
            win_height - FOOT_PAD_HEIGHT - 1, DEVS_PAD_WIDTH - 1
        );
        prefresh(
            disp_pad,
            0, 0,
            0, DEVS_PAD_WIDTH,
            win_height - FOOT_PAD_HEIGHT - 1, DEVS_PAD_WIDTH + DISP_PAD_WIDTH - 1
        );
        prefresh(
            mem_pad,
            mem_pad_y, mem_pad_x,
            0, DEVS_PAD_WIDTH + DISP_PAD_WIDTH,
            win_height - FOOT_PAD_HEIGHT - 1, win_width - 1
        );
        prefresh(
            foot_pad,
            0, 0,
            win_height - FOOT_PAD_HEIGHT, 0,
            win_height, win_width - 1
        );
        doupdate();

        int key = wgetch(mem_pad);
        if (key == KEY_RESIZE || key == KEY_CTRL_L) {
            clear();
            refresh();
        }
        if (key == KEY_RESIZE || key == KEY_MOUSE) {
            key = ERR;
        }
        if (!paused) {
            last_scancode = key == ERR ? 0 : (uint8_t) key;
        }

        switch (key) {
            case 's':
                if (paused) {
                    should_step = true;
                }
                break;
            case 'h':
                if (paused) {
                    mem_pad_x -= MEM_PAD_SCRL_X_SPD;
                }
                break;
            case 'j':
                if (paused) {
                    mem_pad_y += MEM_PAD_SCRL_Y_SPD;
                }
                break;
            case 'k':
                if (paused) {
                    mem_pad_y -= MEM_PAD_SCRL_Y_SPD;
                }
                break;
            case 'l':
                if (paused) {
                    mem_pad_x += MEM_PAD_SCRL_X_SPD;
                }
                break;
            case 'g':
                if (paused) {
                    mem_pad_y = 0;
                }
                break;
            case 'G':
                if (paused) {
                    mem_pad_y = MEM_PAD_SCRL_Y_LIM;
                }
                break;
            default:
                continue;
        }

        if (mem_pad_y < 0) {
            mem_pad_y = 0;
        } else if (mem_pad_y > MEM_PAD_SCRL_Y_LIM) {
            mem_pad_y = MEM_PAD_SCRL_Y_LIM;
        }
        if (mem_pad_x < 0) {
            mem_pad_x = 0;
        } else if (mem_pad_x > MEM_PAD_SCRL_X_LIM) {
            mem_pad_x = MEM_PAD_SCRL_X_LIM;
        }
    }

    status = EXIT_SUCCESS;

end:
    if (dev_pad  != NULL ||
        mem_pad  != NULL ||
        disp_pad != NULL ||
        foot_pad != NULL) {
        if (dev_pad != NULL) {
            delwin(dev_pad);
            dev_pad = NULL;
        }
        if (mem_pad != NULL) {
            delwin(mem_pad);
            mem_pad = NULL;
        }
        if (disp_pad != NULL) {
            delwin(disp_pad);
            disp_pad = NULL;
        }
        if (foot_pad != NULL) {
            delwin(foot_pad);
            foot_pad = NULL;
        }
        endwin();
    }

    if (frm.image != NULL) {
        free(frm.image);
        frm.image = NULL;
        frm.image_size = 0;

        frm.data_sec = NULL;
        frm.data_sec_size = 0;

        frm.text_sec = NULL;
        frm.text_sec_size = 0;
    }

    if (in_file != NULL) {
        fclose(in_file);
        in_file = NULL;
    }

    return status;
}
