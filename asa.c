#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "asc.h"

#define PROGRAM_NAME "asa"

#define MAX_LINE_LEN 4096
#define MAX_TOKEN_LEN 128
#define MAX_LABELS MAX_FIRMWARE_SIZE

typedef struct label_entry
{
    char label[MAX_TOKEN_LEN];
    size_t sec_pos;
    bool taken;
} label_entry_t;

static label_entry_t labels[MAX_LABELS];

static size_t hash_label(char *label)
{
    assert(label != NULL);

    size_t h = 0;
    for (char *p = label; *p; ++p) {
        h = 31 * h + *p;
    }
    return h % MAX_LABELS;
}

static bool record_label(char *label, size_t sec_pos)
{
    assert(label != NULL);

    size_t h = hash_label(label);

    for (size_t i = 0; i < MAX_LABELS; ++i) {
        size_t idx = (h + i) % MAX_LABELS;
        if (!labels[idx].taken || strncmp(labels[idx].label, label, MAX_TOKEN_LEN) == 0) {
            strncpy(labels[idx].label, label, MAX_TOKEN_LEN);
            labels[idx].sec_pos = sec_pos;
            labels[idx].taken = true;
            return true;
        }
    }

    return false;
}

static bool get_label(char *label, size_t *sec_pos)
{
    assert(label != NULL);
    assert(sec_pos != NULL);

    size_t h = hash_label(label);

    for (size_t i = 0; i < MAX_LABELS; ++i) {
        size_t idx = (h + i) % MAX_LABELS;
        if (!labels[idx].taken)
            break;
        if (strncmp(labels[idx].label, label, MAX_TOKEN_LEN) == 0) {
            *sec_pos = labels[idx].sec_pos;
            return true;
        }
    }

    return false;
}

typedef struct missing_label_entry
{
    char label[MAX_TOKEN_LEN];
    uint8_t *sec;
    size_t sec_pos;
    char line[MAX_LINE_LEN];
    size_t line_num;
} missing_label_entry_t;

static size_t missing_labels_count;
static missing_label_entry_t missing_labels[MAX_LABELS];

typedef enum command_type
{
    INVALID,
    BLANK,
    COMMENT,
    DIRECTIVE,
    LABEL,
    INSTRUCTION,
} command_type_t;

typedef struct command
{
    command_type_t type;
    char token[MAX_TOKEN_LEN];
    char arg1[MAX_TOKEN_LEN];
    char arg2[MAX_TOKEN_LEN];
} command_t;

static char* trim(char *str)
{
    assert(str != NULL);

    while (isspace((unsigned char) *str)) {
        ++str;
    }

    size_t len = strlen(str);
    while (len > 0 && isspace(str[len - 1])) {
        --len;
    }

    str[len] = '\0';

    return str;
}

static bool is_label(char *line)
{
    if (!*line) return false;

    while (*line && (isalnum(*line) || *line == '_')) {
        ++line;
    }

    return *line == ':';
}

static void parse_line(char *line, command_t *cmd)
{
    assert(line != NULL);
    assert(cmd != NULL);

    cmd->type = INVALID;
    cmd->token[0] = '\0';
    cmd->arg1[0]  = '\0';
    cmd->arg2[0]  = '\0';

    if (*line == '\0') {
        cmd->type = BLANK;
        return;
    }

    if (*line == '#') {
        cmd->type = COMMENT;
        return;
    }

    if (sscanf(line, ".%s %s %s", cmd->token, cmd->arg1, cmd->arg2) >= 2) {
        cmd->type = DIRECTIVE;
        return;
    }

    if (is_label(line) && sscanf(line, "%[^:]:", cmd->token) == 1) {
        cmd->type = LABEL;
        return;
    }

    if (sscanf(line, "%s %s %s", cmd->token, cmd->arg1, cmd->arg2) >= 1) {
        cmd->type = INSTRUCTION;
        return;
    }
}

static bool get_reg_code(char *reg, uint8_t *code)
{
    assert(reg != NULL);
    assert(code != NULL);

    if (strncmp(reg, REG_ENC_TABLE[CPU_REG_R0].name, MAX_TOKEN_LEN) == 0) {
        *code = REG_ENC_TABLE[CPU_REG_R0].code;
    } else if (strncmp(reg, REG_ENC_TABLE[CPU_REG_R1].name, MAX_TOKEN_LEN) == 0) {
        *code = REG_ENC_TABLE[CPU_REG_R1].code;
    } else if (strncmp(reg, REG_ENC_TABLE[CPU_REG_R2].name, MAX_TOKEN_LEN) == 0) {
        *code = REG_ENC_TABLE[CPU_REG_R2].code;
    } else if (strncmp(reg, REG_ENC_TABLE[CPU_REG_R3].name, MAX_TOKEN_LEN) == 0) {
        *code = REG_ENC_TABLE[CPU_REG_R3].code;
    } else if (strncmp(reg, REG_ENC_TABLE[CPU_REG_R4].name, MAX_TOKEN_LEN) == 0) {
        *code = REG_ENC_TABLE[CPU_REG_R4].code;
    } else if (strncmp(reg, REG_ENC_TABLE[CPU_REG_R5].name, MAX_TOKEN_LEN) == 0) {
        *code = REG_ENC_TABLE[CPU_REG_R5].code;
    } else if (strncmp(reg, REG_ENC_TABLE[CPU_REG_R6].name, MAX_TOKEN_LEN) == 0) {
        *code = REG_ENC_TABLE[CPU_REG_R6].code;
    } else if (strncmp(reg, REG_ENC_TABLE[CPU_REG_R7].name, MAX_TOKEN_LEN) == 0) {
        *code = REG_ENC_TABLE[CPU_REG_R7].code;
    } else if (strncmp(reg, REG_ENC_TABLE[CPU_REG_CS].name, MAX_TOKEN_LEN) == 0) {
        *code = REG_ENC_TABLE[CPU_REG_CS].code;
    } else if (strncmp(reg, REG_ENC_TABLE[CPU_REG_DS].name, MAX_TOKEN_LEN) == 0) {
        *code = REG_ENC_TABLE[CPU_REG_DS].code;
    } else if (strncmp(reg, REG_ENC_TABLE[CPU_REG_SS].name, MAX_TOKEN_LEN) == 0) {
        *code = REG_ENC_TABLE[CPU_REG_SS].code;
    } else if (strncmp(reg, REG_ENC_TABLE[CPU_REG_FP].name, MAX_TOKEN_LEN) == 0) {
        *code = REG_ENC_TABLE[CPU_REG_FP].code;
    } else if (strncmp(reg, REG_ENC_TABLE[CPU_REG_SP].name, MAX_TOKEN_LEN) == 0) {
        *code = REG_ENC_TABLE[CPU_REG_SP].code;
    } else if (strncmp(reg, REG_ENC_TABLE[CPU_REG_IP].name, MAX_TOKEN_LEN) == 0) {
        *code = REG_ENC_TABLE[CPU_REG_IP].code;
    } else if (strncmp(reg, REG_ENC_TABLE[CPU_REG_LS].name, MAX_TOKEN_LEN) == 0) {
        *code = REG_ENC_TABLE[CPU_REG_LS].code;
    } else if (strncmp(reg, REG_ENC_TABLE[CPU_REG_LR].name, MAX_TOKEN_LEN) == 0) {
        *code = REG_ENC_TABLE[CPU_REG_LR].code;
    } else if (strncmp(reg, REG_ENC_TABLE[CPU_REG_FLAGS].name, MAX_TOKEN_LEN) == 0) {
        *code = REG_ENC_TABLE[CPU_REG_FLAGS].code;
    } else {
        return false;
    }

    return true;
}

int main(int argc, char **argv)
{
    int status = EXIT_FAILURE;

    FILE *in_file  = NULL;
    FILE *out_file = NULL;

    firmware_t frm = {
        .image    = NULL,
        .text_sec = NULL,
        .data_sec = NULL
    };

    if (argc != 3) {
        fprintf(
            stderr,
            "Usage:\n"
                "\t%s <path to the assembly file> <path with a file name of the output file>\n",
            PROGRAM_NAME
        );
        goto end;
    }

    const char *in_file_path = (const char *) argv[1];
    in_file = fopen(in_file_path, "rb");
    if (in_file == NULL) {
        perror("Failed to open the assembly file");
        goto end;
    }

    const char *out_file_path = (const char *) argv[2];
    out_file = fopen(out_file_path, "wb");
    if (out_file == NULL) {
        perror("Failed to create the output file");
        goto end;
    }

    frm.image = (uint8_t *) malloc(MAX_FIRMWARE_SIZE);
    if (frm.image == NULL) {
        perror("Not enough memory to create internal data structures to process the assembly file");
        goto end;
    }

    size_t data_sec_pos = 0;
    uint8_t *data_sec = (uint8_t *) malloc(MAX_FIRMWARE_SIZE);
    if (data_sec == NULL) {
        perror("Not enough memory to create internal data structures for the text section of the assembly file");
        goto end;
    }

    size_t text_sec_pos = 0;
    uint8_t *text_sec = (uint8_t *) malloc(MAX_FIRMWARE_SIZE);
    if (text_sec == NULL) {
        perror("Not enough memory to create internal data structures for the data section of the assembly file");
        goto end;
    }

    uint8_t *sec = text_sec;
    size_t *sec_pos = &text_sec_pos;

    size_t line_num = 0;

    char input_buffer[MAX_LINE_LEN];
    while (fgets(input_buffer, sizeof(input_buffer), in_file)) {
        char *line = trim(input_buffer);
        ++line_num;

        command_t cmd;
        parse_line(line, &cmd);

        if (cmd.type == INVALID) {
            fprintf(stderr, "Invalid code at at %zu: %s\n", line_num, line);
            goto end;
        } else if (cmd.type == DIRECTIVE && strncmp(cmd.token, "section", sizeof("section") - 1) == 0) {
            if (strncmp(cmd.arg1, ".data", sizeof(".data") - 1) == 0) {
                sec = data_sec;
                sec_pos = &data_sec_pos;
            } else if (strncmp(cmd.arg1, ".text", sizeof(".text") - 1) == 0) {
                sec = text_sec;
                sec_pos = &text_sec_pos;
            } else {
                fprintf(stderr, "Invalid section name at line %zu: '%s'\n", line_num, line);
                goto end;
            }
        } else if (cmd.type == DIRECTIVE && strncmp(cmd.token, "byte", sizeof("byte") - 1) == 0) {
            char *end_ptr; int base = 16;
            errno = 0;
            unsigned int val = (unsigned int) strtoul(cmd.arg1, &end_ptr, base);
            if (errno != 0 || *end_ptr != '\0' || val > UINT8_MAX) {
                fprintf(stderr, "Invalid byte value at line %zu: '%s'\n", line_num, line);
                goto end;
            }
            unsigned int cnt = 1;
            if (strlen(cmd.arg2) > 0) {
                errno = 0;
                cnt = (unsigned int) strtoul(cmd.arg2, &end_ptr, base);
                if (errno != 0 || *end_ptr != '\0' || val > UINT8_MAX) {
                    fprintf(stderr, "Invalid count value at line %zu: '%s'\n", line_num, line);
                    goto end;
                }
            }
            for (size_t i = 0; i < (size_t) cnt; ++i) {
                if (*sec_pos >= MAX_FIRMWARE_SIZE) {
                    fprintf(stderr, "Maximum binary size has been reached at line %zu: '%s'\n", line_num, line);
                    goto end;
                }

                sec[(*sec_pos)++] = (uint8_t) val;
            }
        } else if (cmd.type == LABEL) {
            if (*sec_pos >= MAX_FIRMWARE_SIZE) {
                fprintf(stderr, "Maximum binary size has been reached at line %zu: '%s'\n", line_num, line);
                goto end;
            }

            if (!record_label(cmd.token, *sec_pos)) {
                fprintf(stderr, "Too many labels. The last one was found on line %zu: '%s'\n", line_num, line);
                goto end;
            }
        } else if (cmd.type == INSTRUCTION) {
            if (*sec_pos + 2 >= MAX_FIRMWARE_SIZE) {
                fprintf(stderr, "Maximum binary size has been reached at line %zu: '%s'\n", line_num, line);
                goto end;
            }

            if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_HLT].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_HLT].code;
                sec[(*sec_pos)++] = 0;
                sec[(*sec_pos)++] = 0;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_LR].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_LR].code;

                uint8_t reg_with_addr_of_src;
                if (!get_reg_code(cmd.arg1, &reg_with_addr_of_src)) {
                    fprintf(stderr, "Invalid register name for the first argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_with_addr_of_src;

                uint8_t reg_dst;
                if (!get_reg_code(cmd.arg2, &reg_dst)) {
                    fprintf(stderr, "Invalid register name for the second argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_dst;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_LI].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_LI].code;

                errno = 0; char *end_ptr; int base = 16;
                unsigned int val = (unsigned int) strtoul(cmd.arg1, &end_ptr, base);
                if (!(errno != 0 || *end_ptr != '\0')) {
                    if (val > UINT8_MAX) {
                        fprintf(stderr, "Invalid immediate value for the first argument on line %zu: '%s'\n", line_num, line);
                        goto end;
                    }

                    sec[(*sec_pos)++] = (uint8_t) val;
                } else {
                    size_t saved_sec_pos = 0;
                    if (!get_label(cmd.arg1, &saved_sec_pos)) {
                        missing_label_entry_t *missing_label = &missing_labels[missing_labels_count++];
                        strcpy(missing_label->label, cmd.arg1);
                        missing_label->sec = sec;
                        missing_label->sec_pos = *sec_pos;
                        strcpy(missing_label->line, line);
                        missing_label->line_num = line_num;
                    }
                    sec[(*sec_pos)++] = (uint8_t) saved_sec_pos;
                }

                uint8_t reg_dst;
                if (!get_reg_code(cmd.arg2, &reg_dst)) {
                    fprintf(stderr, "Invalid register name for the second argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_dst;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_MOV].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_MOV].code;

                uint8_t reg_src;
                if (!get_reg_code(cmd.arg1, &reg_src)) {
                    fprintf(stderr, "Invalid register name for the first argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_src;

                uint8_t reg_dst;
                if (!get_reg_code(cmd.arg2, &reg_dst)) {
                    fprintf(stderr, "Invalid register name for the second argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_dst;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_SR].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_SR].code;

                uint8_t reg_src;
                if (!get_reg_code(cmd.arg1, &reg_src)) {
                    fprintf(stderr, "Invalid register name for the first argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_src;

                uint8_t reg_with_addr_of_dst;
                if (!get_reg_code(cmd.arg2, &reg_with_addr_of_dst)) {
                    fprintf(stderr, "Invalid register name for the second argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_with_addr_of_dst;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_PUSH].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_PUSH].code;

                uint8_t reg_src;
                if (!get_reg_code(cmd.arg1, &reg_src)) {
                    fprintf(stderr, "Invalid register name for the first argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_src;
                sec[(*sec_pos)++] = 0;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_POP].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_POP].code;

                uint8_t reg_dst;
                if (!get_reg_code(cmd.arg1, &reg_dst)) {
                    fprintf(stderr, "Invalid register name for the first argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_dst;
                sec[(*sec_pos)++] = 0;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_ADD].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_ADD].code;

                uint8_t reg_src;
                if (!get_reg_code(cmd.arg1, &reg_src)) {
                    fprintf(stderr, "Invalid register name for the first argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_src;

                uint8_t reg_dst;
                if (!get_reg_code(cmd.arg2, &reg_dst)) {
                    fprintf(stderr, "Invalid register name for the second argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_dst;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_ADC].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_ADC].code;

                uint8_t reg_src;
                if (!get_reg_code(cmd.arg1, &reg_src)) {
                    fprintf(stderr, "Invalid register name for the first argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_src;

                uint8_t reg_dst;
                if (!get_reg_code(cmd.arg2, &reg_dst)) {
                    fprintf(stderr, "Invalid register name for the second argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_dst;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_SUB].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_SUB].code;

                uint8_t reg_src;
                if (!get_reg_code(cmd.arg1, &reg_src)) {
                    fprintf(stderr, "Invalid register name for the first argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_src;

                uint8_t reg_dst;
                if (!get_reg_code(cmd.arg2, &reg_dst)) {
                    fprintf(stderr, "Invalid register name for the second argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_dst;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_CMP].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_CMP].code;

                uint8_t reg_src;
                if (!get_reg_code(cmd.arg1, &reg_src)) {
                    fprintf(stderr, "Invalid register name for the first argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_src;

                uint8_t reg_dst;
                if (!get_reg_code(cmd.arg2, &reg_dst)) {
                    fprintf(stderr, "Invalid register name for the second argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_dst;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_SBB].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_SBB].code;

                uint8_t reg_src;
                if (!get_reg_code(cmd.arg1, &reg_src)) {
                    fprintf(stderr, "Invalid register name for the first argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_src;

                uint8_t reg_dst;
                if (!get_reg_code(cmd.arg2, &reg_dst)) {
                    fprintf(stderr, "Invalid register name for the second argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_dst;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_NOT].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_NOT].code;

                uint8_t reg_dst;
                if (!get_reg_code(cmd.arg1, &reg_dst)) {
                    fprintf(stderr, "Invalid register name for the first argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_dst;
                sec[(*sec_pos)++] = 0;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_AND].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_AND].code;

                uint8_t reg_src;
                if (!get_reg_code(cmd.arg1, &reg_src)) {
                    fprintf(stderr, "Invalid register name for the first argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_src;

                uint8_t reg_dst;
                if (!get_reg_code(cmd.arg2, &reg_dst)) {
                    fprintf(stderr, "Invalid register name for the second argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_dst;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_TEST].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_TEST].code;

                uint8_t reg_src;
                if (!get_reg_code(cmd.arg1, &reg_src)) {
                    fprintf(stderr, "Invalid register name for the first argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_src;

                uint8_t reg_dst;
                if (!get_reg_code(cmd.arg2, &reg_dst)) {
                    fprintf(stderr, "Invalid register name for the second argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_dst;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_OR].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_OR].code;

                uint8_t reg_src;
                if (!get_reg_code(cmd.arg1, &reg_src)) {
                    fprintf(stderr, "Invalid register name for the first argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_src;

                uint8_t reg_dst;
                if (!get_reg_code(cmd.arg2, &reg_dst)) {
                    fprintf(stderr, "Invalid register name for the second argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_dst;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_XOR].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_XOR].code;

                uint8_t reg_src;
                if (!get_reg_code(cmd.arg1, &reg_src)) {
                    fprintf(stderr, "Invalid register name for the first argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_src;

                uint8_t reg_dst;
                if (!get_reg_code(cmd.arg2, &reg_dst)) {
                    fprintf(stderr, "Invalid register name for the second argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_dst;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_SHL].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_SHL].code;

                uint8_t shift_in_reg;
                if (!get_reg_code(cmd.arg1, &shift_in_reg)) {
                    fprintf(stderr, "Invalid register name for the first argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) shift_in_reg;

                uint8_t reg_dst;
                if (!get_reg_code(cmd.arg2, &reg_dst)) {
                    fprintf(stderr, "Invalid register name for the second argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_dst;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_SHR].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_SHR].code;

                uint8_t shift_in_reg;
                if (!get_reg_code(cmd.arg1, &shift_in_reg)) {
                    fprintf(stderr, "Invalid register name for the first argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) shift_in_reg;

                uint8_t reg_dst;
                if (!get_reg_code(cmd.arg2, &reg_dst)) {
                    fprintf(stderr, "Invalid register name for the second argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_dst;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_SAR].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_SAR].code;

                uint8_t shift_in_reg;
                if (!get_reg_code(cmd.arg1, &shift_in_reg)) {
                    fprintf(stderr, "Invalid register name for the first argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) shift_in_reg;

                uint8_t reg_dst;
                if (!get_reg_code(cmd.arg2, &reg_dst)) {
                    fprintf(stderr, "Invalid register name for the second argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_dst;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_JMP].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_JMP].code;

                uint8_t reg_with_addr_of_dst;
                if (!get_reg_code(cmd.arg1, &reg_with_addr_of_dst)) {
                    fprintf(stderr, "Invalid register name for the first argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_with_addr_of_dst;
                sec[(*sec_pos)++] = 0;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_JE].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_JE].code;

                uint8_t reg_with_addr_of_dst;
                if (!get_reg_code(cmd.arg1, &reg_with_addr_of_dst)) {
                    fprintf(stderr, "Invalid register name for the first argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_with_addr_of_dst;
                sec[(*sec_pos)++] = 0;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_JNE].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_JNE].code;

                uint8_t reg_with_addr_of_dst;
                if (!get_reg_code(cmd.arg1, &reg_with_addr_of_dst)) {
                    fprintf(stderr, "Invalid register name for the first argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_with_addr_of_dst;
                sec[(*sec_pos)++] = 0;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_JG].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_JG].code;

                uint8_t reg_with_addr_of_dst;
                if (!get_reg_code(cmd.arg1, &reg_with_addr_of_dst)) {
                    fprintf(stderr, "Invalid register name for the first argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_with_addr_of_dst;
                sec[(*sec_pos)++] = 0;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_JGE].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_JGE].code;

                uint8_t reg_with_addr_of_dst;
                if (!get_reg_code(cmd.arg1, &reg_with_addr_of_dst)) {
                    fprintf(stderr, "Invalid register name for the first argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_with_addr_of_dst;
                sec[(*sec_pos)++] = 0;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_JL].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_JL].code;

                uint8_t reg_with_addr_of_dst;
                if (!get_reg_code(cmd.arg1, &reg_with_addr_of_dst)) {
                    fprintf(stderr, "Invalid register name for the first argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_with_addr_of_dst;
                sec[(*sec_pos)++] = 0;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_JLE].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_JLE].code;

                uint8_t reg_with_addr_of_dst;
                if (!get_reg_code(cmd.arg1, &reg_with_addr_of_dst)) {
                    fprintf(stderr, "Invalid register name for the first argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_with_addr_of_dst;
                sec[(*sec_pos)++] = 0;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_JA].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_JA].code;

                uint8_t reg_with_addr_of_dst;
                if (!get_reg_code(cmd.arg1, &reg_with_addr_of_dst)) {
                    fprintf(stderr, "Invalid register name for the first argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_with_addr_of_dst;
                sec[(*sec_pos)++] = 0;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_JAE].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_JAE].code;

                uint8_t reg_with_addr_of_dst;
                if (!get_reg_code(cmd.arg1, &reg_with_addr_of_dst)) {
                    fprintf(stderr, "Invalid register name for the first argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_with_addr_of_dst;
                sec[(*sec_pos)++] = 0;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_JB].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_JB].code;

                uint8_t reg_with_addr_of_dst;
                if (!get_reg_code(cmd.arg1, &reg_with_addr_of_dst)) {
                    fprintf(stderr, "Invalid register name for the first argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_with_addr_of_dst;
                sec[(*sec_pos)++] = 0;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_JBE].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_JBE].code;

                uint8_t reg_with_addr_of_dst;
                if (!get_reg_code(cmd.arg1, &reg_with_addr_of_dst)) {
                    fprintf(stderr, "Invalid register name for the first argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_with_addr_of_dst;
                sec[(*sec_pos)++] = 0;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_CALL].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_CALL].code;

                uint8_t reg_with_cs_of_dst;
                if (!get_reg_code(cmd.arg1, &reg_with_cs_of_dst)) {
                    fprintf(stderr, "Invalid register name for the first argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_with_cs_of_dst;

                uint8_t reg_with_addr_of_dst;
                if (!get_reg_code(cmd.arg2, &reg_with_addr_of_dst)) {
                    fprintf(stderr, "Invalid register name for the second argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_with_addr_of_dst;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_RET].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_RET].code;
                sec[(*sec_pos)++] = 0;
                sec[(*sec_pos)++] = 0;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_IRET].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_IRET].code;
                sec[(*sec_pos)++] = 0;
                sec[(*sec_pos)++] = 0;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_INB].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_INB].code;

                uint8_t src_port_in_reg;
                if (!get_reg_code(cmd.arg1, &src_port_in_reg)) {
                    fprintf(stderr, "Invalid register name for the first argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) src_port_in_reg;

                uint8_t reg_dst;
                if (!get_reg_code(cmd.arg2, &reg_dst)) {
                    fprintf(stderr, "Invalid register name for the second argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_dst;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_OUTB].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_OUTB].code;

                uint8_t reg_src;
                if (!get_reg_code(cmd.arg1, &reg_src)) {
                    fprintf(stderr, "Invalid register name for the first argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) reg_src;

                uint8_t dst_port_in_reg;
                if (!get_reg_code(cmd.arg2, &dst_port_in_reg)) {
                    fprintf(stderr, "Invalid register name for the second argument on line %zu: '%s'\n", line_num, line);
                    goto end;
                }
                sec[(*sec_pos)++] = (uint8_t) dst_port_in_reg;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_CLI].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_CLI].code;
                sec[(*sec_pos)++] = 0;
                sec[(*sec_pos)++] = 0;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_STI].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_STI].code;
                sec[(*sec_pos)++] = 0;
                sec[(*sec_pos)++] = 0;
            } else if (strncmp(cmd.token, MNEM_ENC_TABLE[CPU_MNEM_NOP].mnem, MAX_TOKEN_LEN) == 0) {
                sec[(*sec_pos)++] = MNEM_ENC_TABLE[CPU_MNEM_NOP].code;
                sec[(*sec_pos)++] = 0;
                sec[(*sec_pos)++] = 0;
            } else {
                fprintf(stderr, "Unknown instruction at line %zu: '%s'\n", line_num, line);
                goto end;
            }
        }
    }

    for (size_t i = 0; i < missing_labels_count; ++i) {
        missing_label_entry_t *missing_label = &missing_labels[i];
        size_t saved_sec_pos;
        if (!get_label(missing_label->label, &saved_sec_pos)) {
            fprintf(stderr, "Invalid label name for the first argument on line %zu: '%s'\n", missing_label->line_num, missing_label->line);
            goto end;
        }
        missing_label->sec[missing_label->sec_pos] = (uint8_t) saved_sec_pos;
    }

    size_t text_sec_size = text_sec_pos;
    size_t data_sec_size = data_sec_pos;
    size_t text_sec_alig = 0;
    size_t rem = text_sec_size % SECTION_STEP;
    if (rem != 0) text_sec_alig = SECTION_STEP - rem;
    size_t data_sec_alig = 0;
    rem = data_sec_size % SECTION_STEP;
    if (rem != 0) data_sec_alig = SECTION_STEP - rem;

    frm.image_size = text_sec_size + text_sec_alig + data_sec_size + data_sec_alig;
    if (frm.image_size > MAX_FIRMWARE_SIZE) {
        fprintf(stderr, "Firmware is too big: %zu bytes out of %d allowed\n", frm.image_size, MAX_FIRMWARE_SIZE);
        goto end;
    }

    frm.text_sec = frm.image;
    frm.text_sec_size = text_sec_size + text_sec_alig;
    memcpy(frm.text_sec, text_sec, text_sec_size);

    frm.data_sec = frm.image + frm.text_sec_size;
    frm.data_sec_size = data_sec_size + data_sec_alig;
    memcpy(frm.data_sec, data_sec, data_sec_size);

    bin_fmt_header_t bin_fmt_header = {
        .magic = BIN_FMT_MAGIC,
        .text_sec_size = (uint16_t) frm.text_sec_size,
        .data_sec_size = (uint16_t) frm.data_sec_size
    };

    if (fwrite(&bin_fmt_header, sizeof(bin_fmt_header_t), 1, out_file) != 1) {
        perror("Failed to write the header to the compiled binary file");
        goto end;
    }

    if (fwrite(frm.image, frm.image_size, 1, out_file) != 1) {
        perror("Failed to write the machine code to the compiled binary file");
        goto end;
    }

    status = EXIT_SUCCESS;

end:
    if (frm.image != NULL) {
        free(frm.image);
        frm.image = NULL;
        frm.image_size = 0;

        frm.data_sec = NULL;
        frm.data_sec_size = 0;

        frm.text_sec = NULL;
        frm.text_sec_size = 0;
    }

    if (out_file != NULL) {
        fclose(out_file);
        out_file = NULL;
    }

    if (in_file != NULL) {
        fclose(in_file);
        in_file = NULL;
    }

    return status;
}
