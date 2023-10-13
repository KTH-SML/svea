#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../arducam_config_parser.h"

void showHelp() {
	LOG("usage: ./toJson <input_name> <output_name>\n");
}

void printCameraParameter(FILE *file, CameraParam *cam_param) {
    const char *camera_param_pattern = 
    "\"camera_parameter\" : {       \n"
	"	\"CFG_MODE\": %d,           \n"
	"	\"TYPE\": \"%s\",           \n"
	"	\"SIZE\": [%d,%d],          \n"
	"	\"BIT_WIDTH\": %d,          \n"
	"	\"FORMAT\": [%d,%d],        \n"
	"	\"I2C_MODE\": %d,           \n"
	"	\"I2C_ADDR\": \"0x%02X\",   \n"
    "	\"TRANS_LVL\": %d           \n"
	"},                             \n";
    fprintf(file, camera_param_pattern,
        cam_param->cfg_mode,
        cam_param->type,
        cam_param->width, cam_param->height,
        cam_param->bit_width,
        cam_param->format >> 8, cam_param->format & 0xFF,
        cam_param->i2c_mode,
        cam_param->i2c_addr,
        cam_param->trans_lvl);
}

void buildStat(Config *config, int offset, int length, char *buff) {
    const char *comma = ", ";
    for (int n = offset; n < offset + length - 1; n++) {
        sprintf(buff + strlen(buff), "\"0x%02x\"", config->params[n]);
        strncat(buff, comma, strlen(comma));
    }
    sprintf(buff + strlen(buff), "\"0x%02x\"", config->params[offset + length - 1]);
}

void printBoardParameter(FILE *file, Config *configs, int configs_length, uint32_t board_type){
    const char *command_pattern_1 = "	[%s, [%s]]";
    const char *command_pattern_2 = ",\n	[%s, [%s]]";
    int frist = 1;
    for(int i = 0; i < configs_length; i++){
        Config config = configs[i];
        if(board_type == (config.type & 0xFFFF0000)){
            char command[1024] = { '\0' };
            char data[1024] = { '\0' };
            buildStat(&configs[i], 0, 4, command);
            buildStat(&configs[i], 4, configs[i].params[3], data);
            if(frist){
                fprintf(file, command_pattern_1, command, data);
                frist = 0;
            }else{
                fprintf(file, command_pattern_2 , command, data);
            }
        }
    }
}

void printBoardParameters(FILE *file, Config *configs, int configs_length) {
    fprintf(file, "\"board_parameter\" : [\n");
    printBoardParameter(file, configs, configs_length, SECTION_TYPE_BOARD);
    fprintf(file, "\n],\n");

    fprintf(file, "\"board_parameter_dev2\" : [\n");
    printBoardParameter(file, configs, configs_length, SECTION_TYPE_BOARD_2);
    fprintf(file, "\n],\n");

    fprintf(file, "\"board_parameter_dev3_inf2\" : [\n");
    printBoardParameter(file, configs, configs_length, SECTION_TYPE_BOARD_3_2);
    fprintf(file, "\n],\n");

    fprintf(file, "\"board_parameter_dev3_inf3\" : [\n");
    printBoardParameter(file, configs, configs_length, SECTION_TYPE_BOARD_3_3);
    fprintf(file, "\n],\n");
}


void printRegisterParameter(FILE *file, Config *configs, int configs_length, uint32_t reg_type){
    const char *reg_pattern_1 = "	[\"0x%04X\", \"0x%04X\"]";
    const char *reg_pattern_2 = ",\n	[\"0x%04X\", \"0x%04X\"]";
    int frist = 1;
    for(int i = 0; i < configs_length; i++){
        Config config = configs[i];
        if(reg_type == (config.type & 0xFFFF0000)){
            if(CONFIG_TYPE_DELAY == (config.type & 0xFFFF)){
                if(frist){
                    fprintf(file, "	[\"DELAY\", \"%d\"]", config.params[0]);
                    frist = 0;
                }else{
                    fprintf(file, ",\n	[\"DELAY\", \"%d\"]", config.params[0]);
                }
            } else {
                if(frist){
                    fprintf(file, reg_pattern_1, config.params[0], config.params[1]);
                    frist = 0;
                }else{
                    fprintf(file, reg_pattern_2, config.params[0], config.params[1]);
                }
            }
        }
    }
}

void printRegisterParameters(FILE *file, Config *configs, int configs_length){
    fprintf(file, "\"register_parameter\" : [\n");
    printRegisterParameter(file, configs, configs_length, SECTION_TYPE_REG);
    fprintf(file, "\n],\n");

    fprintf(file, "\"register_parameter_dev3_inf2\" : [\n");
    printRegisterParameter(file, configs, configs_length, SECTION_TYPE_REG_3_2);
    fprintf(file, "\n],\n");

    fprintf(file, "\"register_parameter_dev3_inf3\" : [\n");
    printRegisterParameter(file, configs, configs_length, SECTION_TYPE_REG_3_3);
    fprintf(file, "\n]\n");
}

void printConfigs(FILE *file, Config *configs, int configs_length) {
    printBoardParameters(file, configs, configs_length);
    printRegisterParameters(file, configs, configs_length);
}
void printJson(CameraConfigs *cam_cfgs, const char *output_name){
    FILE *output_file = fopen(output_name, "wb");
    
    CameraParam *cam_param = &cam_cfgs->camera_param;
	Config *configs = cam_cfgs->configs;
	int configs_length = cam_cfgs->configs_length;

    fprintf(output_file, "{\n");
    printCameraParameter(output_file, cam_param);
    printConfigs(output_file, configs, configs_length);
    fprintf(output_file, "}\n");
}


int main(int argc, char **argv){
    if(argc != 3){
        showHelp();
        exit(-1);
    }
    char *input_file_name = argv[1];
    char *output_file_name = argv[2];
    CameraConfigs cam_cfgs;
	memset(&cam_cfgs, 0x00, sizeof(CameraConfigs));
	if (arducam_parse_config(input_file_name, &cam_cfgs)) {
		LOG("Cannot find configuration file.\n");
        showHelp();
		return -1;
	}
	printJson(&cam_cfgs, output_file_name);
    return 0;
}