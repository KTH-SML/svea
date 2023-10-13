#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../arducam_config_parser.h"

void dump_camera_parameter(CameraParam *camera_param){
    printf("CFG_MODE    = %d\n", camera_param->cfg_mode);
    printf("TYPE        = %s\n", camera_param->type);
    printf("WIDTH       = %d\n", camera_param->width);
    printf("HEIGHT      = %d\n", camera_param->height);
    printf("BIT_WIDTH   = %d\n", camera_param->bit_width);
    printf("FORMAT      = 0x%02X\n", camera_param->format);
    printf("I2C_MODE    = %d\n", camera_param->i2c_mode);
    printf("I2C_ADDR    = %d\n", camera_param->i2c_addr);
    printf("TRANS_LVL   = %d\n", camera_param->trans_lvl);
    printf("\n");
}

void dump_controls(CameraConfigs *configs) {
    for (int i = 0; i < configs->controls_length; i++) {
        printf("[control parameter]\n");
        printf("MIN_VALUE   = %ld\n", configs->controls[i].min);
        printf("MAX_VALUE   = %ld\n", configs->controls[i].max);
        printf("STEP        = %d\n", configs->controls[i].step);
        printf("DEF         = %ld\n", configs->controls[i].def);
        printf("CTRL_NAME   = %s\n", configs->controls[i].name);
        printf("FUNC_NAME   = %s\n", configs->controls[i].func);
        printf("%s\n", configs->controls[i].code);
    }
}

void dump_configs(CameraConfigs *configs){
    uint32_t lastSection = 0;
    for (int i = 0; i < configs->configs_length; i++){
        if(lastSection != (configs->configs[i].type >> 16)){
            printf("[0x%04X]\n", (configs->configs[i].type >> 16));
            lastSection = configs->configs[i].type >> 16;
        }
        for(int j = 0; j < configs->configs[i].params_length; j++)
            printf("0x%04X ", configs->configs[i].params[j]);
        printf("\n");
    }
}

void showHelp() {
	printf("usage: ./dump <input_name>\n");
}

int main(int argc, char **argv){
    if(argc != 2){
        showHelp();
        exit(-1);
    }
    char *input_file_name = argv[1];
    CameraConfigs cam_cfgs;
	memset(&cam_cfgs, 0x00, sizeof(CameraConfigs));
	if (arducam_parse_config(input_file_name, &cam_cfgs)) {
		printf("Cannot find configuration file.\n");
        showHelp();
		return -1;
	}
	dump_camera_parameter(&cam_cfgs.camera_param);
    dump_controls(&cam_cfgs);
    dump_configs(&cam_cfgs);
    return 0;
}