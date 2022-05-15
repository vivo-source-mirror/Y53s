#include "vts_core.h"

#define PARSE_INT(buf, val) do { \
		char *token; \
		\
		token = strsep(&buf, ","); \
		if (!token) \
			return ERR_PTR(-EINVAL); \
		\
		if(kstrtoint(token, 0, val)) \
			return ERR_PTR(-EINVAL); \
	} while (0)

static char* vts_get_rejection_zone(struct vts_rejection_data *buf, char *cmd) {
	PARSE_INT(cmd, &(buf->type));
	PARSE_INT(cmd, &(buf->mode));
	PARSE_INT(cmd, &(buf->block));
	PARSE_INT(cmd, &(buf->x1));
	PARSE_INT(cmd, &(buf->y1));
	PARSE_INT(cmd, &(buf->x2));
	PARSE_INT(cmd, &(buf->y2));
	return cmd;
}

static LIST_HEAD(vts_rejection_zone_config);
static void vts_get_lying_config_data(struct vts_rejection_config *config)
{   
    struct vts_rejection_config *src_config;
    list_for_each_entry(src_config, &vts_rejection_zone_config, list){
    if (src_config == NULL)
        return;
    else if (src_config->scene == SPECIAL_LYING_SCENE)// copy special edge config
            memcpy((config->data + (config->config_num - 1)), (src_config->data + (src_config->config_num -1)), sizeof(struct vts_rejection_data));
    }

    
}
static ssize_t vts_rejection_zone_config_set(struct vts_device *vtsdev, int scene, int config_num, struct vts_rejection_data *data, int from_dts)
{
	struct vts_rejection_config *config;
	if (false == vtsdev->module->upper_cfg && 0 == from_dts){
		VTE("xml cfg version lower, return");
		return 0;
	}
	list_for_each_entry(config, &vts_rejection_zone_config, list) {
		if (config == NULL) {
			break;
		} else if (config->scene == scene && config->config_num == config_num) {
			memcpy(config->data, data, sizeof(struct vts_rejection_data) * config_num);
			return 0;
		}
	}
	config = kzalloc(sizeof(struct vts_rejection_config), GFP_KERNEL);
	if (!config) {
		VTE("no mmeory!\n");
		return -ENOMEM;
	}
	config->scene = scene;
	config->config_num = config_num;
	config->data = kzalloc(sizeof(struct vts_rejection_data) * config_num, GFP_KERNEL);
	if (!config->data) {
		VTE("no mmeory!\n");
		kfree(config);
		return -ENOMEM;
	}
	memcpy(config->data, data, sizeof(struct vts_rejection_data) * config_num);
	
	INIT_LIST_HEAD(&config->list);
	list_add_tail(&config->list, &vts_rejection_zone_config);
	VTI("add new config list");
	return 0;
}

int vts_rejection_zone_config_get_from_dts(struct vts_device *vtsdev, char **tmp)
{
	char *tag;
	int scene = 0;
	int config_num = 0;
	int i;
	struct vts_rejection_data *rejection_zone = NULL;
	//config:2:5:0,0,0,10,0,10,0,0,0,1,30,200,100,30,1,0,0,45,0,45,0,1,0,1,65,300,65,300,256,0,0,50,0,0,0
	if (strstr(*tmp, "config")) {
		tag = strsep(tmp, ":");
		if (!tag) {
			VTE("tag is NULL\n");
			return -EINVAL;
		}
		tag = strsep(tmp, ":");
		if (!tag) {
			VTE("tag is NULL\n");
			return -EINVAL;
		}
		if (kstrtoint(tag, 10, &scene)) {
			VTE("invalid command format!\n");
			return -EINVAL;
		}
		tag = strsep(tmp, ":");
		if (!tag) {
			VTE("tag is NULL\n");
			return -EINVAL;
		}
		if (kstrtoint(tag, 10, &config_num)) {
			VTE("invalid command format!\n");
			return -EINVAL;
		}
		if (config_num > vtsdev->config_length)
			vtsdev->config_length = config_num;
		rejection_zone = kzalloc(sizeof(struct vts_rejection_data) * config_num, GFP_KERNEL);
		if (!rejection_zone) {
			VTE("no mmeory!\n");
			return -ENOMEM;
		}
		tag = strsep(tmp, ":");
		if (!tag) {
			VTE("tag is NULL\n");
			goto return_error;
		}
		VTI("DTS: scene = %d, config_num = %d", scene, config_num);
		for (i = 0; i < config_num; i++) {
			if (IS_ERR(tag = vts_get_rejection_zone(rejection_zone + i, tag))) {
				VTE("get rejection zone error\n");
				goto return_error;
			}
		}
		vts_rejection_zone_config_set(vtsdev, scene, config_num, rejection_zone, 1);
		goto return_ret;
	}
	return_ret:
		if(rejection_zone)
			kfree(rejection_zone);
		return 0;
	return_error:
		if(rejection_zone)
			kfree(rejection_zone);
		return -EINVAL;
}


/* get the mateched rejection data for ic */
struct vts_rejection_config* vts_rejection_zone_config_get(struct vts_device *vtsdev, int scene)
{
	struct vts_rejection_config *config;
	memset(vtsdev->rejection_config->data, 0, vtsdev->config_length);
	list_for_each_entry(config, &vts_rejection_zone_config, list) {
		if (config == NULL) {
			return NULL;
		} else if (config->scene == scene) {
			vtsdev->rejection_config->scene = config->scene;
			vtsdev->rejection_config->config_num = config->config_num;
			vtsdev->rejection_config->checksum = config->checksum;
			memcpy(vtsdev->rejection_config->data, config->data, sizeof(struct vts_rejection_data) * config->config_num);
			if (atomic_read(&vtsdev->special_lying_state)) {
				vts_get_lying_config_data(vtsdev->rejection_config);
			}
			return vtsdev->rejection_config;
		}
	}

	return NULL;
}
int vts_rejection_version_parse(struct vts_device *vtsdev, char **tmp)
{
    u32 support = 0;
    char *tag;
    int cur_ver = 0;
    int last_ver = 0;
    vts_property_get(vtsdev, VTS_PROPERTY_EDGE_REJECTION, &support);
    if (support != 1) {
        VTI("no support edge setting\n");
        return -EINVAL;
    }
    tag = strsep(tmp, ":");
    if (!tag) {
        VTE("tag is NULL\n");
        return -EINVAL;
    } 
    if(*tmp){
        if(kstrtoint(*tmp, 0 , &cur_ver)){
            VTE("kstrtoint cur_ver fail\n");
            return -EINVAL;
        }
        if(kstrtoint(vtsdev->module->edge_cfg_version, 0 , &last_ver)){
            VTE("kstrtoint last_ver fail\n");
            return -EINVAL;
        }
        VTI("last version :%d---cur version :%d", last_ver, cur_ver);
        if(cur_ver > last_ver){
            memset(vtsdev->module->edge_cfg_version, '\0', 256);
            strcpy(vtsdev->module->edge_cfg_version, *tmp);
            vtsdev->module->upper_cfg = true;
            VTI("edge_version: %s", *tmp);
        }
        
    }
    return 0;

}

int vts_rejection_zone_cmd_set(struct vts_device *vtsdev, char **tmp)
{
	u32 support = 0;
	char *tag;
	int scene = 0;
	int config_num = 0;
	int i;
	struct vts_rejection_data *rejection_zone = NULL;

	vts_property_get(vtsdev, VTS_PROPERTY_EDGE_REJECTION, &support);
	if (support != 1) {
		VTI("no support edge setting\n");
		return -EINVAL;
	}
	tag = strsep(tmp, ":");
	if (!tag) {
		VTE("tag is NULL\n");
		return -EINVAL;
	}
	if (strstr(*tmp, "scene")) {
		tag = strsep(tmp, ":");
		if (!tag) {
			VTE("tag is NULL\n");
			return -EINVAL;
		}
		tag = strsep(tmp, ":");
		if (!tag) {
			VTE("tag is NULL\n");
			return -EINVAL;
		}
		if (kstrtoint(tag, 10, &scene)) {
			VTE("invalid command format!\n");
			return -EINVAL;
		}
        if(scene == SPECIAL_NORMAL_SCENE)
            atomic_set(&vtsdev->special_lying_state, 0);
        if(scene == SPECIAL_LYING_SCENE)
            atomic_set(&vtsdev->special_lying_state, 1);
        
		vts_state_set(vtsdev, VTS_STA_REJECTION_ZONE_SCENE, scene);
		goto return_ret;
	}
	if (strstr(*tmp, "lying")) {
		tag = strsep(tmp, ":");
		if (!tag) {
			VTE("tag is NULL\n");
			return -EINVAL;
		}
		tag = strsep(tmp, ":");
		if (!tag) {
			VTE("tag is NULL\n");
			return -EINVAL;
		}
		if (kstrtoint(tag, 10, &scene)) {
			VTE("invalid command format!\n");
			return -EINVAL;
		}
		vts_state_set(vtsdev, VTS_STA_REJECTION_ZONE_SCENE_LYING, scene);
		goto return_ret;
	}
	if (strstr(*tmp, "config")) {
		tag = strsep(tmp, ":");
		if (!tag) {
			VTE("tag is NULL\n");
			return -EINVAL;
		}
		tag = strsep(tmp, ":");
		if (!tag) {
			VTE("tag is NULL\n");
			return -EINVAL;
		}
		if (kstrtoint(tag, 10, &scene)) {
			VTE("invalid command format!\n");
			return -EINVAL;
		}
		tag = strsep(tmp, ":");
		if (!tag) {
			VTE("tag is NULL\n");
			return -EINVAL;
		}
		if (kstrtoint(tag, 10, &config_num)) {
			VTE("invalid command format!\n");
			return -EINVAL;
		}
		rejection_zone = kzalloc(sizeof(struct vts_rejection_data) * config_num, GFP_KERNEL);
		if (!rejection_zone) {
			VTE("no mmeory!\n");
			return -ENOMEM;
		}
		tag = strsep(tmp, ":");
		if (!tag) {
			VTE("tag is NULL\n");
			goto return_error;
		}
		VTI("scene = %d, config_num = %d", scene, config_num);
		for (i = 0; i < config_num; i++) {
			if (IS_ERR(tag = vts_get_rejection_zone(rejection_zone + i, tag))) {
				VTE("get rejection zone error\n");
				goto return_error;
			}
		}
		vts_rejection_zone_config_set(vtsdev, scene, config_num, rejection_zone, 0);
		goto return_ret;
	}
return_ret:
	if(rejection_zone)
   		kfree(rejection_zone);
	return 0;

return_error:
	if(rejection_zone)
		kfree(rejection_zone);
	return -EINVAL;
}

