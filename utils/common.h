#ifndef _RKNN_MODEL_ZOO_COMMON_H_
#define _RKNN_MODEL_ZOO_COMMON_H_

/**
 * @brief Image pixel format
 * 
 */
typedef enum {
    IMAGE_FORMAT_GRAY8,
    IMAGE_FORMAT_RGB888,
    IMAGE_FORMAT_RGBA8888,
    IMAGE_FORMAT_YUV420SP_NV21,
    IMAGE_FORMAT_YUV420SP_NV12,
} image_format_t;

/**
 * @brief Image buffer
 * 
 */
typedef struct {
    int width;
    int height;
    int width_stride;
    int height_stride;
    image_format_t format;
    unsigned char* virt_addr;
    int size;
    int fd;
} image_buffer_t;

/**
 * @brief Image rectangle
 * 
 */
typedef struct {
    int left;
    int top;
    int right;
    int bottom;
} image_rect_t;

typedef struct single_pixel_camera_coordinates{
    float X;
    float Y;
    float Z;

}single_pixel_camera_coordinates;


/**
 * @brief Image obb rectangle
 * 
 */
typedef struct {
    int x;
    int y;
    int w;
    int h;
    float angle;
} image_obb_box_t;

typedef struct box_camera_coordinates{
    single_pixel_camera_coordinates left_top;
    single_pixel_camera_coordinates right_top;
    single_pixel_camera_coordinates right_bottom;
    single_pixel_camera_coordinates left_bottom;

    single_pixel_camera_coordinates *add_edge_point_single_pixel_camera_coordinates;
    int add_edge_point_num;
    
}box_camera_coordinates;


#endif //_RKNN_MODEL_ZOO_COMMON_H_
