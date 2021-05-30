/*
Title:       camera.h
Description: Provide camera function prototypes and macros.
Authors:     Chris Abajian, Becky Reich
Date:        10/31/2020
*/

#if !defined(CAMERA_H_)
    #define CAMERA_H_   /**< Symbol preventing repeated inclusion */
    
    // Default System clock value
    // period = 1/20485760  = 4.8814395e-8
    #define DEFAULT_SYSTEM_CLOCK 20485760u  
    // Integration time (seconds)
    // Determines how high the camera values are
    // Don't exceed 100ms or the caps will saturate
    // Must be above 1.25 ms based on camera clk 
    //  (camera clk is the mod value set in FTM2)
    //#define INTEGRATION_TIME .0075f
    #define INTEGRATION_TIME .0025f // 400 reads/sec
    
    // camera FOV indices
    #define CAM_CENTER      60
    #define CAM_LOW         20
    #define CAM_HIGH        110
    
    // minimum average ADC value to detect a "track" (for off course detection)
    #define ADC_TRACK_THRESH 20000
    
    // debug camera (sent to matlab)
    #define DEBUG_CAM_RAW        0
    #define DEBUG_CAM_FILTERED   1
    #define DEBUG_CAM_DERIVATIVE 2

    void init_FTM2(void);
    void init_GPIO(void);
    void init_PIT(void);
    void init_ADC0(void);
    void FTM2_IRQHandler(void);
    void PIT1_IRQHandler(void);
    void ADC0_IRQHandler(void);
		void convolve(const int *x, const int *h, int *y, const int xSize, const int hSize);
    void send_line(void);
    void debug_camera(const int *data, short camera);
    void init_camera(void);

#endif  /* #if !defined(CAMERA_H_) */
 
/* uart.h, eof. */
