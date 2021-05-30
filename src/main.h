#ifndef MAIN_H_
  #define MAIN_H_
	
	// debug macro
	//   0: send nothing
	//   1: send camera data over uart (raw, smoothed, derivative)
	//   2: send generic UART print statements
	#define DEBUG 0
	
	#define MODE_SLOW    0
	#define MODE_MEDIUM  1
	#define MODE_MEDIUM2 2
	#define MODE_FAST    3
	#define MODE_FAST2   4
	
	#define POS_START        -1
	#define POS_CENTER       0
  #define POS_CORNER       1
  #define POS_INTERSECTION 2
	
	void scoreboard(void);
  void control(int *line, int line_max_left, int line_max_right);
  void car_start_switch(void);
  void car_start_uart(void);
	
#endif
