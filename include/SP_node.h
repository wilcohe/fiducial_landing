 #ifdef __cplusplus
 #define EXTERNC extern "C"
 #else
 #define EXTERNC
 #endif

typedef traj_type struct {
	double[] pos;  
	double[] vel;  
	double[] quat; 
	double[] rates; 
} traj_type; 

 EXTERNC double[] send_traj(void);


 #undef EXTERNC