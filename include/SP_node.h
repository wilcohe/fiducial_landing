 #ifdef __cplusplus
 #define EXTERNC extern "C"
 #else
 #define EXTERNC
 #endif

typedef struct traj_type{
	double* pos;  
	double* vel;  
	double* quat; 
	double* rates; 
} traj_type; 

 EXTERNC traj_type send_traj(void);


 #undef EXTERNC