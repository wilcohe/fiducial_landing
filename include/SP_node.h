 #ifdef __cplusplus
 #define EXTERNC extern "C"
 #else
 #define EXTERNC
 #endif

typedef struct traj_type{
	double pos[3];  
	double vel[3];  
	double quat[4]; 
	double rates[3]; 
} traj_type; 

typdef struct pose{
	double pos[3]; 
	double quat[4]; 
} pose; 

 EXTERNC traj_type send_traj(void);

 EXTERNC pose send_pose(void); 

 EXTERNC bool send_detect(void); 


 #undef EXTERNC