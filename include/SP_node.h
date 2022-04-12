 #ifdef __cplusplus
 #define EXTERNC extern "C"
 #else
 #define EXTERNC
 #endif

typedef struct traj_type{
	double pos[3];  
	double vel[3];  
	double quat[4]; // w x y z
	double rates[3]; 
} traj_type; 

typedef struct pose_type{
	double pos[3]; 
	double quat[4]; 
} pose_type; 

 EXTERNC traj_type send_traj(void);

 EXTERNC pose_type send_pose(void); 


 #undef EXTERNC