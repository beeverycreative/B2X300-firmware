//DR - As trinamics use 16 microsteps like the A4988 we need to make this configuration

// Trinamic stepper drivers defines
#ifdef BEEVC_TMC2208ext
	#define BEEVC_A4988ext
	#define BEEVC_ReverseE
#endif
#ifdef BEEVC_TMC2208all
	#define BEEVC_A4988all
	#define BEEVC_ReverseE
	#define BEEVC_ReverseX
	#define BEEVC_ReverseY
	#define BEEVC_ReverseZ
#endif

// B2X300 defines
#ifdef BEEVC_B2X300
	#define BEEVC_A4988all
	#define BEEVC_ReverseX
	#define BEEVC_ReverseY
	#define BEEVC_Trapezoidal
	#define BEEVC_Extendedbed
	#define BEEVC_Autolevel
	#define BEEVC_Bowden
	#define BEEVC_MKS_MINI_12864
	#define BEEVC_Restore
	#define FILAMENT_RUNOUT_SENSOR
  #define FILAMENT_RUNOUT_DUAL
#endif

// Power Restore defines
#ifdef BEEVC_Restore

	#ifdef BEEVC_Trapezoidal
		#define BEEVC_Restore_LiftZ 2000
	#else
		#define BEEVC_Restore_LiftZ 600
	#endif

	#define SERIAL_DEBUG
#endif
