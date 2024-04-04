

#if defined(NATIVE_CANFD_AVAILABLE)

#define PT_CAN_TORQ 0xA8
#define PT_CAN_TORQ2 0xA9
#define PT_CAN_THRTL 0xAA

// The following CAN IDs were in the MDF log file and matched in the DBC:
// - 0x80 --> SYNC 
// - 0xA8 --> EngineAndBrake 
// - 0xA9 --> Torque2 
// - 0xAA --> AccPedal 
// - 0xAC --> WheelTorqueDrivetrain2 
// - 0xB1 --> Torque_request_steering 
// - 0xB4 --> WheelTorqueDriveTrain1 
// - 0xB5 --> Torque_request_EGS 
// - 0xB6 --> DynamicCruiseControlTorqueDemand 
// - 0xB8 --> TorqueTransmisionRequest 
// - 0xBA --> TransmissionData 
// - 0xBE --> Alive_Counter 
// - 0xBF --> RequestedWheelTorqueDriveTrain 
// - 0xC0 --> Alive_Central_Gateway 
// - 0xC1 --> Alive_counter_telephone 
// - 0xC4 --> SteeringWheelAngle 
// - 0xC8 --> SteeringWheelAngle_slow 
// todo: simulate for now. need to connect abs sensors to separate board
// - 0xCE --> WheelSpeeds 
// - 0xE1 --> Wheel_torque_brake 
// - 0xF7 --> lateral_dynamics_ARS_VDM 
// todo: simulate? (outside of ECU)
// - 0xE2 --> Status_central_locking_BFT 
// - 0xF2 --> Status_central_locking_HK 
// todo: why would window be in PT CAN?
// - 0xFA --> Control_window_lifter_FAT 
// - 0xFC --> Control_window_lifter_FATH 

// - 0xA8 --> TORQ_0A8 
// - 0xAA --> THRTL_0AA 
// - 0xB8 --> DSC_0C0 
// - 0xC4 --> STEER_0C4 
// - 0xC8 --> STEER_0C8 
// - 0xCE --> WLSPD_0CE 
// - 0xE2 --> DOOR_FL_0E2 
// - 0xF2 --> BOOT_0F2 
// - 0xFA --> WIND_FL_0FA 

// The following CAN IDs were in the MDF log file, but not matched in the DBC:
// - 0x2
// - 0xA
// - 0x10
// - 0x11
// - 0x12
// - 0x16
// - 0x17
// - 0x18
// - 0x1A
// - 0x1C
// - 0x26
// - 0x29
// - 0x30
// - 0x32
// - 0x35
// - 0x37
// - 0x3A
// - 0x42
// - 0x48
// - 0x4A
// - 0x4C
// - 0x4E
// - 0x4F
// - 0x52
// - 0x5E
// - 0x74
// - 0x75
// - 0x78
// - 0x7A
// - 0x7D
// - 0x7E
// - 0x81
// - 0x83
// - 0x88
// - 0x8B
// - 0x92
// - 0x93
// - 0x94
// - 0x95
// - 0x97
// - 0x98
// - 0x9E
// - 0xA0
// - 0xA2
// - 0xA6
// - 0xB2
// - 0xB3
// - 0xB9
// - 0xCC
// - 0xCF
// - 0xD0
// - 0xD2
// - 0xD6
// - 0xDE
// - 0xE0
// - 0xED
// - 0xEF
// - 0xF0
// - 0xF1
// - 0xF3
// - 0xF6
// - 0xF8

#endif