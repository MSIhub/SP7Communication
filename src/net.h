#ifndef NET_H_
#define NET_H_
#include <cstdint>


#ifdef __GNUC__
#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

#ifdef _MSC_VER
#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop))
#endif
//This packing of struct ensures that the structure memory padding is minimum


namespace sp7 
{
    class NetState {
    public:
        enum class NetMasterMode 
        {
            UNKNOWN = 0,
            ZEROING,
            OPERATING,
        };

        enum class NetSystems 
        {
            COMM = 0,
            MOTION_CONTROL,
            AXIS_1,
            AXIS_2,
            AXIS_3,
            AXIS_4,
            AXIS_5,
            AXIS_6,
            AXIS_7,
        };

        static const uint32_t MAGIC_WORD = 0x53535441;
    };

    class NetCmd 
    {
    public:
        static const uint32_t MAGIC_WORD = 0x41545353;

        PACK(typedef struct {
            uint32_t magic;
            uint32_t packet_id;
            uint16_t command;
            union 
            {
                PACK(struct{
                    int32_t positions[7];
                    int32_t velocities[7];
                } joint_target_data);

                PACK(struct {
                    int32_t positions[6];
                    int32_t velocities[6];
                } ee_target_data);
                PACK(struct {
                    uint16_t orig_command;
                    int32_t code;
                } cmd_reply);
                PACK(struct {
                    uint16_t sys_state;
                    uint16_t motion_state;
                    uint16_t drive_state[7];
                    int32_t jpos[7];
                    int32_t jvel[7];
                    int8_t ik_state;
                    int8_t fk_state;
                    int32_t jposref[7];
                    int32_t jvelref[7];
                    int32_t positions[6];
                    int32_t velocities[6];
                } motion_data);
            };
        } remote_command_t);

        enum class NetCommands : uint16_t 
        {
            ALIVE = 0,
            CONNECT,
            DISCONNECT,
            START,
            STOP,
            ZERO,
            JOINT_TARGETS,
            EE_TARGETS,
            RESET = 0xeffe,
            HALT = 0xefff,
            MOTION_DATA = 0xfffe,
            CMD_REPLY = 0xffff
        };

        enum class NetReplyCode : int32_t 
        {
            REPLY_OK = 0,
            REPLY_UNABLE = -1,
            REPLY_FAILED = -2,
        };
    };
}
#endif /* NET_H_ */
