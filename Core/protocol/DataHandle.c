// 谢胜
// 2020.1.4
// ICRA2020 串口通信协议
// 数据结构
/*************
typedef struct
{
  uint16_t sof;
  uint16_t ver_data_len;
  uint16_t crc_16;
  uint8_t cmd_set;
  uint8_t cmd_id;
  uint8_t pdata[];
  uint32_t crc32;
} protocol_pack_desc_t;
*************/

#include "DataHandle.h"
#include "string.h"

size_t HEADER_LEN = sizeof(Header);
//size_t CMD_SET_PREFIX_LEN = 2 * sizeof(uint8_t);
//size_t CRC_DATA_LEN = sizeof(uint32_t);
//size_t CRC_HEAD_LEN = sizeof(uint16_t);
//uint8_t SOF = 0xAA;

//// 校验数据，一帧数据中有两处CRC校验位，都校验通过后认为数据有效
//int VerifyData(uint8_t* data)
//{
//    Header *header_ptr = (Header*)data;
//    int is_frame = 0;
//    if(header_ptr->sof == SOF &&
//        CRCHeadCheck(data, HEADER_LEN) &&
//        CRCTailCheck(data, header_ptr->length))
//    {
//        is_frame = 1;
//    }
//    return is_frame;
//}

//// 通过校验后，根据数据中的cmd_set和cmd_id判断数据内容
//uint16_t data_len;
//void ContainerHandler(uint8_t cmd_set, uint8_t cmd_id, uint8_t* data, uint16_t len)
//{
//		data_len = len - HEADER_LEN - CMD_SET_PREFIX_LEN - CRC_DATA_LEN;
//    switch(cmd_set)
//    {
//        case CHASSIS_CMD_SET:
//            switch(cmd_id)
//            {
//								case CMD_SET_GIMBAL_CONTROL:
//								if(data_len == sizeof(cmd_gimbal_control))
//								{
//									infantry.gimbal_control_info = *(cmd_gimbal_control*)data;
//								}
//                default:;
//            }
//            break;
//        default:;
//    }
//}

//// 数据接收回调函数
//void RecvDataCallback(uint8_t *data, uint16_t len)
//{
//    if(VerifyData(data))
//    {
//        ContainerHandler(data[HEADER_LEN], data[HEADER_LEN+1], &data[HEADER_LEN+2], len);
//    }
//}

//// 发送之前调用这个函数进行数据打包
//// cmd_set 发送的消息对应的CMD_SET，如CMD_SET_CHASSIS_SPEED，具体查找protocol.h
//// cmd_id  发送的消息对应的CMD_ID，如CMD_PUSH_CHASSIS_INFO，具体查找protocol.h
//// p_data  要发送的数据，强制转换成uint8_t*类型传入
//// len     要发送的数据的长度
//// tx_buf  打包好的数据存放的地方，需提前定义好
//uint16_t DataPack(uint8_t cmd_set, uint8_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t *tx_buf)
//{
//    uint8_t cmd_set_prefix[2] = {cmd_set, cmd_id};
//    uint32_t crc_data;
//    uint16_t frame_length = HEADER_LEN + CMD_SET_PREFIX_LEN + len + CRC_DATA_LEN;
//    Header *header = (Header*)tx_buf;
//    header->sof = 0xAA;
//    header->length = frame_length;
//    header->crc = CRC16Calc(tx_buf, HEADER_LEN - CRC_HEAD_LEN);
//    memcpy(&tx_buf[HEADER_LEN], cmd_set_prefix, CMD_SET_PREFIX_LEN);
//    memcpy(&tx_buf[HEADER_LEN + CMD_SET_PREFIX_LEN], p_data, len);
//    crc_data = CRC32Calc(tx_buf, frame_length - CRC_DATA_LEN);
//    memcpy(&tx_buf[frame_length - CRC_DATA_LEN], &crc_data, CRC_DATA_LEN);
//    return frame_length;
//}
