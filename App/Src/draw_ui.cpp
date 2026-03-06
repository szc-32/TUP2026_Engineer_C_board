/**
 ******************************************************************************
 * @file    draw_ui.cpp
 * @author  Xushuang
 * @version V1.0.0 Xushuang 基本完成
 *			
 * @date    2023/9/22
 * @brief		此处为UI任务定义
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "draw_ui.h"
#include "cmsis_os.h"
#include "string.h"
#include "super_cap.h"
#define UI_UART huart3
//UI信息
UI_Pub_Msg_t ui_msg;
UI_t ui;
//UI初始化
void UiInit()
{
	CenterPointer()->PointerInit(&ui_msg,UIPUB);
}

//声明
void CharLayerSend(ext_charstring_data_t *tx_char,void (*CharCallback)());
void GraphicLayerSend(ext_graphic_seven_data_t *tx_graphic,void (*GraphicCallback)());
void GraphicLayerSend(ext_graphic_seven_data_t *tx_graphic,void (*GraphicCallback)(fp32[7]),fp32 send_data[7]);
void FloatLayerSend(ext_float_two_data_t *tx_float,void (*FloatCallback)(fp32,fp32),fp32 send_data1,fp32 send_data2);
void IntLayerSend(ext_int_two_data_t *tx_int,void (*IntCallback)(int,int),int send_data1,int send_data2);
//test
void GraphicGroup1LayerSend(ext_graphic_seven_data_t *tx_graphic);
void FloatGroup1LayerSend(ext_float_two_data_t *tx_float,fp32 send_data1,fp32 send_data2);
ext_charstring_data_t    tx_client_char;      //字符发送
ext_float_two_data_t     tx_group1_float;     //浮点数发送
ext_graphic_seven_data_t tx_group1_graphic;
ext_graphic_seven_data_t tx_group2_graphic;

uint8_t CliendTxBuffer[200];

//具体信息
void SetGraphicGroup1()
{
//	//自瞄
//	Figure_Graphic(&tx_group1_graphic.clientData[0],"Tar",ADD,CIRCLE,2,GREEN,0,0,2,(x),(y),radius,0,0);
//	Figure_Graphic(&tx_group1_graphic.clientData[1],"Pre",ADD,CIRCLE,3,YELLOW,0,0,1,(pre_x),(pre_y),pre_radius,0,0);
	
	
}
void UiTask(void const *pvParameters)
{
	uint16_t ui_heart_cnt = 0;
	uint16_t init_step = 0;
	while(1)
	{
		vTaskDelay(2);
		ui_heart_cnt++;
		//初始化
		if(init_step == 0)
		{
			if(ui_heart_cnt <= 20)//Pitch轴字符初始化
			{
				CharLayerSend(&tx_client_char,&UI_t::PitchCharInit);
				continue;
			}
			if(ui_heart_cnt <= 40)//超级电容字符初始化
			{
				CharLayerSend(&tx_client_char,&UI_t::SuperCapCharInit);
				continue;
			}
			if(ui_heart_cnt <= 60)//发射机构字符初始化
			{
				CharLayerSend(&tx_client_char,&UI_t::FirCharInit);
				continue;
			}
			if(ui_heart_cnt <= 80)//自瞄字符初始化
			{
				CharLayerSend(&tx_client_char,&UI_t::AutoCharInit);
				continue;
			}
			if(ui_heart_cnt <= 100)//小陀螺字符初始化
			{
				CharLayerSend(&tx_client_char,&UI_t::SpinCharInit);
				continue;
			}
			if(ui_heart_cnt <= 120)//六轴归位字符初始化
			{
				CharLayerSend(&tx_client_char,&UI_t::HomeCharInit);
				continue;
			}
			if(ui_heart_cnt <= 140)//射击模式字符初始化
			{
				CharLayerSend(&tx_client_char,&UI_t::ShootModeInit);
				continue;
			}
			if(ui_heart_cnt <= 160)//剩余能量
			{
//			     FloatLayerSend(&tx_group1_float,&UI_t::FloatGroup1Init,ChassisPointer()->gimbal_pit_angle,GetCapPointer()->robofuture_cap.ele_quantity);
				 continue;
			}
			if(ui_heart_cnt <=200)
			{
				GraphicLayerSend(&tx_group1_graphic,&UI_t::GraphicGroup1Init);  //圆圈初始化
				continue;
			}
//			if(ui_heart_cnt <=200)
//			{
////				Client_graphic_aim_line_init();  //圆圈初始化
//				continue;
//			}
			init_step = 1;
		}
		
		if(ui_heart_cnt % 7 == 0)//发射模式更新
		{
			CharLayerSend(&tx_client_char,&UI_t::ShootModeUpdate);
			continue;
		}
		
		if(ui_heart_cnt % 3 == 0)//状态更新
		{
			GraphicLayerSend(&tx_group1_graphic,&UI_t::GraphicGroup1Update);  //圆圈更新
			continue;
		}
		
		if(ui_heart_cnt % 4 == 0)//
		{
//			FloatLayerSend(&tx_group1_float,&UI_t::FloatGroup1Update,ChassisPointer()->gimbal_pit_angle,GetCapPointer()->robofuture_cap.ele_quantity);
			continue;
		}
		
//		if(ui_heart_cnt % 3 == 0)
//		{
//			
//			continue;
//		}
	}
}
/********字符更新函数*************/
void UI_t::PitchCharInit()
{
	char pitch_line[10]= {"PITCH:"};//pitch轴角度,float
	Char_Graphic(&tx_client_char.clientData,"PC",ADD,0,ORANGE,20,strlen(pitch_line),2,(1920-200-200),(750),pitch_line);//第0层增加
}

void UI_t::SuperCapCharInit()
{
	char super_line[10]= {"VCAP:"};//超级电容剩余量,float
	Char_Graphic(&tx_client_char.clientData,"Cp",ADD,0,ORANGE,20,strlen(super_line),2,(1920-200-200),(590),super_line);//第0层增加
}

void UI_t::FirCharInit()
{
	char fir_line[10]= {"FIRC:"};//发射机构状态
	Char_Graphic(&tx_client_char.clientData,"Fir",ADD,0,ORANGE,20,strlen(fir_line),2,100,640,fir_line);//第0层增加
}

void UI_t::AutoCharInit()
{
	char auto_line[10]= {"AUTO:"};//自瞄状态
	Char_Graphic(&tx_client_char.clientData,"Auto",ADD,0,ORANGE,20,strlen(auto_line),2,100,840,auto_line);
}

void UI_t::SpinCharInit()
{
	char spin_line[10]= {"SPIN:"};//小陀螺状态
	Char_Graphic(&tx_client_char.clientData,"Spin",ADD,0,ORANGE,20,strlen(spin_line),2,100,740,spin_line);
}

void UI_t::HomeCharInit()
{
	char home_line[10]= {"HOME:"};//六轴归位状态
	Char_Graphic(&tx_client_char.clientData,"Home",ADD,0,ORANGE,20,strlen(home_line),2,100,940,home_line);
}

void UI_t::ShootModeInit()
{
	char shoot_line[10]= {"shoot1"};//发射模式
	Char_Graphic(&tx_client_char.clientData,"St",ADD,0,ORANGE,20,strlen(shoot_line),2,100,540,shoot_line);
}

void UI_t::ShootModeUpdate()
{
	char shoot_1_line[10]= {"shoot1"};//发射模式
	char shoot_3_line[10]= {"shoot3"};//发射模式
	char shoot_continue_line[10]= {"shoot++"};//发射模式
	if(RevolverPointer()->shoot_mode_set == 1)
	{
		Char_Graphic(&tx_client_char.clientData,"St",MODIFY,0,ORANGE,20,strlen(shoot_1_line),2,100,540,shoot_1_line);//修改为"shoot1"
	}else if(RevolverPointer()->shoot_mode_set == 2)
	{
		Char_Graphic(&tx_client_char.clientData,"St",MODIFY,0,FUCHSIA,20,strlen(shoot_3_line),2,100,540,shoot_3_line);//修改为"shoot3"
	}else if(RevolverPointer()->shoot_mode_set == 3)
	{
		Char_Graphic(&tx_client_char.clientData,"St",MODIFY,0,PINK,20,strlen(shoot_continue_line),2,100,540,shoot_continue_line);//修改为"shoot++"
	}
}
/********数字初始化函数*************/
void UI_t::FloatGroup1Init(fp32 first_data,fp32 second_data)
{
	Float_Graphic(&tx_group1_float.clientData[0],"Pit",ADD,FLOAT,4,CYAN_BLUE,30,1,3,((1920-350)),(700),(float)(first_data));
	Float_Graphic(&tx_group1_float.clientData[1],"Sc" ,ADD,FLOAT,4,CYAN_BLUE,30,1,3, (1920-350),  540,( float)(second_data));
}

/********数字更新函数*************/
void UI_t::FloatGroup1Update(fp32 first_data,fp32 second_data)
{
	Float_Graphic(&tx_group1_float.clientData[0],"Pit",MODIFY,FLOAT,4,CYAN_BLUE,30,1,3,((1920-350)),(700),(float)(first_data));
	Float_Graphic(&tx_group1_float.clientData[1],"Sc", MODIFY,FLOAT,4,CYAN_BLUE,30,1,3, (1920-350),  540, (float)(second_data));
}

/********图形初始化函数*************/
void UI_t::GraphicGroup1Init()
{
	//弹舱盖
//	Figure_Graphic(&tx_group1_graphic.clientData[0],"Co",ADD,CIRCLE,2,GREEN,0,0,4,X_CIRCLE_O,540,20,0,0);
    //小陀螺
	Figure_Graphic(&tx_group1_graphic.clientData[0],"Sp", ADD,CIRCLE,2,GREEN ,0,0,4,X_CIRCLE_O,740,20,0,0);
    //自瞄
    Figure_Graphic(&tx_group1_graphic.clientData[1],"Au", ADD,CIRCLE,2,GREEN ,0,0,4,X_CIRCLE_O,840,20,0,0);
	//摩擦轮
	Figure_Graphic(&tx_group1_graphic.clientData[2],"Fi", ADD,CIRCLE,2,GREEN ,0,0,4,X_CIRCLE_O,640,20,0,0);
	//六轴归位
	Figure_Graphic(&tx_group1_graphic.clientData[5],"Hm", ADD,CIRCLE,2,RED_BLUE ,0,0,4,X_CIRCLE_O,940,20,0,0);

	//车体宽度
	Figure_Graphic(&tx_group1_graphic.clientData[3],"LCW",ADD ,LINE ,2,ORANGE,0,0,2, 700,8,0,850,281);
	Figure_Graphic(&tx_group1_graphic.clientData[4],"RCW",ADD ,LINE ,2,ORANGE,0,0,2, 1250,11,0,1135,283);
	
//	//自瞄（葡萄射手）
//	Figure_Graphic(&tx_group1_graphic.clientData[5],"Tar",ADD,CIRCLE,2,GREEN,0,0,2,0,0,0,0,0);
//	Figure_Graphic(&tx_group1_graphic.clientData[6],"Pri",ADD,CIRCLE,3,YELLOW,0,0,1,0,0,0,0,0);
	//可add一个图层
//	Figure_Graphic(&tx_group1_graphic.clientData[5],"FL",ADD,LINE,4,GREEN,0,0,3, 960+(int)200*arm_sin_f32((angle)*2*PI/360.0f),540+(int)200*arm_cos_f32((angle)*2*PI/360.0f),0,960+(int)270*arm_sin_f32((angle)*2*PI/360.0f),540+(int)270*arm_cos_f32((angle)*2*PI/360.0f));
}

void UI_t::GraphicGroup2Init()
{
//	tx_group2_graphic
}
/********图形更新函数*************/
void UI_t::GraphicGroup1Update()
{
	static uint8_t home_blink_div = 0;
	static uint8_t home_blink_on = 1;

	if(SysPointer()->key_flag.spin_flag)
	{
		Figure_Graphic(&tx_group1_graphic.clientData[0],"Sp",MODIFY,CIRCLE,2,CYAN_BLUE,0,0,4,X_CIRCLE_O,740,20,0,0);
	}
	else
	{
		Figure_Graphic(&tx_group1_graphic.clientData[0],"Sp",MODIFY,CIRCLE,2,GREEN,0,0,4,X_CIRCLE_O,740,20,0,0);
	}

	if(SysPointer()->key_flag.auto_aim_flag)
	{
		Figure_Graphic(&tx_group1_graphic.clientData[1],"Au",MODIFY,CIRCLE,2,CYAN_BLUE,0,0,4,X_CIRCLE_O,840,20,0,0);
	}
	else
	{
		Figure_Graphic(&tx_group1_graphic.clientData[1],"Au",MODIFY,CIRCLE,2,GREEN,0,0,4,X_CIRCLE_O,840,20,0,0);
	}

	if(SysPointer()->key_flag.fir_flag)
	{
		Figure_Graphic(&tx_group1_graphic.clientData[2],"Fi",MODIFY,CIRCLE,2,CYAN_BLUE,0,0,4,X_CIRCLE_O,640,20,0,0);
	}
	else
	{
		Figure_Graphic(&tx_group1_graphic.clientData[2],"Fi",MODIFY,CIRCLE,2,GREEN,0,0,4,X_CIRCLE_O,640,20,0,0);
	}

	if(SysPointer()->arm_home_done)
	{
		home_blink_div = 0;
		home_blink_on = 1;
		Figure_Graphic(&tx_group1_graphic.clientData[5],"Hm",MODIFY,CIRCLE,2,GREEN,0,0,4,X_CIRCLE_O,940,20,0,0);
	}
	else
	{
		if (++home_blink_div >= 10)
		{
			home_blink_div = 0;
			home_blink_on = !home_blink_on;
		}

		if (home_blink_on)
		{
			Figure_Graphic(&tx_group1_graphic.clientData[5],"Hm",MODIFY,CIRCLE,2,RED_BLUE,0,0,4,X_CIRCLE_O,940,20,0,0);
		}
		else
		{
			Figure_Graphic(&tx_group1_graphic.clientData[5],"Hm",MODIFY,CIRCLE,2,BLACK,0,0,4,X_CIRCLE_O,940,20,0,0);
		}
	}
//	//车体宽度
//	Figure_Graphic(&tx_group1_graphic.clientData[3],"LCW",MODIFY,LINE,2,ORANGE,0,0,2,625,8,0,775,281);
//	Figure_Graphic(&tx_group1_graphic.clientData[4],"RCW",MODIFY,LINE,2,ORANGE,0,0,2,1190,11,0,1045,283);
	//自瞄（葡萄射手）（单板从视觉获取数据后的处理在communicate文件中）
//	Figure_Graphic(&tx_group1_graphic.clientData[5],"Tar",ADD,CIRCLE,2,GREEN,0,0,2,(x),(y),radius,0,0);
//	Figure_Graphic(&tx_group1_graphic.clientData[6],"Pri",ADD,CIRCLE,3,YELLOW,0,0,1,(pre_x),(pre_y),pre_radius,0,0);
	//可add一个图层
//	Figure_Graphic(&tx_group1_graphic.clientData[5],"FL",MODIFY,LINE,4,GREEN,0,0,3, 960+(int)200*arm_sin_f32((angle)*2*PI/360.0f),540+(int)200*arm_cos_f32((angle)*2*PI/360.0f),0,960+(int)270*arm_sin_f32((angle)*2*PI/360.0f),540+(int)270*arm_cos_f32((angle)*2*PI/360.0f));

}

void UI_t::GraphicGroup2Update()
{
	
}

//字符图层发送
void CharLayerSend(ext_charstring_data_t *tx_char,void (*CharCallback)())//外部放入的数组)
{
	tx_char->txFrameHeader.SOF = JUDGE_FRAME_HEADER;
	tx_char->txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_string_t);
	tx_char->txFrameHeader.Seq = 0;//包序号
	memcpy(CliendTxBuffer,&tx_char->txFrameHeader,sizeof(xFrameHeader));
	append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//头校验
	
	//命令码
	tx_char->CmdID = ID_robot_interactive_header_data;
		
	//数据段头结构
	tx_char->dataFrameHeader.data_cmd_id = INTERACT_ID_draw_char_graphic;
	tx_char->dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
	tx_char->dataFrameHeader.receiver_ID = REF.self_client;
		
	CharCallback();
	memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_char->CmdID, LEN_CMD_ID+tx_char->txFrameHeader.DataLength);//加上命令码长度2
		
	//帧尾
	append_CRC16_check_sum(CliendTxBuffer,sizeof(ext_charstring_data_t));
		
	USARTSend(&UI_UART,CliendTxBuffer,sizeof(ext_charstring_data_t),USART_TRANSFER_BLOCKING);
}
//test
void GraphicGroup1LayerSend(ext_graphic_seven_data_t *tx_graphic)
{
	tx_graphic->txFrameHeader.SOF = JUDGE_FRAME_HEADER;
	tx_graphic->txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)*7;
	tx_graphic->txFrameHeader.Seq = 0;//包序号
	memcpy(CliendTxBuffer,&tx_graphic->txFrameHeader,sizeof(xFrameHeader));
	append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//头校验
	
	//命令码
	tx_graphic->CmdID = ID_robot_interactive_header_data;
		
	//数据段头结构
	tx_graphic->dataFrameHeader.data_cmd_id = INTERACT_ID_draw_seven_graphic;
	tx_graphic->dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
	tx_graphic->dataFrameHeader.receiver_ID = REF.self_client;
		
//	GraphicCallback(send_data);
	ui.GraphicGroup1Update();
	memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_graphic->CmdID, LEN_CMD_ID+tx_graphic->txFrameHeader.DataLength);//加上命令码长度2
		
	//帧尾
	append_CRC16_check_sum(CliendTxBuffer,sizeof(ext_graphic_seven_data_t));
	USARTSend(&UI_UART,CliendTxBuffer,sizeof(ext_graphic_seven_data_t),USART_TRANSFER_BLOCKING);
}
//test
void FloatGroup1LayerSend(ext_float_two_data_t *tx_float,fp32 send_data1,fp32 send_data2)
{
	tx_float->txFrameHeader.SOF = JUDGE_FRAME_HEADER;
	tx_float->txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)*2;
	tx_float->txFrameHeader.Seq = 0;//包序号
	memcpy(CliendTxBuffer,&tx_float->txFrameHeader,sizeof(xFrameHeader));
	append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//头校验
	
	//命令码
	tx_float->CmdID = ID_robot_interactive_header_data;
		
	//数据段头结构
	tx_float->dataFrameHeader.data_cmd_id = INTERACT_ID_draw_two_graphic;
	tx_float->dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
	tx_float->dataFrameHeader.receiver_ID = REF.self_client;
		
	ui.FloatGroup1Update(send_data1,send_data2);
	memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_float->CmdID, LEN_CMD_ID+tx_float->txFrameHeader.DataLength);//加上命令码长度2
		
	//帧尾
	append_CRC16_check_sum(CliendTxBuffer,sizeof(ext_float_two_data_t));
	USARTSend(&UI_UART,CliendTxBuffer,sizeof(ext_float_two_data_t),USART_TRANSFER_BLOCKING);
}

void GraphicLayerSend(ext_graphic_seven_data_t *tx_graphic,void (*GraphicCallback)())
{
	tx_graphic->txFrameHeader.SOF = JUDGE_FRAME_HEADER;
	tx_graphic->txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)*7;
	tx_graphic->txFrameHeader.Seq = 0;//包序号
	memcpy(CliendTxBuffer,&tx_graphic->txFrameHeader,sizeof(xFrameHeader));
	append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//头校验
	
	//命令码
	tx_graphic->CmdID = ID_robot_interactive_header_data;
		
	//数据段头结构
	tx_graphic->dataFrameHeader.data_cmd_id = INTERACT_ID_draw_seven_graphic;
	tx_graphic->dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
	tx_graphic->dataFrameHeader.receiver_ID = REF.self_client;
		
	GraphicCallback();
	memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_graphic->CmdID, LEN_CMD_ID+tx_graphic->txFrameHeader.DataLength);//加上命令码长度2
		
	//帧尾
	append_CRC16_check_sum(CliendTxBuffer,sizeof(ext_graphic_seven_data_t));
	USARTSend(&UI_UART,CliendTxBuffer,sizeof(ext_graphic_seven_data_t),USART_TRANSFER_BLOCKING);
}

void GraphicLayerSend(ext_graphic_seven_data_t *tx_graphic,void (*GraphicCallback)(fp32[7]),fp32 send_data[7])
{
	tx_graphic->txFrameHeader.SOF = JUDGE_FRAME_HEADER;
	tx_graphic->txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)*7;
	tx_graphic->txFrameHeader.Seq = 0;//包序号
	memcpy(CliendTxBuffer,&tx_graphic->txFrameHeader,sizeof(xFrameHeader));
	append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//头校验
	
	//命令码
	tx_graphic->CmdID = ID_robot_interactive_header_data;
		
	//数据段头结构
	tx_graphic->dataFrameHeader.data_cmd_id = INTERACT_ID_draw_seven_graphic;
	tx_graphic->dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
	tx_graphic->dataFrameHeader.receiver_ID = REF.self_client;
		
	GraphicCallback(send_data);
	memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_graphic->CmdID, LEN_CMD_ID+tx_graphic->txFrameHeader.DataLength);//加上命令码长度2
		
	//帧尾
	append_CRC16_check_sum(CliendTxBuffer,sizeof(ext_graphic_seven_data_t));
	USARTSend(&UI_UART,CliendTxBuffer,sizeof(ext_graphic_seven_data_t),USART_TRANSFER_BLOCKING);
}

void FloatLayerSend(ext_float_two_data_t *tx_float,void (*FloatCallback)(fp32,fp32),fp32 send_data1,fp32 send_data2)
{
	tx_float->txFrameHeader.SOF = JUDGE_FRAME_HEADER;
	tx_float->txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)*2;
	tx_float->txFrameHeader.Seq = 0;//包序号
	memcpy(CliendTxBuffer,&tx_float->txFrameHeader,sizeof(xFrameHeader));
	append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//头校验
	
	//命令码
	tx_float->CmdID = ID_robot_interactive_header_data;
		
	//数据段头结构
	tx_float->dataFrameHeader.data_cmd_id = INTERACT_ID_draw_two_graphic;
	tx_float->dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
	tx_float->dataFrameHeader.receiver_ID = REF.self_client;
		
	FloatCallback(send_data1,send_data2);
	memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_float->CmdID, LEN_CMD_ID+tx_float->txFrameHeader.DataLength);//加上命令码长度2
		
	//帧尾
	append_CRC16_check_sum(CliendTxBuffer,sizeof(ext_float_two_data_t));
	USARTSend(&UI_UART,CliendTxBuffer,sizeof(ext_float_two_data_t),USART_TRANSFER_BLOCKING);
}

void IntLayerSend(ext_int_two_data_t *tx_int,void (*IntCallback)(int,int),int send_data1,int send_data2)
{
	tx_int->txFrameHeader.SOF = JUDGE_FRAME_HEADER;
	tx_int->txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(graphic_data_struct_t)*2;
	tx_int->txFrameHeader.Seq = 0;//包序号
	memcpy(CliendTxBuffer,&tx_int->txFrameHeader,sizeof(xFrameHeader));
	append_CRC8_check_sum(CliendTxBuffer, sizeof(xFrameHeader));//头校验
	
	//命令码
	tx_int->CmdID = ID_robot_interactive_header_data;
		
	//数据段头结构
	tx_int->dataFrameHeader.data_cmd_id = INTERACT_ID_draw_two_graphic;
	tx_int->dataFrameHeader.send_ID     = REF.GameRobotStat.robot_id;
	tx_int->dataFrameHeader.receiver_ID = REF.self_client;
		
	IntCallback(send_data1,send_data2);
	memcpy(CliendTxBuffer+LEN_FRAME_HEAD, (uint8_t*)&tx_int->CmdID, LEN_CMD_ID+tx_int->txFrameHeader.DataLength);//加上命令码长度2
		
	//帧尾
	append_CRC16_check_sum(CliendTxBuffer,sizeof(ext_int_two_data_t));
	USARTSend(&UI_UART,CliendTxBuffer,sizeof(ext_int_two_data_t),USART_TRANSFER_BLOCKING);
}
