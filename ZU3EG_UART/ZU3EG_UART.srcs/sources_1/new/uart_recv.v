`timescale 1ns / 1ps
// +FHDR---------------------------------------------------------------------------
// Copyrigth (c) 2024, SIMTECT
// Xingmai Confidential Proprietary
// --------------------------------------------------------------------------------
// FILE NAME		: uart_recv.v
// TYPE			: module
// DEPARTMENT		:
// AUTHOR		:
// AUTHOR'S EMAIL	:
// --------------------------------------------------------------------------------
// Release history
// VERSION	DATE		AUTHOR		DESCRIPTION
//
// --------------------------------------------------------------------------------
// KEYWORDS :  uart_recv
// --------------------------------------------------------------------------------
// PURPOSE :
//      接收串口数据
// -FHDR---------------------------------------------------------------------------

module uart_recv
#(
	parameter CLK_FRE = 50,         //时钟频率，默认时钟频率为50MHz
	parameter DATA_WIDTH = 8,       //有效数据位，缺省为8位
	parameter PARITY_ON = 0,        //校验位，1为有校验位，0为无校验位，缺省为0
	parameter PARITY_TYPE = 0,      //校验类型，1为奇校验，0为偶校验，缺省为偶校验
	parameter BAUD_RATE = 9600      //波特率，缺省为9600
)
(
	input                         i_clk_sys,        //系统时钟
	input                         i_rst_n,          //复位信号
	input                         i_rxd,            //输入串口bit
	output [DATA_WIDTH - 1:0]     o_rx_data,        //解析出的字节数据
	output                        o_ld_parity,      //校验位标志，高电平为校验正确
	output                        o_data_valid      //数据有效标志
);

localparam STATE_IDLE    = 3'b000;         //空闲状态
localparam STATE_START   = 3'b001;         //开始状态
localparam STATE_DATA    = 3'b011;         //数据接收状态
localparam STATE_PARITY  = 3'b100;         //数据校验状态
localparam STATE_END     = 3'b101;         //结束状态
localparam CYCLE = CLK_FRE * 1000000 / BAUD_RATE;   //波特计数周期

//--------------------------------------------------------------------
// 定义变量
//--------------------------------------------------------------------
reg                      rxd_r1,rxd_r2,rxd_r3;
reg  [4:0]               flag_rcv_start  = 'b11111;
reg  [2:0]               current_state   = 'd0;     //当前状态
reg  [2:0]               next_state      = 'd0;     //次态
reg                      baud_valid      = 'b0;     //波特计数有效位
reg  [15:0]              baud_cnt        = 'd0;     //波特率计数器
reg                      baud_pulse      = 'b0;     //波特率采样脉冲
reg  [3:0]               rcv_cnt         = 'd0;     //接收数据位计数
reg  [DATA_WIDTH - 1:0]  data_rcv;
reg                      parity_check;
reg  [DATA_WIDTH - 1:0]  rx_data;
reg                      ld_parity;
reg                      data_valid;

//--------------------------------------------------------------------
// I/O Connections assignments
//--------------------------------------------------------------------
assign o_rx_data        = rx_data;
assign o_ld_parity      = ld_parity;
assign o_data_valid     = data_valid;

//--------------------------------------------------------------------
// 对输入单bit信号打节拍
//--------------------------------------------------------------------
always @ (posedge i_clk_sys) begin
	rxd_r1         <= i_rxd;
	rxd_r2         <= rxd_r1;
	rxd_r3         <= rxd_r2;
end

//--------------------------------------------------------------------
// 连续采样五个接收路电平来判断rx是否有信号传来
// 用五个采样信号来作为判断标准可以有效排除毛刺噪声带来的误判
//--------------------------------------------------------------------
always @ (posedge i_clk_sys) begin
	if(!i_rst_n) begin
		flag_rcv_start <= 'b11111;
	end
	else begin
		flag_rcv_start <= {flag_rcv_start[3:0],rxd_r3};
	end
end

//--------------------------------------------------------------------
// 波特率计数器和波特采样脉冲（位于数据中心）
//--------------------------------------------------------------------
//波特率计数器
always @ (posedge i_clk_sys) begin
	if(!i_rst_n) begin
		baud_cnt <= 'd0;
	end
        else if(!baud_valid) begin
		baud_cnt <= 'd0;
	end
        else if(baud_cnt == CYCLE - 1) begin
		baud_cnt <= 'd0;
	end
        else begin
                baud_cnt <= baud_cnt + 'd1;
	end
end

//波特采样脉冲
always @ (posedge i_clk_sys) begin
	if(!i_rst_n) begin
		baud_pulse <= 'b0;
	end
	else if(baud_cnt == CYCLE/2-7) begin
                baud_pulse <= 'b1;
	end
	else begin
                baud_pulse <= 'b0;
        end
end

//--------------------------------------------------------------------
// 数据接收状态机
//--------------------------------------------------------------------
//状态机状态变化定义
always @ (posedge i_clk_sys) begin
	if(!i_rst_n) begin
		current_state <= STATE_IDLE;
	end
	else if(!baud_valid) begin
		current_state <= STATE_IDLE;
	end
	else if(current_state == STATE_END) begin
		if(baud_valid && baud_cnt == CYCLE - 'd2) begin   //【20250705】提前2个时钟周期进入IDLE，防止连续收数据导致baud_pulse错位
			current_state <= next_state;
		end
	end
	else if(baud_valid && baud_cnt == 'd0) begin
		current_state <= next_state;
        end
end

//状态机次态定义
always @ (*) begin
	case(current_state)
		STATE_IDLE: begin
			next_state <= STATE_START;
		end
		STATE_START: begin
			next_state <= STATE_DATA;
		end
		STATE_DATA: begin
			if(rcv_cnt == DATA_WIDTH) begin
				if(PARITY_ON == 0) begin          //校验位未启用，直接结束
					next_state <= STATE_END;
				end
				else begin
					next_state <= STATE_PARITY;       //校验位开启时进入校验状态
				end
                        end
			else begin
				next_state <= STATE_DATA;
                        end
		end
		STATE_PARITY: begin
			next_state <= STATE_END;
		end
		STATE_END: begin
			next_state <= STATE_IDLE;
		end
		default: begin
			next_state <= STATE_IDLE;
		end
	endcase
end

//状态机输出逻辑
always @ (posedge i_clk_sys) begin
	if(!i_rst_n) begin
		baud_valid    <= 'b0;
		data_rcv      <= 'd0;
		rcv_cnt       <= 'd0;
		parity_check  <= 'b0;
		rx_data       <= 'd0;
		ld_parity     <= 'b0;
		data_valid    <= 'b0;
	end
	else begin
		case(current_state)
			STATE_IDLE: begin
                            //闲置状态下对寄存器进行复位
			    rcv_cnt      <= 'd0;
                            data_rcv     <= 'd0;
                            parity_check <= 'b0;
                            data_valid   <= 'b0;
                            //连续检测到低电平时认为UART传来数据，拉高baud_valid
			    if(flag_rcv_start == 'b00000) begin
				    baud_valid  <= 'b1;
			    end
                        end
			STATE_START: begin
				if(baud_pulse && rxd_r3) begin     //波特率采样脉冲到来时再次检测是否为低电平，如果不为低电平，认为前期误检测，重新进入IDLE状态
					baud_valid <= 'b0;
				end
                        end
			STATE_DATA: begin
				if(baud_pulse) begin
					data_rcv     <= {rxd_r3,data_rcv[DATA_WIDTH-1 : 1]};    //数据移位存储
					rcv_cnt      <= rcv_cnt + 'd1;                          //数据位计数
					parity_check <= parity_check + rxd_r3;                  //校验位做加法验证高电平的奇偶
                                end
                        end
			STATE_PARITY: begin  //校验检测，正确则ld_parity拉高，可输出给led检测，如果闪烁则表示有错误数据发生
				if(baud_pulse) begin
					if(parity_check + rxd_r3 == PARITY_TYPE) begin
						ld_parity <= 'b1;
					end
					else begin
						ld_parity <= 1'b0;
					end
                                end
				else begin
                                        ld_parity  <= ld_parity;
				end
                        end
			STATE_END: begin
				if(baud_pulse) begin
					if(PARITY_ON == 0 || ld_parity) begin   //没有校验位或者校验位正确时才输出数据，否则直接丢弃数据
                                            rx_data      <= data_rcv;
                                            data_valid   <= 'b1;
                                        end
                                end
				else begin
					data_valid   <= 'b0;
                                end

				if(baud_cnt == CYCLE - 'd2) begin
					baud_valid <= 1'b0;
				end
			end
			default: begin
				rcv_cnt      <= 'd0;
				data_rcv     <= 'd0;
				parity_check <= 'b0;
				data_valid   <= 'b0;
				baud_valid   <= 'b0;
			end
		endcase
	end
end

endmodule
